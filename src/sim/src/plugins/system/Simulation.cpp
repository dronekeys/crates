// Needed for SDF parsing
#include <tinyxml2.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo/msgs/contacts.pb.h>

// SDF
#include <sdf/sdf.hh>

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

// Global clock tick
#include <rosgraph_msgs/Clock.h>

//	World Wireless Handler
#include <dronkey_wireless/Wireless.h>

// ROS topics
#include "sim/Contacts.h"

// ROS services
#include "sim/Insert.h"
#include "sim/Delete.h"
#include "sim/Step.h"
#include "sim/Reset.h"
#include "sim/Resume.h"
#include "sim/Pause.h"
#include "sim/Noise.h"
#include "sim/Seed.h"
#include "sim/Wireless.h"

// Protobuf messages
#include "noise.pb.h"

#define ROS_TIMEOUT_SECONDS 1.0

namespace gazebo
{
  typedef const boost::shared_ptr<const msgs::Contacts> ContactsPtr;

  class Simulation : public SystemPlugin
  {

  private:

    // Plugin state management
    bool loaded, created, stop;

	// Used to queu incoming requests
  	boost::shared_ptr<boost::thread> threadQueue;

    // Lock access to fields that are used in ROS message callbacks
    boost::mutex lock;

    // ROS comm
    boost::shared_ptr<ros::NodeHandle>    rosNode;
    boost::shared_ptr<ros::AsyncSpinner>  async;

    // Gazebo event handling
    event::ConnectionPtr eventSigint, eventLoad, eventClock;

    // gazebo world
    physics::WorldPtr world;

    // For communication on gazebo backbone
    transport::NodePtr gazeboNode;
    transport::PublisherPtr pubFactory, pubRequest, pubNoise;
    transport::SubscriberPtr subResponse, subContacts;

	// For injecting an ID into model code
	sdf::SDFPtr sdfPtr;

    // Used to buffer callbacks
  	ros::CallbackQueue queue;

  	// The clock will be published by the simulator
  	ros::Publisher topicClock, topicContacts;

  	// Five services we will offer to users of the simulator
	ros::ServiceServer serviceReset, serviceResume, servicePause, serviceWireless,
		serviceInsert, serviceDelete,serviceStep, serviceNoise, serviceSeed;

	// Two time representations
	rosgraph_msgs::Clock timeRos;

	// For storing collisions
	sim::Contacts msgContacts;

	//Wireless Simulator
	dronkey::Wireless wirelessSimulator;

  public:

    // Constructor
    Simulation() : created(false), stop(false), loaded(false)
    {
    	sdfPtr.reset(new sdf::SDF);
    }

    // Destructor
    ~Simulation()
    {
		// Unload the sigint event
		event::Events::DisconnectSigInt(eventSigint);

		// Don't attempt to unload this plugin if it was never loaded in the Load() function
		if(!loaded)
			return;
		  
		// Stop the multi threaded ROS spinner
		async->stop();

		// Shutdown the ROS node
		rosNode->shutdown();

		// Shutdown ROS queue
		threadQueue->join();
    }

    // Called when CTRL+C is pressed
    void Shutdown()
    {
      stop = true;
    }

    // Called to load the plugin
    void Load(int argc, char** argv)
    {
		// When a SIGINT is receiver, send the sudown signal
		eventSigint = event::Events::ConnectSigInt(
			boost::bind(&Simulation::Shutdown,this)
		);

		// Start ROS without SIGINT ability
		if (!ros::isInitialized())
			ros::init(argc,argv,"sim",ros::init_options::NoSigintHandler);
		else
			ROS_ERROR("Something started ros::init(...) prior to libbssim Load()");

		// Wait until the ROS master becomes available
		while(!ros::master::check())
		{
			usleep(500*1000);
			if(stop)
		  		return;
		}

		// Advertise topics and services in this node's namespace
		rosNode.reset(new ros::NodeHandle("~")); 

		// Built-in multi-threaded ROS spinnin
		async.reset(new ros::AsyncSpinner(0)); 
		async->start();

		/// setup custom callback queue
		threadQueue.reset(
			new boost::thread(&Simulation::SetupThreadQueue, this)
		);

		// On connecting to the world, initialise all services
		eventLoad = event::Events::ConnectWorldCreated(
			boost::bind(&Simulation::Init,this,_1)
		);

		eventClock = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&Simulation::Tick,this)
		);

		// Load success
		loaded = true;
    }

    // Called when world is connected to
    void Init(std::string world_name)
    {
		// Ensures that this is called only once
		event::Events::DisconnectWorldCreated(eventLoad);
		lock.lock();
		if (created)
		{
			lock.unlock();
			return;
		}
		created = true;
		lock.unlock();

		// Check to see that the world is valid
		world = physics::get_world(world_name);
		if (!world)
		{
			ROS_FATAL("physics::get_world() fails to return a valid world pointer");
			return;
		}

		// Initialise the messaging 
		gazeboNode = transport::NodePtr(new transport::Node());
		gazeboNode->Init(world_name);

		// Set up gazebo publishers
		pubFactory  = gazeboNode->Advertise<msgs::Factory>("~/factory");
		pubRequest  = gazeboNode->Advertise<msgs::Request>("~/request");
		pubNoise    = gazeboNode->Advertise<msgs::Noise>("~/noise");
		subResponse = gazeboNode->Subscribe("~/response",&Simulation::Response, this);
		subContacts = gazeboNode->Subscribe("~/physics/contacts",&Simulation::Contacts, this);

	  	// Publish clock for simulated ros time
		topicClock = rosNode->advertise<rosgraph_msgs::Clock>("/clock",10);

	  	// Add a model
		ros::AdvertiseServiceOptions adInsert = ros::AdvertiseServiceOptions::create<sim::Insert>(
			"Insert",boost::bind(&Simulation::Insert,this,_1,_2),ros::VoidPtr(), &queue
		);
		serviceInsert = rosNode->advertiseService(adInsert);

	  	// Delete a model
		ros::AdvertiseServiceOptions adDelete = ros::AdvertiseServiceOptions::create<sim::Delete>(
			"Delete",boost::bind(&Simulation::Delete,this,_1,_2),ros::VoidPtr(), &queue
		);
		serviceDelete = rosNode->advertiseService(adDelete);

		// Advertise more services on the custom queue
		ros::AdvertiseServiceOptions adPause = ros::AdvertiseServiceOptions::create<sim::Pause>(
			"Pause",boost::bind(&Simulation::Pause,this,_1,_2),ros::VoidPtr(), &queue
		);
		servicePause = rosNode->advertiseService(adPause);

		// Advertise more services on the custom queue
		ros::AdvertiseServiceOptions adResume = ros::AdvertiseServiceOptions::create<sim::Resume>(
			"Resume",boost::bind(&Simulation::Resume,this,_1,_2),ros::VoidPtr(), &queue
		);
		serviceResume = rosNode->advertiseService(adResume);

		// Advertise more services on the custom queue
		ros::AdvertiseServiceOptions adReset = ros::AdvertiseServiceOptions::create<sim::Reset>(
			"Reset",boost::bind(&Simulation::Reset,this,_1,_2),ros::VoidPtr(), &queue
		);
		serviceReset = rosNode->advertiseService(adReset);

		// Advertise more services on the custom queue
		ros::AdvertiseServiceOptions adStep = ros::AdvertiseServiceOptions::create<sim::Step>(
			"Step",boost::bind(&Simulation::Step,this,_1,_2),ros::VoidPtr(), &queue
		);
		serviceStep = rosNode->advertiseService(adStep);

		// Advertise more services on the custom queue
		ros::AdvertiseServiceOptions adNoise = ros::AdvertiseServiceOptions::create<sim::Noise>(
			"Noise",boost::bind(&Simulation::Noise,this,_1,_2),ros::VoidPtr(), &queue
		);
		serviceNoise = rosNode->advertiseService(adNoise);

		// Advertise more services on the custom queue
		ros::AdvertiseServiceOptions adSeed = ros::AdvertiseServiceOptions::create<sim::Seed>(
			"Seed",boost::bind(&Simulation::Seed,this,_1,_2),ros::VoidPtr(), &queue
		);
		serviceSeed = rosNode->advertiseService(adSeed);

		//Receiver
		ros::AdvertiseServiceOptions adSendPacket = ros::AdvertiseServiceOptions::create<sim::Wireless>(
			"Wireless",boost::bind(&Simulation::Wireless,this,_1,_2),ros::VoidPtr(), &queue
		);
		serviceWireless = rosNode->advertiseService(adSendPacket);

	  	// Publish clock for simulated ros time
		topicContacts = rosNode->advertise<sim::Contacts>("Contacts",10);

		//Initialize Wireless
		wirelessSimulator.SetWorld(world);
		
		// Set param for use_sim_time if not set by user already
		rosNode->setParam("/use_sim_time",  true);
    }

    // Setup the thread queue
	void SetupThreadQueue()
	{
		while (rosNode->ok())
			queue.callAvailable(ros::WallDuration(0.001));
	}

    // Called on every time tick
	void Tick()
	{
		// Convert to a ros time
		timeRos.clock.fromSec(world->GetSimTime().Double());
	  	
	  	//  publish time to ros
		topicClock.publish(timeRos);
	}

    // When a response is received from the gazebo server
    void Response(ConstResponsePtr&response)
    {
      // Do nothing
    }

    // Subscribe to messages
    void Contacts(ContactsPtr& msg)
    {
    	// Only publish if we actually have some contacts!
    	if (msg->contact_size() > 0)
    	{
	    	// Clear existing contacts
			msgContacts.contacts.clear();

	  	    // Add the current contacts
			for (int i = 0; i < msg->contact_size(); i++)
			{
				sim::Contact contact;
				contact.name1 = msg->contact(i).collision1();
				contact.name2 = msg->contact(i).collision2();
				msgContacts.contacts.push_back(contact);
			}

		  	//  publish time to ros
			topicContacts.publish(msgContacts);
		}
    }

    ////////////////////////////////////////////////////////////////////////////////

	bool Insert(sim::Insert::Request &req, sim::Insert::Response &res)
	{
		// Resolve the model file name
  		std::string filename = common::ModelDatabase::Instance()->GetModelFile(req.model_type);
		
		// For XML
		tinyxml2::XMLDocument   xmlDocument;
		tinyxml2::XMLPrinter    xmlPrinter;

		// Create a new tinyxml document
		if (xmlDocument.LoadFile(filename.c_str()))
		{
			res.success = false;
			res.status_message = std::string("Could not load model file.");
			return true;
		}

		// Get the first <sdf> element
  		tinyxml2::XMLElement* xmlElementSDF = xmlDocument.FirstChildElement("sdf");
		if (!xmlElementSDF)
		{
			res.success = false;
			res.status_message = std::string("No <sdf> tag in model file.");
			return true;
		}
		
		// Get the first <model> element
  		tinyxml2::XMLElement* xmlElementMODEL = xmlElementSDF->FirstChildElement("model");
		if (!xmlElementMODEL)
		{
			res.success = false;
			res.status_message = std::string("No <model> tage in model file.");
			return true;
		}

		// Update the model name
		xmlElementMODEL->SetAttribute("name",req.model_name.c_str());

		// Extract the XML data
		xmlDocument.Print(&xmlPrinter);

		// Create and publish the message
		msgs::Factory msg;
		msg.set_sdf(xmlPrinter.CStr());
		pubFactory->Publish(msg);
		
		// Wait until the model has been deleted
		ros::Time timeout = ros::Time::now() + ros::Duration(ROS_TIMEOUT_SECONDS);
		while (true)
		{
			if (ros::Time::now() > timeout)
			{
				res.success = false;
				res.status_message = std::string("Added to the insertion queue, but failed to appear in the world. Did you forget the model:// prefix?");
				return true;
			}
			if (world->GetModel(req.model_name)) 
				break;
			ROS_DEBUG("Waiting for model insertion (%s)",req.model_name.c_str());
			usleep(1000);
		}

		res.success = true;
		res.status_message = std::string("Successfully inserted model");
		return true;
	}

	bool Delete(sim::Delete::Request &req, sim::Delete::Response &res)
	{
	  	// Clear forces, etc for the body in question
		physics::ModelPtr model = world->GetModel(req.model_name);
		if (!model)
		{
			ROS_ERROR("Delete: model [%s] does not exist",req.model_name.c_str());
			res.success = false;
			res.status_message = "Cannot delete model, because it does not exist";
			return true;
		}

	  	// Send delete model request
		msgs::Request *msg = msgs::CreateRequest("entity_delete",req.model_name);
		pubRequest->Publish(*msg,true);

		// Wait until the model has been deleted
		ros::Time timeout = ros::Time::now() + ros::Duration(ROS_TIMEOUT_SECONDS);
		while (true)
		{
			if (ros::Time::now() > timeout)
			{
				res.success = false;
				res.status_message = std::string("Model pushed to delete queue, but delete service timed out waiting for model to disappear from simulation");
				return true;
			}
			if (!world->GetModel(req.model_name)) 
				break;
			ROS_DEBUG("Waiting for model deletion (%s)",req.model_name.c_str());
			usleep(1000);
		}
		res.success = true;
		res.status_message = std::string("Successfully deleted model");
		return true;
	}

	// Reset the world
	bool Reset(sim::Reset::Request &req,sim::Reset::Response &res)
	{
		world->Reset();
		res.success = true;
		res.status_message = std::string("Successfully paused the simulation.");
		return true;
	}

	// Pause physics
	bool Pause(sim::Pause::Request &req,sim::Pause::Response &res)
	{
		world->SetPaused(true);
		res.success = true;
		res.status_message = std::string("Successfully paused the simulation.");
		return true;
	}

	// Resume physics
	bool Resume(sim::Resume::Request &req,sim::Resume::Response &res)
	{
		world->SetPaused(false);
		res.success = true;
		res.status_message = std::string("Successfully resumed the simulation");
		return true;
	}

	// Resume physics
	bool Step(sim::Step::Request &req, sim::Step::Response &res)
	{
		// Keep steps to a reasonable length
		if (req.num_steps > 128)
		{
			res.success = false;
			res.status_message = std::string("Number of steps must be less than 128");
			return true;
		}

		// Step the world forward
		world->Step(req.num_steps);

		// Error message
		res.success = true;
		res.status_message = std::string("Successfully stepped the simulaor");
		return true;
	}

	// Resume physics
	bool Noise(sim::Noise::Request &req, sim::Noise::Response &res)
	{
		// Create and publish the message to gazebo
		msgs::Noise msg;
		msg.set_enable(req.enable);
		pubNoise->Publish(msg);

		// Keep steps to a reasonable length
		res.success = true;
		res.status_message = std::string("Noise message sent");	
		return true;
	}

	// Resume physics
	bool Seed(sim::Seed::Request &req, sim::Seed::Response &res)
	{
		// Create and publish the message
		math::Rand::SetSeed(req.seed);

		// Keep steps to a reasonable length
		res.success = true;
		res.status_message = std::string("Random generated seeded");
	}

	bool Wireless(sim::Wireless::Request &req, sim::Wireless::Response &res)
	{
		//Handle Wireless
		wirelessSimulator.Send();
		return true;
	}

  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(Simulation)

}