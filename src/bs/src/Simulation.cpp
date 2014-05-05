// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

// Global clock tick
#include <rosgraph_msgs/Clock.h>

// For blank requests
#include <std_srvs/Empty.h>

// Services
#include <bs/Insert.h>
#include <bs/Delete.h>
#include <bs/Step.h>

namespace gazebo
{
  class Simulation : public SystemPlugin
  {

  private:

    // Plugin state management
    bool loaded, created, stop;

    // Lock access to fields that are used in ROS message callbacks
    boost::mutex lock;

    // ROS comm
    boost::shared_ptr<ros::NodeHandle>    rosNode;
    boost::shared_ptr<ros::AsyncSpinner>  async;

    // Gazebo event handling
    event::ConnectionPtr  eventSigint;
    event::ConnectionPtr  eventLoad;

    // gazebo world
    physics::WorldPtr world;

    // For communication on gazebo backbone
    transport::NodePtr        gazeboNode;
    transport::PublisherPtr   pubFactory;
    transport::PublisherPtr   pubRequest;
    transport::SubscriberPtr  subResponse;

    // Used to buffer callbacks
  	ros::CallbackQueue queue;

  	// Used to queu incoming requests
  	boost::shared_ptr<boost::thread> threadQueue;

  	// The clock will be published by the simulator
  	ros::Publisher     		topicClock;

  	// Five services we will offer to users of the simulator
	ros::ServiceServer 		serviceReset;
	ros::ServiceServer 		serviceResume;
	ros::ServiceServer 		servicePause;
	ros::ServiceServer 		serviceInsert;
	ros::ServiceServer 		serviceDelete;
	ros::ServiceServer 		serviceStep;

	// Two time representations
	rosgraph_msgs::Clock 	timeRos;

  public:

    // Constructor
    Simulation() : created(false), stop(false), loaded(false)
    {
      // Do nothing
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
			ros::init(argc,argv,"bs",ros::init_options::NoSigintHandler);
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
		subResponse = gazeboNode->Subscribe("~/response",&Simulation::Response, this);

	  	// Publish clock for simulated ros time
		topicClock = rosNode->advertise<rosgraph_msgs::Clock>("/clock",10);

	  	// Add a model
		ros::AdvertiseServiceOptions adInsert = ros::AdvertiseServiceOptions::create<bs::Insert>(
			"insert",boost::bind(&Simulation::Insert,this,_1,_2),ros::VoidPtr(), &queue
		);
		serviceInsert = rosNode->advertiseService(adInsert);

	  	// Delete a model
		ros::AdvertiseServiceOptions adDelete = ros::AdvertiseServiceOptions::create<bs::Delete>(
			"delete",boost::bind(&Simulation::Delete,this,_1,_2),ros::VoidPtr(), &queue
		);
		serviceDelete = rosNode->advertiseService(adDelete);

		// Advertise more services on the custom queue
		ros::AdvertiseServiceOptions adPause = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
			"pause",boost::bind(&Simulation::Pause,this,_1,_2),ros::VoidPtr(), &queue
		);
		servicePause = rosNode->advertiseService(adPause);

		// Advertise more services on the custom queue
		ros::AdvertiseServiceOptions adResume = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
			"resume",boost::bind(&Simulation::Resume,this,_1,_2),ros::VoidPtr(), &queue
		);
		serviceResume = rosNode->advertiseService(adResume);

		// Advertise more services on the custom queue
		ros::AdvertiseServiceOptions adReset = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
			"reset",boost::bind(&Simulation::Reset,this,_1,_2),ros::VoidPtr(), &queue
		);
		serviceReset = rosNode->advertiseService(adReset);

		// Advertise more services on the custom queue
		ros::AdvertiseServiceOptions adStep = ros::AdvertiseServiceOptions::create<bs::Step>(
			"step",boost::bind(&Simulation::Step,this,_1,_2),ros::VoidPtr(), &queue
		);
		serviceStep = rosNode->advertiseService(adStep);

		// Set param for use_sim_time if not set by user already
		rosNode->setParam("/use_sim_time", true);
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

    // Subscribe to messages
    void Response(ConstResponsePtr &response)
    {
      // Do nothing
    }

    ////////////////////////////////////////////////////////////////////////////////

	bool Insert(bs::Insert::Request &req, bs::Insert::Response &res)
	{
		world->InsertModelFile((std::string)"model://"+req.model_type);
		res.success = true;
		res.status_message = std::string("Insert: successfully inserted model");
		return true;
	}

	bool Delete(bs::Delete::Request &req, bs::Delete::Response &res)
	{
	  	// Clear forces, etc for the body in question
		physics::ModelPtr model = world->GetModel(req.model_name);
		if (!model)
		{
			ROS_ERROR("Delete: model [%s] does not exist",req.model_name.c_str());
			res.success = false;
			res.status_message = "Delete: model does not exist";
			return true;
		}

	  	// Send delete model request
		msgs::Request *msg = msgs::CreateRequest("entity_delete",req.model_name);
		pubRequest->Publish(*msg,true);

		// Wait until the model has been deleted
		ros::Time timeout = ros::Time::now() + ros::Duration(60.0);
		while (true)
		{
			if (ros::Time::now() > timeout)
			{
				res.success = false;
				res.status_message = std::string("Delete: Model pushed to delete queue, but delete service timed out waiting for model to disappear from simulation");
				return true;
			}
			if (!world->GetModel(req.model_name)) 
				break;
			ROS_DEBUG("Waiting for model deletion (%s)",req.model_name.c_str());
			usleep(1000);
		}
		res.success = true;
		res.status_message = std::string("Delete: successfully deleted model");
		return true;
	}

	// Reset the world
	bool Reset(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
	{
		world->Reset();
		return true;
	}

	// Pause physics
	bool Pause(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
	{
		world->SetPaused(true);
		return true;
	}

	// Resume physics
	bool Resume(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
	{
		world->SetPaused(false);
		return true;
	}

	// Resume physics
	bool Step(bs::Step::Request &req, bs::Step::Response &res)
	{
		// Keep steps to a reasonable length
		if (req.num_steps > 128)
		{
			res.success = false;
			res.status_message = std::string("Step: number of steps must be less than 128");
			return true;
		}

		// Step the world forward
		world->Step(req.num_steps);

		// Error message
		res.success = true;
		res.status_message = std::string("Step: successfully stepped the simulaor");
		return true;
	}

  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(Simulation)

}