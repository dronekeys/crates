// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// ROS
#include <ros/ros.h>

namespace gazebo
{
  class Experiment : public SystemPlugin
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

  public:

    // Constructor
    Experiment() : created(false), stop(false), loaded(false)
    {
      // Do nothing
    }

    // Destructor
    ~Experiment()
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
        boost::bind(&Experiment::Shutdown,this)
      );

      // Start ROS without SIGINT ability
      if (!ros::isInitialized())
        ros::init(argc,argv,"bs",ros::init_options::NoSigintHandler);
      else
        ROS_ERROR("Something started ros::init(...) prior to libbsexp Load()");

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

      // On connecting to the world, initialise all services
      eventLoad = event::Events::ConnectWorldCreated(
        boost::bind(&Experiment::Init,this,_1)
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
      subResponse = gazeboNode->Subscribe("~/response",&Experiment::Response, this);

      // Set param for use_sim_time if not set by user already
      rosNode->setParam("/use_sim_time", false);
    }

    // Subscribe to messages
    void Response(ConstResponsePtr &response)
    {
      // Do nothing
    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(Experiment)

}