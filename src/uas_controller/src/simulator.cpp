// ROS includes
#include <ros/ros.h>

// Gazebo includes
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

// This package's services
#include <uas_controller/Resume.h>
#include <uas_controller/Reset.h>
#include <uas_controller/Pause.h>
#include <uas_controller/Insert.h>
#include <uas_controller/Remove.h>
#include <uas_controller/Step.h>

namespace uas_controller
{
    class Simulator : public gazebo::WorldPlugin
    {
  
    private:

      // Shared pointer to the gazebo world
      gazebo::physics::WorldPtr wp;

        // ROS node handle
      ros::NodeHandle nh;

      // Initialise services
      ros::ServiceServer ser_Pause;
      ros::ServiceServer ser_Reset;
      ros::ServiceServer ser_Resume;
      ros::ServiceServer ser_Insert;
      ros::ServiceServer ser_Remove;
      ros::ServiceServer ser_Step;

    public:
  
      // Initialise the Hardware Abstraction Layer
      void Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf)
      {
        // Make sure the ROS node for Gazebo has already been initialized                                                                                    
        if (!ros::isInitialized())
        {
          ROS_FATAL("A ROS node for Gazebo has not been initialized. Unable to load plugin.");
          return;
        }
        ROS_INFO("Simulator plugin loaded");

        // advertise topics and services in this node's namespace
        wp = _parent;
        nh = ros::NodeHandle("controller"); 

        // Initialise services
        ser_Pause       = nh.advertiseService("Pause",      &Simulator::Pause,      this);  // pause simulation
        ser_Resume      = nh.advertiseService("Resume",     &Simulator::Resume,     this);  // resume simulation
        ser_Reset       = nh.advertiseService("Reset",      &Simulator::Reset,      this);  // reset simulation
        ser_Insert      = nh.advertiseService("Insert",     &Simulator::Insert,     this);  // insert a model
        ser_Remove      = nh.advertiseService("Remove",     &Simulator::Remove,     this);  // remove a model
        ser_Step        = nh.advertiseService("Step",       &Simulator::Step,       this);  // step the simulation
      }

      // Pause the simulator
      bool Pause(
        uas_controller::Pause::Request&  req,
        uas_controller::Pause::Response& res)
      {
        if (wp->IsPaused())
        {
          res.success = false;
          res.status_message = std::string("Pause: simulator cannot be paused, as it is already paused.");
        }
        else
        {
          res.success = true;
          res.status_message = std::string("Pause: simulator paused successfully");
          wp->SetPaused(true);
        }
        return true;
      }

      // Reset the simulator
      bool Reset(
        uas_controller::Reset::Request&  req, 
        uas_controller::Reset::Response& res)
      {
        wp->Reset();
        res.success = true;
        res.status_message = std::string("Reset: simulator reset successfully");
        return true;
      }

      // Resume the simulator
      bool Resume(
        uas_controller::Resume::Request&  req, 
        uas_controller::Resume::Response& res) 
      {
        if (wp->IsPaused())
        {
          res.success = true;
          res.status_message = std::string("Resume: simulator resumed successfully");
          wp->SetPaused(false);
        }
        else
        {
          res.success = false;
          res.status_message = std::string("Resume: simulator cannot be resumed, as it is not paused.");
        }
        return true;
      }

      // Step the simulator
      bool Step(
        uas_controller::Step::Request&  req, 
        uas_controller::Step::Response& res) 
      {
        if (req.epochs < 1)
        {
          res.success = false;
          res.status_message = std::string("Resume: positive integer epoch step count required");
        }
        else if (wp->IsPaused())
        {
          wp->StepWorld(req.epochs);
          res.success = true;
          res.status_message = std::string("Resume: simulator stepped successfully");
        }
        else
        {
          res.success = false;
          res.status_message = std::string("Resume: simulator cannot be stepped, as it is not paused.");
        }
        return true;
      }


      // Resume the simulator
      bool Insert(
        uas_controller::Insert::Request&  req, 
        uas_controller::Insert::Response& res) 
      {
        wp->InsertModelFile(req.type);
        res.success = true;
        res.status_message = std::string("Insert: model inserted successfully");
        return true;
      }

      // Remove a model from the simulation
      bool Remove(
        uas_controller::Remove::Request&  req, 
        uas_controller::Remove::Response& res)
      {
        /*
        // Get a pointer to the model and check that it exists!
        gazebo::physics::ModelPtr model = wp->GetModel(req.id);
        if (!model)
        {
          res.status = false;
          res.status_message = "Remove: model does not exist";
          return true;
        }

        // Delete any wrench jobs on the body
        for (unsigned int i = 0 ; i < model->GetChildCount(); i ++)
        {
          gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(model->GetChild(i));
          if (body)
            clearBodyWrenches(body->GetScopedName());
        }

        // Delete any force jobs on the body
        gazebo::physics::Joint_V joints = model->GetJoints();
        for (unsigned int i=0;i< joints.size(); i++)
          clearJointForces(joints[i]->GetName());

        // send delete model request
        gazebo::msgs::Request *msg = gazebo::msgs::CreateRequest("entity_delete",req.id);
        request_pub_->Publish(*msg,true);

        ros::Duration model_spawn_timeout(60.0);
        ros::Time timeout = ros::Time::now() + model_spawn_timeout;
        
        // Wait to see if the model actuallt gets deleted
        while (true)
        {

          if (ros::Time::now() > timeout)
          {
            res.success = false;
            res.status_message = std::string("DeleteModel: Model pushed to delete queue, but delete service timed out waiting for model to disappear from simulation");
            return true;
          }
          {
            //boost::recursive_mutex::scoped_lock lock(*world->GetMRMutex());
            if (!world_->GetModel(req.model_name)) break;
          }
          ROS_DEBUG("Waiting for model deletion (%s)",req.model_name.c_str());
          usleep(1000);
        }
      */
        // set result
        res.success = true;
        res.status_message = std::string("Remove: successfully removed model");
        return true;
      }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(Simulator)

}