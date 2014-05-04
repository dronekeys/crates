#include <gazebo/common/Events.hh>
#include "gazebo_ros_api_plugin.h"

namespace gazebo
{

	GazeboRosApiPlugin::GazeboRosApiPlugin() : world_created_(false), stop_(false), plugin_loaded_(false)
	{
		robot_namespace_.clear();
	}

	GazeboRosApiPlugin::~GazeboRosApiPlugin()
	{
		ROS_DEBUG_STREAM_NAMED("api_plugin","GazeboRosApiPlugin Deconstructor start");

		// Unload the sigint event
		gazebo::event::Events::DisconnectSigInt(sigint_event_);
		ROS_DEBUG_STREAM_NAMED("api_plugin","After sigint_event unload");

  		// Don't attempt to unload this plugin if it was never loaded in the Load() function
		if(!plugin_loaded_)
		{
			ROS_DEBUG_STREAM_NAMED("api_plugin","Deconstructor skipped because never loaded");
			return;
		}

  		// Disconnect slots
		gazebo::event::Events::DisconnectWorldUpdateBegin(wrench_update_event_);
		gazebo::event::Events::DisconnectWorldUpdateBegin(force_update_event_);
		gazebo::event::Events::DisconnectWorldUpdateBegin(time_update_event_);
		ROS_DEBUG_STREAM_NAMED("api_plugin","Slots disconnected");

		// disconnect if there are subscribers on exit
  		if (pub_link_states_connection_count_ > 0) 
  			gazebo::event::Events::DisconnectWorldUpdateBegin(pub_link_states_event_);
  		if (pub_model_states_connection_count_ > 0) 
  			gazebo::event::Events::DisconnectWorldUpdateBegin(pub_model_states_event_);
  		ROS_DEBUG_STREAM_NAMED("api_plugin","Disconnected World Updates");

		// Stop the multi threaded ROS spinner
		async_ros_spin_->stop();
		ROS_DEBUG_STREAM_NAMED("api_plugin","Async ROS Spin Stopped");

		// Shutdown the ROS node
		nh_->shutdown();
		ROS_DEBUG_STREAM_NAMED("api_plugin","Node Handle Shutdown");

		// Shutdown ROS queue
		gazebo_callback_queue_thread_->join();
		ROS_DEBUG_STREAM_NAMED("api_plugin","Callback Queue Joined");

		// Delete Force and Wrench Jobs
		lock_.lock();
		for (std::vector<GazeboRosApiPlugin::ForceJointJob*>::iterator iter=force_joint_jobs_.begin();iter!=force_joint_jobs_.end();)
			delete (*iter);
		force_joint_jobs_.clear();
		ROS_DEBUG_STREAM_NAMED("api_plugin","ForceJointJobs deleted");
		for (std::vector<GazeboRosApiPlugin::WrenchBodyJob*>::iterator iter=wrench_body_jobs_.begin();iter!=wrench_body_jobs_.end();)
			delete (*iter);
		wrench_body_jobs_.clear();
		lock_.unlock();
		ROS_DEBUG_STREAM_NAMED("api_plugin","WrenchBodyJobs deleted");
		ROS_DEBUG_STREAM_NAMED("api_plugin","Unloaded");
	}

	void GazeboRosApiPlugin::shutdownSignal()
	{
		ROS_DEBUG_STREAM_NAMED("api_plugin","shutdownSignal() recieved");
		stop_ = true;
	}

	void GazeboRosApiPlugin::Load(int argc, char** argv)
	{
		ROS_DEBUG_STREAM_NAMED("api_plugin","Load");

	  	// When a SIGINT is receiver, send the sudown signal
		sigint_event_ = gazebo::event::Events::ConnectSigInt(
			boost::bind(&GazeboRosApiPlugin::shutdownSignal,this)
		);

	  	// Start ROS without SIGINT ability
		if (!ros::isInitialized())
			ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler);
		else
			ROS_ERROR("Something other than this gazebo_ros_api plugin started ros::init(...), command line arguments may not be parsed properly.");

	  	// Wait until the ROS master becomes available
		while(!ros::master::check())
		{
			ROS_WARN_STREAM_NAMED("api_plugin","No ROS master - start roscore to continue...");
			
			// wait 0.5 second (can't use ROS Time here b/c node handle is not yet initialized()
			usleep(500*1000);
			if(stop_)
			{
				ROS_WARN_STREAM_NAMED("api_plugin","Canceled loading Gazebo ROS API plugin by sigint event");
				return;
			}
		}

	  	// Advertise topics and services in this node's namespace
		nh_.reset(new ros::NodeHandle("~")); 

	  	// Built-in multi-threaded ROS spinning (will use a thread for each CPU core)
		async_ros_spin_.reset(new ros::AsyncSpinner(0)); 
		async_ros_spin_->start();

	  	/// setup custom callback queue
		gazebo_callback_queue_thread_.reset(
			new boost::thread(&GazeboRosApiPlugin::gazeboQueueThread, this)
		);

	  	// On connecting to the world, load the rest of the worls
		load_gazebo_ros_api_plugin_event_ = gazebo::event::Events::ConnectWorldCreated(
			boost::bind(&GazeboRosApiPlugin::loadGazeboRosApiPlugin,this,_1)
		);

		plugin_loaded_ = true;
		ROS_INFO("Finished loading Gazebo ROS API Plugin.");
	}

	void GazeboRosApiPlugin::loadGazeboRosApiPlugin(std::string world_name)
	{
	  	// Esures that this is called only once
		gazebo::event::Events::DisconnectWorldCreated(load_gazebo_ros_api_plugin_event_);
		lock_.lock();
		if (world_created_)
		{
			lock_.unlock();
			return;
		}

	  	// World has not been created
		world_created_ = true;
		lock_.unlock();

		// Check to see that the world is valid
		world_ = gazebo::physics::get_world(world_name);
		if (!world_)
		{
			ROS_FATAL("cannot load gazebo ros api server plugin, physics::get_world() fails to return world");
			return;
		}

		gazebonode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
		gazebonode_->Init(world_name);

		// Set up gazebo publishers
		factory_pub_ = gazebonode_->Advertise<gazebo::msgs::Factory>("~/factory");
		request_pub_ = gazebonode_->Advertise<gazebo::msgs::Request>("~/request");
		response_sub_ = gazebonode_->Subscribe("~/response",&GazeboRosApiPlugin::onResponse, this);

	  	// reset topic connection counts
		pub_link_states_connection_count_ = 0;
		pub_model_states_connection_count_ = 0;

	  	///////////////////////////////////////////////////////////
	  	// ADVERTISE ALL SIMULATION SERVICES ON THE ROS BACKBONE //
	  	///////////////////////////////////////////////////////////

	  	// publish clock for simulated ros time
		pub_clock_ = nh_->advertise<rosgraph_msgs::Clock>("/clock",10);

	  	// Add a model
		ros::AdvertiseServiceOptions spawn_sdf_model_aso = ros::AdvertiseServiceOptions::create<sim::AddModel>(
			"add_model",
			boost::bind(&GazeboRosApiPlugin::addModel,this,_1,_2),
			ros::VoidPtr(), &gazebo_queue_
			);
		spawn_sdf_model_service_ = nh_->advertiseService(spawn_sdf_model_aso);

	  	// Delete a model
		ros::AdvertiseServiceOptions delete_aso = ros::AdvertiseServiceOptions::create<sim::DelModel>(
			"del_model",
			boost::bind(&GazeboRosApiPlugin::delModel,this,_1,_2),
			ros::VoidPtr(), &gazebo_queue_
			);
		delete_model_service_ = nh_->advertiseService(delete_aso);

	  	// Get a model state
		ros::AdvertiseServiceOptions get_model_state_aso = ros::AdvertiseServiceOptions::create<sim::GetModelState>(
			"get_model_state",
			boost::bind(&GazeboRosApiPlugin::getModelState,this,_1,_2),
			ros::VoidPtr(), &gazebo_queue_
			);
		get_model_state_service_ = nh_->advertiseService(get_model_state_aso);

	  	// Set model state
		ros::AdvertiseServiceOptions set_model_state_aso = ros::AdvertiseServiceOptions::create<sim::SetModelState>(
			"set_model_state",
			boost::bind(&GazeboRosApiPlugin::setModelState,this,_1,_2),
			ros::VoidPtr(), &gazebo_queue_
			);
		set_model_state_service_ = nh_->advertiseService(set_model_state_aso);

	  	// Advertise more services on the custom queue
		ros::AdvertiseServiceOptions pause_physics_aso = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
			"pause",
			boost::bind(&GazeboRosApiPlugin::pausePhysics,this,_1,_2),
			ros::VoidPtr(), &gazebo_queue_
			);
		pause_physics_service_ = nh_->advertiseService(pause_physics_aso);

	  	// Advertise more services on the custom queue
		ros::AdvertiseServiceOptions unpause_physics_aso = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
			"resume",
			boost::bind(&GazeboRosApiPlugin::unpausePhysics,this,_1,_2),
			ros::VoidPtr(), &gazebo_queue_
			);
		unpause_physics_service_ = nh_->advertiseService(unpause_physics_aso);

	  	// Advertise more services on the custom queue
		std::string reset_simulation_service_name("reset");
		ros::AdvertiseServiceOptions reset_simulation_aso = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
			reset_simulation_service_name,
			boost::bind(&GazeboRosApiPlugin::resetSimulation,this,_1,_2),
			ros::VoidPtr(), &gazebo_queue_
			);
		reset_simulation_service_ = nh_->advertiseService(reset_simulation_aso);

	  	// set param for use_sim_time if not set by user already
		nh_->setParam("/use_sim_time", true);

	  	// hooks for applying forces, publishing simtime on /clock
		time_update_event_ = gazebo::event::Events::ConnectWorldUpdateBegin(
			boost::bind(&GazeboRosApiPlugin::publishSimTime,this)
		);
	}

	void GazeboRosApiPlugin::onResponse(ConstResponsePtr &response)
	{
		// Do nothing
	}

	void GazeboRosApiPlugin::gazeboQueueThread()
	{
		static const double timeout = 0.001;
		while (nh_->ok())
		{
			gazebo_queue_.callAvailable(ros::WallDuration(timeout));
		}
	}

	void GazeboRosApiPlugin::onLinkStatesConnect()
	{
		pub_link_states_connection_count_++;
	  	if (pub_link_states_connection_count_ == 1) // connect on first subscriber
	  	{
	  		pub_link_states_event_   = gazebo::event::Events::ConnectWorldUpdateBegin(
	  			boost::bind(&GazeboRosApiPlugin::publishLinkStates,this)
  			);
	  	}
	}

	void GazeboRosApiPlugin::onModelStatesConnect()
	{
		pub_model_states_connection_count_++;
	  	if (pub_model_states_connection_count_ == 1) // connect on first subscriber
	  	{
	  		pub_model_states_event_ = gazebo::event::Events::ConnectWorldUpdateBegin(
	  			boost::bind(&GazeboRosApiPlugin::publishModelStates,this)
  			);
		}
	}

	void GazeboRosApiPlugin::onLinkStatesDisconnect()
	{
		pub_link_states_connection_count_--;
		if (pub_link_states_connection_count_ <= 0) // disconnect with no subscribers
		{
		  	gazebo::event::Events::DisconnectWorldUpdateBegin(pub_link_states_event_);
			if (pub_link_states_connection_count_ < 0) // should not be possible
				ROS_ERROR("one too many disconnect from pub_link_states_ in gazebo_ros.cpp? something weird");
		}
	}

	void GazeboRosApiPlugin::onModelStatesDisconnect()
	{
		pub_model_states_connection_count_--;
		if (pub_model_states_connection_count_ <= 0) // disconnect with no subscribers
		{
			gazebo::event::Events::DisconnectWorldUpdateBegin(pub_model_states_event_);
			if (pub_model_states_connection_count_ < 0) // should not be possible
				ROS_ERROR("one too many disconnect from pub_model_states_ in gazebo_ros.cpp? something weird");
		}
	}

	bool GazeboRosApiPlugin::addModel(sim::AddModel::Request &req, sim::AddModel::Response &res)
	{
	  	// incoming robot name
		std::string model_name = req.model_name;

	  	// get name space for the corresponding model plugins
		robot_namespace_ = req.robot_namespace;

	  	// get initial pose of model
		gazebo::math::Vector3 initial_xyz(
			req.initial_pose.position.x,
			req.initial_pose.position.y,
			req.initial_pose.position.z
			);

	  	// get initial roll pitch yaw (fixed frame transform)
		gazebo::math::Quaternion initial_q(
			req.initial_pose.orientation.w,
			req.initial_pose.orientation.x,
			req.initial_pose.orientation.y,
			req.initial_pose.orientation.z
			);

	  	// reference frame for initial pose definition, modify initial pose if defined
		gazebo::physics::LinkPtr frame = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.reference_frame));
		if (frame)
		{
		// convert to relative pose
			gazebo::math::Pose frame_pose = frame->GetWorldPose();
			initial_xyz = frame_pose.rot.RotateVector(initial_xyz);
			initial_xyz += frame_pose.pos;
			initial_q *= frame_pose.rot;
		}

	  /// @todo: map is really wrong, need to use tf here somehow
		else if (req.reference_frame == "" || req.reference_frame == "world" || req.reference_frame == "map" || req.reference_frame == "/map")
		{
			ROS_DEBUG("SpawnModel: reference_frame is empty/world/map, using inertial frame");
		}
		else
		{
			res.success = false;
			res.status_message = "SpawnModel: reference reference_frame not found, did you forget to scope the link by model name?";
			return true;
		}

	  // incoming robot model string
		std::string model_xml = req.model_xml;

	  // store resulting Gazebo Model XML to be sent to spawn queue
	  // get incoming string containg either an URDF or a Gazebo Model XML
	  // grab from parameter server if necessary convert to SDF if necessary
		stripXmlDeclaration(model_xml);

	  // put string in TiXmlDocument for manipulation
		TiXmlDocument gazebo_model_xml;
		gazebo_model_xml.Parse(model_xml.c_str());

	  // optional model manipulations: update initial pose && replace model name
		if (isSDF(model_xml))
		{
			updateSDFAttributes(gazebo_model_xml, model_name, initial_xyz, initial_q);

		// Walk recursively through the entire SDF, locate plugin tags and
		// add robotNamespace as a child with the correct namespace
			if (!this->robot_namespace_.empty())
			{
		  // Get root element for SDF
				TiXmlNode* model_tixml = gazebo_model_xml.FirstChild("sdf");
				model_tixml = (!model_tixml) ?
				gazebo_model_xml.FirstChild("gazebo") : model_tixml;
				if (model_tixml)
				{
					walkChildAddRobotNamespace(model_tixml);
				}
				else
				{
					ROS_WARN("Unable to add robot namespace to xml");
				}
			}
		}
		else if (isURDF(model_xml))
		{
			updateURDFModelPose(gazebo_model_xml, initial_xyz, initial_q);
			updateURDFName(gazebo_model_xml, model_name);

		// Walk recursively through the entire URDF, locate plugin tags and
		// add robotNamespace as a child with the correct namespace
			if (!this->robot_namespace_.empty())
			{
		  // Get root element for URDF
				TiXmlNode* model_tixml = gazebo_model_xml.FirstChild("robot");
				if (model_tixml)
				{
					walkChildAddRobotNamespace(model_tixml);
				}
				else
				{
					ROS_WARN("Unable to add robot namespace to xml");
				}
			}
		}
		else
		{
			ROS_ERROR("GazeboRosApiPlugin SpawnModel Failure: input xml format not recognized");
			res.success = false;
			res.status_message = std::string("GazeboRosApiPlugin SpawnModel Failure: input model_xml not SDF or URDF, or cannot be converted to Gazebo compatible format.");
			return true;
		}

	  // do spawning check if spawn worked, return response
		return spawnAndConform(gazebo_model_xml, model_name, res);
	}

	bool GazeboRosApiPlugin::delModel(sim::DelModel::Request &req, sim::DelModel::Response &res)
	{
	  	// Clear forces, etc for the body in question
		gazebo::physics::ModelPtr model = world_->GetModel(req.model_name);
		if (!model)
		{
			ROS_ERROR("DeleteModel: model [%s] does not exist",req.model_name.c_str());
			res.success = false;
			res.status_message = "DeleteModel: model does not exist";
			return true;
		}

	  // delete wrench jobs on bodies
		for (unsigned int i = 0 ; i < model->GetChildCount(); i ++)
		{
			gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(model->GetChild(i));
			if (body)
			{
		  // look for it in jobs, delete body wrench jobs
				clearBodyWrenches(body->GetScopedName());
			}
		}

	  // delete force jobs on joints
		gazebo::physics::Joint_V joints = model->GetJoints();
		for (unsigned int i=0;i< joints.size(); i++)
		{
		// look for it in jobs, delete joint force jobs
			clearJointForces(joints[i]->GetName());
		}

	  // send delete model request
		gazebo::msgs::Request *msg = gazebo::msgs::CreateRequest("entity_delete",req.model_name);
		request_pub_->Publish(*msg,true);

		ros::Duration model_spawn_timeout(60.0);
		ros::Time timeout = ros::Time::now() + model_spawn_timeout;
	  // wait and verify that model is deleted
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

	  // set result
		res.success = true;
		res.status_message = std::string("DeleteModel: successfully deleted model");
		return true;
	}

	bool GazeboRosApiPlugin::getModelState(sim::GetModelState::Request &req, sim::GetModelState::Response &res)
	{
		gazebo::physics::ModelPtr model = world_->GetModel(req.model_name);
		gazebo::physics::LinkPtr frame = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.relative_entity_name));
		if (!model)
		{
			ROS_ERROR("GetModelState: model [%s] does not exist",req.model_name.c_str());
			res.success = false;
			res.status_message = "GetModelState: model does not exist";
			return true;
		}
		else
		{
		// get model pose
			gazebo::math::Pose       model_pose = model->GetWorldPose();
			gazebo::math::Vector3    model_pos = model_pose.pos;
			gazebo::math::Quaternion model_rot = model_pose.rot;

		// get model twist
			gazebo::math::Vector3 model_linear_vel  = model->GetWorldLinearVel();
			gazebo::math::Vector3 model_angular_vel = model->GetWorldAngularVel();


			if (frame)
			{
		  // convert to relative pose
				gazebo::math::Pose frame_pose = frame->GetWorldPose();
				model_pos = model_pos - frame_pose.pos;
				model_pos = frame_pose.rot.RotateVectorReverse(model_pos);
				model_rot *= frame_pose.rot.GetInverse();

		  // convert to relative rates
		  gazebo::math::Vector3 frame_vpos = frame->GetWorldLinearVel(); // get velocity in gazebo frame
		  gazebo::math::Vector3 frame_veul = frame->GetWorldAngularVel(); // get velocity in gazebo frame
		  model_linear_vel = frame_pose.rot.RotateVector(model_linear_vel - frame_vpos);
		  model_angular_vel = frame_pose.rot.RotateVector(model_angular_vel - frame_veul);
		}
		/// @todo: FIXME map is really wrong, need to use tf here somehow
		else if (req.relative_entity_name == "" || req.relative_entity_name == "world" || req.relative_entity_name == "map" || req.relative_entity_name == "/map")
		{
			ROS_DEBUG("GetModelState: relative_entity_name is empty/world/map, using inertial frame");
		}
		else
		{
			res.success = false;
			res.status_message = "GetModelState: reference relative_entity_name not found, did you forget to scope the body by model name?";
			return true;
		}

		// fill in response
		res.pose.position.x = model_pos.x;
		res.pose.position.y = model_pos.y;
		res.pose.position.z = model_pos.z;
		res.pose.orientation.w = model_rot.w;
		res.pose.orientation.x = model_rot.x;
		res.pose.orientation.y = model_rot.y;
		res.pose.orientation.z = model_rot.z;

		res.twist.linear.x = model_linear_vel.x;
		res.twist.linear.y = model_linear_vel.y;
		res.twist.linear.z = model_linear_vel.z;
		res.twist.angular.x = model_angular_vel.x;
		res.twist.angular.y = model_angular_vel.y;
		res.twist.angular.z = model_angular_vel.z;

		res.success = true;
		res.status_message = "GetModelState: got properties";
		return true;
	}
	return true;
	}

	bool GazeboRosApiPlugin::setModelState(sim::SetModelState::Request &req, sim::SetModelState::Response &res)
	{
		gazebo::math::Vector3 target_pos(req.model_state.pose.position.x,req.model_state.pose.position.y,req.model_state.pose.position.z);
		gazebo::math::Quaternion target_rot(req.model_state.pose.orientation.w,req.model_state.pose.orientation.x,req.model_state.pose.orientation.y,req.model_state.pose.orientation.z);
	  target_rot.Normalize(); // eliminates invalid rotation (0, 0, 0, 0)
	  gazebo::math::Pose target_pose(target_pos,target_rot);
	  gazebo::math::Vector3 target_pos_dot(req.model_state.twist.linear.x,req.model_state.twist.linear.y,req.model_state.twist.linear.z);
	  gazebo::math::Vector3 target_rot_dot(req.model_state.twist.angular.x,req.model_state.twist.angular.y,req.model_state.twist.angular.z);

	  gazebo::physics::ModelPtr model = world_->GetModel(req.model_state.model_name);
	  if (!model)
	  {
	  	ROS_ERROR("Updating ModelState: model [%s] does not exist",req.model_state.model_name.c_str());
	  	res.success = false;
	  	res.status_message = "SetModelState: model does not exist";
	  	return true;
	  }
	  else
	  {
	  	gazebo::physics::LinkPtr relative_entity = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.model_state.reference_frame));
	  	if (relative_entity)
	  	{
		  gazebo::math::Pose  frame_pose = relative_entity->GetWorldPose(); // - myBody->GetCoMPose();
		  gazebo::math::Vector3 frame_pos = frame_pose.pos;
		  gazebo::math::Quaternion frame_rot = frame_pose.rot;

		  //std::cout << " debug : " << relative_entity->GetName() << " : " << frame_pose << " : " << target_pose << std::endl;
		  //target_pose = frame_pose + target_pose; // seems buggy, use my own
		  target_pose.pos = frame_pos + frame_rot.RotateVector(target_pos);
		  target_pose.rot = frame_rot * target_pose.rot;
		}
		/// @todo: FIXME map is really wrong, need to use tf here somehow
		else if (req.model_state.reference_frame == "" || req.model_state.reference_frame == "world" || req.model_state.reference_frame == "map" || req.model_state.reference_frame == "/map" )
		{
			ROS_DEBUG("Updating ModelState: reference frame is empty/world/map, usig inertial frame");
		}
		else
		{
			ROS_ERROR("Updating ModelState: for model[%s], specified reference frame entity [%s] does not exist",
				req.model_state.model_name.c_str(),req.model_state.reference_frame.c_str());
			res.success = false;
			res.status_message = "SetModelState: specified reference frame entity does not exist";
			return true;
		}

		//ROS_ERROR("target state: %f %f %f",target_pose.pos.x,target_pose.pos.y,target_pose.pos.z);
		bool is_paused = world_->IsPaused();
		world_->SetPaused(true);
		model->SetWorldPose(target_pose);
		world_->SetPaused(is_paused);
		//gazebo::math::Pose p3d = model->GetWorldPose();
		//ROS_ERROR("model updated state: %f %f %f",p3d.pos.x,p3d.pos.y,p3d.pos.z);

		// set model velocity
		model->SetLinearVel(target_pos_dot);
		model->SetAngularVel(target_rot_dot);

		res.success = true;
		res.status_message = "SetModelState: set model state done";
		return true;
	}
	}

	bool GazeboRosApiPlugin::resetSimulation(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
	{
		world_->Reset();
		return true;
	}

	bool GazeboRosApiPlugin::pausePhysics(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
	{
		world_->SetPaused(true);
		return true;
	}

	bool GazeboRosApiPlugin::unpausePhysics(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
	{
		world_->SetPaused(false);
		return true;
	}

	bool GazeboRosApiPlugin::isURDF(std::string model_xml)
	{
		TiXmlDocument doc_in;
		doc_in.Parse(model_xml.c_str());
		if (doc_in.FirstChild("robot"))
			return true;
		else
			return false;
	}

	bool GazeboRosApiPlugin::isSDF(std::string model_xml)
	{
	  // FIXME: very crude check
		TiXmlDocument doc_in;
		doc_in.Parse(model_xml.c_str());
		if (doc_in.FirstChild("gazebo") ||
		  doc_in.FirstChild("sdf")) // sdf
			return true;
		else
			return false;
	}

	void GazeboRosApiPlugin::publishSimTime(const boost::shared_ptr<gazebo::msgs::WorldStatistics const> &msg)
	{
		ROS_ERROR("CLOCK2");
		gazebo::common::Time currentTime = gazebo::msgs::Convert( msg->sim_time() );
		rosgraph_msgs::Clock ros_time_;
		ros_time_.clock.fromSec(currentTime.Double());
	  	
	  	// Publish time to ros
		pub_clock_.publish(ros_time_);
	}

	void GazeboRosApiPlugin::publishSimTime()
	{
		gazebo::common::Time currentTime = world_->GetSimTime();
		rosgraph_msgs::Clock ros_time_;
		ros_time_.clock.fromSec(currentTime.Double());
	  //  publish time to ros
		pub_clock_.publish(ros_time_);
	}

	void GazeboRosApiPlugin::publishLinkStates()
	{
		sim::LinkStates link_states;

	  // fill link_states
		for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
		{
			gazebo::physics::ModelPtr model = world_->GetModel(i);

			for (unsigned int j = 0 ; j < model->GetChildCount(); j ++)
			{
				gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(model->GetChild(j));

				if (body)
				{
					link_states.name.push_back(body->GetScopedName());
					geometry_msgs::Pose pose;
			gazebo::math::Pose  body_pose = body->GetWorldPose(); // - myBody->GetCoMPose();
			gazebo::math::Vector3 pos = body_pose.pos;
			gazebo::math::Quaternion rot = body_pose.rot;
			pose.position.x = pos.x;
			pose.position.y = pos.y;
			pose.position.z = pos.z;
			pose.orientation.w = rot.w;
			pose.orientation.x = rot.x;
			pose.orientation.y = rot.y;
			pose.orientation.z = rot.z;
			link_states.pose.push_back(pose);
			gazebo::math::Vector3 linear_vel  = body->GetWorldLinearVel();
			gazebo::math::Vector3 angular_vel = body->GetWorldAngularVel();
			geometry_msgs::Twist twist;
			twist.linear.x = linear_vel.x;
			twist.linear.y = linear_vel.y;
			twist.linear.z = linear_vel.z;
			twist.angular.x = angular_vel.x;
			twist.angular.y = angular_vel.y;
			twist.angular.z = angular_vel.z;
			link_states.twist.push_back(twist);
		}
	}
	}

	pub_link_states_.publish(link_states);
	}

	void GazeboRosApiPlugin::publishModelStates()
	{
		sim::ModelStates model_states;

	  // fill model_states
		for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
		{
			gazebo::physics::ModelPtr model = world_->GetModel(i);
			model_states.name.push_back(model->GetName());
			geometry_msgs::Pose pose;
		gazebo::math::Pose  model_pose = model->GetWorldPose(); // - myBody->GetCoMPose();
		gazebo::math::Vector3 pos = model_pose.pos;
		gazebo::math::Quaternion rot = model_pose.rot;
		pose.position.x = pos.x;
		pose.position.y = pos.y;
		pose.position.z = pos.z;
		pose.orientation.w = rot.w;
		pose.orientation.x = rot.x;
		pose.orientation.y = rot.y;
		pose.orientation.z = rot.z;
		model_states.pose.push_back(pose);
		gazebo::math::Vector3 linear_vel  = model->GetWorldLinearVel();
		gazebo::math::Vector3 angular_vel = model->GetWorldAngularVel();
		geometry_msgs::Twist twist;
		twist.linear.x = linear_vel.x;
		twist.linear.y = linear_vel.y;
		twist.linear.z = linear_vel.z;
		twist.angular.x = angular_vel.x;
		twist.angular.y = angular_vel.y;
		twist.angular.z = angular_vel.z;
		model_states.twist.push_back(twist);
	}
	pub_model_states_.publish(model_states);
	}

	void GazeboRosApiPlugin::stripXmlDeclaration(std::string &model_xml)
	{
	  // incoming robot model string is a string containing a Gazebo Model XML
	  /// STRIP DECLARATION <? ... xml version="1.0" ... ?> from model_xml
	  /// @todo: does tinyxml have functionality for this?
	  /// @todo: should gazebo take care of the declaration?
		std::string open_bracket("<?");
		std::string close_bracket("?>");
		size_t pos1 = model_xml.find(open_bracket,0);
		size_t pos2 = model_xml.find(close_bracket,0);
		if (pos1 != std::string::npos && pos2 != std::string::npos)
			model_xml.replace(pos1,pos2-pos1+2,std::string(""));
	}

	void GazeboRosApiPlugin::updateSDFAttributes(TiXmlDocument &gazebo_model_xml, std::string model_name,
		gazebo::math::Vector3 initial_xyz, gazebo::math::Quaternion initial_q)
	{
	  // This function can handle both regular SDF files and <include> SDFs that are used with the
	  // Gazebo Model Database

	  TiXmlElement* pose_element; // This is used by both reguar and database SDFs

	  // Check SDF for requires SDF element
	  TiXmlElement* gazebo_tixml = gazebo_model_xml.FirstChildElement("sdf");
	  if (!gazebo_tixml)
	  {
	  	ROS_WARN("Could not find <sdf> element in sdf, so name and initial position cannot be applied");
	  	return;
	  }

	  // Check SDF for optional model element. May not have one
	  TiXmlElement* model_tixml = gazebo_tixml->FirstChildElement("model");
	  if (model_tixml)
	  {
		// Update model name
	  	if (model_tixml->Attribute("name") != NULL)
	  	{
		  // removing old model name
	  		model_tixml->RemoveAttribute("name");
	  	}
		// replace with user specified name
	  	model_tixml->SetAttribute("name",model_name);
	  }
	  else
	  {
		// Check SDF for world element
	  	TiXmlElement* world_tixml = gazebo_tixml->FirstChildElement("world");
	  	if (!world_tixml)
	  	{
	  		ROS_WARN("Could not find <model> or <world> element in sdf, so name and initial position cannot be applied");
	  		return;
	  	}
		// If not <model> element, check SDF for required include element
	  	model_tixml = world_tixml->FirstChildElement("include");
	  	if (!model_tixml)
	  	{
	  		ROS_WARN("Could not find <include> element in sdf, so name and initial position cannot be applied");
	  		return;
	  	}

		// Check for name element
	  	TiXmlElement* name_tixml = model_tixml->FirstChildElement("name");
	  	if (!name_tixml)
	  	{
		  // Create the name element
	  		name_tixml = new TiXmlElement("name");
	  		model_tixml->LinkEndChild(name_tixml);
	  	}

		// Set the text within the name element
	  	TiXmlText* text = new TiXmlText(model_name);
	  	name_tixml->LinkEndChild( text );
	  }


	  // Check for the pose element
	  pose_element = model_tixml->FirstChildElement("pose");
	  gazebo::math::Pose model_pose;

	  // Create the pose element if it doesn't exist
	  // Remove it if it exists, since we are inserting a new one
	  if (pose_element)
	  {
		// save pose_element in math::Pose and remove child
	  	model_pose = this->parsePose(pose_element->GetText());
	  	model_tixml->RemoveChild(pose_element);
	  }

	  // Set and link the pose element after adding initial pose
	  {
		// add pose_element Pose to initial pose
	  	gazebo::math::Pose new_model_pose = model_pose + gazebo::math::Pose(initial_xyz, initial_q);

		// Create the string of 6 numbers
	  	std::ostringstream pose_stream;
		gazebo::math::Vector3 model_rpy = new_model_pose.rot.GetAsEuler(); // convert to Euler angles for Gazebo XML
		pose_stream << new_model_pose.pos.x << " " << new_model_pose.pos.y << " " << new_model_pose.pos.z << " "
		<< model_rpy.x << " " << model_rpy.y << " " << model_rpy.z;

		// Add value to pose element
		TiXmlText* text = new TiXmlText(pose_stream.str());
		TiXmlElement* new_pose_element = new TiXmlElement("pose");
		new_pose_element->LinkEndChild(text);
		model_tixml->LinkEndChild(new_pose_element);
	}
	}

	gazebo::math::Pose GazeboRosApiPlugin::parsePose(const std::string &str)
	{
		std::vector<std::string> pieces;
		std::vector<double> vals;

		boost::split(pieces, str, boost::is_any_of(" "));
		for (unsigned int i = 0; i < pieces.size(); ++i)
		{
			if (pieces[i] != "")
			{
				try
				{
					vals.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
				}
				catch(boost::bad_lexical_cast &e)
				{
					sdferr << "xml key [" << str
						<< "][" << i << "] value [" << pieces[i]
						<< "] is not a valid double from a 3-tuple\n";
	return gazebo::math::Pose();
	}
	}
	}

	if (vals.size() == 6)
		return gazebo::math::Pose(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]);
	else
	{
		ROS_ERROR("Beware: failed to parse string [%s] as gazebo::math::Pose, returning zeros.", str.c_str());
		return gazebo::math::Pose();
	}
	}

	gazebo::math::Vector3 GazeboRosApiPlugin::parseVector3(const std::string &str)
	{
		std::vector<std::string> pieces;
		std::vector<double> vals;

		boost::split(pieces, str, boost::is_any_of(" "));
		for (unsigned int i = 0; i < pieces.size(); ++i)
		{
			if (pieces[i] != "")
			{
				try
				{
					vals.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
				}
				catch(boost::bad_lexical_cast &e)
				{
					sdferr << "xml key [" << str
						<< "][" << i << "] value [" << pieces[i]
						<< "] is not a valid double from a 3-tuple\n";
	return gazebo::math::Vector3();
	}
	}
	}

	if (vals.size() == 3)
		return gazebo::math::Vector3(vals[0], vals[1], vals[2]);
	else
	{
		ROS_ERROR("Beware: failed to parse string [%s] as gazebo::math::Vector3, returning zeros.", str.c_str());
		return gazebo::math::Vector3();
	}
	}

	void GazeboRosApiPlugin::updateURDFModelPose(TiXmlDocument &gazebo_model_xml, gazebo::math::Vector3 initial_xyz, gazebo::math::Quaternion initial_q)
	{
		TiXmlElement* model_tixml = (gazebo_model_xml.FirstChildElement("robot"));
		if (model_tixml)
		{
		// replace initial pose of robot
		// find first instance of xyz and rpy, replace with initial pose
			TiXmlElement* origin_key = model_tixml->FirstChildElement("origin");

			if (!origin_key)
			{
				origin_key = new TiXmlElement("origin");
				model_tixml->LinkEndChild(origin_key);
			}

			gazebo::math::Vector3 xyz;
			gazebo::math::Vector3 rpy;
			if (origin_key->Attribute("xyz"))
			{
				xyz = this->parseVector3(origin_key->Attribute("xyz"));
				origin_key->RemoveAttribute("xyz");
			}
			if (origin_key->Attribute("rpy"))
			{
				rpy = this->parseVector3(origin_key->Attribute("rpy"));
				origin_key->RemoveAttribute("rpy");
			}

		// add xyz, rpy to initial pose
			gazebo::math::Pose model_pose = gazebo::math::Pose(xyz, rpy) + gazebo::math::Pose(initial_xyz, initial_q);

			std::ostringstream xyz_stream;
			xyz_stream << model_pose.pos.x << " " << model_pose.pos.y << " " << model_pose.pos.z;

			std::ostringstream rpy_stream;
		gazebo::math::Vector3 model_rpy = model_pose.rot.GetAsEuler(); // convert to Euler angles for Gazebo XML
		rpy_stream << model_rpy.x << " " << model_rpy.y << " " << model_rpy.z;

		origin_key->SetAttribute("xyz",xyz_stream.str());
		origin_key->SetAttribute("rpy",rpy_stream.str());
	}
	else
		ROS_WARN("could not find <model> element in sdf, so name and initial position is not applied");
	}

	void GazeboRosApiPlugin::updateURDFName(TiXmlDocument &gazebo_model_xml, std::string model_name)
	{
		TiXmlElement* model_tixml = gazebo_model_xml.FirstChildElement("robot");
	  // replace model name if one is specified by the user
		if (model_tixml)
		{
			if (model_tixml->Attribute("name") != NULL)
			{
		  // removing old model name
				model_tixml->RemoveAttribute("name");
			}
		// replace with user specified name
			model_tixml->SetAttribute("name",model_name);
		}
		else
			ROS_WARN("could not find <robot> element in URDF, name not replaced");
	}

	void GazeboRosApiPlugin::walkChildAddRobotNamespace(TiXmlNode* robot_xml)
	{
		TiXmlNode* child = 0;
		child = robot_xml->IterateChildren(child);
		while (child != NULL)
		{
			if (child->ValueStr().find(std::string("plugin")) == 0)
			{
				if (child->FirstChildElement("robotNamespace") == NULL)
				{
					TiXmlElement* child_elem = child->ToElement()->FirstChildElement("robotNamespace");
					while (child_elem)
					{
						child->ToElement()->RemoveChild(child_elem);
						child_elem = child->ToElement()->FirstChildElement("robotNamespace");
					}
					TiXmlElement* key = new TiXmlElement("robotNamespace");
					TiXmlText* val = new TiXmlText(robot_namespace_);
					key->LinkEndChild(val);
					child->ToElement()->LinkEndChild(key);
				}
			}
			walkChildAddRobotNamespace(child);
			child = robot_xml->IterateChildren(child);
		}
	}

	bool GazeboRosApiPlugin::spawnAndConform(TiXmlDocument &gazebo_model_xml, std::string model_name,
		sim::SpawnModel::Response &res)
	{
	  // push to factory iface
		std::ostringstream stream;
		stream << gazebo_model_xml;
		std::string gazebo_model_xml_string = stream.str();
		ROS_DEBUG("Gazebo Model XML\n\n%s\n\n ",gazebo_model_xml_string.c_str());

	  // publish to factory topic
		gazebo::msgs::Factory msg;
		gazebo::msgs::Init(msg, "spawn_model");
		msg.set_sdf( gazebo_model_xml_string );

	  //ROS_ERROR("attempting to spawn model name [%s] [%s]", model_name.c_str(),gazebo_model_xml_string.c_str());

	  // FIXME: should use entity_info or add lock to World::receiveMutex
	  // looking for Model to see if it exists already
		gazebo::msgs::Request *entity_info_msg = gazebo::msgs::CreateRequest("entity_info", model_name);
		request_pub_->Publish(*entity_info_msg,true);
	  // todo: should wait for response response_sub_, check to see that if _msg->response == "nonexistant"

		gazebo::physics::ModelPtr model = world_->GetModel(model_name);
		if (model)
		{
			ROS_ERROR("SpawnModel: Failure - model name %s already exist.",model_name.c_str());
			res.success = false;
			res.status_message = "SpawnModel: Failure - model already exists.";
			return true;
		}

	  // Publish the factory message
		factory_pub_->Publish(msg);
	  /// FIXME: should change publish to direct invocation World::LoadModel() and/or
	  ///        change the poll for Model existence to common::Events based check.

	  /// \brief poll and wait, verify that the model is spawned within Hardcoded 10 seconds
		ros::Duration model_spawn_timeout(10.0);
		ros::Time timeout = ros::Time::now() + model_spawn_timeout;

		while (ros::ok())
		{
			if (ros::Time::now() > timeout)
			{
				res.success = false;
				res.status_message = std::string("SpawnModel: Model pushed to spawn queue, but spawn service")
				+ std::string(" timed out waiting for model to appear in simulation under the name ")
				+ model_name;
				return true;
			}

			{
		  //boost::recursive_mutex::scoped_lock lock(*world->GetMRMutex());
				if (world_->GetModel(model_name))
					break;
			}

			ROS_DEBUG_STREAM_ONCE_NAMED("api_plugin","Waiting for " << timeout - ros::Time::now()
				<< " for model " << model_name << " to spawn");

			usleep(2000);
		}

	  // set result
		res.success = true;
		res.status_message = std::string("SpawnModel: Successfully spawned model");
		return true;
	}

	// Register this plugin with the simulator
	GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosApiPlugin)
}
