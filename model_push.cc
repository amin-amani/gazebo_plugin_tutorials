#ifndef _MODEL_PUSH_HH_
#define _MODEL_PUSH_HH_
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
 #include <gazebo/gazebo.hh>
 #include <gazebo/physics/physics.hh>
 #include <gazebo/transport/transport.hh>
 #include <gazebo/msgs/msgs.hh>


namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
      
      /// \brief A node use for ROS transport
private: std::unique_ptr<ros::NodeHandle> rosNode;

/// \brief A ROS subscriber
private: ros::Subscriber rosSub;

/// \brief A ROS callbackqueue that helps process messages
private: ros::CallbackQueue rosQueue;

/// \brief A thread the keeps running the rosQueue
private: std::thread rosQueueThread;

/// \brief A node used for transport
private: transport::NodePtr node;

/// \brief A subscriber to a named topic.
private: transport::SubscriberPtr sub;

    /// \brief Pointer to the model.
private: physics::ModelPtr model;

/// \brief Pointer to the joint.
private: physics::JointPtr joint1;

/// \brief A PID controller for the joint.
private: common::PID pid1;

/// \brief Pointer to the joint.
private: physics::JointPtr joint2;

/// \brief A PID controller for the joint.
private: common::PID pid2;
      
       /// \brief Constructor
       public:  ModelPush() {}
       
       /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        if (_model->GetJointCount() == 0)
  {
    std::cerr << "Invalid joint count, ModelPush plugin not loaded\n";
    return;
  }
  
   // Store the model pointer for convenience.
  this->model = _model;
  
  
// Get the first joint. We are making an assumption about the model
  // having one joint that is the rotational joint.
  this->joint1 = _model->GetJoints()[0];
    // Setup a P-controller, with a gain of 0.1.
  this->pid1 = common::PID(1000, 0, 300);
  // Apply the P-controller to the joint.
  this->model->GetJointController()->SetPositionPID(this->joint1->GetScopedName(), this->pid1);
  // Set the joint's target velocity. This target velocity is just
  // for demonstration purposes.
//   this->model->GetJointController()->SetPositionTarget(this->joint1->GetScopedName(), 0.5);
//   this->model->GetJointController()->SetVelocityTarget(this->joint1->GetScopedName(), 0);
//   
  
  this->joint2 = _model->GetJoints()[1];
    // Setup a P-controller, with a gain of 0.1.
  this->pid2 = common::PID(1000, 0, 300);
  // Apply the P-controller to the joint.
  this->model->GetJointController()->SetPositionPID(this->joint2->GetScopedName(), this->pid2);
  // Set the joint's target velocity. This target velocity is just
  // for demonstration purposes.
//   this->model->GetJointController()->SetPositionTarget(this->joint2->GetScopedName(),0.3);
//   this->model->GetJointController()->SetVelocityTarget(this->joint2->GetScopedName(), 0);
   this->SetPosition(0,0);
    //this->SetVelocity(0,0);
 
   // Just output a message for now
      std::cerr << "\nThe velodyne plugin is attach to model[" <<_model->GetName() << "]\n";      
      // Create the node

// Initialize ros, if it has not already bee initialized.
if (!ros::isInitialized())
{
  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gazebo_client",
      ros::init_options::NoSigintHandler);
}

// Create our ROS node. This acts in a similar manner to
// the Gazebo node
this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

// Create a named topic, and subscribe to it.
ros::SubscribeOptions so =
  ros::SubscribeOptions::create<std_msgs::Float32>(
      "/" + this->model->GetName() + "/vel_cmd",
      1,
      boost::bind(&ModelPush::OnRosMsg, this, _1),
      ros::VoidPtr(), &this->rosQueue);
this->rosSub = this->rosNode->subscribe(so);

// Spin up the queue helper thread.
this->rosQueueThread =
  std::thread(std::bind(&ModelPush::QueueThread, this));
    }
    
    /// \brief Set the velocity of the Velodyne
/// \param[in] _vel New target velocity
// public: void SetVelocity(const double &_vel1,const double &_vel2)
// {
//   // Set the joint's target velocity.
// 
//    this->model->GetJointController()->SetVelocityTarget(this->joint1->GetScopedName(), _vel1);
//    this->model->GetJointController()->SetVelocityTarget(this->joint2->GetScopedName(), _vel2);
// }

 public: void SetVelocity(const double &_vel)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(
          this->joint1->GetScopedName(), _vel);
    }

public: void SetPosition(const double &_pos1,const double &_pos2)
{
  // Set the joint's target position.
  this->model->GetJointController()->SetPositionTarget(this->joint1->GetScopedName(), _pos1);
  this->model->GetJointController()->SetPositionTarget(this->joint2->GetScopedName(), _pos2);  
}

/// \brief Handle an incoming message from ROS
/// \param[in] _msg A float value that is used to set the velocity
/// of the Velodyne.
public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
{
  this->SetVelocity(_msg->data);
}

/// \brief ROS helper function that processes messages
private: void QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
    


  };

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
#endif
