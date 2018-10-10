#define _USE_MATH_DEFINES
#define RAND_MAX 360
#include <thread>
#include <cstdlib>
#include "math.h"
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class SphereDrive : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      std::cerr << "\nThe SphereDrive plugin is attached to model[" <<
        model->GetName() << "]\n";

      // Initialize ros, if it has not already been initialized.
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
      
      // Create a named topic for twist, and subscribe to it.
      ros::SubscribeOptions so_twist =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(
            "/" + this->model->GetName() + "/cmd_vel",
            1,
            boost::bind(&SphereDrive::OnRosMsgTwist, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSubTwist = this->rosNode->subscribe(so_twist);
      
      // Create a named topic for setting heading, and subscribe to it.
      ros::SubscribeOptions so_heading =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->model->GetName() + "/set_heading",
            1,
            boost::bind(&SphereDrive::OnRosMsgHeading, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSubHeading = this->rosNode->subscribe(so_heading);
      
      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&SphereDrive::QueueThread, this));
    }

    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A twist message that is used to set the angular 
    /// velocity of the Sphere.
    public: void OnRosMsgTwist(const geometry_msgs::Twist::ConstPtr& _msg)
    {
      // Get message information
      double vel = _msg->linear.x;
      double yaw = _msg->angular.z * M_PI / 180;
      std::cerr << "\n[" << model->GetName() << "] vel set to " << 
        vel << ", heading set to " << yaw << "\n";
      // Adjust x and y velocities based on heading (yaw) and drift
      double x_vel = vel * cos(yaw + this->drift);
      double y_vel = vel * sin(yaw + this->drift);
      this->model->SetAngularVel(ignition::math::Vector3d(x_vel, y_vel, 0)); 
    }
    
    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A Float32 message that is used to set the heading 
    /// angle of the Sphere.
    public: void OnRosMsgHeading(const std_msgs::Float32::ConstPtr& _msg)
    {
      this->drift = this->drift + _msg->data * M_PI / 180;
      std::cerr << "\n[" << model->GetName() << "] heading adjusted by " << 
        _msg->data << "degrees. New heading is " << this->drift << "\n";
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

    // Initialize drift to a random heading
    // private: double drift = rand() * M_PI / 180;
    private: double drift = 0;

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    
    /// \brief A ROS subscriber for twist
    private: ros::Subscriber rosSubTwist;
    
    /// \brief A ROS subscriber for heading
    private: ros::Subscriber rosSubHeading;
    
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SphereDrive)
}

