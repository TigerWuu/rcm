#ifndef _JOY_CONTROL_HH_
#define _JOY_CONTROL_HH_

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  	class JoyControl : public ModelPlugin
  {
    public: 
		JoyControl():ModelPlugin(){
			// Just output a message for now
      		std::cout << "I am Tiger"<<"\n";
		}

		virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
			// Safety check
			if (_model->GetJointCount() == 0){
				std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
				return;
			}
			
			// Store the model pointer for convenience.
			this->model = _model;
			// Get the first joint. We are making an assumption about the model
			// having one joint that is the rotational joint.
			this->Link1_joint = _model->GetJoints()[0];
			this->Link2_1_joint = _model->GetJoints()[1];
			this->Link_E_joint = _model->GetJoints()[7];

			// Setup a P-controller, with a gain of 0.1.
			this->pid = common::PID(1, 0.1, 0.3);

			// Apply the P-controller to the joint.
			this->model->GetJointController()->SetVelocityPID(this->Link1_joint->GetScopedName(), this->pid);
			this->model->GetJointController()->SetVelocityPID(this->Link2_1_joint->GetScopedName(), this->pid);
			// this->model->GetJointController()->SetVelocityPID(this->Link_E_joint->GetScopedName(), this->pid);

			// Initialize ros, if it has not already bee initialized.
			if (!ros::isInitialized()){
				int argc = 0;
				char **argv = NULL;
				ros::init(argc, argv, "gazebo_client",ros::init_options::NoSigintHandler);
			}

			// Create our ROS node. This acts in a similar manner to the Gazebo node
			this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

			// Create a named topic, and subscribe to it.
			ros::SubscribeOptions so =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
				"/" + this->model->GetName() + "/vel_cmd",
				1,
				boost::bind(&JoyControl::OnRosMsg, this, _1),
				ros::VoidPtr(), &this->rosQueue);
			this->rosSub = this->rosNode->subscribe(so);

			// Spin up the queue helper thread.
			this->rosQueueThread = std::thread(std::bind(&JoyControl::QueueThread, this));

			// Create the node
			// this->node = transport::NodePtr(new transport::Node());
			// #if GAZEBO_MAJOR_VERSION < 8
			// this->node->Init(this->model->GetWorld()->GetName());
			// #else
			// this->node->Init(this->model->GetWorld()->Name());
			// #endif

			// // Create a topic name
			// std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";
      		// std::cout << topicName <<"\n";
			// // Subscribe to the topic, and register a callback
			// this->sub = this->node->Subscribe(topicName, &JoyControl::OnMsg, this);

		}
		void OnRosMsg(const std_msgs::Float64MultiArrayConstPtr &_msg){
			this->SetVelocity(this->Link1_joint, _msg->data[1]);
			this->SetVelocity(this->Link2_1_joint, _msg->data[2]);
			this->SetVelocity(this->Link_E_joint, _msg->data[0]);
		}

		/// Set the velocity of the Velodyne
		/// _vel New target velocity
		void SetVelocity(physics::JointPtr joint, const double &_vel){
			// Set the joint's target velocity.
			this->model->GetJointController()->SetVelocityTarget(joint->GetScopedName(), _vel);
    	}

    private: 
		/// \brief A node use for ROS transport
		private: std::unique_ptr<ros::NodeHandle> rosNode;
		/// \brief A ROS subscriber
		private: ros::Subscriber rosSub;
		/// \brief A ROS callbackqueue that helps process messages
		private: ros::CallbackQueue rosQueue;
		/// \brief A thread the keeps running the rosQueue
		private: std::thread rosQueueThread;

		/// \brief Pointer to the model.
		physics::ModelPtr model;
    	/// \brief Pointer to the joint.
		physics::JointPtr Link1_joint;
		physics::JointPtr Link2_1_joint;
		physics::JointPtr Link_E_joint;
    	/// \brief A PID controller for the joint.
    	common::PID pid;

		/// \brief ROS helper function that processes messages
		void QueueThread(){
			static const double timeout = 0.01;
			while (this->rosNode->ok()){
				this->rosQueue.callAvailable(ros::WallDuration(timeout));
			}
		}

		// /// \brief A node used for transport
		// transport::NodePtr node;
		// /// \brief A subscriber to a named topic.
		// transport::SubscriberPtr sub;


		/// \brief Handle incoming message
		/// \param[in] _msg Repurpose a vector3 message. This function will
		/// only use the x component.
		// void OnMsg(ConstVector3dPtr &_msg){
		// 	this->SetVelocity(_msg->x());
		// }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(JoyControl)
}
#endif