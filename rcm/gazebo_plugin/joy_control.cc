#ifndef _JOY_CONTROL_HH_
#define _JOY_CONTROL_HH_

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>
#include <unistd.h>

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
			this->pid_vel = common::PID(1, 0.1, 0.3);
			this->pid_pos = common::PID(150, 0.01, 50);

			// Apply the P-controller to the joint.
			SetPID(this->Link1_joint, "vel", this->pid_vel);
			SetPID(this->Link2_1_joint, "vel", this->pid_vel);
			SetPID(this->Link1_joint, "pos", this->pid_pos);
			SetPID(this->Link2_1_joint, "pos", this->pid_pos);
			SetPID(this->Link_E_joint, "pos", this->pid_pos);

			// this->model->GetJointController()->SetVelocityPID(this->Link1_joint->GetScopedName(), this->pid);
			// this->model->GetJointController()->SetVelocityPID(this->Link2_1_joint->GetScopedName(), this->pid);
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
			std::string velocity = "/" + this->model->GetName() + "/vel_cmd";
			std::string position = "/" + this->model->GetName() + "/pos_cmd";

			ros::SubscribeOptions so =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(velocity,1,boost::bind(&JoyControl::RosVelMsg, this, _1),ros::VoidPtr(), &this->rosQueue);

			ros::SubscribeOptions so2 =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(position,1,boost::bind(&JoyControl::RosPosMsg, this, _1),ros::VoidPtr(), &this->rosQueue);

			this->rosSub = this->rosNode->subscribe(so);
			this->rosSub2 = this->rosNode->subscribe(so2);

			// Spin up the queue helper thread.
			this->rosQueueThread = std::thread(std::bind(&JoyControl::QueueThread, this));


		}
		void RosVelMsg(const std_msgs::Float64MultiArrayConstPtr &_msg){
			this->SetVelocity(this->Link1_joint, _msg->data[1]);
			this->SetVelocity(this->Link2_1_joint, _msg->data[2]);
			this->SetVelocity(this->Link_E_joint, _msg->data[0]);
		}

		void RosPosMsg(const std_msgs::Float64MultiArrayConstPtr &_msg){
			this->model->GetJointController()->SetPositionTarget(this->Link_E_joint->GetScopedName(), 0.0);
			sleep(1);
			this->SetPosition(this->Link1_joint, _msg->data[0]);
			this->SetPosition(this->Link2_1_joint, _msg->data[1]);
			sleep(1);
			this->SetPosition(this->Link_E_joint, _msg->data[2]);
		}

		/// Set the velocity of the Velodyne
		/// _vel New target velocity
		void SetVelocity(physics::JointPtr joint, const double &_vel){
			// Set the joint's target velocity.
			this->model->GetJointController()->SetVelocityTarget(joint->GetScopedName(), _vel);
    	}

		void SetPosition(physics::JointPtr joint, const double &_pos){
			// Set the joint's target position.
			this->model->GetJointController()->SetPositionTarget(joint->GetScopedName(), _pos);
    	}



    private: 
		/// \brief A node use for ROS transport
		private: std::unique_ptr<ros::NodeHandle> rosNode;
		/// \brief A ROS subscriber
		private: ros::Subscriber rosSub;
		private: ros::Subscriber rosSub2;
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
    	common::PID pid_vel;
		common::PID pid_pos;

		/// \brief ROS helper function that processes messages
		void QueueThread(){
			static const double timeout = 0.01;
			while (this->rosNode->ok()){
				this->rosQueue.callAvailable(ros::WallDuration(timeout));
			}
		}

		void SetPID(physics::JointPtr joint, std::string name, common::PID pid){
			if (name == "vel"){
				this->model->GetJointController()->SetVelocityPID(joint->GetScopedName(), pid);
			}
			else if(name == "pos"){
				this->model->GetJointController()->SetPositionPID(joint->GetScopedName(), pid);
			}
		}
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(JoyControl)
}
#endif