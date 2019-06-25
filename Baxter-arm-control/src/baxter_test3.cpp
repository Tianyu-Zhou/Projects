#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "baxter_core_msgs/JointCommand.h"
#include "baxter_core_msgs/EndEffectorCommand.h"
#include "baxter_core_msgs/EndEffectorState.h"
#include "baxter_core_msgs/EndpointState.h"
#include "sensor_msgs/Range.h"
#include "baxter_core_msgs/SolvePositionIK.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

#include <iostream>
#include <string>
#include <typeinfo>

class Test
{
	ros::NodeHandle n;
	ros::Subscriber test_sub_limb, test_sub_gripstate, test_sub_endpoint, test_sub_right_hand_range;
	ros::Publisher test_pub_right_limb, test_pub_left_limb, test_pub_right_grip;
	ros::ServiceClient client_endpoint_command;
	int k;
	ros::Time time_init;
	public:
		Test()
		{
			//std::cout << "Object is being created" << std::endl;
			//Subscriber
			test_sub_limb = n.subscribe("robot/joint_states", 1, &Test::limb_move, this);
			test_sub_gripstate = n.subscribe("robot/end_effector/right_gripper/state", 1, &Test::gripstate, this);
			test_sub_endpoint = n.subscribe("/robot/limb/right/endpoint_state", 1, &Test::endpointstate, this);
			test_sub_right_hand_range = n.subscribe("/robot/range/right_hand_range/state", 1, &Test::righthandrange, this);
			//Publisher
			test_pub_right_limb = n.advertise<baxter_core_msgs::JointCommand>("robot/limb/right/joint_command", 1);
			test_pub_left_limb = n.advertise<baxter_core_msgs::JointCommand>("robot/limb/left/joint_command", 1);
			test_pub_right_grip = n.advertise<baxter_core_msgs::EndEffectorCommand>("robot/end_effector/right_gripper/command", 1);
			k = 1;
			time_init = ros::Time::now();

		}
		
	void righthandrange(const sensor_msgs::Range right_hand_msg)
	{
		//std::cout << right_hand_msg.header << std::endl;
		//std::cout << right_hand_msg.radiation_type << std::endl;
		//std::cout << right_hand_msg.field_of_view << std::endl;
		//std::cout << right_hand_msg.min_range << std::endl;
		//std::cout << right_hand_msg.max_range << std::endl;
		//std::cout << right_hand_msg.range << std::endl;
		// Don't know the value meaning.	
		return;
	}	
	
		
	void endpointstate(const baxter_core_msgs::EndpointState endpoint_msg)
	{
		//std::cout << "Herder : " << std::endl << endpoint_msg.header << std::endl;
		//std::cout << "Pose : " << std::endl << endpoint_msg.pose << std::endl;
		//std::cout << "Twist : " << std::endl << endpoint_msg.twist << std::endl;
		//std::cout << "Wrench : " << std::endl << endpoint_msg.wrench << std::endl;
		return;
	}
	
	
	
	void gripstate(const baxter_core_msgs::EndEffectorState grip_msg)
	{/*
		std::cout << grip_msg.enabled << std::endl;
		std::cout << grip_msg.calibrated << std::endl;
		std::cout << grip_msg.ready << std::endl;
		std::cout << grip_msg.moving << std::endl;
		std::cout << grip_msg.gripping << std::endl;
		std::cout << grip_msg.missed << std::endl;
		std::cout << grip_msg.error << std::endl;
		std::cout << grip_msg.reverse << std::endl;
		std::cout << grip_msg.id << std::endl;
		std::cout << grip_msg.timestamp << std::endl;
		std::cout << grip_msg.position << std::endl;
		std::cout << grip_msg.force << std::endl;
		std::cout << grip_msg.state << std::endl;
		std::cout << grip_msg.command << std::endl;
		std::cout << grip_msg.command_sender << std::endl;
		std::cout << grip_msg.command_sequence << std::endl;	*/	
		
		return;
	}
		
	void limb_move(const sensor_msgs::JointState msg)
	{/*
		for(int i = 0; i < msg.name.size(); i++)
		{
			std::cout << "name: " << msg.name[i] << "  ";
			std::cout << "position: " << msg.position[i] << "  ";
			std::cout << "velocity: " << msg.velocity[i] << "  ";
			std::cout << "effort: " << msg.effort[i] << std::endl;
		}*/
		
					//client
			client_endpoint_command = n.serviceClient<baxter_core_msgs::SolvePositionIK>("ExternalTools/right/PositionKinematicsNode/IKService"); 				//request
			baxter_core_msgs::SolvePositionIK srv;
			
			geometry_msgs::PoseStamped pose_stamp;
			geometry_msgs::PoseStamped pose_array[1];
			
			//pose_stamp.header.stamp = ros::Time::now();
			//pose_stamp.header.seq = 0;
			pose_stamp.header.frame_id = "base";
			pose_stamp.pose.position.x = 0.657579481614;
			pose_stamp.pose.position.y = -0.851981417433;
			pose_stamp.pose.position.z = 0.0388352386502;
			pose_stamp.pose.orientation.x = 0.366894936773;
			pose_stamp.pose.orientation.y = 0.885980397775;
			pose_stamp.pose.orientation.z = -0.108155782462;
			pose_stamp.pose.orientation.w = 0.262162481772;
			
			//pose_stamp.pose.position.....
			//pose_array[1] = pose_stamp;
			//pose_stamp.pose.position.....
			//pose_array[2] = pose_stamp;
			
			srv.request.pose_stamp.push_back(pose_stamp);
			//srv.request.seed_mode = 2;
			std::cout << "pose_stamp: " << srv.request.pose_stamp.at(0).pose << std::endl;
			client_endpoint_command.call(srv);
			if(client_endpoint_command.call(srv)) std::cout << "srv succese" << std::endl;
			else std::cout << "srv false" << std::endl;		
			
			if(srv.response.isValid[0]) std::cout << "Solution found" << std::endl;
			else std::cout << "Solution not found" << std::endl;
			
			// std::cout << "Result type " << srv.response.result_type << std::endl;

/*			for(int i = 0; i < 1; i++)
		{

			std::cout << "name: " << srv.response.joints[0].name[i] << "  ";
			std::cout << "position: " << srv.response.joints[0].position[i] << "  ";
			std::cout << "velocity: " << srv.response.joints[0].velocity[i] << "  ";
			std::cout << "effort: " << srv.response.joints[0].effort[i] << std::endl;
		}*/
		for (int i = 0; i < srv.response.joints.size(); i++)
		{
			std::cout << "position: " << srv.response.joints[i] << std::endl;
		}
		
		
		
			baxter_core_msgs::JointCommand cmd1;
			baxter_core_msgs::JointCommand cmd2;

			cmd1.mode = 1;
			cmd2.mode = 1;
			
			
			baxter_core_msgs::EndEffectorCommand cmd3;
			
			//rosrun baxter_interface gripper_action_server.py
			
			//cmd3.id = 65538;
			//cmd3.command = "grip";
			//cmd3.command = "release";

			
			//std::cout << cmd3.command << std::endl;
			//test_pub_right_grip.publish(cmd3);   

			
			cmd1.names.push_back("right_s0");
			cmd1.names.push_back("right_s1");			
			cmd1.names.push_back("right_e0");					
			cmd1.names.push_back("right_e1");					
			cmd1.names.push_back("right_w0");						
			cmd1.names.push_back("right_w1");			
			cmd1.names.push_back("right_w2");
			cmd1.command.resize(cmd1.names.size());
			
			cmd2.names.push_back("left_s0");
			cmd2.names.push_back("left_s1");			
			cmd2.names.push_back("left_e0");					
			cmd2.names.push_back("left_e1");					
			cmd2.names.push_back("left_w0");						
			cmd2.names.push_back("left_w1");			
			cmd2.names.push_back("left_w2");
			cmd2.command.resize(cmd2.names.size());
			
			
			float angle_command[3][7];
			angle_command[0][0] = -0.459;
			angle_command[0][1] = -0.202;
			angle_command[0][2] = 1.807;
			angle_command[0][3] = 1.714;
			angle_command[0][4] = -0.906;
			angle_command[0][5] = -1.545;
			angle_command[0][6] = -0.276;
			
			
			
			angle_command[1][0] = -0.395;
			angle_command[1][1] = -0.202;
			angle_command[1][2] = 1.831;
			angle_command[1][3] = 1.981;
			angle_command[1][4] = -1.979;
			angle_command[1][5] = -1.100;
			angle_command[1][6] = -0.448;	
			
			for(int i = 0 ; i < srv.response.joints[0].position.size(); i ++)
			{
				angle_command[2][i] = srv.response.joints.at(0).position.at(i);
			}
//sine wave
/*			double t = (ros::Time::now()-time_init).toSec();
			double w = 2*M_PI/5.0;
			if (k == 1)
			{
				std::cout << "sine wave" << std::endl;
				for (int i = 0; i < cmd1.names.size(); i++)
				{
					if(fabs(angle_command[0][i])<fabs(angle_command[1][i])) w = -w;
					cmd1.command[i] = fabs(angle_command[0][i] - angle_command[1][i])/2 * sin(w * t) + (angle_command[0][i] + angle_command[1][i])/2;
				}
				test_pub_right_limb.publish(cmd1);
				///error = msg.position[11] - angle_command[0][0];
				//if (fabs(error) <= 0.001) k++;
				//std::cout << fabs(error) << std::endl;
			}  */
// Test joint vector
			/*
			angle_command[2][0] = 0.000;
			angle_command[2][1] = 0.000;
			angle_command[2][2] = 0.000;
			angle_command[2][3] = 0.000;
			angle_command[2][4] = 0.000;
			angle_command[2][5] = 0.000;
			angle_command[2][6] = 0.000;*/		
			
//left			
/*			if (k == 1)
			{
				std::cout << "Condition 1" << std::endl;
				for (int i = 0; i < cmd2.names.size(); i++)
				{
					cmd2.command[i] = angle_command[2][i];
				}
				test_pub_left_limb.publish(cmd2); 
			}  */
//right		
float error;	
			if (k == 1)
			{
				cmd3.id = 65538;
				cmd3.command = "release";
				test_pub_right_grip.publish(cmd3); 
				
				std::cout << "Condition 1" << std::endl;
				for (int i = 0; i < cmd1.names.size(); i++)
				{
					cmd1.command[i] = angle_command[2][i];
				}
				test_pub_right_limb.publish(cmd1); 
				error = msg.position[11] - angle_command[2][0];
				if (fabs(error) <= 0.001) k++;
			} 
			else if (k == 2)
			{
				ros::Duration(1.0).sleep();
				k++;
			}
			else if (k == 3)
			{
				std::cout << "Condition 2" << std::endl;
				cmd3.id = 65538;
				cmd3.command = "grip";
				test_pub_right_grip.publish(cmd3);   
				k++;
			}
			else if (k == 4)
			{
				ros::Duration(1.0).sleep();
				k++;
			}
			else if (k == 5)
			{
				std::cout << "Condition 3" << std::endl;
				for (int i = 0; i < cmd1.names.size(); i++)
				{
					cmd1.command[i] = angle_command[0][i];
				}
				test_pub_right_limb.publish(cmd1);
				error = msg.position[11] - angle_command[0][0];
				if(fabs(error) <= 0.001) k++;
			}
			else if (k == 6)
			{
				ros::Duration(1.0).sleep();
				k++;
			}
			else if (k == 7)
			{
				std::cout << "Condition 4" << std::endl;
				cmd3.id = 65538;
				cmd3.command = "release";
				test_pub_right_grip.publish(cmd3);   
				k++;
			}
			else if(k == 8)
			{
				ros::Duration(1.0).sleep();
				k = 1;
			}
			
						
// Wave hand
/*			float error;
			
			if (k == 1)
			{
				std::cout << "Condition 1" << std::endl;
				for (int i = 0; i < cmd1.names.size(); i++)
				{
					cmd1.command[i] = angle_command[0][i];
				}
				test_pub_right_limb.publish(cmd1);
				error = msg.position[11] - angle_command[0][0];
				if (fabs(error) <= 0.001) k++;
				std::cout << fabs(error) << std::endl;
			}
			
			else if(k == 2)
			{
				std::cout << "Condition 2" << std::endl;
				for (int i = 0; i < cmd1.names.size(); i++)
				{
					cmd1.command[i] = angle_command[1][i];
				}
				test_pub_right_limb.publish(cmd1);
				error = msg.position[11] - angle_command[1][0];
				if(fabs(error) <= 0.001) k--;
				std::cout << fabs(error) << std::endl;
				
			}  */
	
		return;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "baxter_test3");
	Test test; 
	ros::spin();
	return 0;
}
