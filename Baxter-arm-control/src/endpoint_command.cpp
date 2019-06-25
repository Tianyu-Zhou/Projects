#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "baxter_core_msgs/SolvePositionIK.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/JointState.h"

#include <iostream>
#include <string>
#include <typeinfo>
#include <cstdlib>


class Test
{
	ros::NodeHandle n;
	ros::ServiceClient client_endpoint_command;
	int k;
	ros::Time time_init;
	public:
		Test()
		{
			
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
		//std::cout << "isValid: " << srv.response.names[0] << std::endl;
			//srv.request.pose_stamp.resize(pose_array.size());
			//srv.request.pose_stamp = pose_array;
			//std::cout << "1" << std::endl;
			k = 1;
			time_init = ros::Time::now();
		}

};




int main(int argc, char **argv)
{
	ros::init(argc, argv, "endpoint_command");
	Test test; 
	ros::spin();
	return 0;
}
