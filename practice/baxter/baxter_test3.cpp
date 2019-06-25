#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <string>

class Test
{
	ros::NodeHandle n;
	ros::Subscriber test_sub;
	ros::Publisher test_pub;
	
	public:
		Test()
		{
			std::cout << "Object is being created" << std::endl;
			//Subscriber
			test_sub = n.subscriber("robot/joint_states", 1, &Test::limb_move, this);
			//Publisher
			//test_pub = n.advertise<sensor_msgs::JointState>("robot/joint_states", 1)

		}
		
	void limb_move(const sensor_msgs::JointState msg)
	{
		for(int i = 0; i < msg.name.size(); i++)
		{
			std::cout << "name: " << msg.name[i] << "  ";
			std::cout << "position: " << msg.position[i] << "  ";
			std::cout << "velocity: " << msg.velocity[i] << "  ";
			std::cout << "effort: " << msg.effort[i] << std::endl;
			/* position[1][1] = 0.500;
			 * position[1][2] = 0.500;
			 * position[1][3] = 0.500;
			 * position[1][4] = 0.500;
			 * position[1][5] = 0.500;
			 * position[1][6] = 0.500;
			 * position[1][7] = 0.500;
			 * */
		}
		
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
