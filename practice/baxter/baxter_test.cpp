#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
//#include
class Test
{
	ros::NodeHandle n;
	ros::Publisher test_pub;
	ros::Subscriber test_sub;
	
	public:
		/*constructor?*/
		//Subscriber
		test_sub = n.subscriber("subscriber", 1, /*function*/, /*this?*/)
		//Publisher
		test_pub = n.advertise<std_msgs::Float32>("publisher", 1);

void /*function*/(const std_msgs::Float32::ConstPtr& msg)
{
	double x, x_des, e, kp, u;
	x = msg;
	//cin >> x_des;
	x_des = 2.0;
	e = x - x_des;
	kp = 1.0;
	u = -kp * e;
	cmd = u;
}
	test_pub.publish(cmd);
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "");
	
	ros::spin()
	return 0;
}
