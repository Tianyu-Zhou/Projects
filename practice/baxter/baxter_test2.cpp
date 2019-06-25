#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
//#include
class Test
{
	ros::NodeHandle n;
	ros::Publisher test_pub_left, test_pub_right, test_pub_grip_left, test_pub_grip_right, test_pub_lj, test_pub_rj;
	ros::Subscriber test_sub_left, test_sub_right, test_sub_grip_left, test_sub_grip_right, test_sub_lj, test_sub_rj;
	
	public:
		/*constructor?*/
		//Subscriber
		test_sub_left = n.subscriber("baxter_interface.Limb('left')", 1, /*left_limb*/, /*this?*/)
		test_sub_right = n.subscriber("baxter_interface.Limb('right')", 1, /*function*/, /*this?*/)
		test_sub_grip_letf = n.subscriber("baxter_interface.Gripper('left')", 1, /*function*/, /*this?*/)
		test_sub_grip_right = n.subscriber("baxter_interface.Gripper('right')", 1, /*function*/, /*this?*/)
		test_sub_lj = n.subscriber("left.joint_name", 1, /*function*/, /*this?*/)
		test_sub_rj = n.subscriber("right.joint_name", 1, /*function*/, /*this?*/)
		
		
		/*limb.joint_angles() */
		
		//Publisher
		//test_pub = n.advertise<std_msgs::Float32>("publisher", 1);
		test_pub_left = n.advertise<std_msgs::Float32MultiArray>("baxter_interface.Limb('left')", 1)
		test_pub_right = n.advertise<std_msgs::Float32MultiArray>("baxter_interface.Limb('right')", 1)
		test_pub_grip_letf = n.advertise<std_msgs::Float32MultiArray>("baxter_interface.Gripper('left')", 1,)
		test_pub_grip_right = n.advertise<std_msgs::Float32MultiArray>("baxter_interface.Gripper('right')", 1)
		test_pub_lj = n.advertise<std_msgs::Float32MultiArray>("left.joint_name", 1)
		test_pub_rj = n.advertise<std_msgs::Float32MultiArray>("right.joint_name", 1)


void /*left_limb_1*/(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	double current_position[10], position_design[10], error[10], kp, u[10];
	current_position = msg;
	// left_s0, left_s1, left_e0, left_e1, left_w0, left_w1, left_w2
	//cin >> x_des;
	position_design[10] = {'left_s0' = -0.459, 'left_s1' = -0.202, 'left_e0' = 1.807, 'left_e1' = 1.714, 'left_w0' = -0.906, 'left_w1' = -1.545, 'left_w2' = -0.276};
	error[10] = current_position[10] - position_design[10];
	kp = 1.0;
	u[10] = -kp * error;
	cmd = u[10];
	test_pub_left.publish(cmd);
}

viod /*left_limb_2*/(const std::msgs::Float32::Constptr& msg)
{
	double current_position[10], position_design[10], error[10], kp, u[10];
	current_position = msg;
	//cin >> x_des;
	position_design[10] = {-0.395, -0.202, 1.831, 1.981, -1.979, -1.100, -0.448};
	error[10] = current_position[10] - position_design[10];
	kp = 1.0;
	u[10] = -kp * error;
	cmd = u[10];
	test_pub_left.publish(cmd);	
}

};



/*	if (grip_left.error())
		grip_left.reset();
	if (grip_right.error())
		grip_tight.reset();
	if (not grip_left.calibrated() and grip_left.type() != 'custom')
		grip_left.calibrate();
	if (not grip_right.calibrated() and grip_right.type() != 'custom')
		grip_right.calibrate();*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "");
	ros::Rate loop_rate(10);
	
	ros::spin()
	return 0;
}
