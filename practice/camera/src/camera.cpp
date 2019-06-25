#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/RegionOfInterest.h"
#include "sensor_msgs/SetCameraInfo.h"


#include <iostream>
#include <string>
#include <typeinfo>
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/core/core.hpp>  

using namespace cv; 

class Test
{
	ros::NodeHandle n;
	ros::Subscriber image_sub, camerainfo_sub;
	//ros::Publisher ;
	ros::ServiceClient set_camerainfo_srv;
	int k;
	ros::Time time_init;
	public:
		Test()
		{
			//Subscriber
			image_sub = n.subscribe("camera/image_raw", 1, &Test::image, this);
			camerainfo_sub = n.subscribe("camera/camera_info", 1, &Test::camerainfo, this);
			//Publisher

			k = 1;
			time_init = ros::Time::now();

		}
		
	void image(const sensor_msgs::Image image_msg)
	{/*
		std::cout << "Herder : " << std::endl << image_msg.header << std::endl;
		std::cout << "Pose : " << std::endl << image_msg.height << std::endl;
		std::cout << "Twist : " << std::endl << image_msg.width << std::endl;
		std::cout << "Wrench : " << std::endl << image_msg.encoding << std::endl;
		std::cout << "Wrench : " << std::endl << image_msg.is_bigendian << std::endl;
		std::cout << "Wrench : " << std::endl << image_msg.step << std::endl;
		//std::cout << "Wrench : " << std::endl << image_msg.data << std::endl;
*/
		return;
	}	
	
		
	void camerainfo(const sensor_msgs::CameraInfo camerainfo_msg)
	{/*
		std::cout << "Herder : " << std::endl << camerainfo_msg.header << std::endl;
		std::cout << "Pose : " << std::endl << camerainfo_msg.height << std::endl;
		std::cout << "Twist : " << std::endl << camerainfo_msg.width << std::endl;
		std::cout << "Wrench : " << std::endl << camerainfo_msg.distortion_model << std::endl;
		//std::cout << "Wrench : " << std::endl << camerainfo_msg.D << std::endl;
		//std::cout << "Wrench : " << std::endl << camerainfo_msg.K << std::endl;
		//std::cout << "Wrench : " << std::endl << camerainfo_msg.R << std::endl;
		//std::cout << "Wrench : " << std::endl << camerainfo_msg.P << std::endl;
		std::cout << "Wrench : " << std::endl << camerainfo_msg.binning_x << std::endl;
		std::cout << "Wrench : " << std::endl << camerainfo_msg.binning_y << std::endl;
		std::cout << "Wrench : " << std::endl << camerainfo_msg.roi << std::endl;*/
		
		set_camerainfo_srv = n.serviceClient<sensor_msgs::SetCameraInfo>("camera/set_camera_info"); 				//request
		sensor_msgs::SetCameraInfo srv;
		
		
		
		return;
	}
	
	
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera");
		VideoCapture cap(0);//打开默认的摄像头
	if(!cap.isOpened())  
	{  
		return -1;  
	}  
	Mat frame;  	
	bool stop = false;  
	while(!stop)  
	{  		
		cap.read(frame); //  或cap>>frame;			
		imshow("Video",frame);
		
		if(waitKey(30)==27) //Esc键退出
		{
			stop = true;  
		}  
	}
	Test test; 
	ros::spin();
	return 0;
}
