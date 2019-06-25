#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/core/core.hpp> 
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <string>
#include <iterator>
#include <algorithm>
#include <opencv2/objdetect/objdetect.hpp>  
#include "cv.h"
#include <tf/transform_broadcaster.h>
#include <math.h>
 
static bool readCameraParameters(std::string filename, cv::Mat &camMatrix, cv::Mat &distCoeffs) {
    //std::cout << "0" << std::endl;
    cv::FileStorage fs;
    
    //TEST read
    fs.open(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
		//std::cout << "File not open" << std::endl;
        return false;
	}
	else
	{
		//std::cout << "File is opened" << std::endl;
	}
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    /*int image_width;
    std::string camera_name;
    fs["image_width"] >> image_width;
    fs["camera_name"] >> camera_name;
    std::cout << "image_width: " << image_width << std::endl;
    std::cout << "camera_name: " << camera_name << std::endl;
    std::cout << "camMatrix: " << camMatrix << std::endl;
    std::cout << "distCoeffs: " << distCoeffs << std::endl;*/
    return true;
}

 
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
  
  
  	cv::VideoCapture inputVideo;
	inputVideo.open(0);
	sensor_msgs::ImagePtr msg;
    ros::Rate loop_rate(5);
		while (nh.ok()) 
		{
			cv::Mat camMatrix, distCoeffs;
			std::string filename = "/home/tianyu/aruco_ws/calibrationdata/test1.yaml";
			bool readOk = readCameraParameters(filename, camMatrix, distCoeffs);
			if(!readOk) 
			{
				std::cerr << "Invalid camera file" << std::endl;
				return 0;
			}
    
			cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
			//std::cout << "1" << std::endl;
			if(!inputVideo.grab())  
			{  
				return -1;  
			}  
			cv::Mat image, imageCopy;				
			//std::cout << "2" << std::endl;
			inputVideo.read(image); //cap>>frame;			
			image.copyTo(imageCopy);
			std::vector<int> ids; 
			std::vector<std::vector<cv::Point2f> > corners;
			//std::vector< cv::Vec3d > rvecs, tvecs;
			cv::aruco::detectMarkers(image, dictionary, corners, ids);
			// if at least one marker detected 

			if (ids.size() > 0) 
			{
				std::cout << "find marker" << std::endl;
				cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
				std::vector< cv::Vec3d > rvecs, tvecs;
		
				cv::aruco::estimatePoseSingleMarkers(corners, 0.120, camMatrix, distCoeffs, rvecs, tvecs); // draw axis for each marker 
				for(int i=0; i<ids.size(); i++) 
				{
					cv::aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1); 
					std::cout << "tvecs: " << tvecs[i] << std::endl;
					std::cout << "rvecs: " << rvecs[i] << std::endl;
				}
				#if 0
				static tf::TransformBroadcaster br1;
				tf::Transform transform1;
				transform1.setOrigin( tf::Vector3(0.0, 0.0, 1) );
				tf::Quaternion q1;
				q1.setRPY(M_PI/2, 0.0, 0.0);
				transform1.setRotation(q1);
				br1.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "world", "marker"));
				
				/*static tf::TransformBroadcaster br2;
				tf::Transform transform2;
				transform2.setOrigin( tf::Vector3(-tvecs[0][0], -tvecs[0][1], -tvecs[0][2]) );
				tf::Quaternion q2;
				q2.setRPY(-rvecs[0][0], -rvecs[0][1], -rvecs[0][2]);
				transform2.setRotation(q2);
				br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "marker", "camera"));*/				
				
				static tf::TransformBroadcaster br2;
				tf::Transform transform2;
				transform2.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
				tf::Quaternion q2;
				q2.setRPY(0.0, 0.0, -rvecs[0][2]);
				transform2.setRotation(q2);
				br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "marker", "marker1"));
				
				static tf::TransformBroadcaster br3;
				tf::Transform transform3;
				transform3.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
				tf::Quaternion q3;
				q3.setRPY(0.0, -rvecs[0][1], 0.0);
				transform3.setRotation(q3);
				br3.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "marker1", "marker2"));				
				
				static tf::TransformBroadcaster br4;
				tf::Transform transform4;
				transform4.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
				tf::Quaternion q4;
				q4.setRPY(-rvecs[0][0], 0.0, 0.0);
				transform4.setRotation(q4);
				br4.sendTransform(tf::StampedTransform(transform4, ros::Time::now(), "marker2", "marker3"));			
				
				/*static tf::TransformBroadcaster br2;
				tf::Transform transform2;
				transform2.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
				tf::Quaternion q2;
				q2.setRPY(-rvecs[0][0], -rvecs[0][1], -rvecs[0][2]);
				transform2.setRotation(q2);
				br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "marker", "marker3"));	*/			
				
				static tf::TransformBroadcaster br5;
				tf::Transform transform5;
				transform5.setOrigin( tf::Vector3(-tvecs[0][0], -tvecs[0][1], -tvecs[0][2]) );
				tf::Quaternion q5;
				q5.setRPY(0.0, 0.0, 0.0);
				transform5.setRotation(q5);
				br5.sendTransform(tf::StampedTransform(transform5, ros::Time::now(), "marker3", "camera"));				
				
				#endif
				///////////////////////////////////
				#if 0
				static tf::TransformBroadcaster br1;
				tf::Transform transform1;
				transform1.setOrigin( tf::Vector3(0.0, 0.0, 1) );
				tf::Quaternion q1;
				q1.setRPY(-M_PI/2, 0.0, 0.0);
				transform1.setRotation(q1);
				br1.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "world", "camera"));
				
				static tf::TransformBroadcaster br5;
				tf::Transform transform5;
				transform5.setOrigin( tf::Vector3(tvecs[0][0], tvecs[0][1], tvecs[0][2]) );
				tf::Quaternion q5;
				q5.setRPY(0.0, 0.0, 0.0);
				transform5.setRotation(q5);
				br5.sendTransform(tf::StampedTransform(transform5, ros::Time::now(), "camera", "marker1"));					
				
				static tf::TransformBroadcaster br2;
				tf::Transform transform2;
				transform2.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
				tf::Quaternion q2;
				q2.setRPY(rvecs[0][0], 0.0, 0.0);
				transform2.setRotation(q2);
				br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "marker1", "marker2"));
				
				static tf::TransformBroadcaster br3;
				tf::Transform transform3;
				transform3.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
				tf::Quaternion q3;
				q3.setRPY(0.0, rvecs[0][1], 0.0);
				transform3.setRotation(q3);
				br3.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "marker2", "marker3"));				
				
				static tf::TransformBroadcaster br4;
				tf::Transform transform4;
				transform4.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
				tf::Quaternion q4;
				q4.setRPY(0.0, 0.0, rvecs[0][2]);
				transform4.setRotation(q4);
				br4.sendTransform(tf::StampedTransform(transform4, ros::Time::now(), "marker3", "marker"));				
				
			
				
				#endif				
				///////////////////////////////////////////////

				#if 0
				static tf::TransformBroadcaster br1;
				tf::Transform transform1;
				transform1.setOrigin( tf::Vector3(0.0, 0.0, 1) );
				tf::Quaternion q1;
				q1.setRPY(-M_PI/2, 0.0, 0.0);
				transform1.setRotation(q1);
				br1.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "world", "camera"));
							
				static tf::TransformBroadcaster br2;
				tf::Transform transform2;
				transform2.setOrigin( tf::Vector3(tvecs[0][0], tvecs[0][1], tvecs[0][2]) );
				tf::Quaternion q2;
				q2.setRPY(rvecs[0][0], rvecs[0][1], rvecs[0][2]);
				transform2.setRotation(q2);
				br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "camera", "marker"));
				
				#endif


				
				//try matrix
	float DR, RD, alx, aly, alz;
	float s1, c1, s2, c2, s3, c3;
	float R3F[3][3];
	
	DR=M_PI/180.0;
	RD=1/DR;
	
	/*alx = 37; aly = -26; alz = 49;
	
	s1 = sin(alx * DR);
	c1 = cos(alx * DR);
	s2 = sin(aly * DR);
	c2 = cos(aly * DR);
	s3 = sin(alz * DR);
	c3 = cos(alz * DR);*/
	
	s1 = sin(rvecs[0][0]);
	c1 = cos(rvecs[0][0]);
	s2 = sin(rvecs[0][1]);
	c2 = cos(rvecs[0][1]);
	s3 = sin(rvecs[0][2]);
	c3 = cos(rvecs[0][2]);		
	
	float RF1[3][3] = 
	{
		{1,0,0},
		{0,c1,-s1},
		{0,s1,c1}
	};
	float R12[3][3] = 
	{
		{c2,0,s2},
		{0,1,0},
		{-s2,0,c2}
	};
	float R23[3][3] = 
	{
		{c3,-s3,0},
		{s3,c3,0},
		{0,0,1}
	};
	
	float RF3[3][3] = 
	{
		{0,0,0},
		{0,0,0},
		{0,0,0}
	};	
	
	//RF3[3][3] = RF1[3][3] * R12[3][3] * R23[3][3];
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			for(int k = 0; k < 3; k++)
			{
				RF3[i][j] += RF1[i][k] * R12[k][j];
			}
		}
	}
	
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			for(int k = 0; k < 3; k++)
			{
				RF3[i][j] += RF3[i][k] * R23[k][j];
			}
		}
	}
	
	/*for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			std::cout << " " << RF3[i][j];
		}
	}
	std::cout << std::endl;*/
	
	//transpose matrix
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			R3F[j][i]=RF3[i][j];
		}
	}
	
	/*for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			std::cout << " " << R3F[i][j];
		}
	}
	std::cout << std::endl;*/
				
				
				
				
				
				
				//end try
				
				
				float mx, my, mz, v, c, s, theta;
				//std::vector< cv::Vec3d > nrvecs;
				//nrvecs = norm(rvecs[0])

				theta = sqrt(pow(rvecs[0][0], 2.0) + pow(rvecs[0][1], 2.0) + pow(rvecs[0][2], 2.0));
				mx = rvecs[0][0]/theta;
				my = rvecs[0][1]/theta;
				mz = rvecs[0][2]/theta;				
				
				s = sin(theta);
				c = cos(theta);
				v = (1 - c);
				float RAB[3][3] = 
				{
					{mx * mx * v + c,       mx * my * v - mz * s,  mx * mz * v + my * s},
					{mx * my * v + mz * s,  my * my * v + c,       my * mz * v - mx * s},
					{mx * mz * v - my * s,  my * mz * v + mx * s,  mz * mz * v + c}
				};
				float RBA[3][3] = 
				{
					{0,0,0},
					{0,0,0},
					{0,0,0}
				};	
				
				//transpose matrix
				for(int i = 0; i < 3; i++)
				{
					for(int j = 0; j < 3; j++)
					{
						RBA[j][i]=RAB[i][j];
					}
				}
				
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			std::cout << " " << RAB[i][j];
		}
	}
	std::cout << std::endl;				
				
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			std::cout << " " << RBA[i][j];
		}
	}
	std::cout << std::endl;				
				#if 0
				static tf::TransformBroadcaster br1;
				tf::Transform transform1;
				transform1.setOrigin( tf::Vector3(0.0, 0.0, 1) );
				tf::Quaternion q1;
				q1.setRPY(-M_PI/2, 0.0, 0.0);
				transform1.setRotation(q1);
				br1.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "world", "camera"));
							
				static tf::TransformBroadcaster br2;
				tf::Transform transform2;
				transform2.setOrigin( tf::Vector3(tvecs[0][0], tvecs[0][1], tvecs[0][2]) );
				transform2.setBasis(tf::Matrix3x3(RAB[0][0],RAB[0][1],RAB[0][2],RAB[1][0],RAB[1][1],RAB[1][2],RAB[2][0],RAB[2][1],RAB[2][2]));					
				br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "camera", "marker"));				
								
				#endif
				
				///////////////////////////////
				#if 1
				static tf::TransformBroadcaster br1;
				tf::Transform transform1;
				transform1.setOrigin( tf::Vector3(0.0, 0.0, 1) );
				tf::Quaternion q1;
				q1.setRPY(M_PI/2, 0.0, 0.0);
				transform1.setRotation(q1);
				br1.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "world", "marker"));
				
				static tf::TransformBroadcaster br2;
				tf::Transform transform2;
				transform2.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
				transform2.setBasis(tf::Matrix3x3(RBA[0][0],RBA[0][1],RBA[0][2],RBA[1][0],RBA[1][1],RBA[1][2],RBA[2][0],RBA[2][1],RBA[2][2]));
				br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "marker", "marker3"));				
				
				static tf::TransformBroadcaster br5;
				tf::Transform transform5;
				transform5.setOrigin( tf::Vector3(-tvecs[0][0], -tvecs[0][1], -tvecs[0][2]) );
				tf::Quaternion q5;
				q5.setRPY(0.0, 0.0, 0.0);
				transform5.setRotation(q5);
				br5.sendTransform(tf::StampedTransform(transform5, ros::Time::now(), "marker3", "camera"));				
				
				#endif
			}

/*			cv::imshow("out", imageCopy); 
	
			
			if(cv::waitKey(30)==27) 
			{
				stop = true;  
			}  */


			msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageCopy).toImageMsg(); 
			pub.publish(msg);
			cv::waitKey(1);

		} 
    ros::spinOnce();
    loop_rate.sleep();
    return 0;
}






