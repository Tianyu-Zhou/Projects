#include<opencv2/objdetect/objdetect.hpp>  
#include<opencv2/highgui/highgui.hpp>  
#include<opencv2/imgproc/imgproc.hpp>  
#include "highgui.h"//包含opencv库头文件
#include "cv.h"
#include <iostream>
 
using namespace cv;  
using namespace std;
  
CascadeClassifier faceCascade;  
  
int main()  
{  
    faceCascade.load("/home/tianyu/opencv/data/haarcascades/haarcascade_frontalface_alt2.xml");
 
    //CvCapture *capture   = NULL;// 视频获取结构，用来作为视频获取函数的一个参数
 
    //capture = cvCreateCameraCapture(0);//打开摄像头，从摄像头中获取视频
 
	VideoCapture capture;
	capture.open(0);    // 打开摄像头
//      capture.open("video.avi");    // 打开视频
	if(!capture.isOpened())
	{
	  cout << "open camera failed. " << endl;
	  return -1;
	}
 
    Mat img, imgGray;
    vector<Rect> faces;
	while(1)
	{
		capture >> img;    // 读取图像至img
		if(img.empty())	
		{
			continue;
		}
 
		if(img.channels() == 3)	
		{  
		   cvtColor(img, imgGray, CV_RGB2GRAY);  
		}  
		else  
		{  
		   imgGray = img;  
		}  
 
	    faceCascade.detectMultiScale(imgGray, faces, 1.2, 6, 0, Size(0, 0));    // 检测人脸
		
		if(faces.size()>0)	
		{  
		   for(int i =0; i<faces.size(); i++)  
		   {  
			   rectangle(img, Point(faces[i].x, faces[i].y), Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height), Scalar(0, 255, 0), 1, 8);	
		   }  
		}  
		
		imshow("CamerFace", img);      // 显示
		
		if(waitKey(1) > 0)		// delay ms 等待按键退出
		{
			break;
		}
	}
  
    return 0;  
}  

