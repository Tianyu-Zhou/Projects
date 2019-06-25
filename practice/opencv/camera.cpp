#if 0
#include "highgui.h"//包含opencv库头文件
#include "cv.h"
 
int main( int argc, char** argv ) {//主函数
 
  cvNamedWindow( "Example2", CV_WINDOW_AUTOSIZE );//创建窗口，（名字，默认大小）
 CvCapture *capture   = NULL;// 视频获取结构，用来作为视频获取函数的一个参数
 
   capture = cvCreateCameraCapture(0);//打开摄像头，从摄像头中获取视频
   IplImage* frame;//申请IplImage类型指针，就是申请内存空间来存放每一帧图像
    while(1) {
        frame = cvQueryFrame( capture );// 从摄像头中抓取并返回每一帧
        if( !frame ) break;//如果抓取帧为空   break 打破循环否则将抓取的那一帧显示在创建的窗口上
        cvShowImage( "Example2", frame );//在窗口上显示每一帧
        char c = cvWaitKey(33);//延时，每秒钟约33帧；符合人眼观看速度；
        if( c == 27 ) break;//由于是死循环，而且没有控制台，当按下键盘esc键，将按键的ASCII值给C,如果C为ESC（ASCII 为27）循环退出退出循环；
            }
    cvReleaseCapture( &capture );//释放内存；
    cvDestroyWindow( "Example2" );//销毁窗口
 return 0;
}
#endif

#if 1

#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/core/core.hpp>  
 
using namespace cv; 
 
int main(int argc,char *argv[])  
{  
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
	return 0;  
}  

#endif
