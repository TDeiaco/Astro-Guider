//**********************************************
//
//  Clarify - Image Stacking Software
//
//  Using SDL and OpenCV
//
//
//**********************************************

#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <cerrno>
#include <termios.h>
#include <unistd.h>
#include <math.h>
#include <vector>


using namespace std;
using namespace cv;


IplImage* imgTracking;
int lastX = -1;
int lastY = -1;

int port_file;


//This function threshold the HSV image and create a binary image
IplImage* GetThresholdedImage(IplImage* imgHSV){       
  IplImage* imgThresh=cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
  cvInRangeS(imgHSV, cvScalar(0,0,20), cvScalar(180,256,256), imgThresh); 
  return imgThresh;
}

void trackObject(IplImage* imgThresh){
  // Calculate the moments of 'imgThresh'
  CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
  cvMoments(imgThresh, moments, 1);
  double moment10 = cvGetSpatialMoment(moments, 1, 0);
  double moment01 = cvGetSpatialMoment(moments, 0, 1);
  double area = cvGetCentralMoment(moments, 0, 0);

  // if the area<1000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
  if(area>50){
    // calculate the position of the ball
    int posX = moment10/area;
    int posY = moment01/area;  

    //The delta x and y shouldn't be more than 0xFF
    //Therefore 0xFF can be used as a header to organize
    //the coordinate transfer
    char head = 0xFF;
    char xpos = (char)posX;
    char ypos = (char)posY;

    write(port_file, &head, 1);
    write(port_file, &xpos, 1);
    write(port_file, &ypos, 1);
        
    if(lastX>=0 && lastY>=0 && posX>=0 && posY>=0)
      {
	// Draw a yellow line from the previous point to the current point
	cvLine(imgTracking, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(0,0,255), 4);
      }

    lastX = posX;
    lastY = posY;
  }

  free(moments);
}


int main(){

  struct termios terminalAttributes;

  port_file  = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY | O_FSYNC );

  // clear terminalAttributes data
  memset(&terminalAttributes, 0, sizeof(struct termios));

  terminalAttributes.c_cflag = B57600 | CS8 | CLOCAL | CREAD; 
  terminalAttributes.c_iflag = IGNPAR |  ONLCR;
  terminalAttributes.c_oflag = OPOST;
  terminalAttributes.c_cc[VTIME] = 0;
  terminalAttributes.c_cc[VMIN] = 1;

  tcsetattr(port_file, TCSANOW, &terminalAttributes);

  
  CvCapture* capture =0;       
  capture = cvCaptureFromFile("mars2.avi");
  //capture = cvCaptureFromCAM(0);
  if(!capture){
    printf("Capture failure\n");
    return -1;
  }
      
  IplImage* frame=0;
  frame = cvQueryFrame(capture);           
  if(!frame) return -1;
  
  //create a blank image and assigned to 'imgTracking' which has the same size of original video
  imgTracking=cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U, 3);
  cvZero(imgTracking); //covert the image, 'imgTracking' to black

  cvNamedWindow("Video");     
  cvNamedWindow("Ball");

  //iterate through each frames of the video     
  while(true){
    

    frame = cvQueryFrame(capture);           
    if(!frame) break;
    frame=cvCloneImage(frame); 
            
    cvSmooth(frame, frame, CV_GAUSSIAN,3,3); //smooth the original image using Gaussian kernel

    IplImage* imgHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3); 
    cvCvtColor(frame, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV
    IplImage* imgThresh = GetThresholdedImage(imgHSV);
          
    cvSmooth(imgThresh, imgThresh, CV_GAUSSIAN,3,3); //smooth the binary image using Gaussian kernel
            
    //track the possition of the ball
    trackObject(imgThresh);

    // Add the tracking image and the frame
    cvAdd(frame, imgTracking, frame);

    cvShowImage("Ball", imgThresh);           
    cvShowImage("Video", frame);
           
    //Clean up used images
    cvReleaseImage(&imgHSV);
    cvReleaseImage(&imgThresh);            
    cvReleaseImage(&frame);

    //Wait 10mS
    int c = cvWaitKey(30);
    //If 'ESC' is pressed, break the loop
    if(c >=0 ) break;      
  }

  close(port_file);

  cvDestroyAllWindows() ;
  cvReleaseImage(&imgTracking);
  cvReleaseCapture(&capture);     

  return 0;
}
