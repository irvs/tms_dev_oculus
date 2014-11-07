//------------------------------------------------------------------------------
// @file   : oculus_real_vision.cpp
// @brief  : allow to see to real world via Oculus using a camera
// @author : Alaoui Hassani Atlas Omar
// @version: Ver1.0.0 (since 2014.07.15)
// @date   : 2014.07.17
//------------------------------------------------------------------------------
#include "opencv/cv.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/legacy/compat.hpp"
//#include "opencv2/nonfree/nonfree.hpp"

int main(int argc, char **argv)
{
	CvCapture *capture = 0;
	IplImage *frame = 0;
	double w = 800 , h = 800;
	int c;

	//Find camera
	if (argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0]))){
		capture = cvCreateCameraCapture(argc == 2 ? argv[1][0] - '0' : 0);
	}

	//Set properties
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, w);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, h);
	cvSetCaptureProperty(capture, CV_CAP_PROP_CONVERT_RGB, true);

	//Set Windows for each eye, and the proper position in Oculus' display
	cvNamedWindow("Right Eye", CV_WINDOW_AUTOSIZE);
	cvMoveWindow("Right Eye", 750, 575);
	cvNamedWindow("Left Eye", CV_WINDOW_AUTOSIZE);
	cvMoveWindow("Left Eye", -80, 575);
	
	while (1) {
		if(frame = cvQueryFrame(capture)){
		//-----------------------------------------
		  /* Right eye */
			cvSetImageROI(frame, cvRect(150, 0, w-150, h));
			/* create destination image */
			IplImage *imgR = cvCreateImage(cvGetSize(frame), frame->depth, frame->nChannels);
			/* copy subimage */
			cvCopy(frame, imgR, NULL);
			/* always reset the Region of Interest */
			cvResetImageROI(frame);
			/* display current frame */
			cvShowImage( "Right Eye", imgR );
			/* destrot the image */
			cvReleaseImage(&imgR);

			/* Left eye */
			cvSetImageROI(frame, cvRect(0, 0, w-150, h));
			/* create destination image */
			IplImage *imgL = cvCreateImage(cvGetSize(frame), frame->depth, frame->nChannels);
			/* copy subimage */
			cvCopy(frame, imgL, NULL);
			/* always reset the Region of Interest */
			cvResetImageROI(frame);
			/* display current frame */
			cvShowImage( "Left Eye", imgL );
			/* destrot the image */
			cvReleaseImage(&imgL);
		//-----------------------------------------
		}

		c = cvWaitKey(2);
		if (c == '\x1b')
			break;
	}

	//Clear memory
	cvReleaseImage(&frame);
	cvReleaseCapture(&capture);
	cvDestroyAllWindows();

	return 0;
}

//------------------------------------------------------------------------------
//EOF