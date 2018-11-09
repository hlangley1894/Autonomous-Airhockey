// OpenCVCam.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

// These three header files required by OpenCV 
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <windows.h>
#include <iomanip>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <stdio.h>
#include "time.h"
#include "iostream"
#include <math.h>
// Header for the PSEye
#include "CLEyeMulticam.h"
int resultingx;
int resultingy;


#define FRAME_RATE		60
#define FRAME_SIZE		CLEYE_VGA
#define FRAME_FORMAT	CLEYE_COLOR_RAW

//Code changes HL
int Slider_max = 255;
int H_min = 26;
int S_min = 40;
int V_min = 0;
int H_max = 85;
int S_max = 192;
int V_max = 223;
cv::string trackbarWindowName = "HSV Values";
int x_p = 0;
int y_p = 0;
int x_prev = 999;
int y_prev = 999;
int x_a = 0;
int y_a = 400;
int count = 0;

//DEclare these to be array with num rows entries
int xtraject[400];
int ytraject[400];

typedef cv::vector<cv::Point2f> Point2fVector;

#define MAX_OBJ			50
#define MIN_AREA		10
#define MAX_AREA		(640*480)/1.5

void MousCallback(int mEvent, int x, int y, int flags, void* param)
{
	Point2fVector* pPointVec = (Point2fVector*)param;
	if (mEvent == CV_EVENT_LBUTTONDOWN)
	{
		pPointVec->push_back(cv::Point2f(float(x), float(y)));
	}
}

int Trajectory(int x1, int y1, int x2, int y2, int xborderneg, int xborderpos)
{
	int i = 3; // dummy counting variable
	int moving = -1; // used to determine direction, 0 - left, 1 - right
					 // Vaiables used to mark where on the board the separate points are
	int left1, right1, left2, right2;
	int newy, newx; // the next position on the board
	int ydiff = abs(y2 - y1);
	int xdiff = 0;
	int flag = 1;

	left1 = right1 = left2 = right2 = 0;
	newy = y2;
	newx = x2;
	xtraject[0] = x1;
	ytraject[0] = y1;
	xtraject[1] = x2;
	ytraject[1] = y2;

	// Determine which half of the board the two frames are in

	if (x1 < 0)
	{
		left1 = 1;
	}
	else
	{
		right1 = 1;
	}

	if (x2 < 0)
	{
		left2 = 1;
	}
	else
	{
		right2 = 1;
	}

	// Calculate the change in x based of the starting two frames
	if (right1 && right2) // both on right side
		xdiff = x2 - x1;
	else if (right1 && left2) // frame 1 right, frame 2 left
		xdiff = x1 - x2;
	else if (left1 && right2) // frame 1 left, frame 2 right
		xdiff = x2 - x1;
	else if (left1 && left2) // both on left side
		xdiff = abs(abs(x2) - abs(x1));
	else { // this should never happen
		xdiff = 0;
		return 0;
	}
	// Determine intial direction stored in moving variable
	// 0 - left, 1 - right
	if (right1 && right2) // both on right
	{
		if (xdiff < 0)
		{
			xdiff = abs(xdiff);
			moving = 0;
		}
		else {
			moving = 1;
		}
	}
	else if (right1 & left2)// first on right, second on left
		moving = 0;
	else if (left1 && right2)// first on left, second on right
		moving = 1;
	else if (left1 && left2) // both on left side
	{
		if (x2 < x1)
		{
			moving = 0;
		}
		else
		{
			moving = 1;
		}

	}
	else { // this should never happen
		moving = -1;
		return 0;
	}
	int temp = 0;
	// Determine and create trajectory
	while (newy > 0)
	{
		flag = 1;
		// right border
		if (moving && flag) // to the right
		{
			newx = newx + xdiff;
			if (newx >= xborderpos)
			{
				temp = newx - xborderpos;
				newx = xborderpos - temp;
				moving = 0;
				flag = 0;
			}
		}
		else if (!moving && flag)// to the left
		{
			newx = newx - xdiff;
			if (newx <= xborderneg)
			{
				temp = newx + abs(xborderneg);
				newx = xborderneg + abs(temp);
				moving = 1;
				flag = 0;

			}
		}
		newy = newy - ydiff;

		//printf("%d\n",i);
		if (newy >= 0)
		{
			xtraject[i] = newx;
			ytraject[i] = newy;
			i++;
		}

	}
	return i - 1;
}

float motor1;
float motor2;
#define PI 3.14159265

void Angles(int x, int y)
{
	//ydist is theoretical
	double distance;
	double arm1length;
	double arm2length;
	double arm1referenceangle = 90.0;
	double thetad;
	double theta1, theta2;
	double conversion; // used to convert pixel number to distance in inches
	double ydist = 6.0;

	conversion = 53.0 / 400.0;
	distance = sqrt((pow((x*conversion), 2)) + (pow((y*conversion + ydist), 2)));
	arm1length = 18.0;
	arm2length = 13.5;
	//printf("abs(y) %d abs(x) %d\n",abs(y),abs(x));
	double temp;
	temp = ((double)(abs(y + ydist))) / ((double)(abs(x)));
	thetad = atan(temp) * 180 / PI;
	theta1 = acos((pow(arm1length, 2) - pow(arm2length, 2) + pow(distance, 2)) / (2 * arm1length*distance)) * 180 / PI;
	theta2 = acos((pow(arm1length, 2) + pow(arm2length, 2) - pow(distance, 2)) / (2 * arm1length*arm2length)) * 180 / PI;

	//printf("theta1 %lf, theta2 %lf thetad %lf\n", theta1,theta2,thetad);

	if (x >= 0)
	{
		motor1 = (thetad - theta1) /*- arm1referenceangle*/;
		motor2 = 180 - theta2;
	}
	else
	{
		motor1 = 180 - thetad - theta1 /*- arm1referenceangle*/;
		motor2 = 180 - theta2;
	}

}




/* use this function later*/

void Position(double motor1, double motor2, cv::Mat &feed)
{
	double arm1, arm2;
	double distance, theta, thetarelative, conversion = 0.0;
	arm1 = 18.0;
	arm2 = 13.5;
	double ydist = 6.0; //Theoretical right now


	distance = sqrt(pow(arm1, 2) + pow(arm2, 2) - (2 * arm1*arm2*cos(180.0 - motor2)));
	theta = acos((pow(distance, 2) + pow(arm1, 2) - pow(arm2, 2)) / (2 * distance*arm1)) *180 / PI ;
	thetarelative = theta + motor1;

	conversion = 400.0 / 53.0;
	resultingx = distance * cos(thetarelative)  * conversion;
	resultingy = (distance * sin(thetarelative)   * conversion) - (ydist*conversion);

	//cv::circle(feed, cv::Point(resultingx + x_a, y_a - resultingy), 10, cv::Scalar(0, 0, 255),CV_FILLED);

}

/*
hsvSilders()
Creates window with trackbars to adjust HSV values.
The HSV values for the green puck are hardcoded already, but if these values need to be changed then this function can be called.
*/
void hsvSliders()
{
	//create window for trackbar
	cv::namedWindow(trackbarWindowName, 0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf(TrackbarName, "H_MIN", H_min);
	sprintf(TrackbarName, "H_MAX", H_max);
	sprintf(TrackbarName, "S_MIN", S_min);
	sprintf(TrackbarName, "S_MAX", S_max);
	sprintf(TrackbarName, "V_MIN", V_min);
	sprintf(TrackbarName, "V_MAX", V_max);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
	cv::createTrackbar("H_MIN", trackbarWindowName, &H_min, Slider_max);
	cv::createTrackbar("H_MAX", trackbarWindowName, &H_max, Slider_max);
	cv::createTrackbar("S_MIN", trackbarWindowName, &S_min, Slider_max);
	cv::createTrackbar("S_MAX", trackbarWindowName, &S_max, Slider_max);
	cv::createTrackbar("V_MIN", trackbarWindowName, &V_min, Slider_max);
	cv::createTrackbar("V_MAX", trackbarWindowName, &V_max, Slider_max);


}

/*
trackObject(int &x, &y, cv::Mat green, cv::Mat &feed)
Creates target on puck and returns x,y pixel values.
*/

void trackObject(int &x, int &y, cv::Mat green, cv::Mat &feed)
{
	cv::Mat temp_green;
	cv::vector< cv::vector<cv::Point> > contours;
	cv::vector<cv::Vec4i> hierarchy;
	double refArea = 0;
	bool objFound = false;
	int numObj = 0;
	int i, j;
	int trajectIndex = 0;

	//if (green.empty());
	//return;

	//copies binary image to temp location for manipulation. 
	green.copyTo(temp_green);

	//outlines white puck in binary image using contours
	cv::findContours(temp_green, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	//if nothing is in image (puck off board) then there is nothing to track so we exit
	if (!contours.capacity())
		return;

	//printf("rows %d, cols %d\n", feed.rows, feed.cols);

	//find how many objects are in picture
	if (hierarchy.size() > 0)
	{
		numObj = hierarchy.size();
		//printf("number of objects: %d\n",numObj);
	}

	//only use tracking if less than the max number of objects is found. helps with noise
	if (numObj < MAX_OBJ)
	{
		//for each object...
		for (i = 0; i >= 0; i = hierarchy[i][0])
		{
			cv::Moments moment = cv::moments((cv::Mat)contours[i]);
			double area = moment.m00;
			//printf("area: %f\n", area);

			//...find area of that object and get coordinates for the center of that object.
			if (area > MIN_AREA && area < MAX_AREA && area > refArea)
			{
				x = moment.m10 / area;
				y = moment.m01 / area;

				x_p = x - x_a;
				y_p = y_a - y;

				if (x_prev <= 400 && y_prev <= 400 && y_prev > y_p)
				{
					// only recalculate the trajectory after 10 iterations to sample and average data

					trajectIndex = Trajectory(x_prev, y_prev, x_p, y_p, -(feed.cols / 2), feed.cols / 2);

					//motor1 = -2;
					//motor2 = -2;
					//Angles(5,5);
					for (j = 0; j < trajectIndex; j++)
					{
						if (ytraject[j] > 30 && ytraject[j] < 40)
						{
							Angles(xtraject[j], ytraject[j]);
							Position(motor1, motor2, feed);
							printf("motor1 %lf, motor2 %lf\n", motor1, motor2);
							//printf("ANGLESS!!!\n");
						}
					}



					//trajectIndex = Trajectory(10,360,12,350,-(feed.cols/2),feed.cols/2);
					//printf("index: %d\n", trajectIndex);
					if (trajectIndex > 0)
					{
						for (j = 0; j <= trajectIndex; j++)
						{
							//printf("%d %d\n", xtraject[j] + x_a, y_a - ytraject[j]);
							cv::circle(feed, cv::Point(xtraject[j] + x_a, y_a - ytraject[j]), 5, cv::Scalar(0, 0, 0));
						}
					}
					//printf("after traject\n");

				}

				x_prev = x_p;
				y_prev = y_p;



				//printf("%d,%d\n", x_prev,y_prev);
				objFound = true;
				refArea = area;
			}
			else
			{
				objFound = false;
			}

			//if object is found, create crosshares on target
			if (objFound == true)
			{
				//printf("Object Found!!\n");
				/*if ((y - 25) > 0)
				cv::line(feed, cv::point(x, y), cv::point(x, y - 25), cv::scalar(0, 255, 0), 2);
				else cv::line(feed, cv::point(x, y), cv::point(x, 0), cv::scalar(0, 255, 0), 2);
				if ((y + 25) < 480)
				cv::line(feed, cv::point(x, y), cv::point(x, y + 25), cv::scalar(0, 255, 0), 2);
				else cv::line(feed, cv::point(x, y), cv::point(x, 480), cv::scalar(0, 255, 0), 2);
				if ((x - 25) > 0)
				cv::line(feed, cv::point(x, y), cv::point(x - 25, y), cv::scalar(0, 255, 0), 2);
				else cv::line(feed, cv::point(x, y), cv::point(0, y), cv::scalar(0, 255, 0), 2);
				if ((x + 25) < 640)
				cv::line(feed, cv::point(x, y), cv::point(x + 25, y), cv::scalar(0, 255, 0), 2);
				else cv::line(feed, cv::point(x, y), cv::point(640, y), cv::scalar(0, 255, 0), 2);*/
				//cv::putText(feed, "Tracking...", cv::Point(0, 50), 2, 1, cv::Scalar(0, 255, 0), 2);
				cv::putText(feed, std::to_string(x_p) + "," + std::to_string(y_p), cv::Point(x, y + 30), 1, 1, cv::Scalar(0, 255, 0), 2);
				//cv::circle(feed, cv::Point(x, y), 20, cv::Scalar(0, 255, 0), 2);
				imshow("Unwarped", feed);
				//printf("After imshow\n");
			}
		}
		//printf("end tracking\n");
	}
}

typedef struct {
	CLEyeCameraInstance CameraInstance;
	PBYTE FramePointer;
}CAMERA_AND_FRAME;

static DWORD WINAPI CaptureThread(LPVOID ThreadPointer);

int _tmain(int argc, _TCHAR* argv[])
{

	///////MY VARS////////
	PBYTE FramePointer = NULL;
	int width, height, CameraCount, FramerCounter = 0;
	CLEyeCameraInstance EyeCamera = NULL;
	GUID CameraID;
	IplImage *frame;
	clock_t StartTime, EndTime;
	CAMERA_AND_FRAME ThreadPointer;
	HANDLE _hThread;
	//////////////////////

	//Check for presence of EYE
	CameraCount = CLEyeGetCameraCount();
	if (CameraCount>0) printf("Number of EYE's detected: %d\n\n", CameraCount);
	else {
		printf("No camera detected, press any key to exit...");
		getchar();
		return 0;
	}
	// Get ID of first PSEYE
	CameraID = CLEyeGetCameraUUID(0);
	// Get connection to camera and send it running parameters
	EyeCamera = CLEyeCreateCamera(CameraID, FRAME_FORMAT, FRAME_SIZE, FRAME_RATE);
	//Couldn't Connect to camera
	if (EyeCamera == NULL) {
		printf("Couldn't connect to camera, press any key to exit...");
		getchar();
		return 0;
	}
	// Set some camera parameters;
	CLEyeSetCameraParameter(EyeCamera, CLEYE_EXPOSURE, 511);
	CLEyeSetCameraParameter(EyeCamera, CLEYE_GAIN, 0);
	// Get camera frame dimensions;
	CLEyeCameraGetFrameDimensions(EyeCamera, width, height);
	// Create a window in which the captured images will be presented
	//cvNamedWindow( "Camera", CV_WINDOW_AUTOSIZE );
	//Make a image to hold the frames captured from the camera
	frame = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 4);
	// GetPointer To Image Data For frame
	cvGetImageRawData(frame, &FramePointer);

	//Start the eye camera
	CLEyeCameraStart(EyeCamera);

	//Need to copy vars into one var to launch the second thread
	ThreadPointer.CameraInstance = EyeCamera;
	ThreadPointer.FramePointer = FramePointer;
	//Launch thread and confirm its running
	_hThread = CreateThread(NULL, 0, &CaptureThread, &ThreadPointer, 0, 0);
	if (_hThread == NULL)
	{
		printf("failed to create thread...");
		getchar();
		return false;
	}

	//Code edit HL

	//display HSV trackbar window
	//hsvSliders();

	while (1) {
		//Display the captured frame
		//cvShowImage( "Camera", frame );
		//If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
		//remove higher bits using AND operator
		if ((cvWaitKey(1) & 255) == 27) break;
		//}

		//YOU DEE PEE


	}

	CLEyeCameraStop(EyeCamera);
	CLEyeDestroyCamera(EyeCamera);
	EyeCamera = NULL;
	cvDestroyWindow("Camera");

	return 0;
}

static DWORD WINAPI CaptureThread(LPVOID ThreadPointer) {
	CAMERA_AND_FRAME *Instance = (CAMERA_AND_FRAME*)ThreadPointer;
	CLEyeCameraInstance Camera = Instance->CameraInstance;
	PBYTE FramePtr = Instance->FramePointer;
	int FramerCounter = 0;
	clock_t StartTime, EndTime;

	//Code Change HL
	cv::Mat image, hsv, green; //Matrix to store converted images
	int x, y; //for pixel coordinates
	cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));		//looks for 3x3 ellipse every call
	cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
	cv::Mat unwarpImage; //will hold image after homography
	Sleep(30); //wait for a frame


			   //grab frame
	CLEyeCameraGetFrame(Camera, FramePtr);

	//create window for one frame. this is used for the user to click corners for homography
	image = cv::Mat(480, 640, CV_8UC4, FramePtr);
	imshow("image", image);
	MessageBoxA(NULL, "please click four corners of the air hockey table.\n"
		"click the left up corner first and clockwise for the rest.",
		"click", MB_OK);
	Point2fVector points;

	cv::setMouseCallback("image", MousCallback, &points);

	while (1)
	{
		// wait for mouse clicks
		cvWaitKey(10);
		//if(points.size()> 0) printf("clicks: %d\n",points.size());
		if (points.size() == 4)
			break;
	}


	//set up for homography
	double scale = 5.0;
	Point2fVector points2;
	Point2fVector points3;
	points2.push_back(cv::Point2f(0.0, 2000.0 / scale));
	points2.push_back(cv::Point2f(1200.0 / scale, 2000.0 / scale));
	points2.push_back(cv::Point2f(1200.0 / scale, 0.0));
	points2.push_back(cv::Point2f(0.0, 0.0));

	cv::Mat H = cvCreateMat(3, 3, CV_64F);

	MessageBoxA(NULL, "click the puck,please\n", "click", MB_OK);
	cvSetMouseCallback("image", MousCallback, &points3);
	while (1)
	{
		// wait for mouse clicks
		cvWaitKey(10);
		if (points3.size() == 1)
			break;
	}

	cv::Mat_<double> puckpixcoordh(3, 1);
	puckpixcoordh(0, 0) = points[0].x;
	puckpixcoordh(1, 0) = points[0].y;
	puckpixcoordh(2, 0) = 1.0;

	cv::Mat_<double> puckworldcoordh = cv::Mat(H)*puckpixcoordh;
	cv::setMouseCallback("image", NULL, NULL);
	cv::destroyWindow("image");



	//printf("%d,%d\n", H.rows, H.cols);
	//printf("%d\n",H.type());

	//find 3x3 homography matrix and create base for unwarpped image
	H = cv::findHomography((cv::Mat(points)), (cv::Mat(points2)));	//was moved from inside the while loop
	unwarpImage = cvCreateImage(cvSize(1200.0 / scale, 2000.0 / scale), IPL_DEPTH_8U, 3);	//IPL_DEPTH_8U
	int count = 0;
	int dumbcount = 0;
	while (1) {


		//Get Frame From Camera
		CLEyeCameraGetFrame(Camera, FramePtr);

		// put your vision code here

		//Code change 9/28 HL
		image = cv::Mat(480, 640, CV_8UC4, FramePtr);
		//printf("%d,%d,%d\n", points.size(),points2.size(),H.size());
		//printf("%d,%d,%d,\n", image.rows, image.cols,image.type());

		//perform homography
		cv::warpPerspective(image, unwarpImage, H, unwarpImage.size(),0);

		//marks center of x axis for velocity caluculation purposes.
		cv::line(unwarpImage, cv::Point(unwarpImage.cols / 2, 400), cv::Point(unwarpImage.cols / 2, 400 - 25), cv::Scalar(255, 0, 255), 2);

		count++;
		if (count == 5)
		{
			count = 0;
			//imshow("Unwarped", unwarpImage);
		}
		imshow("Unwarped", unwarpImage);
		x_a = unwarpImage.cols / 2;

		//convert to hsv
		cv::cvtColor(unwarpImage, hsv, CV_RGB2HSV);
		//imshow("HSV", hsv);

		//create binary image based on max/min hsv values
		cv::inRange(hsv, cv::Scalar(H_min, S_min, V_min), cv::Scalar(H_max, S_max, V_max), green);

		//use erode and dilate to remove noise and enlarge puck
		cv::erode(green, green, erodeElement);
		cv::dilate(green, green, dilateElement);
		imshow("Green", green);

		//call track object to follow puck

		//if (dumbcount % 3 == 0)
		//{
		trackObject(x, y, green, unwarpImage);
		//}
		dumbcount++;
		//RECOMMENT THIS
		//printf("(%d,%d)\n", x,y);

		//Sleep(1);
		cvWaitKey(1); //Needed for images to appear
					  // Track FPS
		if (FramerCounter == 0) StartTime = clock();
		FramerCounter++;
		EndTime = clock();
		if ((EndTime - StartTime) / CLOCKS_PER_SEC >= 1)
		{
			printf("FPS: %d\n", FramerCounter);
			FramerCounter = 0;
		}
	}
	return 0;
}