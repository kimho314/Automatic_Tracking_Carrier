#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>
//#include <time.h>

#define DIST_TH 0.8
#define PI 3.14

using namespace cv;
using namespace std;

typedef struct _boxsize
{
	int height;
	int width;
}boxSize;

typedef struct _boxinfo
{
	Point2f centerOfROI;
	float angle;
	boxSize size;
	int dist_x;
	int dist_y;
	int prevArea;
	int curArea;
}boxInfo;

Point2f Center;
Mat image;	//input image

bool backprojMode = false;
bool selectObject = false;
int trackObject = 0;
bool showHist = true;
Point origin;
Rect selection;
//int vmin = 10, vmax = 256, smin = 30;

int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;

/*int H_MIN = 0;
int H_MAX = 180;
int S_MIN = 74;
int S_MAX = 256;
int V_MIN = 48;
int V_MAX = 141;*/

RotatedRect LuCamShift(InputArray _probImage, Rect& window, TermCriteria criteria)
{
	const int TOLERANCE = 10;
	Size size;
	Mat mat;
	UMat umat;
	bool isUMat = _probImage.isUMat();

	if (isUMat)
		umat = _probImage.getUMat(), size = umat.size();
	else
		mat = _probImage.getMat(), size = mat.size();

	meanShift(_probImage, window, criteria);

	window.x -= TOLERANCE;
	if (window.x < 0)
		window.x = 0;

	window.y -= TOLERANCE;
	if (window.y < 0)
		window.y = 0;

	window.width += 2 * TOLERANCE;
	if (window.x + window.width > size.width)
		window.width = size.width - window.x;

	window.height += 2 * TOLERANCE;
	if (window.y + window.height > size.height)
		window.height = size.height - window.y;

	// Calculating moments in new center mass
	Moments m = isUMat ? moments(umat(window)) : moments(mat(window));

	double m00 = m.m00, m10 = m.m10, m01 = m.m01;
	double mu11 = m.mu11, mu20 = m.mu20, mu02 = m.mu02;

	if (fabs(m00) < DBL_EPSILON)
		return RotatedRect();

	double inv_m00 = 1. / m00;
	int xc = cvRound(m10 * inv_m00 + window.x);
	int yc = cvRound(m01 * inv_m00 + window.y);
	double a = mu20 * inv_m00, b = mu11 * inv_m00, c = mu02 * inv_m00;

	// Calculating width & height
	double square = std::sqrt(4 * b * b + (a - c) * (a - c));

	// Calculating orientation
	double theta = atan2(2 * b, a - c + square);

	// Calculating width & length of figure
	double cs = cos(theta);
	double sn = sin(theta);

	double rotate_a = cs * cs * mu20 + 2 * cs * sn * mu11 + sn * sn * mu02;
	double rotate_c = sn * sn * mu20 - 2 * cs * sn * mu11 + cs * cs * mu02;
	double length = std::sqrt(rotate_a * inv_m00) * 4;
	double width = std::sqrt(rotate_c * inv_m00) * 4;

	// In case, when tetta is 0 or 1.57... the Length & Width may be exchanged
	if (length < width)
	{
		std::swap(length, width);
		std::swap(cs, sn);
		theta = PI*0.5 - theta;
	}

	// Saving results
	int _xc = cvRound(xc);
	int _yc = cvRound(yc);

	int t0 = cvRound(fabs(length * cs));
	int t1 = cvRound(fabs(width * sn));

	t0 = MAX(t0, t1) + 2;
	window.width = MIN(t0, (size.width - _xc) * 2);

	t0 = cvRound(fabs(length * sn));
	t1 = cvRound(fabs(width * cs));

	t0 = MAX(t0, t1) + 2;
	window.height = MIN(t0, (size.height - _yc) * 2);

	window.x = MAX(0, _xc - window.width / 2);
	window.y = MAX(0, _yc - window.height / 2);

	window.width = MIN(size.width - window.x, window.width);
	window.height = MIN(size.height - window.y, window.height);

	RotatedRect box;
	box.size.height = (float)length;
	box.size.width = (float)width;
	box.angle = (float)((PI*0.5 + theta)*180. / PI);
	while (box.angle < 0)
		box.angle += 360;
	while (box.angle >= 360)
		box.angle -= 360;
	if (box.angle >= 180)
		box.angle -= 180;
	box.center = Point2f(window.x + window.width*0.5f, window.y + window.height*0.5f);

	return box;
}

/* mouse event */
static void onMouse(int event, int x, int y, int, void*)
{
	if (selectObject)
	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);
		selection.height = std::abs(y - origin.y);

		selection &= Rect(0, 0, image.cols, image.rows);
	}

	switch (event)
	{
	case EVENT_LBUTTONDOWN:	//when the left button clicked
		origin = Point(x, y);
		selection = Rect(x, y, 0, 0);
		selectObject = true;
		break;
	case EVENT_LBUTTONUP:	//when the left button unclicked
		selectObject = false;
		if (selection.width > 0 && selection.height > 0)
			trackObject = -1;
		break;
	}
}

const char* keys =
{
	"{help h | | show help message}{@camera_number| 0 | camera number}"
};


int main(int argc, const char** argv)
{
	//clock_t start, end;

	/* wiring pi code */
	int fd;
	int send_f = 1;
	int send_l = 3;
	int send_r = 4;
	int send_dummy = 0;

	if((fd = serialOpen("/dev/ttyS0", 115200)) < 0){
		fprintf(stderr, "Unable to open Serial device : %s\n", strerror(errno));
		return 1;
	}

	if(wiringPiSetup() == -1){
		fprintf(stdout, "Unable to start wiringPi : %s\n", strerror(errno));
		return 1;
	}
	/* 		end 		*/
	VideoCapture cap;
	Rect trackWindow;
	int hsize = 16;
	int hist_sizes[] = { hsize, hsize, hsize };
	float hranges[] = { 0,180 };
	float vranges[] = { 0,255 };
	//const float* phranges = hranges;
	const float* phranges[] = { hranges, vranges, vranges };
	//const float* phranges[] = { hranges, vranges };
	int channels[] = { 0, 1, 2 };
	//int channels[] = { 0, 0 };
	CommandLineParser parser(argc, argv, keys);

	boxInfo box;
	box.prevArea = 0; box.curArea = 0;

	int camNum = parser.get<int>(0);
	cap.set(CAP_PROP_EXPOSURE, -10);
	cap.open(camNum);
	

	if (!cap.isOpened())
	{
		cout << "***Could not initialize capturing...***\n";
		cout << "Current parameter's value: \n";
		//parser.printMessage();
		return -1;
	}

	namedWindow("CamShift Demo", 0);
	setMouseCallback("CamShift Demo", onMouse, 0);

	createTrackbar("H_MIN", "CamShift Demo", &H_MIN, H_MAX, 0);
	createTrackbar("H_MAX", "CamShift Demo", &H_MAX, H_MAX, 0);
	createTrackbar("S_MIN", "CamShift Demo", &S_MIN, S_MAX, 0);
	createTrackbar("S_MAX", "CamShift Demo", &S_MAX, S_MAX, 0);
	createTrackbar("V_MIN", "CamShift Demo", &V_MIN, V_MAX, 0);
	createTrackbar("V_MAX", "CamShift Demo", &V_MAX, V_MAX, 0);


	Mat frame, hsv, hue, sat, val, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
	Mat hist_3D;
	bool paused = false;
	int count = 0;
	int flag = 0;

	/* Kalman Filter */
	Point2f ptPredicted;
	Point2f ptEstimated;
	Point2f ptMeasured;

	KalmanFilter KF(4, 2, 0);
	Mat measurement(2, 1, CV_32F);

	float dt = 1.0;

	const float A[] = { 1, 0, dt, 0,
						0, 1, 0, dt,
						0, 0, 1, 0,
						0, 0, 0, 1 };
	memcpy(KF.transitionMatrix.data, A, sizeof(A));

	//Initialize Kalman parameters
	double Q = 1e-5;
	double R = 1e-6;
	const float H[] = { 1, 0, 0, 0,
						0, 1, 0, 0 };
	memcpy(KF.measurementMatrix.data, H, sizeof(H));

	setIdentity(KF.processNoiseCov, Scalar::all(Q));
	KF.processNoiseCov.at<float>(2, 2) = 0;
	KF.processNoiseCov.at<float>(3, 3) = 0;

	setIdentity(KF.measurementNoiseCov, Scalar::all(R));

	Mat hist1, hist2; // for histogram matching

	//CAMShift Algorithm begins
	for (;;)
	{
		//start = clock();
		//capturing video frames
		if (!paused)
		{
			cap >> frame;
			Center.x = image.cols / 2;
			Center.y = image.rows / 2;
			if (frame.empty())
				break;
		}

		frame.copyTo(image);
		/*get the center of image frame*/
		//Center.x = image.cols / 2;
		//Center.y = image.rows / 2;

		//box.curArea = 0; 

		if (!paused)
		{
			cvtColor(image, hsv, COLOR_BGR2HSV); //change BGR to HSV

			if (trackObject)
			{
				//int _vmin = vmin, _vmax = vmax;

				//inRange(hsv, Scalar(0, smin, MIN(_vmin, _vmax)), Scalar(180, 256, MAX(_vmin, _vmax)), mask);
				inRange(hsv, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), mask);
				
				int ch[] = { 0,0, 1,1, 2,2 };
				//int ch[] = { 0,0 };
				hue.create(hsv.size(), hsv.depth());
				sat.create(hsv.size(), hsv.depth());
				val.create(hsv.size(), hsv.depth());
				
				Mat out[] = { hue, sat, val };
				//Mat out[] = { hue, sat };
				//mixChannels(&hsv, 1, &hue, 1, ch, 1);
				mixChannels(&hsv, 1, out, 3, ch, 3);
				//val = Mat::zeros(val.size(), val.depth());
				//sat = Mat::zeros(sat.size(), sat.depth());
				mixChannels(out, 3, &hsv, 1, ch, 3);
				

				if (trackObject < 0)
				{
					//Mat roi(hue, selection), maskroi(mask, selection);
					Mat roi(hsv, selection), maskroi(mask, selection);
					
					//calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges[0]); //calculates histogram
					calcHist(&roi, 1, channels, maskroi, hist_3D, 3, hist_sizes, phranges);
					hist_3D.copyTo(hist1);
					normalize(hist1, hist1, 1.0); // for matching
					//normalize(hist, hist, 0, 255, NORM_MINMAX); //normalizes histogram

					trackWindow = selection;
					trackObject = 1;

					//initialize the state vector(position and velocity)
					ptMeasured = Point2f(trackWindow.x + trackWindow.width / 2.0,
						trackWindow.y + trackWindow.height / 2.0);
					KF.statePost.at<float>(0, 0) = ptMeasured.x;
					KF.statePost.at<float>(1, 0) = ptMeasured.y;
					KF.statePost.at<float>(2, 0) = 0;
					KF.statePost.at<float>(3, 0) = 0;

					setIdentity(KF.errorCovPost, Scalar::all(1));

					/*histimg = Scalar::all(0);
					int binW = histimg.cols / hsize;
					Mat buf(1, hsize, CV_8UC3);
					for (int i = 0; i < hsize; i++)
						buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180. / hsize), 255, 255);
					cvtColor(buf, buf, COLOR_HSV2BGR); //change HSV to BGR

					for (int i = 0; i < hsize; i++)
					{
						int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows / 255);
						rectangle(histimg, Point(i*binW, histimg.rows),
							Point((i + 1)*binW, histimg.rows - val),
							Scalar(buf.at<Vec3b>(i)), -1, 8);
					}*/
				}

				Mat prediction = KF.predict(); //predict
				ptPredicted.x = prediction.at<float>(0, 0);
				ptPredicted.y = prediction.at<float>(1, 0);

				//calcBackProject(&hue, 1, 0, hist, backproj, &phranges[0]); //calculates the back projection of a histogram
				calcBackProject(&hsv, 1, channels, hist_3D, backproj, phranges);
				backproj &= mask;
				//RotatedRect trackBox;
				RotatedRect trackBox = LuCamShift(backproj, trackWindow,
					TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 100, 1));
				
				//box.centerOfROI = trackBox.center; //get the center of ROI
				//box.angle = trackBox.angle; //get the angle of ROI
				//box.size.height = trackBox.size.height; //get the height of ROI 
				//box.size.width = trackBox.size.width; //get the width of ROI
				//box.curArea = trackBox.size.height * trackBox.size.width; //calculates the area of ROI
				

				/*calculates the distance between center of the frame and center of ROI*/
				box.dist_x = (int)(Center.x - box.centerOfROI.x);
				box.dist_y = (int)(Center.y - box.centerOfROI.y);

				/*when failed to track the object*/
				if (trackWindow.area() <= 1)
				{
					int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5) / 6;
					trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
						trackWindow.x + r, trackWindow.y + r) &
						Rect(0, 0, cols, rows);
				}

				if (backprojMode)
					cvtColor(backproj, image, COLOR_GRAY2BGR);
				//ellipse(image, trackBox, Scalar(0, 0, 255), 3, LINE_AA);

				//Validate the result of CamShift
				Mat hsvImageROI(hsv, trackWindow), maskROI(mask, trackWindow);
				calcHist(&hsvImageROI, 1, channels, maskROI, hist2, 3, hist_sizes, phranges);
				normalize(hist2, hist2, 1.0);

				double dist = compareHist(hist1, hist2, HISTCMP_BHATTACHARYYA);
				//A tracking object is detected by CamShift
				if (dist < DIST_TH)
				{
					ptMeasured = Point2f(trackWindow.x + trackWindow.width / 2.0,
						trackWindow.y + trackWindow.height / 2.0);

					//measurements : the center point of the trackWindow
					measurement.at<float>(0, 0) = ptMeasured.x;
					measurement.at<float>(1, 0) = ptMeasured.y;

					Mat estimated = KF.correct(measurement); //update

					ptEstimated.x = estimated.at<float>(0, 0);
					ptEstimated.y = estimated.at<float>(1, 0);

					trackWindow = Rect(ptEstimated.x - selection.width / 2,
						ptEstimated.y - selection.height / 2,
						selection.width, selection.height);

					trackBox = LuCamShift(backproj, trackWindow,
						TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 100, 1));
					ellipse(image, trackBox, Scalar(0, 0, 255), 3, LINE_AA);
					
					box.centerOfROI = trackBox.center; //get the center of ROI
					box.angle = trackBox.angle; //get the angle of ROI
					box.size.height = trackBox.size.height; //get the height of ROI 
					box.size.width = trackBox.size.width; //get the width of ROI
					box.curArea = trackBox.size.height * trackBox.size.width; //calculates the area of ROI
				}
				//A tracking object is not detected by CamShift
				else
				{
					trackWindow = Rect(ptPredicted.x - selection.width / 2,
						ptPredicted.y - selection.height / 2,
						selection.width, selection.height);
					
					trackBox = LuCamShift(backproj, trackWindow,
						TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 100, 1));
					ellipse(image, trackBox, Scalar(255, 0, 0), 3, LINE_AA);

				}

				cout << "current area: " << box.curArea << " " << "prevArea: " << box.prevArea << " " <<
					"distance x: " << box.dist_x << endl;
				//cout << "selectObject : " << selectObject << "  selection.width : " << selection.width <<
					//"  selection.height : " << selection.height << endl;
				//cout << "trackBox.width : " << trackBox.size.width <<
					//"  trackBox.height : " << trackBox.size.height << endl;
			}
			box.prevArea = box.curArea;
			
		}
		else if (trackObject < 0)
			paused = false;

		if (selectObject && (selection.width > 0) && (selection.height > 0))
		{
			Mat roi(image, selection);
			bitwise_not(roi, roi);
		}

		circle(image, Point(Center.x, Center.y), 5, Scalar(255, 0, 0), 3, LINE_AA);
		circle(image, Point(box.centerOfROI.x, box.centerOfROI.y), 5, Scalar(255, 0, 255), 3, LINE_AA);

		imshow("CamShift Demo", image);
				
		char c = (char)waitKey(10);
		if (c == 27)
			break;
		switch (c)
		{
		case 'b':
			backprojMode = !backprojMode;
			break;
		case 'c':
			trackObject = 0;
			histimg = Scalar::all(0);
			break;
		case 'p':
			paused = !paused;
			break;
		default:
			;
		}

		/* wiring pi code */
		fflush(stdout);
		if((trackObject == 1) && (box.curArea >= 7000) && (flag == 0)){
			if(box.dist_x > 100){
				serialPutchar(fd, send_l);
			}
			else if(box.dist_x < -100){
				serialPutchar(fd, send_r);
			}
			else{
				serialPutchar(fd, send_dummy);
			}
		}
		else if((trackObject == 1) && (box.curArea < 7000) && (flag == 0)){
			if((box.dist_x <= 100) && (box.dist_x >= -100)){
				serialPutchar(fd, send_f);
			}
			else{
				flag = 1;
			}
		}

		if((trackObject == 1) && (box.dist_x > 100) && (flag == 1)){
			serialPutchar(fd, send_f);
			if(box.curArea >= 15000){
				flag = 0;
			}
		}
		else if((trackObject == 1) && (box.dist_x < -100) && (flag == 1)){
			serialPutchar(fd, send_f);
			if(box.curArea >= 15000){
				flag = 0;
			}
		}
		//end = clock();
		//cout << "time : " << (double)(end - start)/CLOCKS_PER_SEC << "\n" << endl;
	}

	return 0;
}
