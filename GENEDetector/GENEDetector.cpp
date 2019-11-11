#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	VideoCapture cap(0); //capture the video from webcam

	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	int iLowH = 20;
	int iHighH = 30;

	int iLowS = 100;
	int iHighS = 255;

	int iLowV = 100;
	int iHighV = 255;
	/*
	//Create trackbars in "Control" window
	createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &iHighH, 179);

	createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &iHighS, 255);

	createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
	createTrackbar("HighV", "Control", &iHighV, 255);
	*/
	int iLastX = -1;
	int iLastY = -1;

	//Capture a temporary image from the camera
	Mat imgTmp;
	cap.read(imgTmp);

	while (true)
	{
		Mat imgOriginal;

		bool bSuccess = cap.read(imgOriginal); // read a new frame from video

		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}

		Mat imgHSV;

		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

		Mat imgThresholded;

		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

		//morphological opening (removes small objects from the foreground)

		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (removes small holes from the foreground)
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//Calculate the moments of the thresholded image
		Moments oMoments = moments(imgThresholded);

		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;

		// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
		if (dArea > 10000)
		{
			//calculate the position of the ball
			int posX = dM10 / dArea;
			int posY = dM01 / dArea;
		}

		vector<vector<Point> > contours;
		findContours(imgThresholded, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

		vector<vector<Point> > contours_poly(contours.size());
		vector<Rect> boundRect(contours.size());
		vector<Point2f>centers(contours.size());
		vector<float>radius(contours.size());
		for (size_t i = 0; i < contours.size(); i++)
		{
			approxPolyDP(contours[i], contours_poly[i], 3, true);
			boundRect[i] = boundingRect(contours_poly[i]);
		}

		Mat drawing = Mat::zeros(imgThresholded.size(), CV_8UC3);

		for (size_t i = 0; i < contours.size(); i++)
		{
			Scalar color = Scalar(0, 255, 0, 10);
			rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
			if ((boundRect[i].height * boundRect[i].width) > 5000)
			{
				drawContours(drawing, contours_poly, (int)i, color);
				cout << boundRect[i].height << " is the x dimension" << endl;
				cout << boundRect[i].width << " is the y dimension" << endl;
			}
		}

		imshow("Drawing", drawing);
		waitKey(1);
	}

	return 0;
}