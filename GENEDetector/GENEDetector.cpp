#include <iostream>
#include <fstream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

// Acceptable Pixel Deviation #

const int ACCEPTABLE_DIFFERENCE = 30;

// Empirical Inch/Px of Camera

const int INCH_PER_PIXEL = 0.009074194088;

bool getDifference(int x0, int x1)
{
	if (fabs(x0 - x1) < ACCEPTABLE_DIFFERENCE)
		return true;
	else
		return false;
}

int main(int argc, char** argv)
{
	VideoCapture cap(1); //capture the video from webcam

	int iLowH = 20;
	int iHighH = 30;

	int iLowS = 100;
	int iHighS = 255;

	int iLowV = 100;
	int iHighV = 255;
	int iLastX = -1;
	int iLastY = -1;

	// Pixel positions relative to top-left of plane
	int posX = 0;
	int posY = 0;

	// Average width and height values
	int avgHeight = 0;
	int avgWidth = 0;

	// Dimensioned height and width values
	int HEIGHT = 0;
	int WIDTH = 0;

	// Counter for average readings
	int counter = 0;

	// Center pixel numbers for x and y
	int centerX = 320;
	int centerY = 230;

	// Output file opening in truncated form (no overlapping requests)
	ofstream centeredOut("centeredOut.txt", STRUNCATE);

	// Output file default zero for conveyor state
	centeredOut << "0" << endl;

	cout << "0";

	// Original image stream for debugging purposes
	Mat imgOriginal;

	// HSV-Filtered image stream
	Mat imgHSV;

	// Thresholded image stream (based off HSV sliders)
	Mat imgThresholded;

	while (true)
	{

		bool bSuccess = cap.read(imgOriginal); // read a new frame from video

		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			return EXIT_SUCCESS;
		}

		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

		//Threshold the image
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

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


		if (dArea > 10000)
		{
			//calculate the position of the object
			posX = dM10 / dArea;
			posY = dM01 / dArea;
		}

		vector<vector<Point> > contours;
		findContours(imgThresholded, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

		vector<vector<Point> > contours_poly(contours.size());
		vector<Rect> boundRect(contours.size());
		vector<Point2f>centers(contours.size());
		for (size_t i = 0; i < contours.size(); i++)
		{
			approxPolyDP(contours[i], contours_poly[i], 3, true);
			boundRect[i] = boundingRect(contours_poly[i]);
		}

		Mat drawing = Mat::zeros(imgThresholded.size(), CV_8UC3);

		if (contours.size() > 0)
		{
			for (size_t i = 0; i < contours.size(); i++)
			{
				Scalar color = Scalar(0, 255, 0, 10);

				// boundRect Area filter
				if ((boundRect[i].height * boundRect[i].width) > 5000)
				{
					rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
					system("CLS");
					cout << "X position is " << posX << endl;
					cout << "Y position is " << posY << endl;
					cout << "Average height is " << avgHeight << endl;
					cout << "Average width is " << avgWidth << endl;
					cout << "Width is " << WIDTH << endl;
					cout << "Height is " << HEIGHT << endl;
					
					// If large boundRect is centered
					if (getDifference(posY, centerY))
					{
						cout << "Is centered" << endl;
						if (counter < 25)
						{
							ofstream centeredOut("centeredOut.txt", STRUNCATE);
							centeredOut << "1";
							counter++;
							avgHeight += boundRect[i].height;
							avgWidth += boundRect[i].width;
						}
						else
						{
							cout << "Counter is " << counter << endl;
							HEIGHT = (avgHeight / 25) * INCH_PER_PIXEL;
							WIDTH = (avgWidth / 25) * INCH_PER_PIXEL;

							ofstream centeredOut("centeredOut.txt", STRUNCATE);
							// Formatted file output HEIGHT AND WIDTH FUNCTIONS GO HERE
							centeredOut << "-1" << endl
								<< HEIGHT << endl
								<< WIDTH << endl;
						}
					}
					else if ((!(getDifference(posY, centerY))) && (counter == 25))
					{
						system("CLS");
						cout << "Resetting" << endl;

						ofstream centeredOut("centeredOut.txt", STRUNCATE);
						centeredOut << "0";
						avgHeight = 0;
						avgWidth = 0;
						counter = 1;
						HEIGHT = 0;
						WIDTH = 0;
					}
				}
			}
		}
		imshow("Drawing", drawing);
		waitKey(1);
	}

	return 0;
}