#include <stdio.h>
#include "Header.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#define PI 3.14159265

using namespace cv;

priv	pv_vars;
pub		pb_vars;

Mat frame,gray;
int main()
{
	int i, j, k, l, m, n;
	if (0) 
	{
		//Open the default video camera
		VideoCapture cap(0);
		Mat frame, gray, binar;
		// if not success, exit program
		if (cap.isOpened() == false)
		{
			cout << "Cannot open the video camera" << endl;
			cin.get(); //wait for any key press
			return -1;
		}

		string window_name = "My Camera Feed";
		namedWindow(window_name); //create a window called "My Camera Feed"
		string gray_img = "gray_img";
		namedWindow(gray_img);
		string binary_img = "binary";
		namedWindow(binary_img); //create a window called "My Camera Feed"
		
		std::vector <std::vector<cv::Point> > contours_WHITE; // Vector for storing contour
		std::vector<cv::Vec4i> hierarchy;
		cv::Mat threshold_output;
		std::vector<cv::Vec3f> v3fCircles;

		real_t x_mea, y_mea;
		while (true)
		{
	
			bool bSuccess = cap.read(frame); // read a new frame from video 
			cvtColor(frame, gray, CV_BGR2GRAY);
			binar = gray > 230;
			//morphological opening (remove small objects from the foreground)
			//erode(binar, binar, getStructuringElement(MORPH_ELLIPSE, Size(11,11)));
			//dilate(binar, binar, getStructuringElement(MORPH_ELLIPSE, Size(15, 15)));
			cv::GaussianBlur(binar, binar, Size(9, 9), 2, 2);			//Blur Effect

			//morphological closing (fill small holes in the foreground)
			//dilate(binar, binar, getStructuringElement(MORPH_ELLIPSE, Size(15, 15)));
			//erode(binar, binar, getStructuringElement(MORPH_ELLIPSE, Size(15, 15)));
			//Find and draw contours_WHITE
			

			cv::HoughCircles(binar, v3fCircles, CV_HOUGH_GRADIENT, 2, binar.rows / 4, 100, 50, 10, 800);  // algorithm for detecting circles		
			for (int i = 0; i < v3fCircles.size(); i++) 	// for each circle
			{					
				x_mea = (+v3fCircles[0][0] - 359)*0.15 / 250;
				y_mea = (-v3fCircles[0][1] + 245)*0.15 / 225;
				std::cout << "Ball position X = " << x_mea			// x position of center point of circle
					<< ",\tY = " << y_mea								// y position of center point of circle
					<< "\n";					// radius of circle

																					// draw small green circle at center of object detected
				cv::circle(frame,												// draw on original image
					cv::Point((int)v3fCircles[i][0], (int)v3fCircles[i][1]),		// center point of circle
					3,																// radius of circle in pixels
					cv::Scalar(0, 255, 0),											// draw green
					1);														// thickness

																					// draw red circle around object detected 
				cv::circle(frame,												// draw on original image
					cv::Point((int)v3fCircles[i][0], (int)v3fCircles[i][1]),		// center point of circle
					(int)v3fCircles[i][2],											// radius of circle in pixels
					cv::Scalar(0, 0, 255),											// draw red
					3);																// thickness
			}
			//cv::findContours(binar, contours_WHITE, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
			// Approximate contours_WHITE to polygons + get bounding rects and circles
			
			//show the frame in the created window
			imshow(window_name, frame);
			imshow(binary_img, binar);
			imshow(gray_img, gray);
			if (waitKey(10) == 27)
			{
				cout << "Esc key is pressed by user. Stoppig the video" << endl;
				break;
			}
		}
	}

	/* SOLVER SQP FULL STEP*/
	// INIT:
	if(0)
	{
		for (i = 0; i < 8; i++)
			pb_vars.x0[i] = 0;

		for (i = 0; i < 40; i++)
			pb_vars.u[i] = 0;

		for (i = 0; i < 8; i++) 
			pb_vars.x_des[i] = 0;

		for (i = 0; i < 2; i++)
			pb_vars.u0[i] = 0;


		next_20_xk(pb_vars.x0, pb_vars.u, pb_vars.x);
		cost_function_k_0_N();

		real_t A[64];
		real_t B[16];
		real_t AB[16];
		A_m(pb_vars.x0, pb_vars.u0, A);
		B_m(pb_vars.x0, pb_vars.u0, B);
		multi_AB(A, B, AB);
	}
	inverse_kinematics(10*PI/180, 0);
	return 0;
	

}
