/*
 * AutomodelLineDetector.cpp
 *
 *  Created on: Mar 16, 2018
 *      Author: dougbel
 */

#include "automodel_line_detector/AutomodelLineDetector.h"


namespace automodel {


	cv::Mat detectedEdges;

	AutomodelLineDetector::AutomodelLineDetector(ros::NodeHandle& nodeHandle_, cv::Mat transfMatrix, cv::Size transfSize) :
			nodeHandle(nodeHandle_) {

		  readDefaultParameters();
			this->transfMatrix = transfMatrix;
			this->transfSize = transfSize;

		 // subsribe topic
		  ros::Subscriber sub = nodeHandle.subscribe(image_topic, 1000,
					&AutomodelLineDetector::detect, this);

		  if(debug)
			  createGUI();


		  pubLeft = nodeHandle.advertise<std_msgs::Float32MultiArray>("leftLine", 1);
		  pubRight = nodeHandle.advertise<std_msgs::Float32MultiArray>("rightLine", 1);

		  ros::spin();

	}

	AutomodelLineDetector::~AutomodelLineDetector() {
		cv::destroyWindow(IN_NAMED_WINDOW);
		cv::destroyWindow(OUT_NAMED_WINDOW);
		ROS_INFO_STREAM("Destroying Automodel Line Detector");
	}

	void AutomodelLineDetector::detect(
		const sensor_msgs::ImageConstPtr& msg) {


		cv::Mat image = cv_bridge::toCvShare(msg, "mono8")->image;
		//cv::warpPerspective(image, image, transfMatrix, transfSize, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(127, 127, 127) );


		///////////choosing pixels////

		//color
		//lower_yellow = np.array([20, 100, 100], dtype = “uint8”)
		//upper_yellow = np.array([30, 255, 255], dtype=”uint8")
		//mask_yellow = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
		//mask_yw = cv2.bitwise_or(mask_white, mask_yellow)
		//mask_yw_image = cv2.bitwise_and(gray_image, mask_yw)

		//gray scale
		Mat mask_white;
		inRange(image, canny_lowThreshold, canny_highThreshold, mask_white);
		bitwise_and(image,mask_white, mask_yw_image);

		//erasing the horizon
		Rect region_of_interest = Rect(0, 0, image.cols, .01*canny_perBlindHorizon*image.rows);
		Mat image_roi = mask_yw_image(region_of_interest);
		image_roi= cv::Mat::zeros(image_roi.size(), image_roi.type());


		//cv::warpPerspective(mask_yw_image, mask_yw_image, transfMatrix, transfSize, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(127, 127, 127) );
		//cv::imshow("alñsdkjfaslñkfjsald", mask_yw_image);
		cv::Canny(mask_yw_image,mask_yw_image,canny_lowThreshold,canny_highThreshold);


        rho = hough_int_rho;
        theta = hough_int_theta*3.1416/180;

		HoughLines( mask_yw_image, linesLeft,rho, theta, hough_threshold, 0, 0, 0, 1.0472 );  //max 60 grad
		HoughLines( mask_yw_image, linesRight,rho, theta, hough_threshold, 0, 0, 2.0944, CV_PI);  //min 120 grad

		if(debug)
			visualize( msg);

		publishLines();

	}

	void AutomodelLineDetector::publishLines(){

		if(linesLeft.size()>0) {
			float rhot = linesLeft[0][0];
			float thetat = linesLeft[0][1];
			std_msgs::Float32MultiArray left;

			left.layout.dim.push_back(std_msgs::MultiArrayDimension());
			left.layout.dim[0].size = 3;
			left.layout.dim[0].stride = 1;
			left.layout.dim[0].label = "left";
			left.data.push_back( cos(thetat) );
			left.data.push_back( sin(thetat) );
			left.data.push_back( rhot );

			pubLeft.publish(left);
		}

		if(linesRight.size()>0) {
			float rhot = linesRight[0][0];
			float thetat = linesRight[0][1];
			std_msgs::Float32MultiArray right;

			right.layout.dim.push_back(std_msgs::MultiArrayDimension());
			right.layout.dim[0].size = 3;
			right.layout.dim[0].stride = 1;
			right.layout.dim[0].label = "right";
			right.data.push_back( cos(thetat) );
			right.data.push_back( sin(thetat) );
			right.data.push_back( rhot );

			pubRight.publish(right);
		}



	}

	void AutomodelLineDetector::createGUI() {
		cv::namedWindow(IN_NAMED_WINDOW);
		cv::namedWindow(OUT_NAMED_WINDOW);

		createTrackbar( "Threshold low", IN_NAMED_WINDOW, &canny_lowThreshold, 255);
		createTrackbar( "Threshold high", IN_NAMED_WINDOW, &canny_highThreshold, 255);
		createTrackbar( "Perc horizon", IN_NAMED_WINDOW, &canny_perBlindHorizon, 100);


		createTrackbar( "Hough ro", IN_NAMED_WINDOW, &hough_int_rho, 50);
		createTrackbar( "Hough theta", IN_NAMED_WINDOW, &hough_int_theta, 360);
		createTrackbar( "Hough threshold", IN_NAMED_WINDOW, &hough_threshold, 300);

	}

	void AutomodelLineDetector::readDefaultParameters() {

		image_topic = "/app/camera/rgb/image_raw";
		canny_lowThreshold = 172;
		canny_highThreshold = 179;
		canny_perBlindHorizon = 46;

		hough_int_rho = 1;
		hough_int_theta =1;
		hough_threshold = 45;

		debug = false;

		if (!nodeHandle.getParam("/automodel_line_detector/image_topic", image_topic)) {
			ROS_ERROR("Could not find image_topic parameter!");
			//ros::requestShutdown();
		}
		if (!nodeHandle.getParam("/automodel_line_detector/canny_lowThreshold", canny_lowThreshold)) {
			ROS_ERROR("Could not find canny_lowThreshold parameter!");
			//ros::requestShutdown();
		}
		if (!nodeHandle.getParam("/automodel_line_detector/canny_highThreshold", canny_highThreshold)) {
			ROS_ERROR("Could not find canny_highThreshold parameter!");
			//ros::requestShutdown();
		}
		if (!nodeHandle.getParam("/automodel_line_detector/canny_perBlindHorizon", canny_perBlindHorizon)) {
			ROS_ERROR("Could not find canny_perBlindHorizon parameter!");
			//ros::requestShutdown();
		}
		if (!nodeHandle.getParam("/automodel_line_detector/hough_int_rho", hough_int_rho)) {
			ROS_ERROR("Could not find hough_int_rho parameter!");
			//ros::requestShutdown();
		}
		if (!nodeHandle.getParam("/automodel_line_detector/hough_int_theta", hough_int_theta)) {
				ROS_ERROR("Could not find hough_int_theta parameter!");
				//ros::requestShutdown();
			}
		if (!nodeHandle.getParam("/automodel_line_detector/hough_threshold", hough_threshold)) {
			ROS_ERROR("Could not find hough_threshold parameter!");
			//ros::requestShutdown();
		}

		if (!nodeHandle.getParam("/automodel_line_detector/debug", debug)) {
			ROS_ERROR("Could not find debug parameter!");
			//ros::requestShutdown();
		}

		ROS_INFO_STREAM("Image topic: "<< image_topic);
		ROS_INFO_STREAM("Canny lowThreshold: "<< canny_lowThreshold);
		ROS_INFO_STREAM("Canny highThreshold: "<< canny_highThreshold);
		ROS_INFO_STREAM("Percentage blind horizon: "<< canny_perBlindHorizon);
		ROS_INFO_STREAM("Hough rho (distance accumulador): "<< hough_int_rho);
		ROS_INFO_STREAM("Hough theta (angle accumulator): "<< hough_int_theta);
		ROS_INFO_STREAM("Hough accumulator threshold: "<< hough_threshold);
		ROS_INFO_STREAM("Debug: "<< debug);

	}

	void AutomodelLineDetector::visualize(const sensor_msgs::ImageConstPtr& msg) {

		imageColor = cv_bridge::toCvShare(msg, "bgr8")->image;
		cv::imshow(IN_NAMED_WINDOW,imageColor);

		if(linesRight.size()>0){

			float rhot = linesRight[0][0], thetat = linesRight[0][1];
			Point pt1, pt2;
			double a = cos(thetat), b = sin(thetat);
			double x0 = a*rhot, y0 = b*rhot;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a));
			line( imageColor, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
			// putText(imageColor, "RIGTH: rho "+to_string(rhot)+" angle: "+to_string(thetat), cvPoint(30,30),
				//FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,250), 1, CV_AA);
		}
		if(linesLeft.size()>0){
			float rhot = linesLeft[0][0];
			float thetat = linesLeft[0][1];
			Point pt1, pt2;
			double a = cos(thetat), b = sin(thetat);
			double x0 = a*rhot, y0 = b*rhot;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a));
			line( imageColor, pt1, pt2, Scalar(0,255,0), 3, CV_AA);
			// putText(imageColor, "LEFT: rho "+to_string(rhot)+" angle: "+to_string(thetat),cvPoint(30,80),
		//		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,250,0), 1, CV_AA);


		}

		cv::imshow(OUT2_NAMED_WINDOW,imageColor);
		cv::imshow(OUT_NAMED_WINDOW,mask_yw_image);

		if(cv::waitKey(15)=='s')
		{
			saveParameters();
		}
	}

	void AutomodelLineDetector::saveParameters() {


		FileStorage fs("./config.yaml", FileStorage::WRITE);

		fs << "/automodel_line_detector/debug" << debug;
		fs << "/automodel_line_detector/image_topic" << image_topic;
		fs << "/automodel_line_detector/canny_lowThreshold" << canny_lowThreshold;
		fs << "/automodel_line_detector/canny_highThreshold" << canny_highThreshold;
		fs << "/automodel_line_detector/canny_perBlindHorizon" << canny_perBlindHorizon;
		fs << "/automodel_line_detector/hough_int_rho" << hough_int_rho;
		fs << "/automodel_line_detector/hough_int_theta" << hough_int_theta;
		fs << "/automodel_line_detector/hough_threshold" << hough_threshold;

		ROS_INFO_STREAM("File Saved");

		fs.release();
	}

} /* namespace automodel */
