#include "CrossDetector.h"

CrossDetector::CrossDetector()
{
	this->flagInverted = 1; 
	
}

cv::Mat CrossDetector::segmentationCross(cv::Mat image,  bool &cross)
{	
	//std::cout<<"aaaaaaabbbbbbbbbbbbaaaaaa"<<std::endl;
	cv::Mat imageRoi=cv::Mat(image, cv::Rect(0, 360,image.cols,119 ));
	cv::Mat imageGray;
	cv::Mat imageThreshold;
	

	cv::cvtColor(imageRoi, imageGray, CV_BGR2GRAY);
	//cv::equalizeHist( imageGray, imageGray );
	cv::blur(imageGray,imageGray, cv::Size(5,11)); 

	/*cv::Ptr<cv::CLAHE> clahe =cv::createCLAHE();
	clahe->setClipLimit(2);
	clahe->apply(imageGray,imageGray);*/

	cv::threshold(imageGray, imageThreshold, 130, 255, CV_THRESH_BINARY ); ///dice jesus del pasado 




	//cv::Mat imageCross = cv::Mat::zeros(imageThreshold.rows,imageThreshold.cols,CV_8U);
	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS,cv::Size(4,4));
	cv::erode(imageThreshold,imageThreshold, element,cv::Point(-1,-1), 14);
	std::vector< cv::Vec4i > linePoints; 
	cv::HoughLinesP( imageThreshold, linePoints, 1, CV_PI/180, 150, 400,20); 
	cv::Mat imageCross = cv::Mat::zeros(imageThreshold.rows,imageThreshold.cols,CV_8U);
	int c=0;
	for (int i = 0; i <linePoints.size(); ++i)
	{

		cv::Vec4i l= linePoints[i];
		double m = (double)(l[1]-l[3])/(double)(l[0]-l[2]);
		
		if (m < 0.09 && m > -0.09 )
		{
			++c;
			cv::line(imageCross, cv::Point(l[0], l[1]),cv::Point(l[2], l[3]), cv::Scalar(255,0,0));
			cross=true;
		}

		 
	}
	std::cout<<c<<std::endl;
	//cv::imshow("img",imageCross);
	//cv::imshow("img2",imageGray);
	//cv::imshow("img3",imageThreshold);

	return imageCross;


}


