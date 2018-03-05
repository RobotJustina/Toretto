#include "LinesDetector.h"

LinesDetector::LinesDetector()
{
	this->flagInverted = 1; 
	
}

cv::Mat LinesDetector::segmentationLines(cv::Mat image, std::vector<geometry_msgs::PoseStamped> &poses)
{	
	//std::cout<<"aaaaaaabbbbbbbbbbbbaaaaaa"<<std::endl;
	cv::Mat imageRoi=cv::Mat(image, cv::Rect(0, 330,image.cols,149 ));
	cv::Mat imageGray;
	cv::Mat imageThreshold;
	cv::cvtColor(imageRoi, imageGray, CV_BGR2GRAY);
	cv::blur(imageGray,imageGray, cv::Size(5,11)); 
	cv::Ptr<cv::CLAHE> clahe =cv::createCLAHE();
	clahe->setClipLimit(4);
	clahe->apply(imageGray,imageGray);
	//imageGray/=2;
	
	//cv::equalizeHist( imageGray, imageGray );
	
	cv::threshold(imageGray, imageThreshold, 200, 255, CV_THRESH_BINARY);//CV_THRESH_OTSU ); ///dice jesus del pasado 
	//cv::adaptiveThreshold(imageGray,imageThreshold, 255 , CV_ADAPTIVE_THRESH_GAUSSIAN_C, this->flagInverted, 15,0);
	//cv::Mat images; 
	//cv::vconcat(imageGray, imageThreshold, images );

	//std::cout<<"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"<<std::endl;
	
	cv::Mat kernel = getStructuringElement(cv::MORPH_CROSS,cv::Size(3,3));
	cv::Mat imageLines = cv::Mat::zeros(imageThreshold.rows,imageThreshold.cols,CV_8U);
	/*
	std::vector< cv::Vec4i > linePoints; 
	cv::HoughLinesP( imageThreshold, linePoints, 1, CV_PI/180, 35, 40, 4); 
	cv::Mat imageLines = cv::Mat::zeros(imageThreshold.rows,imageThreshold.cols,CV_8U);
	for (int i = 0; i <linePoints.size(); ++i)
	{
		cv::line(imageLines, cv::Point(linePoints[i][0], linePoints[i][1]),
            cv::Point(linePoints[i][2], linePoints[i][3]), cv::Scalar(255,0,0), 4 , 8 );
	}
	//cv::dilate(imageLines,imageLines,kernel,cv::Point(-1,-1),10);
	
	for (int i = 0; i < 7; ++i)
	{
		cv::dilate(imageLines,imageLines,kernel,cv::Point(-1,-1) ,1);
		cv::erode(imageLines,imageLines,kernel,cv::Point(-1,-1) ,1 );
	}
	
	
	imageLines= 255- imageLines;
	for (int i = 0; i < 10; ++i)
	{
		cv::erode(imageLines,imageLines,kernel,cv::Point(-1,-1) ,1);
		cv::dilate(imageLines,imageLines,kernel,cv::Point(-1,-1) ,1);
	}
*/
	//	cv::erode(imageLines,imageLines,kernel,cv::Point(-1,-1) ,20);

	cv::erode(imageThreshold,imageLines,kernel,cv::Point(-1,-1) ,0);
	//cv::imshow("img",imageLines);
	imageLines= 255- imageLines;
	std::vector<cv::Point> points;
	int x_ant;
	/*
	bool found = false; 
	int cnt = 0; 
	cv::Point2f avg; 
	for (int i = imageLines.rows; i >0 ; --i)
	{	
		for( int j = imageLines.cols -1 ; j > 0 ; j--)
		{
			uchar first = imageLines.at<uchar>(i,j+1); 
			uchar second = imageLines.at<uchar>(i,j); 

			int diff = (int)second - (int)first; 

			if( diff > 0)
			{
				avg = cv::Point2f(i,j); 
				cnt = 1;
			}

			if( diff == 0)
			{
				avg += cv::Point2f(j,i); 
				cnt++; 
			}

			if( diff < 0)
			{
				avg *= (1/(float)cnt); 
				found = true; 
			}


		}
	}
*/

	//std::cout<<"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"<<std::endl;

	for (int i = imageLines.rows; i >0 ; --i)
	{	
		int start = -1;
		int end = -1;
		int j= imageLines.cols;
		bool pat=false;



/*		while(imageLines.at<uchar>(i,j) == 255 && j<imageLines.cols)
			++j;
		while(imageLines.at<uchar>(i,j) == 0 && j<imageLines.cols)
		{
			++j;
			start=j;
		}
		if (start>320) continue;
		while(imageLines.at<uchar>(i,j) == 255 && j<imageLines.cols)
		{
			++j;
			end=j;
		}
		if (start != -1 && end != -1)// && start >  && end <540)
			points.push_back(cv::Point(start+(end-start)/2,i));*/
		/*while(imageLines.at<uchar>(i,j) == 0 && j>0){
			pat = true;
			--j;
		}
		if (!pat) continue;
		pat=false;*/
		int count=0;
		while(imageLines.at<uchar>(i,j) == 255 && j>0){
			pat= true;
			--j;
			++count;
		}
		if (!pat || count > 500 ) 
			continue;

		pat=false;
		while(imageLines.at<uchar>(i,j) == 0 && j>0)
		{
			pat=true;
			start=j;
			--j;
			
		}
		if (!pat) continue;
		pat=false;
		while(imageLines.at<uchar>(i,j) == 255 && j>0)
		{
			pat= true;
			end=j;
			--j;
			
		}

		if (start != -1 && pat){
			points.push_back(cv::Point(start,i)); 
			cv::circle(imageGray,cv::Point(start,i),5,cv::Scalar(0,0,255));
			//cv::circle(imageGray,cv::Point(start-(start-end)/2,i),5,cv::Scalar(180,0,255));
			cv::circle(imageGray,cv::Point(end,i),5,cv::Scalar(255,0,255));
		}
		
	}

	std::cout<<points.size()<<std::endl;

	for (int i = 1; i <points.size(); ++i)
	{
		geometry_msgs::PoseStamped pose;
		points[i].x=(abs(points[i].x - points[i-1].x) > 20 )?  320:points[i].x;
		
		pose.pose.position.x=points[i].x;
		pose.pose.position.y=points[i].y;
		pose.pose.position.z=0;
		//if (points[i].x!=320)
		//{
			//cv::circle(imageGray,points[i],3,cv::Scalar(0,0,255));
			poses.push_back(pose);
		//}
		
	}

	//
	
	//
	//cv::imshow("img2",imageGray);
	//cv::vconcat(images, imageLines, images );
	//cv::vconcat(images, imageGray, images);
	return imageGray;
	//return images;
	/*
	
	cv::erode(imageLines,imageLines,kernel,cv::Point(-1,-1),30);
	cv::dilate(imageLines,imageLines,kernel,cv::Point(-1,-1),29);
	

	cv::Mat mask = cv::Mat::ones(imageThreshold.rows,imageThreshold.cols,CV_8U)*255;
	cv::Mat img;
	cv::rectangle(mask, cv::Rect(0,0,imageLines.cols,imageLines.rows),cv::Scalar(0,0,0),10);
	imageLines.copyTo(img,mask);
	cv::Mat dist;
	cv::Mat dist8U;
    cv::distanceTransform(img, dist, CV_DIST_L2, 3);
    
    cv::threshold(dist, dist, 0.1, 1, cv::THRESH_BINARY );
    dist.convertTo(dist8U,CV_8U);
    
    
	return dist8U;	
	*/
}


