#include "LinesDetector.h"

LinesDetector::LinesDetector()
{
	this->flagInverted = 1; 
	//cv::namedWindow( "bars", CV_WINDOW_AUTOSIZE );
	this->thresh=80;
	this->thresh2=14;
	//this->thrlines = 43;
	this->thrlines = 266 ; // contours
	this->minLineLength = 867;
	this->maxLineGap = 33;
	// cv::createTrackbar( "thresh 1:", "bars", &thresh, 255 );
	// cv::createTrackbar( "thresh 2:", "bars", &thresh2, 255 );
	// cv::createTrackbar( "thrlines:", "bars", &thrlines, 5000 );
	// cv::createTrackbar( "minLineLength:", "bars", &minLineLength, 15000 );
	// cv::createTrackbar( "maxLineGap:", "bars", &maxLineGap, 500 );
}


void LinesDetector::imageSave(cv::Mat image)
{
	cv::Mat imageGray;
	cv::cvtColor(image, imageGray, CV_BGR2GRAY);
	//cv::imshow( "imageGray", imageGray ); 
	if (cv::waitKey(60)=='s')
	{
		cv::imshow( "Save??", imageGray ); 
		if (cv::waitKey()=='y'){
			std::string name;
			std::cout<<"Name image: ";
			std::cin>>name;
			cv::imwrite( name, imageGray );
		}
	}
}

cv::Mat LinesDetector::homography(cv::Mat image , cv::Mat matrixH)
{

    cv::Mat transform;
    cv::Mat resized;
    cv::warpPerspective(image,transform,matrixH,cv::Size(image.cols*2, image.rows*2.5));
    //cv::imshow("image",image);
    cv::resize(transform,resized,cv::Size(transform.cols/2, transform.rows/2)); 
    //cv::imshow("resized",resized);
    //cv::waitKey(1);
    return resized;
}


cv::Mat LinesDetector::segmentationLines(cv::Mat image, std::vector<geometry_msgs::PoseStamped> &center , 
														std::vector<geometry_msgs::PoseStamped> &lf ,
														std::vector<geometry_msgs::PoseStamped> &rg)
{	
	cv::RNG rng(10);
	
	cv::Mat imageRoi=cv::Mat(image, cv::Rect(0, 0,image.cols, image.rows-40));
	cv::Mat imageGray;
	cv::Mat imageEdges;

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::cvtColor(imageRoi, imageGray, CV_BGR2GRAY);
	//imshow("imageGray", imageGray);
	cv::threshold(imageGray, imageGray, 160, 255, CV_THRESH_BINARY);
	//imshow("imageThreshold", imageGray);

	cv::blur(imageGray,imageGray, cv::Size(7,7)); 
    cv::Canny(imageGray,imageEdges,this->thresh,this->thresh2,3);
	cv::Mat kernel = getStructuringElement(cv::MORPH_CROSS,cv::Size(5,5));
	cv::Mat imgDilate;
	cv::dilate(imageEdges,imgDilate,kernel,cv::Point(-1,-1),4);
	for (int i = 0; i < 5; ++i)
	{
		
		cv::erode(imgDilate,imgDilate,kernel,cv::Point(-1,-1),1);
		cv::dilate(imgDilate,imgDilate,kernel,cv::Point(-1,-1),1);
	}
	cv::erode(imgDilate,imgDilate,kernel,cv::Point(-1,-1),5);
	//imshow("imgDilate", imgDilate);


	cv::findContours( imgDilate, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
	cv::Mat drawing = cv::Mat::zeros( imgDilate.size(), CV_8UC3 );
	std::vector<cv::Rect> boundRect;
	std::vector<cv::Point> linesExt;
	float centerMean=0.0;
	int cntMean=0;
	for( int i = 0; i< contours.size(); i++ )
    {
    	double area = cv::contourArea( contours[i] , false );
    	if (  area > thrlines )
      	{
	     	cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      		//cv::drawContours( imageRoi, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
      		
      		if( area < minLineLength)
      		{
      	 		cv::RotatedRect r = cv::minAreaRect( contours[i] ) ;
      	 		//cv::circle( imageRoi, r.center , 5 ,  cv::Scalar(0,255,0), -1 );
      	 		geometry_msgs::PoseStamped pose;
      	 		pose.pose.position.x=r.center.x;
		 		pose.pose.position.y=r.center.y;
 				pose.pose.position.z=0;
 				center.push_back(pose);
      	 		centerMean+=r.center.x; 
      	 		++cntMean;
      	 	}
      	 	else
      	 	{
      	 		for( int j=0; j<contours[i].size(); j+=70){
					
      	 			linesExt.push_back(contours[i][j]);
      	 		}
      	 	}
      	} 
      
    }

    centerMean/=cntMean; 

    for (int i = 0; i < linesExt.size(); ++i)
    {
    	geometry_msgs::PoseStamped pose;
    	if (linesExt[i].x > centerMean)
    	{
    		pose.pose.position.x=linesExt[i].x;
		 	pose.pose.position.y=linesExt[i].y;
 			pose.pose.position.z=0;
 			rg.push_back(pose);
    		//cv::circle( imageRoi, linesExt[i] , 5 ,  cv::Scalar(0,0,255), -1 );
    	}else{
    		pose.pose.position.x=linesExt[i].x;
		 	pose.pose.position.y=linesExt[i].y;
 			pose.pose.position.z=0;
 			lf.push_back(pose);
    		//cv::circle( imageRoi, linesExt[i] , 5 ,  cv::Scalar(255,0,0), -1 );
    	}
    	
    }
	


/*	cv::HoughLinesP(imgDilate, lines, 1, CV_PI/180, thrlines, minLineLength, maxLineGap );
	cv::Mat drawing = cv::Mat::zeros( imgDilate.size(), CV_8UC3 );
	


	double distThr = 100 * 100;
	for( int i = 0; i< lines.size(); i++ )
    {
    	
	    cv::Scalar color = cv::Scalar( 0, 0, rng.uniform(0,255) );
      	
      	cv::Point p1 = cv::Point(lines[i][0],lines[i][1]);
      	cv::Point p2 = cv::Point(lines[i][2],lines[i][3]);

      	double dist = (p1.x - p2.x) *(p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y); 
      	if( dist < distThr )
      		color = cv::Scalar( 255, 0 , 0 );

      	cv::line( imageRoi, cv::Point(lines[i][0],lines[i][1]), cv::Point(lines[i][2],lines[i][3]),color,3,8);
      	
      
    }

*/

	
	//imshow("imageEdges", imageEdges);
	//imshow("drawing", imageRoi);
	


	//cv::waitKey(5);
	return imageGray;
}



// cv::Mat segmentationLines2(cv::Mat image, std::vector<geometry_msgs::PoseStamped> &poses)
// {	
// 	//std::cout<<"aaaaaaabbbbbbbbbbbbaaaaaa"<<std::endl;
// 	//cv::Mat imageRoi=cv::Mat(image, cv::Rect(0, 250,image.cols,229 ));
// 	cv::Mat imageGray;
// 	cv::Mat imageEdges;
// 	cv::cvtColor(image, imageGray, CV_BGR2GRAY);
// 	cv::blur(imageGray,imageGray, cv::Size(7,7)); 
// 	/*cv::Ptr<cv::CLAHE> clahe =cv::createCLAHE();
// 	clahe->setClipLimit(1);
// 	clahe->apply(imageGray,imageGray);*/

// 	//cv::imshow( "imageGray", imageGray ); 

	
// 	//cv::equalizeHist( imageGray, imageGray );
// 	cv::Canny(imageGray,imageEdges,100,200,7);
// 	//cv::threshold(imageGray, imageThreshold, 150, 255, CV_THRESH_OTSU ); ///dice jesus del pasado  CV_THRESH_BINARY);//
// 	//cv::adaptiveThreshold(imageGray,imageThreshold, 255 , CV_ADAPTIVE_THRESH_GAUSSIAN_C, this->flagInverted, 17,0);
// 	//cv::Mat images; 
// 	//cv::vconcat(imageGray, imageThreshold, images );

// 	//std::cout<<"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"<<std::endl;
// 	cv::imshow( "imageThreshold", imageEdges ); 

	


// /*



// 	cv::Mat kernel = getStructuringElement(cv::MORPH_CROSS,cv::Size(3,3));
// 	cv::Mat imageLines = cv::Mat::zeros(imageThreshold.rows,imageThreshold.cols,CV_8U);
// 	/*
// 	std::vector< cv::Vec4i > linePoints; 
// 	cv::HoughLinesP( imageThreshold, linePoints, 1, CV_PI/180, 35, 40, 4); 
// 	cv::Mat imageLines = cv::Mat::zeros(imageThreshold.rows,imageThreshold.cols,CV_8U);
// 	for (int i = 0; i <linePoints.size(); ++i)
// 	{
// 		cv::line(imageLines, cv::Point(linePoints[i][0], linePoints[i][1]),
//             cv::Point(linePoints[i][2], linePoints[i][3]), cv::Scalar(255,0,0), 4 , 8 );
// 	}
// 	//cv::dilate(imageLines,imageLines,kernel,cv::Point(-1,-1),10);
	
// 	for (int i = 0; i < 7; ++i)
// 	{
// 		cv::dilate(imageLines,imageLines,kernel,cv::Point(-1,-1) ,1);
// 		cv::erode(imageLines,imageLines,kernel,cv::Point(-1,-1) ,1 );
// 	}
	
	
// 	imageLines= 255- imageLines;
// 	for (int i = 0; i < 10; ++i)
// 	{
// 		cv::erode(imageLines,imageLines,kernel,cv::Point(-1,-1) ,1);
// 		cv::dilate(imageLines,imageLines,kernel,cv::Point(-1,-1) ,1);
// 	}
// */
// 	//	cv::erode(imageLines,imageLines,kernel,cv::Point(-1,-1) ,20);

// 	cv::erode(imageThreshold,imageLines,kernel,cv::Point(-1,-1) ,0);
// 	imageLines= 255- imageLines;
// 	std::vector<cv::Point> points;
// 	int x_ant;

// 	cv::imshow( "imageLines", imageLines ); 
    

	
// 	bool found = false; 
// 	int cnt = 0; 
// 	cv::Point2f avg; 
// 	for (int i = imageLines.rows; i >0 ; --i)
// 	{	
// 		for( int j = imageLines.cols -1 ; j > 0 ; j--)
// 		{
// 			uchar first = imageLines.at<uchar>(i,j+1); 
// 			uchar second = imageLines.at<uchar>(i,j); 

// 			int diff = (int)second - (int)first; 

// 			if( diff > 0)
// 			{
// 				avg = cv::Point2f(i,j); 
// 				cnt = 1;
// 			}

// 			if( diff == 0)
// 			{
// 				avg += cv::Point2f(j,i); 
// 				cnt++; 
// 			}

// 			if( diff < 0)
// 			{
// 				avg *= (1/(float)cnt); 
// 				found = true; 
// 			}


// 		}
// 	}


// 	//std::cout<<"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"<<std::endl;

// 	for (int i = imageLines.rows; i >0 ; --i)
// 	{	
// 		int start = -1;
// 		int end = -1;
// 		int j= imageLines.cols;
// 		bool pat=false;



// /*		while(imageLines.at<uchar>(i,j) == 255 && j<imageLines.cols)
// 			++j;
// 		while(imageLines.at<uchar>(i,j) == 0 && j<imageLines.cols)
// 		{
// 			++j;
// 			start=j;
// 		}
// 		if (start>320) continue;
// 		while(imageLines.at<uchar>(i,j) == 255 && j<imageLines.cols)
// 		{
// 			++j;
// 			end=j;
// 		}
// 		if (start != -1 && end != -1)// && start >  && end <540)
// 			points.push_back(cv::Point(start+(end-start)/2,i));*/
// 		/*while(imageLines.at<uchar>(i,j) == 0 && j>0){
// 			pat = true;
// 			--j;
// 		}
// 		if (!pat) continue;
// 		pat=false;*/
// 		int count=0;
// 		while(imageLines.at<uchar>(i,j) == 255 && j>0){
// 			pat= true;
// 			--j;
// 			++count;
// 		}
// 		if (!pat || count > 500 ) 
// 			continue;

// 		pat=false;
// 		while(imageLines.at<uchar>(i,j) == 0 && j>0)
// 		{
// 			pat=true;
// 			start=j;
// 			--j;
			
// 		}
// 		if (!pat) continue;
// 		pat=false;
// 		while(imageLines.at<uchar>(i,j) == 255 && j>0)
// 		{
// 			pat= true;
// 			end=j;
// 			--j;
			
// 		}

// 		if (start != -1 && pat){
// 			points.push_back(cv::Point(start,i)); 
// 			cv::circle(imageGray,cv::Point(start,i),5,cv::Scalar(0,0,255));
// 			//cv::circle(imageGray,cv::Point(start-(start-end)/2,i),5,cv::Scalar(180,0,255));
// 			cv::circle(imageGray,cv::Point(end,i),5,cv::Scalar(255,0,255));
// 		}
		
// 	}
// 	std::cout<<points.size()<<std::endl;

// 	for (int i = 1; i <points.size(); ++i)
// 	{
// 		geometry_msgs::PoseStamped pose;
// 		points[i].x=(abs(points[i].x - points[i-1].x) > 20 )?  320:points[i].x;
		
// 		pose.pose.position.x=points[i].x;
// 		pose.pose.position.y=points[i].y;
// 		pose.pose.position.z=0;
// 		//if (points[i].x!=320)
// 		//{
// 			//cv::circle(imageGray,points[i],3,cv::Scalar(0,0,255));
// 			poses.push_back(pose);
// 		//}
		
// 	}

// 	//cv::imshow("img",imageLines);
// 	cv::imshow("img2",imageGray);
// 	//cv::vconcat(images, imageLines, images );
// 	//cv::vconcat(images, imageGray, images);
// 	cv::waitKey(1);
// 	return imageGray;
// 	//return images;
// 	/*
	
// 	cv::erode(imageLines,imageLines,kernel,cv::Point(-1,-1),30);
// 	cv::dilate(imageLines,imageLines,kernel,cv::Point(-1,-1),29);
	

// 	cv::Mat mask = cv::Mat::ones(imageThreshold.rows,imageThreshold.cols,CV_8U)*255;
// 	cv::Mat img;
// 	cv::rectangle(mask, cv::Rect(0,0,imageLines.cols,imageLines.rows),cv::Scalar(0,0,0),10);
// 	imageLines.copyTo(img,mask);
// 	cv::Mat dist;
// 	cv::Mat dist8U;
//     cv::distanceTransform(img, dist, CV_DIST_L2, 3);
    
//     cv::threshold(dist, dist, 0.1, 1, cv::THRESH_BINARY );
//     dist.convertTo(dist8U,CV_8U);
    
    
// 	return dist8U;	
// 	*/


	
// }


