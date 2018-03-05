#include "LinesDetector.h"

LinesDetector::LinesDetector()
{
	this->flagInverted = 1; 
	
	this->thresh=80;
	this->thresh2=14;
	//this->thrlines = 43;
	this->thrlines = 1011 ; // contours
	//this->minLineLength = 2451;
	this->minLineLength = 5237 ;
	this->maxLineGap = 33;
	
	this->widthRecLines = 20; 
	this->heightRecLines = 200;

	//cv::namedWindow( "bars", CV_WINDOW_AUTOSIZE );
	//cv::createTrackbar( "thresh 1:", "bars", &thresh, 255 );
	//cv::createTrackbar( "thresh 2:", "bars", &thresh2, 255 );
	//cv::createTrackbar( "thrlines:", "bars", &thrlines, 5000 );
	//cv::createTrackbar( "minLineLength:", "bars", &minLineLength, 15000 );
	//cv::createTrackbar( "minH_rec:", "bars", &widthRecLines, 500 );
	//cv::createTrackbar( "minW_rec:", "bars", &heightRecLines, 500 );
	
	this->DebugMode = false; 

 	//cv::FileStorage fs("/home/edd/homoMat4.yaml", cv::FileStorage::READ);
 	cv::FileStorage fs("/root/catkin_ws_user/homoMat4.yaml", cv::FileStorage::READ);
 	fs ["Homography"] >> this->transfMatrix;
 	fs.release(); 
	cv::Mat divMat = cv::Mat::zeros(3,3, CV_64FC1); 
	divMat.at<double>(0,0) = (double)0.25; 
	divMat.at<double>(1,1) = (double)0.25; 
	divMat.at<double>(2,2) = (double)1.0; 
	this->transfMatrix = divMat *  this->transfMatrix; 

	this->kernel = getStructuringElement(cv::MORPH_CROSS,cv::Size(3,3));

	this->transfSize = cv::Size(323, 270);
}


void LinesDetector::imageSave(cv::Mat image)
{
	cv::Mat imageGray;
	cv::cvtColor(image, imageGray, CV_BGR2GRAY);
	cv::imshow( "imageGray", imageGray ); 
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

void LinesDetector::TrasformImage(cv::Mat &ima, cv::Mat &transformed)
{
	cv::Mat imageGray;
	cv::cvtColor(ima, imageGray, CV_BGR2GRAY);

	cv::warpPerspective(imageGray, transformed, this->transfMatrix, this->transfSize, cv::INTER_LINEAR, cv::BORDER_REPLICATE, cv::Scalar(127, 127, 127) );

	if(this->DebugMode)
		imshow("transformed", transformed);

}

void LinesDetector::GetLines(	cv::Mat &ima, 	
								std::vector<geometry_msgs::PoseStamped> &center, 
								std::vector<geometry_msgs::PoseStamped> &lf,
								std::vector<geometry_msgs::PoseStamped> &rg)
{
	cv::Mat imaOri; 
	cv::Mat imaFin; 
	if( DebugMode )
	{
		cvtColor( ima, imaOri, CV_GRAY2BGR);
		cvtColor( ima, imaFin, CV_GRAY2BGR);
	}

	cv::Mat imaThres; 
	

	cv::adaptiveThreshold( ima, imaThres, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 15, 2 );

	// Pal dia
	//cv::threshold(ima, imaThres, 160, 255, CV_THRESH_BINARY);
    
	// Pa la noche
    //cv::threshold(ima, imaThres, 115, 255, CV_THRESH_BINARY);
	if( DebugMode)
		imshow("imaThres", imaThres);

	cv::erode(imaThres, imaThres, kernel,cv::Point(-1,-1), 1);
	cv::dilate(imaThres, imaThres, kernel,cv::Point(-1,-1), 1);
	cv::erode(imaThres, imaThres, kernel,cv::Point(-1,-1), 1);
	cv::dilate(imaThres, imaThres, kernel,cv::Point(-1,-1), 1);
	cv::erode(imaThres, imaThres, kernel,cv::Point(-1,-1), 1);
	cv::dilate(imaThres, imaThres, kernel,cv::Point(-1,-1), 1);


	if( DebugMode)
		imshow("imaThres2", imaThres);

	// cv::Mat imaCanny;
	// cv::blur(imaThres,imaThres, cv::Size(7,7));
 //    cv::Canny(imaThres,imaCanny,this->thresh, this->thresh2, 3);
 //    if(DebugMode)
 //    	imshow("imaCanny", imaCanny);

	// cv::Mat imaMorph;
	// cv::dilate(imaCanny, imaMorph, kernel,cv::Point(-1,-1), 4);
	// morphologyEx(imaMorph, imaMorph, cv::MORPH_CLOSE,kernel, cv::Point(-1,-1),5);
	// cv::erode(imaMorph, imaMorph, kernel,cv::Point(-1,-1), 4);
	// if(DebugMode)
	// 	imshow("imaMorph", imaMorph);

	std::vector<std::vector<cv::Point> > contours;
	cv::findContours( imaThres, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
        
//        std::cout << "CONTOUR SIZE!!!!!!!!!!!!   " << contours.size() << std::endl;
        
	for (size_t i = 0; i < contours.size(); ++i)
	{
		if( DebugMode)
		{
			cv::Scalar color = cv::Scalar( rand()%255, rand()%255, rand()%255 );
			cv::drawContours( imaOri, contours, i, color, 3, 8, cv::Mat(), 0, cv::Point() );	
		}

		cv::RotatedRect rotRec = cv::minAreaRect( contours[i] );
		
		if( DebugMode )
		{	
			cv::circle( imaOri, rotRec.center, 5 ,  cv::Scalar(0,255,0), -1 );
			cv::Point2f vert[4]; 
			rotRec.points( vert );
			cv::line( imaOri , vert[0], vert[1], cv::Scalar(0,255,0), 2 );
			cv::line( imaOri , vert[1], vert[2], cv::Scalar(0,255,0), 2 );
			cv::line( imaOri , vert[2], vert[3], cv::Scalar(0,255,0), 2 );
			cv::line( imaOri , vert[3], vert[0], cv::Scalar(0,255,0), 2 );
		}

		if( 50 > rotRec.size.height )
			continue;

		if( 50 < rotRec.size.width )
			continue;

		if( rotRec.size.height > 80 && rotRec.size.width > 80)
		 	continue;
               // std::cout << "CONTOUR SIZE!!!!!!!!!!!!   " << contours.size() << std::endl;

		if( 50 < rotRec.size.height && rotRec.size.height < heightRecLines  
			&& 
			5 < rotRec.size.width && rotRec.size.width < widthRecLines )
		{
				cv::Point2f vert[4]; 
				rotRec.points( vert );

			geometry_msgs::PoseStamped pose;

    		pose.pose.position.x= vert[0].x;
		 	pose.pose.position.y= vert[0].y;
 			pose.pose.position.z=0;
 			center.push_back(pose);

    		pose.pose.position.x= vert[1].x;
		 	pose.pose.position.y= vert[1].y;
 			pose.pose.position.z=0;
 			center.push_back(pose);

    		pose.pose.position.x= rotRec.center.x;
		 	pose.pose.position.y= rotRec.center.y;
 			pose.pose.position.z=0;
 			center.push_back(pose);

                       // std::cout << "Nao puxe" << std::endl;

			if( DebugMode )
			{	
				cv::circle( imaFin, rotRec.center, 5 ,  cv::Scalar(0,255,0), -1 );
				cv::circle( imaFin , vert[0], 5 , cv::Scalar(0,255,0), -1 );
				cv::circle( imaFin , vert[2], 5 , cv::Scalar(0,255,0), -1 );
			}

			continue; 
		}
		
		for (size_t j = 0; j < contours[i].size()	; j+= 30)
		{
			geometry_msgs::PoseStamped pose;
    		pose.pose.position.x= contours[i][j].x;
		 	pose.pose.position.y= contours[i][j].y;
 			pose.pose.position.z=0;
 			rg.push_back(pose);

			cv::circle( imaFin, contours[i][j] , 5 ,  cv::Scalar(0,0,255), -1 );
		}
	}

	if( DebugMode)
	{
		cv::imshow("imaOri", imaOri);
		cv::imshow("imaFin", imaFin);
	}
}

cv::Mat LinesDetector::homography(cv::Mat image , cv::Mat matrixH)
{	
	//long int t; 
	//t = cv::getTickCount(); 

	cv::Mat imageGray;
	cv::cvtColor(image, imageGray, CV_BGR2GRAY);
	
	//std::cout << "cvtColor time: " << (cv::getTickCount() - t) / cv::getTickFrequency()<< std::endl; 
    
	//t = cv::getTickCount(); 
    cv::Mat transform;
    cv::Mat resized;

    cv::warpPerspective(imageGray,transform,matrixH,cv::Size(image.cols, image.rows*1.25));
    //cv::imshow("image",transform);
    //cv::resize(transform,resized,cv::Size(transform.cols/2, transform.rows/2)); 
    //cv::imshow("resized",resized);
    //cv::waitKey(1);
    //std::cout << "homography time: " << (cv::getTickCount() - t) / cv::getTickFrequency()<< std::endl; 
    

    return transform;
}


cv::Mat LinesDetector::segmentationLines(cv::Mat image,cv::Mat kernel,std::vector<geometry_msgs::PoseStamped> &center , 
																	  std::vector<geometry_msgs::PoseStamped> &lf ,
																	  std::vector<geometry_msgs::PoseStamped> &rg)
{	
	//int t = cv::getTickCount(); 

	
	//cv::Mat imageRoi=cv::Mat(image, cv::Rect(0, 0,image.cols, image.rows-40));
	cv::Mat imageGray = cv::Mat(image, cv::Rect(0, 0,image.cols, image.rows-40));
	cv::Mat imageEdges;

	std::vector<std::vector<cv::Point> > contours;
	//std::vector<cv::Vec4i> hierarchy;
	//cv::cvtColor(imageRoi, imageGray, CV_BGR2GRAY);
	
	//imageGray = imageRoi; 

	imshow("imageO", imageGray);
	cv::threshold(imageGray, imageGray, 160, 255, CV_THRESH_BINARY);
	

	cv::blur(imageGray,imageEdges, cv::Size(7,7)); 
    cv::Canny(imageGray,imageEdges,this->thresh,this->thresh2,3);

    //std::cout << "Canny time: " << (cv::getTickCount() - t) / cv::getTickFrequency()<< std::endl; 

    //t = cv::getTickCount();
	
	cv::Mat imgDilate;
	cv::dilate(imageEdges,imgDilate,kernel,cv::Point(-1,-1),4);
	morphologyEx(imgDilate,imgDilate,cv::MORPH_CLOSE,kernel,cv::Point(-1,-1),5);
	cv::erode(imgDilate,imgDilate,kernel,cv::Point(-1,-1),5);
	/*for (int i = 0; i < 5; ++i)
	{
		
		cv::erode(imgDilate,imgDilate,kernel,cv::Point(-1,-1),1);
		cv::dilate(imgDilate,imgDilate,kernel,cv::Point(-1,-1),1);
	}*/
	
	//imshow("imgDilate", imgDilate);

	// std::cout << "Morph time: " << (cv::getTickCount() - t) / cv::getTickFrequency()<< std::endl; 


    //t = cv::getTickCount();
	cv::findContours( imgDilate, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
	//cv::Mat drawing = cv::Mat::zeros( imgDilate.size(), CV_8UC3 );

	//std::cout << "findContours time: " << (cv::getTickCount() - t) / cv::getTickFrequency()<< std::endl; 

    //t = cv::getTickCount();

	std::vector<cv::Rect> boundRect;
	std::vector<cv::Point> linesExt;
	float centerMean=0.0;
	int cntMean=0;
	for( int i = 0; i< contours.size(); i++ )
    {
    	double area = cv::contourArea( contours[i] , false );
    	if (  area > thrlines )
      	{
	     	//cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      		//cv::drawContours( imageRoi, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
      		
      		if( area < minLineLength)
      		{
      	 		cv::RotatedRect r = cv::minAreaRect( contours[i] ) ;
      	 		cv::circle( imageGray, r.center , 5 ,  cv::Scalar(200,255,0), -1 );
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
      	 		for( int j=0; j<contours[i].size(); j+=35){
					
      	 			linesExt.push_back(contours[i][j]);
      	 		}
      	 	}
      	} 
      
    }

    centerMean=(center.size()<1)?0:centerMean/cntMean; 

	//std::cout << "Clasification C time: " << (cv::getTickCount() - t) / cv::getTickFrequency()<< std::endl; 
    //t = cv::getTickCount();

    for (int i = 0; i < linesExt.size(); ++i)
    {
    	geometry_msgs::PoseStamped pose;
    	if (linesExt[i].x < centerMean)
    	{
    		pose.pose.position.x=linesExt[i].x;
		 	pose.pose.position.y=linesExt[i].y;
 			pose.pose.position.z=0;
 			lf.push_back(pose);
    		cv::circle( imageGray, linesExt[i] , 5 ,  cv::Scalar(0,0,255), -1 );
    	}else{
    		pose.pose.position.x=linesExt[i].x;
		 	pose.pose.position.y=linesExt[i].y;
 			pose.pose.position.z=0;
 			rg.push_back(pose);
    		cv::circle( imageGray, linesExt[i] , 5 ,  cv::Scalar(255,0,0), -1 );
    	}
    	
    }
	
	//std::cout << "Clasification LR time: " << (cv::getTickCount() - t) / cv::getTickFrequency()<< std::endl; 

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
	imshow("imageGray", imageGray);
	


	cv::waitKey(5);
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


