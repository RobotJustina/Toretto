#include "LinesDetector.h"

bool compare(cv::Point2f p1,cv::Point2f p2){ return (p1.x > p2.x);}
bool compareY(cv::Point2f p1,cv::Point2f p2){ return (p1.y > p2.y);}
std::vector<cv::Point2f> v;

void CallBackFunc(int event, int x, int y, int flags,void *vec)
{
	
	if  ( event == cv::EVENT_LBUTTONDOWN )
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
        v.push_back(cv::Point2f(x,y));
    }

}


LinesDetector::LinesDetector(bool debug)
{
	this->debug=debug;
	
}


void LinesDetector::transformMatrix(cv::Mat img)
{

	
	cv::Mat temp(img);
	cv::imshow("Select Points", temp);
	cv::setMouseCallback("Select Points", CallBackFunc, NULL);
	cv::waitKey(0);

	if (v[0].y<v[1].y)
	{
		v[1].y=v[0].y;
	}else{
		v[0].y=v[1].y;
	}

	if (v[2].y>v[3].y)
	{
		v[3].y=v[2].y;
	}else{
		v[2].y=v[3].y;
	}

	for (int i = 0; i < v.size(); ++i)
	{
		
		std::cout<<v[i]<<std::endl;
	}
	int t=0;
	for (; t < v.size()-1; ++t)
	{
		line(temp,cv::Point(v[t].x,v[t].y), cv::Point(v[t+1].x,v[t+1].y),cv::Scalar(0,255,0),2);
	}
	line(temp,cv::Point(v[t].x,v[t].y), cv::Point(v[0].x,v[0].y),cv::Scalar(0,255,0),2);

	std::vector< cv::Point2f> dstPoints; 
	//dstPoints.push_back( cv::Point2f(v[3].x, v[0].y)); 
	//dstPoints.push_back( cv::Point2f(v[2].x, v[1].y)); 
	//dstPoints.push_back( cv::Point2f(v[2].x, v[2].y)); 
	//dstPoints.push_back( cv::Point2f(v[3].x, v[3].y)); 

	dstPoints.push_back( cv::Point2f(0, 0)); 
	dstPoints.push_back( cv::Point2f(639, 0)); 
	dstPoints.push_back( cv::Point2f(639, 700)); 
	dstPoints.push_back( cv::Point2f(0, 700)); 


	for (t=0; t < v.size()-1; ++t)
	{
		line(temp,cv::Point(dstPoints[t].x,dstPoints[t].y), cv::Point(dstPoints[t+1].x,dstPoints[t+1].y),cv::Scalar(255,0,0),2);
	}
	line(temp,cv::Point(dstPoints[t].x,dstPoints[t].y), cv::Point(dstPoints[0].x,dstPoints[0].y),cv::Scalar(255,0,0),2);

	cv::Size transfSize = cv::Size(640, 700); 
	cv::Mat tMatrix = getPerspectiveTransform(v, dstPoints );
  	cv::Mat transformed;
  	cv::warpPerspective(temp, transformed, tMatrix, transfSize, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(127, 127, 127) );

  	cv::imshow("Transformed", transformed);
	cv::imshow("Select Points", temp);
	
	int key;
	while(key != 32)
	{
		key=cv::waitKey(1);
	}

	cv::FileStorage fs("Matrix.yaml", cv::FileStorage::WRITE);
	fs<<"Homography"<<tMatrix;
	fs<<"tSize"<<transfSize;
	fs.release();  


}

cv::Mat LinesDetector::segmentationLines(cv::Mat image, std_msgs::Float32MultiArray &lRight,std_msgs::Float32MultiArray &lLeft)
{	
	
	cv::Mat imageThreshold;
	cv::inRange(image,cv::Scalar(172,172,172),cv::Scalar(255,255,255),imageThreshold);
	std::vector<cv::Point2f> peaks = LinesDetector::peakHistrogram(imageThreshold);

	for (int i = 1; i < 14; ++i)
	{
		line(imageThreshold,cv::Point(0,i*50 ), cv::Point(640,i*50),cv::Scalar(0,0,0),2);
	}
	
	std::vector<std::vector<cv::Point> > contours;
	
	cv::findContours( imageThreshold, contours,  CV_RETR_EXTERNAL , CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
	cv::Mat drawing = cv::Mat::zeros( imageThreshold.size(), CV_8UC3 );
  	

	std::vector<cv::Moments> mu;
	for( int i = 0; i < contours.size(); i++ ){
		mu.push_back(  cv::moments( contours[i], false )); 
	}
	std::vector<cv::Point2f> mc;
	for( int i = 0; i < contours.size(); i++ ){
		cv::Point2f p( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
		if (p.y<10)
		{
			continue;
		}
		mc.push_back(p ); 
	}


	std::vector<cv::Point2f> left, right;

	if (peaks.size()>0)
	{
		std::cout<<"derecha: "<<peaks[0]<<std::endl;
		std::cout<<"mc antes: "<<mc.size()<<std::endl;

		right.push_back(peaks[0]);
		peaks.erase(peaks.begin());

		int cR=0;
		for (int i = 0; i < mc.size(); ++i)
		{
			if (distance(right[cR],mc[i])<110*110)
			{
				right.push_back(mc[i]);
				mc.erase(mc.begin()+i);

				++cR;
			}
		}
	}

	if (peaks.size()>0)
	{

		left.push_back(peaks[0]);
		std::cout<<"izquierda: "<<peaks[0]<<std::endl;
		std::cout<<"mc despues: "<<mc.size()<<std::endl;
		int cL=0;
		for (int i = 0; i < mc.size(); ++i)
		{
			if (distanceX(left[cL],mc[i])<60 && distanceX(right[0],mc[i])>60)
			{
				left.push_back(mc[i]);
				++cL;
			}
		}
	}

	if (right.size()>0)
	{
		cv::Vec4f lineR;
		cv::fitLine(right,lineR, CV_DIST_WELSCH, 0, 0.01,0.01);
		float lefty = (-lineR[2]*lineR[1]/lineR[0])+lineR[3];
    	float righty = ((imageThreshold.cols-lineR[2])*lineR[1]/lineR[0])+lineR[3];
    	float m =( righty - lefty)/(imageThreshold.cols-1);
    	float a = lefty;
    	if (this->debug)
    	{
    		cv::line(drawing,cv::Point(imageThreshold.cols-1,righty),cv::Point(0,lefty),cv::Scalar(200,200,0),2);
    	}    	
    	lRight.data.push_back(m);
    	lRight.data.push_back(a);
	}

	if (left.size()>0)
	{
		cv::Vec4f lineL;
		cv::fitLine(left,lineL, CV_DIST_L2, 0, 0.01,0.01);
		float lefty = (-lineL[2]*lineL[1]/lineL[0])+lineL[3];
    	float righty = ((imageThreshold.cols-lineL[2])*lineL[1]/lineL[0])+lineL[3];
    	float m =( righty - lefty)/(imageThreshold.cols-1);
    	float a = lefty;
    	if (this->debug)
    	{
    		cv::line(drawing,cv::Point(imageThreshold.cols-1,righty),cv::Point(0,lefty),cv::Scalar(0,200,200),2);
    	}    	
    	lLeft.data.push_back(m);
    	lLeft.data.push_back(a);
	}


	if (this->debug)
	{
		for( int i = 0; i< contours.size(); i++ )
    	{
    		cv::drawContours( drawing, contours, i, cv::Scalar( 0, 255, 0 ), 2);
    	}
    	for (int i = 0; i < right.size(); ++i)
    	{
   			circle( drawing, right[i], 4, cv::Scalar( 0, 0, 255 ), -1, 8, 0 );//red
    	}
    	for (int i = 0; i < left.size(); ++i)
    	{
   			circle( drawing, left[i], 4, cv::Scalar( 255, 0, 0 ), -1, 8, 0 );//blue
    	}
    	cv::imshow("transformed",image);
    	cv::imshow("thres",imageThreshold);
   		cv::imshow( "Draw", drawing );
	}

	return imageThreshold;

}



std::vector<cv::Point2f> LinesDetector::peakHistrogram(cv::Mat image)
{
 	std::vector<int> histo;
	cv::reduce(image, histo,0,CV_REDUCE_SUM);
	int MAX=0;
	std::vector<cv::Point2f> peaks;
	int p=0;
	bool peak=false;
	for (int i = 0; i < histo.size()-1; ++i)
	{
		histo[i]/=255;

		if (histo[i]==0)
		{
			peak=false;
			p=0;
			MAX=0;
		}
		else{
			if (histo[i]>MAX)
			{
				MAX = histo[i];
				p=i;
			}
			if ((histo[i+1]==0 || i+2> histo.size()-1) && MAX>10)
			{
				peaks.push_back(cv::Point2f(p,700));
			}
		}
	}

	 if (peaks.size()>0)
	 {
	 	std::sort(peaks.begin(),peaks.end(),compare);
	 }
	

	if (peaks.size()>2)
	{
		for (int i = 0; i < peaks.size()-1; ++i)
		{
			if (abs(peaks[i].x-peaks[i+1].x)<150)
			{
				float mean = (peaks[i].x+peaks[i+1].x)/2;
				peaks[i].x = mean;
				peaks.erase(peaks.begin()+i+1);
			}
		}
	}
	if (this->debug)
	{
		cv::Mat H= cv::Mat::zeros(image.size(),CV_8U);
		for (int i = 0; i < peaks.size(); ++i)
		{
			circle(H,peaks[i],20,cv::Scalar(255,255,255),-1);
		}
		for (int i = 0; i < histo.size()-1; ++i)
		{
			line(H,cv::Point(i,480-histo[i]), cv::Point(i+1,480-histo[i+1]),cv::Scalar(255,255,255),3);
		}
		cv::imshow("points",H);
	}
  	return peaks;
}

float LinesDetector::distance(cv::Point2f point1,cv::Point2f point2)
{
	return ((point1.x - point2.x)*(point1.x - point2.x))+((point1.y - point2.y)*(point1.y - point2.y));
}

float LinesDetector::distanceX(cv::Point2f point1,cv::Point2f point2)
{
	return abs(point1.x - point2.x);
}


