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


LinesDetector::LinesDetector(bool debug, std::string path)
{
	this->debug=debug;
	this->path=path;
	this->prevPoint= cv::Point2f(-1.0,-1.0);
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

	dstPoints.push_back( cv::Point2f(0, 0)); 
	dstPoints.push_back( cv::Point2f(img.cols/2, 0)); 
	dstPoints.push_back( cv::Point2f(img.cols/2, img.rows)); 
	dstPoints.push_back( cv::Point2f(0, img.rows)); 


	for (t=0; t < v.size()-1; ++t)
	{
		line(temp,cv::Point(dstPoints[t].x,dstPoints[t].y), cv::Point(dstPoints[t+1].x,dstPoints[t+1].y),cv::Scalar(255,0,0),2);
	}
	line(temp,cv::Point(dstPoints[t].x,dstPoints[t].y), cv::Point(dstPoints[0].x,dstPoints[0].y),cv::Scalar(255,0,0),2);

	cv::Size transfSize = cv::Size(img.cols/2, img.rows); 
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

	cv::FileStorage fs(this->path, cv::FileStorage::WRITE);
	fs<<"Homography"<<tMatrix;
	fs<<"tSize"<<transfSize;
	fs.release();  
}

void LinesDetector::linesVector(std::vector<cv::Point2f> p,std_msgs::Float32MultiArray &l, int cols, cv::Mat &drawing ,cv::Scalar color)
{
	cv::Vec4f line;
	cv::fitLine(p,line, CV_DIST_WELSCH, 0, 0.01,0.01);
    int x0 = line[2];
    int y0 = line[3];
	int x1 = x0+100*line[0];
	int y1 = y0+100*line[1];
    int A= y1-y0;//y2-y1
    int B= x0-x1; //x1-x2
    int C= -A*x0-B*y0; //-A*x1-B*y1
    if (this->debug)
    {
    	cv::line(drawing,cv::Point(x0,y0),cv::Point(x1,y1),color,2);
    }    	
    l.data.push_back(A);
    l.data.push_back(B);
    l.data.push_back(C);
}

cv::Mat LinesDetector::segmentationLines(cv::Mat image, std_msgs::Float32MultiArray &lRight,std_msgs::Float32MultiArray &lLeft)
{	
	
	cv::Mat imageThreshold;
	

	//cv::inRange(image,cv::Scalar(172,172,172),cv::Scalar(255,255,255),imageThreshold);
	image=3*image-100; //contraste
	cv::inRange(image,cv::Scalar(0,0,0),cv::Scalar(200,200,200),imageThreshold);
	
	//cv::Mat imageGray;
	//cv::cvtColor(image,imageGray,CV_BGR2GRAY);
	//imageGray=255 - imageGray;	
	//cv::threshold(imageGray, imageThreshold, 240, 255, CV_THRESH_OTSU);

	std::cout<<"xxxxxxxxxxxxxxxxxxxxxxxx Vive?"  <<"[prevPoint]" << this->prevPoint <<std::endl;
	std::vector<cv::Point2f> peaks = LinesDetector::peakHistrogram(imageThreshold);
	std::cout<<"xxxxxxxxxxxxxxxxxxxxxxxx Si Vive"  <<"[prevPoint]" << this->prevPoint <<std::endl;

	for (int i = 1; i < 12; ++i)
	{
		line(imageThreshold,cv::Point(0,i*20 ), cv::Point(image.cols,i*20),cv::Scalar(0,0,0),2);
	}
	

	std::vector<std::vector<cv::Point> > contours;
	
	cv::findContours( imageThreshold, contours,  CV_RETR_EXTERNAL , CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
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
		//std::cout<<"derecha: "<<peaks[0]<<std::endl;
		//std::cout<<"mc antes: "<<mc.size()<<std::endl;

		right.push_back(peaks[0]);
		peaks.erase(peaks.begin());

		int cR=0;
		for (int i = 0; i < mc.size(); ++i)
		{
			if (distance(right[cR],mc[i])<60*60)
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
		//std::cout<<"izquierda: "<<peaks[0]<<std::endl;
		//std::cout<<"mc despues: "<<mc.size()<<std::endl;
		int cL=0;
		for (int i = 0; i < mc.size(); ++i)
		{
			if (distanceX(left[cL],mc[i])<30 && distanceX(right[0],mc[i])>60)
			{
				left.push_back(mc[i]);
				++cL;
			}
		}
	}

	if (right.size()>2)
	{
		LinesDetector::linesVector(right,lRight,imageThreshold.cols,drawing,cv::Scalar(200,200,0));

    	std::cout<<"[A]: ["<<lRight.data[0]<<" [B]: "<<lRight.data[1]<<" [C]: "<<lRight.data[2]<<std::endl;
	}

	if (left.size()>1)
	{
		LinesDetector::linesVector(left,lLeft,imageThreshold.cols,drawing,cv::Scalar(0,200,200));
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


	
	if(this->prevPoint.x < 0 && this->prevPoint.y<0){
		for (int i = 0; i < histo.size()-1; ++i)
		{
			//histo[i]/=255;
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
				if (((histo[i+1]==0 || i+2> histo.size()-1)) && MAX>5*255)
				{
					peaks.push_back(cv::Point2f(p,image.rows));
					this->prevPoint= cv::Point2f(p,image.rows);
				}
			}
		}
	}else{
		
		int th=20;
		int init = (this->prevPoint.x-th>0)?this->prevPoint.x-th:0;
		int endit = (this->prevPoint.x+th<histo.size()-1)?this->prevPoint.x+th:histo.size()-1;
		for (int i = init; i < endit; ++i)
		{
			//histo[i]/=255;
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
				if (((histo[i+1]==0 || i+2> endit)) && MAX>5)
				{
					peaks.push_back(cv::Point2f(p,image.rows));
					this->prevPoint= cv::Point2f(p,image.rows);
				}
			}
		}
		
	}
	
	if (peaks.size()>0)
	{
		std::sort(peaks.begin(),peaks.end(),compare);
		//this->prevPoint=cv::Point2f(peaks[0].x,peaks[0].y);
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
		for (int i = 0; i < histo.size()-1; ++i)
		{
		histo[i]/=255;
		}
		cv::Mat H= cv::Mat::zeros(image.size(),CV_8U);
		for (int i = 0; i < peaks.size(); ++i)
		{
			circle(H,peaks[i],15,cv::Scalar(255,255,255),-1);
		}
		for (int i = 0; i < histo.size()-1; ++i)
		{
			line(H,cv::Point(i,image.rows-histo[i]), cv::Point(i+1,image.rows-histo[i+1]),cv::Scalar(255,255,255),3);
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



