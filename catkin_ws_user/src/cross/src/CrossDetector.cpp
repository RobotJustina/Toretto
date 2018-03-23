#include "CrossDetector.h"
#define DEG2RAD M_PI/180
#define RAD2DEG 180/M_PI

CrossDetector::CrossDetector( int hough_thr,double minLen, double gapLen, int lowValThr,int highValThr, int canny_thr_low, int canny_thr_high)
{
								this->hough_thr=hough_thr;
								this->minLen=minLen;
								this->gapLen=gapLen;
								this->lowValThr=lowValThr;
								this->highValThr=highValThr;
								this->canny_thr_low=canny_thr_low;
								this->canny_thr_high=canny_thr_high;

}

void CrossDetector::get_borders(cv::Mat &image, cv::Mat &edges, bool color=false)
{
								cv::Mat imageGray,hsv;
								cv::Mat imageThreshold;

								if (color)
								{
																cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
																cv::Mat chan_hsv[3];
																cv::split(hsv,chan_hsv);
																imageGray = chan_hsv[1].clone();
								}
								else
								{
																cv::cvtColor(image, imageGray, CV_BGR2GRAY);
								}
								//cv::equalizeHist( imageGray, imageGray );
								cv::blur(imageGray,imageGray, cv::Size(5,5));

								//130,255 as trheshols
								cv::inRange(imageGray, lowValThr, highValThr, imageThreshold); ///dice jesus del pasado

								//cv::Mat imageCross = cv::Mat::zeros(imageThreshold.rows,imageThreshold.cols,CV_8U);
								// cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS,cv::Size(4,4));
								// cv::erode(imageThreshold,imageThreshold, element,cv::Point(-1,-1), 14);
								cv::Canny( imageThreshold, edges,canny_thr_low,canny_thr_high);
}

void CrossDetector::segmentationCross(cv::Mat edges, cv::Mat &imageCross,  bool &cross)
{
								//std::cout<<"aaaaaaabbbbbbbbbbbbaaaaaa"<<std::endl;
								int cols=edges.cols;
								int rows=edges.rows;
								cv::Point roi_corner(0,rows/2); //lower half of image
								cv::Size roi_size(cols-1,rows/2-1);
								cv::Rect roi(roi_corner,roi_size);
								cv::Mat imageRoi(edges,roi);

								std::vector< cv::Vec4i > linePoints;
								cv::HoughLinesP( edges, linePoints, 8, CV_PI/180, hough_thr, minLen, gapLen);

								cross=false;
								int hor_cnt=0;
								for (int i = 0; i <linePoints.size(); i++)
								{

																cv::Vec4i l= linePoints[i];
																float m = atan2(l[1]-l[3],l[0]-l[2]);

																//std::cout<<m*RAD2DEG<<std::endl;
																m=fabs(m);
																if ((m < 5.14*DEG2RAD) || (m> 170 *DEG2RAD))
																{
																								cv::line(imageCross, cv::Point(l[0], l[1]),cv::Point(l[2], l[3]), cv::Scalar(0,51,255),3);
																								cross=true;
																								hor_cnt++;
																}
																else
																{
																								linePoints.erase(linePoints.begin()+i);
																}


								}


								std::cout<<"Found "<< hor_cnt<<" horizontal lines"<<std::endl;
								//cv::imshow("img",imageCross);
								//cv::imshow("img2",imageGray);
								//cv::imshow("img3",imageThreshold);

								return;


}
