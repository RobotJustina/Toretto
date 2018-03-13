#define BOXES 12
#define MAX_VAL_IN_PIXEL 255
#include "lane_detection.h"

lane_extractor::lane_extractor( int hough_thr,double minLen, double gapLen, int lowValThr,int highValThr, int canny_thr_low, int canny_thr_high)
{
        this->hough_thr=hough_thr;
        this->minLen=minLen;
        this->gapLen=gapLen;
        this->lowValThr=lowValThr;
        this->highValThr=highValThr;
        this->canny_thr_low=canny_thr_low;
        this->canny_thr_high=canny_thr_high;
}

cv::Mat lane_extractor::extract_lane_angle_hough(cv::Mat image)
{
        int cols=image.cols;
        int rows=image.rows;
        cv::Point roi_corner(cols/2,rows*2/3);
        cv::Size roi_size(cols/2-1,rows/3-1);
        cv::Rect roi(roi_corner,roi_size);

        cv::Mat cropped_img(image,roi);

        cv::Mat hsv;
        cv::cvtColor(cropped_img, hsv, cv::COLOR_BGR2HSV);
        cv::blur(hsv,hsv,cv::Size(11,11));
        cv::Mat chan_hsv[3],binarized;
        cv::split(hsv,chan_hsv);
        cv::inRange(chan_hsv[2], lowValThr,highValThr, binarized);
        cv::Mat border;
        cv::Canny( binarized, border,50,50*2);
        //cv::cvtColor(cropped_img,gray_img,cv::COLOR_BGR2GRAY );
        //cv::blur(gray_img,gray_img,cv::Size(11,11));

        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(border, lines,1,CV_PI/180, hough_thr, minLen, gapLen);
        for (int i=0; i<lines.size(); i++)
        {
                std::cout<<"Number of lines: "<< lines.size()<<std::endl;
                cv::Point ini(lines[i][0],lines[i][1]);
                cv::Point fin(lines[i][2],lines[i][3]);
                cv::line(image,ini+roi_corner,fin+roi_corner,cv::Scalar(0,250,0),3);
        }

        //Order points before fitting
        std::vector<cv::Point> puntos;

        for(int i =0; i<lines.size(); i++)
        {
                cv::Point temp;
                temp.x=lines[i][0];
                temp.y=lines[i][1];
                puntos.push_back(temp);

                temp.x=lines[i][2];
                temp.y=lines[i][3];
                puntos.push_back(temp);
        }
        //fitline
        cv::Vec4f lineR;
        if(puntos.size()>0)
        {
                cv::fitLine(puntos,lineR, CV_DIST_WELSCH, 0, 0.01,0.01);
                cv::Point2f ini(cols/2,rows/2);
                cv::Point2f dir(lineR[0],lineR[1]);
                cv::Point2f fin(ini+200*dir);
                cv::line(image,ini,fin, cv::Scalar(255,0,0),5);

        } //Fits a stright line to Points

        //cv::cvtColor(border,border, cv::COLOR_GRAY2BGR);
        return image;


}

void draw_angle_arrows(cv::Mat &image, cv::Mat fase, int tam, cv::Point localizacion)
{
        cv::circle(image, localizacion, 5, cv::Scalar(255,0,0),5);
        //Draw angle lines
        float theta =fase.at<float>(localizacion)-M_PI/2;
        float fin_lin_x=localizacion.x+tam*cos(theta);
        float fin_lin_y=localizacion.y+tam*sin(theta);
        cv::Point fin_lin(fin_lin_x,fin_lin_y);
        cv::line(image, localizacion, fin_lin, cv::Scalar(0,200,0),5);
}

void draw_angle_arrows(cv::Mat &image, cv::Point origin, int tam, float theta)
{
        //Draw angle lines
        float fin_lin_x=origin.x+tam*cos(theta);
        float fin_lin_y=origin.y+tam*sin(theta);
        cv::Point fin_lin(fin_lin_x,fin_lin_y);
        cv::line(image, origin, fin_lin, cv::Scalar(0,200,0),5);
}




cv::Mat extract_lane(cv::Mat image,  int lowValThr,int highValThr,std::vector<geometry_msgs::PoseStamped> &poses_right, std::vector<geometry_msgs::PoseStamped> &poses_left)
{
        int cols=image.cols;
        int rows=image.rows;
        //std::cout<<cols<<" "<<rows<<endl;
        //Change to HSV
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        cv::blur(hsv,hsv,cv::Size(11,11));
        //Split channels and binarize based on value
        //TODO: Try HSL.
        cv::Mat chan_hsv[3],binarized;
        cv::split(hsv,chan_hsv);
        cv::Mat binarized_val;
        //cv::inRange(hsv, cv::Scalar(10,10,lowValThr),cv::Scalar(250,250,highValThr), binarized_val);
        cv::inRange(chan_hsv[2], lowValThr,highValThr, binarized_val);
        //Morphological filtering
        cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(10,10));
        cv:: morphologyEx(binarized_val, binarized_val, cv::MORPH_OPEN, element);
        // std::cout << cols <<" "<<rows<< std::endl;
        int thr_val_per_box = rows/(cols/2)*(255*0.7); //Threshold to prnue noise

        //Gradients
        // cv::Mat dev_x,dev_y, fase;
        // cv::Sobel(chan_hsv[2], dev_x, 5, 1,0 );
        // cv::Sobel(chan_hsv[2], dev_y, 5, 0,1 );
        // cv::phase(dev_x,dev_y, fase);

        // Right side

        for(int i=0; i<BOXES; i++)
        {
                cv::Rect micro_roi_der;
                geometry_msgs::PoseStamped pose_now;
                micro_roi_der.x=cols/2;
                micro_roi_der.y=rows/BOXES*(i)+1; //roi wont accept index 0 for whateer reason
                micro_roi_der.height=rows/BOXES-1;
                micro_roi_der.width=cols/2-1; //wont accept image overflow either

                cv::Point micro_roi_corner(micro_roi_der.x,micro_roi_der.y);
                //cv::rectangle(image,micro_roi_der,cv::Scalar(0,0,255),5);
                cv::Mat region_der(binarized_val,micro_roi_der);
                cv::Mat profile_hor, profile_ver;
                cv::reduce(region_der,profile_hor,0,CV_REDUCE_SUM,CV_32SC1);
                cv::reduce(region_der,profile_ver,1,CV_REDUCE_SUM,CV_32SC1);

                double max_hor, max_ver;
                cv::Point loc_max_hor, loc_max_ver;
                cv::minMaxLoc(profile_hor,NULL,&max_hor,NULL,&loc_max_hor,cv::noArray());
                cv::minMaxLoc(profile_ver,NULL,&max_ver,NULL,&loc_max_ver,cv::noArray());
                //cant discriminate y y axis
                cv::Point locale = loc_max_hor+loc_max_ver;
                //std::cout<<locale<<std::endl;
                if (max_hor < thr_val_per_box)
                {
                        //Cant decide if found lane or noise, set to 0 and continue
                        cv::circle(image, micro_roi_corner, 5, cv::Scalar(0,0,255),5);
                }
                else
                {
                        cv::Point abs_loc=locale+micro_roi_corner;
                        pose_now.pose.position.x=abs_loc.x;
                        pose_now.pose.position.y=abs_loc.y;
                        pose_now.pose.position.z=0;
                        poses_right.push_back(pose_now);
                        cv::circle(image, abs_loc, 5, cv::Scalar(0,255,0),5);
                        //draw_angle_arrows(image,fase,100,abs_loc);


                }


        }

        //left Side
        for(int i=0; i<BOXES; i++)
        {
                geometry_msgs::PoseStamped pose_now;

                cv::Rect micro_roi_izq;
                micro_roi_izq.x=1;
                micro_roi_izq.y=rows/BOXES*(i)+1; //roi wont accept index 0 for whateer reason
                micro_roi_izq.height=rows/BOXES-1;
                micro_roi_izq.width=cols/2-1; //wont accept image overflow either

                cv::Point micro_roi_corner(micro_roi_izq.x,micro_roi_izq.y);
                //cv::rectangle(image,micro_roi_izq,cv::Scalar(0,0,255),5);
                //Rigth Side
                cv::Mat region_izq(binarized_val,micro_roi_izq);
                cv::Mat profile_hor, profile_ver;
                cv::reduce(region_izq,profile_hor,0,CV_REDUCE_SUM,CV_32SC1);
                cv::reduce(region_izq,profile_ver,1,CV_REDUCE_SUM,CV_32SC1);

                double max_hor, max_ver;
                cv::Point loc_max_hor, loc_max_ver;
                cv::minMaxLoc(profile_hor,NULL,&max_hor,NULL,&loc_max_hor,cv::noArray());
                cv::minMaxLoc(profile_ver,NULL,&max_ver,NULL,&loc_max_ver,cv::noArray());
                //cant discriminate y y axis
                cv::Point locale = loc_max_hor+loc_max_ver;
                //std::cout<<locale<<std::endl;
                if (max_hor < thr_val_per_box)
                {
                        //Cant decide if found lane or noise, set to 0 and continue
                        cv::circle(image, micro_roi_corner, 5, cv::Scalar(0,0,255),5);
                }
                else
                {
                        cv::Point abs_loc=locale+micro_roi_corner;
                        pose_now.pose.position.x=abs_loc.x;
                        pose_now.pose.position.y=abs_loc.y;
                        pose_now.pose.position.z=0;
                        poses_left.push_back(pose_now);
                        cv::circle(image, abs_loc, 5, cv::Scalar(255,0,0),5);
                        //draw_angle_arrows(image,fase,100,abs_loc);
                }
        }

        return image;
}


cv::Mat extract_lane_angle(cv::Mat image,  int lowValThr,int highValThr,std_msgs::Float32MultiArray &angle_right, std_msgs::Float32MultiArray &angle_left)
{
        int cols=image.cols;
        int rows=image.rows;
        //std::cout<<cols<<" "<<rows<<endl;
        //Change to HSV
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        cv::blur(hsv,hsv,cv::Size(11,11));
        //Split channels and binarize based on value
        //TODO: Try HSL.
        cv::Mat chan_hsv[3],binarized;
        cv::split(hsv,chan_hsv);
        cv::Mat binarized_val;
        //cv::inRange(hsv, cv::Scalar(10,10,lowValThr),cv::Scalar(250,250,highValThr), binarized_val);
        cv::inRange(chan_hsv[2], lowValThr,highValThr, binarized_val);
        //Morphological filtering
        cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(10,10));
        cv:: morphologyEx(binarized_val, binarized_val, cv::MORPH_OPEN, element);
        // std::cout << cols <<" "<<rows<< std::endl;
        int thr_val_per_box = rows/(cols/2)*(255*0.7); //Threshold to prnue noise

        std::vector<cv::Point2f> left, right;
        //Gradients
        // cv::Mat dev_x,dev_y, fase;
        // cv::Sobel(chan_hsv[2], dev_x, 5, 1,0 );
        // cv::Sobel(chan_hsv[2], dev_y, 5, 0,1 );
        // cv::phase(dev_x,dev_y, fase);

        // Right side

        for(int i=0; i<BOXES; i++)
        {
                cv::Rect micro_roi_der;
                geometry_msgs::PoseStamped pose_now;
                micro_roi_der.x=cols/2;
                micro_roi_der.y=rows/BOXES*(i)+1; //roi wont accept index 0 for whateer reason
                micro_roi_der.height=rows/BOXES-1;
                micro_roi_der.width=cols/2-1; //wont accept image overflow either

                cv::Point micro_roi_corner(micro_roi_der.x,micro_roi_der.y);
                //cv::rectangle(image,micro_roi_der,cv::Scalar(0,0,255),5);
                cv::Mat region_der(binarized_val,micro_roi_der);
                cv::Mat profile_hor, profile_ver;
                cv::reduce(region_der,profile_hor,0,CV_REDUCE_SUM,CV_32SC1);
                cv::reduce(region_der,profile_ver,1,CV_REDUCE_SUM,CV_32SC1);

                double max_hor, max_ver;
                cv::Point loc_max_hor, loc_max_ver;
                cv::minMaxLoc(profile_hor,NULL,&max_hor,NULL,&loc_max_hor,cv::noArray());
                cv::minMaxLoc(profile_ver,NULL,&max_ver,NULL,&loc_max_ver,cv::noArray());
                //cant discriminate y y axis
                cv::Point locale = loc_max_hor+loc_max_ver;
                //std::cout<<locale<<std::endl;
                if (max_hor < thr_val_per_box)
                {
                        //Cant decide if found lane or noise, set to 0 and continue
                        cv::circle(image, micro_roi_corner, 5, cv::Scalar(0,0,255),5);
                }
                else
                {
                        cv::Point abs_loc=locale+micro_roi_corner;
                        right.push_back(abs_loc);
                        cv::circle(image, abs_loc, 5, cv::Scalar(0,255,0),5);
                        //draw_angle_arrows(image,fase,100,abs_loc);


                }


        }

        //left Side
        for(int i=0; i<BOXES; i++)
        {
                geometry_msgs::PoseStamped pose_now;

                cv::Rect micro_roi_izq;
                micro_roi_izq.x=1;
                micro_roi_izq.y=rows/BOXES*(i)+1; //roi wont accept index 0 for whateer reason
                micro_roi_izq.height=rows/BOXES-1;
                micro_roi_izq.width=cols/2-1; //wont accept image overflow either

                cv::Point micro_roi_corner(micro_roi_izq.x,micro_roi_izq.y);
                //cv::rectangle(image,micro_roi_izq,cv::Scalar(0,0,255),5);
                //Rigth Side
                cv::Mat region_izq(binarized_val,micro_roi_izq);
                cv::Mat profile_hor, profile_ver;
                cv::reduce(region_izq,profile_hor,0,CV_REDUCE_SUM,CV_32SC1);
                cv::reduce(region_izq,profile_ver,1,CV_REDUCE_SUM,CV_32SC1);

                double max_hor, max_ver;
                cv::Point loc_max_hor, loc_max_ver;
                cv::minMaxLoc(profile_hor,NULL,&max_hor,NULL,&loc_max_hor,cv::noArray());
                cv::minMaxLoc(profile_ver,NULL,&max_ver,NULL,&loc_max_ver,cv::noArray());
                //cant discriminate y y axis
                cv::Point locale = loc_max_hor+loc_max_ver;
                //std::cout<<locale<<std::endl;
                if (max_hor < thr_val_per_box)
                {
                        //Cant decide if found lane or noise, set to 0 and continue
                        cv::circle(image, micro_roi_corner, 5, cv::Scalar(0,0,255),5);
                }
                else
                {
                        cv::Point abs_loc=locale+micro_roi_corner;
                        left.push_back(abs_loc);
                        cv::circle(image, abs_loc, 5, cv::Scalar(255,0,0),5);
                        //draw_angle_arrows(image,fase,100,abs_loc);
                }
        }

        cv::Vec4f lineR, lineL;
        if(right.size()>0)
        {
                cv::fitLine(right,lineR, CV_DIST_WELSCH, 0, 0.01,0.01); //Fits a stright line to Points
                cv::Point2f ini(lineR[2],lineR[3]);
                cv::Point2f dir(lineR[0],lineR[1]);
                cv::Point2f fin(ini+200*dir);
                cv::line(image,ini,fin, cv::Scalar(255,0,0),5);
                angle_right.data.push_back(lineR[0]);
                angle_right.data.push_back(lineR[1]);
                angle_right.data.push_back(lineR[2]);
                angle_right.data.push_back(lineR[3]);

        }
        if(left.size()>0)
        {
                cv::fitLine(left,lineL, CV_DIST_WELSCH, 0, 0.01,0.01); //Fits a stright line to Points
                cv::Point2f ini(lineL[2],lineL[3]);
                cv::Point2f dir(lineL[0],lineL[1]);
                cv::Point2f fin(ini+200*dir);
                cv::line(image,ini,fin, cv::Scalar(0,255,0),5);
                angle_left.data.push_back(lineL[0]);
                angle_left.data.push_back(lineL[1]);
                angle_left.data.push_back(lineL[2]);
                angle_left.data.push_back(lineL[3]);
        }
        return image;
}



float calculate_lane_angle(std::vector<geometry_msgs::PoseStamped> &poses)
{
        float theta =0;
        for (int i=1; i<poses.size(); i++)
        {
                float x =poses[i].pose.position.x;
                float y =poses[i].pose.position.y;

                float x_i =poses[i-1].pose.position.x;
                float y_i =poses[i-1].pose.position.y;

                theta +=atan2( (y-y_i),(x-x_i));
        }
        return theta/poses.size();
}

cv::Point getAverageCenterLanePosition(std::vector<geometry_msgs::PoseStamped> &poses)
{

        float center_x =0;
        float center_y =0;
        int n = poses.size();
        for (int i=0; i<poses.size(); i++)
        {
                center_x +=poses[i].pose.position.x;
                center_y +=poses[i].pose.position.y;


        }
        cv::Point center(center_x/n,center_y/n);
        return center;
}
