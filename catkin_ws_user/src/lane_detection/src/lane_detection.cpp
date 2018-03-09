#define BOXES 8
#define MAX_VAL_IN_PIXEL 255
#include "lane_detection.h"


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

cv::Mat extract_lane(cv::Mat image,  int lowValThr,int highValThr,std::vector<geometry_msgs::PoseStamped> &poses)
{
        int cols=image.cols;
        int rows=image.rows;
        //std::cout<<cols<<" "<<rows<<endl;
        //Change to HSV
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        cv::blur(hsv,hsv,cv::Size(5,11));
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
        cv::Mat dev_x,dev_y, fase;
        cv::Sobel(chan_hsv[2], dev_x, 5, 1,0 );
        cv::Sobel(chan_hsv[2], dev_y, 5, 0,1 );
        cv::phase(dev_x,dev_y, fase);

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
                        pose_now.pose.position.x=loc_max_hor.x;
                        pose_now.pose.position.y=loc_max_ver.y;
                        pose_now.pose.position.z=0;
                        poses.push_back(pose_now);
                        cv::Point abs_loc=locale+micro_roi_corner;

                        draw_angle_arrows(image,fase,100,abs_loc);


                }


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
