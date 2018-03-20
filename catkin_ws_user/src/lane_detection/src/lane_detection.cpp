#define BOXES 12
#define MAX_VAL_IN_PIXEL 255
#include "lane_detection.h"
#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180

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

void lane_extractor::get_borders(cv::Mat &image, cv::Mat &edges, bool color = false)
{
        cv::Mat gray_img,hsv;
        cv::Mat binarized;
        if (color)
        {
                cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
                cv::blur(hsv,hsv,cv::Size(7,7));
                cv::Mat chan_hsv[3];
                cv::split(hsv,chan_hsv);
                cv::inRange(chan_hsv[1], lowValThr,highValThr, binarized);
        }
        else{
                cv::cvtColor(image, gray_img, cv::COLOR_BGR2GRAY);
                cv::blur(gray_img,gray_img,cv::Size(11,11));
                cv::inRange(gray_img, lowValThr,highValThr, binarized);
        }

        cv::Canny( binarized, edges,canny_thr_low,canny_thr_high);
}

std_msgs::Float32MultiArray lane_extractor::extract_right_lane_hough(cv::Mat &edges, cv::Mat &viz)
{
        //Extract lines from lane using hough transform
        //Return line equation: Ax+By+C=0 coefficents
        int cols=edges.cols;
        int rows=edges.rows;

        // cv::Point roi_corner(cols/2,rows*2/3);
        // cv::Size roi_size(cols/2-1,rows/3-1);
        // cv::Rect roi(roi_corner,roi_size);
        // cv::Mat cropped_img(edges,roi);

        cv::Point roi_corner(cols/2,0); //Right hand side of the edges
        cv::Size roi_size(cols/2-1,rows-1);
        cv::Rect roi(roi_corner,roi_size);
        cv::Mat cropped_img(edges,roi);


        //cv::blur(gray_img,gray_img,cv::Size(11,11));

        std::vector<cv::Vec4i> lines;
        std::vector<cv::Point> puntos;
        cv::HoughLinesP(cropped_img, lines,1,CV_PI/180, hough_thr, minLen, gapLen);

        std::cout<<"Number of lines Right: "<< lines.size()<<std::endl;
        int ver_cnt=0;
        for (int i=0; i<lines.size(); i++)
        {

                cv::Point ini(lines[i][0],lines[i][1]);
                cv::Point fin(lines[i][2],lines[i][3]);
                //cv::line(image,ini+roi_corner,fin+roi_corner,cv::Scalar(0,250,0),3);
                float m=atan2(fin.y-ini.y,fin.x-ini.x);
              //  std::cout<<m*RAD2DEG<<std::endl;

                m=fabs(m);
                //cv::line(image,ini+roi_corner,fin+roi_corner,cv::Scalar(0,250,0),3);
                if ((m > 20*DEG2RAD) && (m< 160 *DEG2RAD))
                {
                        //lines.erase(lines.begin()+i); does not work no idea why, vector is unchanged
                        cv::line(viz,ini+roi_corner,fin+roi_corner,cv::Scalar(0,0,250),3);
                        puntos.push_back(ini);
                        puntos.push_back(fin);

                }
                else
                {
                        continue;
                }

        }

        //Order points before fitting
        // for(int i =0; i<lines.size(); i++)
        // {
        //         cv::Point temp;
        //         temp.x=lines[i][0];
        //         temp.y=lines[i][1];
        //         puntos.push_back(temp);
        //
        //         temp.x=lines[i][2];
        //         temp.y=lines[i][3];
        //         puntos.push_back(temp);
        // }
        // //fitline
        cv::Vec4f lineR;
        if(puntos.size()>0)
        {
                cv::fitLine(puntos,lineR, CV_DIST_WELSCH, 0, 0.01,0.01);
                lineR[2]+=roi_corner.x; lineR[3]+=roi_corner.y;
                cv::Point2f ini(lineR[2],lineR[3]);
                //ini=ini+roi_corner;
                cv::Point2f dir(lineR[0],lineR[1]);
                cv::Point2f fin(ini+100*dir);
                cv::line(viz,ini,fin, cv::Scalar(27,232, 232),5); //amarillo
                msg_direction.data.clear();

                float A=1/lineR[0];
                float B=-1/lineR[1];
                float C=-lineR[2]/lineR[0]+lineR[3]/lineR[1];
                msg_direction.data.push_back(A);
                msg_direction.data.push_back(B);
                msg_direction.data.push_back(C);

        } //Fits a stright line to Points

        //cv::cvtColor(border,border, cv::COLOR_GRAY2BGR);
        return msg_direction;


}


std_msgs::Float32MultiArray lane_extractor::extract_left_lane_hough(cv::Mat &edges, cv::Mat & viz )
{
        int cols=edges.cols;
        int rows=edges.rows;

        // cv::Point roi_corner(cols/2,rows*2/3);
        // cv::Size roi_size(cols/2-1,rows/3-1);
        // cv::Rect roi(roi_corner,roi_size);
        // cv::Mat cropped_img(image,roi);

        cv::Point roi_corner(0,0); //left hand side of the image
        cv::Size roi_size(cols/2-1,rows-1);
        cv::Rect roi(roi_corner,roi_size);
        cv::Mat cropped_img(edges,roi);


        //cv::cvtColor(cropped_img,gray_img,cv::COLOR_BGR2GRAY );
        //cv::blur(gray_img,gray_img,cv::Size(11,11));

        std::vector<cv::Vec4i> lines;
        std::vector<cv::Point> puntos;
        cv::HoughLinesP(cropped_img, lines,1,CV_PI/180, hough_thr, minLen, gapLen);
        std::cout<<"Number of lines left: "<< lines.size()<<std::endl;
        for (int i=0; i<lines.size(); i++)
        {

                cv::Point ini(lines[i][0],lines[i][1]);
                cv::Point fin(lines[i][2],lines[i][3]);
                float m=atan2(fin.y-ini.y,fin.x-ini.x);
                //std::cout<<m*RAD2DEG<<std::endl;

                m=fabs(m);
                //cv::line(image,ini+roi_corner,fin+roi_corner,cv::Scalar(0,250,0),3);
                if ((m > 20*DEG2RAD)  && (m< 160 *DEG2RAD))
                {
                        //lines.erase(lines.begin()+i); does not work either
                        cv::line(viz,ini,fin,cv::Scalar(0,250,0),3);
                        puntos.push_back(ini);
                        puntos.push_back(fin);
                }
                else{
                      continue;
                }
        }

        //Order points before fitting
        // for(int i =0; i<lines.size(); i++)
        // {
        //         cv::Point temp;
        //         temp.x=lines[i][0];
        //         temp.y=lines[i][1];
        //         puntos.push_back(temp);
        //
        //         temp.x=lines[i][2];
        //         temp.y=lines[i][3];
        //         puntos.push_back(temp);
        // }
        // //fitline
        cv::Vec4f lineL;
        if(puntos.size()>0)
        {
                cv::fitLine(puntos,lineL, CV_DIST_WELSCH, 0, 0.01,0.01);
                lineL[2]+=roi_corner.x; lineL[3]+=roi_corner.y;
                cv::Point2f ini(lineL[2],lineL[3]);
                //ini=ini+roi_corner;
                cv::Point2f dir(lineL[0],lineL[1]);
                cv::Point2f fin(ini+100*dir);
                cv::line(viz,ini,fin, cv::Scalar(188,193,46),5); //azul clarito
                msg_direction.data.clear();

                float A=1/lineL[0];
                float B=-1/lineL[1];
                float C=-lineL[2]/lineL[0]+lineL[3]/lineL[1];
                msg_direction.data.push_back(A);
                msg_direction.data.push_back(B);
                msg_direction.data.push_back(C);

        } //Fits a stright line to Points

        //cv::cvtColor(border,border, cv::COLOR_GRAY2BGR);
        return msg_direction;


}


void draw_angle_arrows(cv::Mat &image, cv::Point origin, int tam, float theta)
{
        //Draw angle lines
        float fin_lin_x=origin.x+tam*cos(theta);
        float fin_lin_y=origin.y+tam*sin(theta);
        cv::Point fin_lin(fin_lin_x,fin_lin_y);
        cv::line(image, origin, fin_lin, cv::Scalar(0,200,0),5);
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
