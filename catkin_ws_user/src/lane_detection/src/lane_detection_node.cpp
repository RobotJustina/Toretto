#include "ros/ros.h"
#include "lane_detection.h"

#define RADINDEG 180/M_PI
cv::Mat Image;
bool image_flag=false;
cv::Mat persp;
cv::Mat transf;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
        try{
                Image = cv_bridge::toCvShare(msg, "bgr8")->image;
                image_flag=true;
//     std::cout << "Received image" << std::endl;
        }catch ( cv_bridge::Exception& e)
        {
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
                return;
        }
}

int main(int argc, char** argv)
{
        ros::init(argc, argv, "image_publisher");
        ros::NodeHandle nh;
        image_transport::ImageTransport its(nh);
        image_transport::Subscriber sub = its.subscribe("image",1,imageCallback);

        image_transport::ImageTransport itp(nh);
        image_transport::Publisher pub = itp.advertise("lines",1);
        sensor_msgs::ImagePtr msg;

        // ros::Publisher path_pub_r= nh.advertise<nav_msgs::Path>("/right",1000);
        // ros::Publisher path_pub_l= nh.advertise<nav_msgs::Path>("/left",1000);
        // nav_msgs::Path msg_path;

        // ros::Publisher path_pub_r= nh.advertise<std_msgs::Float32MultiArray>("/right",1000);
        // ros::Publisher path_pub_r= nh.advertise<std_msgs::Float32MultiArray>("/left",1000);
        // std_msgs::Float32MultiArray

        ros::Publisher angle_pub_r = nh.advertise<std_msgs::Float32MultiArray>("right",100);
        ros::Publisher angle_pub_l = nh.advertise<std_msgs::Float32MultiArray>("left",100);
        //std_msgs::Float32 msg_angle;
        ros::Rate loop_rate(10);

        cv::Mat transfMatrix;
        std::string filepath;

        int value_thr_high, value_thr_low;
        nh.param<std::string>("calib_file",filepath,"Matrix2.yaml"); //Set file path_pub
        nh.param<int>("value_thr_low",value_thr_low,170);
        nh.param<int>("value_thr_high",value_thr_high,190);

        cv::FileStorage fs(filepath, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
                std::cout<<"No homoMat4 @"<<filepath<<std::endl;
                return 0;
        }else
        {
                std::cout<<"*********** homoMat4 FOUND @ "<<filepath<<"***********"<<std::endl;
        }
        fs["Homography"] >> transfMatrix;
        cv::Size transfSize;
        fs["tSize"] >> transfSize;
        fs.release();

        //
        // cv::Mat divMat = cv::Mat::zeros(3,3, CV_64FC1);
        // divMat.at<double>(0,0) = (double)0.5;
        // divMat.at<double>(1,1) = (double)0.5;
        // divMat.at<double>(2,2) = (double)1.0;
        // transfMatrix = divMat *  transfMatrix;

        std::cout<<"Starting processing"<<std::endl;
        int i= 0;
        while (ros::ok()) {
                if (image_flag)
                {
                        image_flag = false;
                        cv::Mat trans;
                        //std::vector<geometry_msgs::PoseStamped> poses_right, poses_left;
                        std_msgs::Float32MultiArray angle_r, angle_l;

                        cv::warpPerspective(Image, trans, transfMatrix, transfSize, cv::INTER_LINEAR, cv::BORDER_REPLICATE, cv::Scalar(127, 127, 127) );
                        //trans = extract_lane(trans,value_thr_low,value_thr_high, poses_right, poses_left);
                        trans =extract_lane_angle(trans,value_thr_low,value_thr_high,angle_r,angle_l);
                        if(angle_r.data.size()>0)
                        {
                                // msg_path.poses = poses_right;
                                // path_pub_r.publish(msg_path);
                                //
                                // angle_r = calculate_lane_angle(poses_right);
                                // msg_angle.data = 180-angle_r*RADINDEG;
                                // angle_pub_r.publish(msg_angle);
                                //Visualization
                                // cv::Point centro=getAverageCenterLanePosition(poses_right);
                                // cv::circle(trans, centro, 5, cv::Scalar(0,100,100),10);
                                // draw_angle_arrows(trans,centro,100,angle_r);
                                angle_pub_r.publish(angle_r);

                        }

                        if(angle_l.data.size()>0)
                        {
                              angle_pub_l.publish(angle_l);
                        }

                        sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",trans).toImageMsg();

                        pub.publish(msg);
                }
                ros::spinOnce();
                loop_rate.sleep();
        }
        return 0;
}
