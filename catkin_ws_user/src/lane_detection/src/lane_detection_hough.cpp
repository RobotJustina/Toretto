#include "ros/ros.h"
#include "lane_detection.h"

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

        ros::Publisher angle_pub_r = nh.advertise<std_msgs::Float32MultiArray>("right",100);
        ros::Publisher angle_pub_l = nh.advertise<std_msgs::Float32MultiArray>("left",100);

        ros::Rate loop_rate(15);

        int hough_thr;
        double min_lin_len,max_gap_len;
        int value_thr_low, value_thr_high;
        int canny_thr_low, canny_thr_high;

        nh.param<int>("value_thr_low",value_thr_low,170);
        nh.param<int>("value_thr_high",value_thr_high,190);

        nh.param<int>("hough_thr",hough_thr,50);
        nh.param<double>("min_lin_len",min_lin_len,30);
        nh.param<double>("max_gap_len",max_gap_len,10);

        nh.param<int>("canny_thr_low",canny_thr_low,50);
        nh.param<int>("canny_thr_high",canny_thr_high,100);

        lane_extractor extractor(hough_thr,min_lin_len,max_gap_len,value_thr_low,
                                 value_thr_high,canny_thr_low,canny_thr_high);
        std::cout<<"Starting processing"<<std::endl;

        int i= 0;
        while (ros::ok()) {
                if (image_flag)
                {
                        //std::cout << "Image Received" << '\n';
                        image_flag = false;

                        std_msgs::Float32MultiArray angle_r;
                        angle_r=extractor.extract_right_lane_angle_hough(Image);
                        sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",Image).toImageMsg();

                        pub.publish(msg);
                        if(angle_r.data.size()>0)
                        {
                          angle_pub_r.publish(angle_r);
                        }
                }
                ros::spinOnce();
                loop_rate.sleep();
        }
        return 0;
}
