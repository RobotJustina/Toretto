#include "ros/ros.h"
#include "CrossDetector.h"

cv::Mat Image;
bool image = false;
cv::Mat persp;
cv::Mat transf;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
        try{
                Image = cv_bridge::toCvShare(msg, "bgr8")->image;

                image = true;
        }catch ( cv_bridge::Exception& e)
        {
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
}

int main(int argc, char** argv)
{
        ros::init(argc, argv, "cross_publisher");
        ros::NodeHandle nh;
        image_transport::ImageTransport its(nh);
        image_transport::Subscriber sub = its.subscribe("app/camera/rgb/image_raw",1,imageCallback);

        image_transport::ImageTransport itp(nh);
        image_transport::Publisher pub = itp.advertise("crossImg",1);
        sensor_msgs::ImagePtr msg;

        ros::Publisher cross_pub= nh.advertise<std_msgs::Bool>("cross",1000);

        ros::Rate loop_rate(35);

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

        CrossDetector cross = CrossDetector(hough_thr,min_lin_len,max_gap_len,value_thr_low,
                                            value_thr_high,canny_thr_low,canny_thr_high);
        std_msgs::Bool msg_cross;

        std::cout<<"Starting processing"<<std::endl;

        /*std::vector< cv::Point2f> srcPoints;
           srcPoints.push_back( cv::Point2f( 132, 470) );
           srcPoints.push_back( cv::Point2f( 195, 390) );
           srcPoints.push_back( cv::Point2f( 486, 479) );
           srcPoints.push_back( cv::Point2f( 415, 386) );

           std::vector< cv::Point2f> dstPoints;
           dstPoints.push_back( cv::Point2f( 132, 470) );
           dstPoints.push_back( cv::Point2f( 132, (float)390*0.8) );
           dstPoints.push_back( cv::Point2f( 486, 479) );
           dstPoints.push_back( cv::Point2f( 486, (float)386*0.8) );



           transf = getPerspectiveTransform(srcPoints, dstPoints );
         */
        while (ros::ok()) {

                if (image)
                {
                        bool crss= false;
                        cv::Rect rect(0,240,640,240); //lower half of image
                        cv::Mat resizeImage= Image(rect);
                        cv::Mat img_cross,edges;
                        cross.get_borders(resizeImage,edges, true);
                        cv: cvtColor(edges,img_cross,cv::COLOR_GRAY2BGR);
                        cross.segmentationCross(edges, img_cross, crss);
                        cv::addWeighted(img_cross, 0.5, resizeImage, 0.5, 0, img_cross);

                        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_cross).toImageMsg();
                        pub.publish(msg);
                        msg_cross.data = crss;
                        cross_pub.publish(msg_cross);


                        image= false;

                }
                ros::spinOnce();
                loop_rate.sleep();
        }
        return 0;
}
