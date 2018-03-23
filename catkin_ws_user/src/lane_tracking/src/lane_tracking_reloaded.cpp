#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16.h"
#include "Filter.h"

#define filter_order 5

float K_dist  = 0.5;
float K_angle_max = 16.0;
float K_angle_min = 16.0;
float K_brake = 1.0;
int max_speed  = 800;
int turn_speed = 400;
int dist_to_lane = 100;
int cutoff = 0.1;

int16_t steering = 100;;
int16_t speed = 0;

std::vector<float> input_buffer;
std::vector<float> output_buffer;
std::vector<float> butter_A;
std::vector<float> butter_B;

bool shutdown=false;
void callback_right_line(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
        float A = msg->data[0];
        float B = msg->data[1];
        float C = msg->data[2];
        if (A==0 && B==0)
                return;
        //La imagen homografeada es de 640x700
        float angle_error = atan(B/A);
        input_buffer.erase(input_buffer.begin());
        input_buffer.push_back(angle_error);

	float filtered_angle = 0;
	output_buffer.erase(output_buffer.begin());
	output_buffer.push_back(0);
	for(int i =0; i <= filter_order; i++)
	    filtered_angle += input_buffer[i] * butter_B[i];
	for(int i=1; i <= filter_order; i++)
	    filtered_angle -= output_buffer[i] * butter_A[i];
	output_buffer[5] = filtered_angle;
	//angle_error = filtered_angle;

        float dist_error = (fabs(A*160 + B*190 +C)/sqrt(A*A + B*B) - dist_to_lane);
        speed    = (int16_t)(-(max_speed - K_brake * fabs(angle_error) * (max_speed - turn_speed)));
	float K_angle;
	if(-speed < 700)
	    K_angle = K_angle_max;
	else if (-speed > 1500)
	    K_angle = K_angle_min;
	else
	    K_angle = K_angle_max - (-speed - 700.0)/800.0*(K_angle_max - K_angle_min);
	steering = (int16_t)(100 + K_dist * dist_error + K_angle * angle_error);
        // std::cout << "Found line: " << A << "\t" << B << "\t" << C << std::endl;
        // std::cout << "Angle error= " << angle_error << std::endl;
        //std::cout << "Distance: " << dist_error<<'\n';
}

void Callback_stop(const std_msgs::Int16::ConstPtr& msg)
{
        if(msg->data==1)
        {
                printf("!!!!!!Requesting shutdown!!!!!\n");
                shutdown=true;
        }
        else if(msg->data==0)
        {
                //printf("!!!!!!Disabling shutdown!!!!!\n");
                shutdown=false;
        }

}

int main(int argc, char** argv)
{
        std::cout << "INITIALIZING LANE TRACKING BY MARCOSOFT..."  << std::endl;
        ros::init(argc, argv, "lane_tracking");
        ros::NodeHandle n;


        n.param<float>("K_dist", K_dist, 0.5);
        n.param<float>("K_angle_max", K_angle_max, 30.0);
	n.param<float>("K_angle_min", K_angle_min, 12.0);
        n.param<float>("K_brake", K_brake, 1.0);
        n.param<int>("max_speed", max_speed, 800);
        n.param<int>("turn_speed", turn_speed, 400);
        n.param<int>("dist_to_lane", dist_to_lane, 90);
	n.param<int>("cutoff", cutoff, 1);

        std::cout << "K_dist=" << K_dist << "\tK_angle_max=" << K_angle_max << "\tK_angle_min=" << K_angle_min;
	std::cout << "\tK_brake=" << K_brake << "\tmax_speed=" << max_speed;
        std::cout << "\tturn_speed=" << turn_speed << "\tdist_to_lane=" << dist_to_lane;
	std::cout << "\tcutoff freq=" << cutoff << std::endl;
        ros::Publisher pub_steering   = n.advertise<std_msgs::Int16>("/manual_control/steering", 1);
        ros::Publisher pub_speed      = n.advertise<std_msgs::Int16>("/manual_control/speed", 1);
        ros::Subscriber sub_lane_right = n.subscribe("/rightLine", 1, callback_right_line);

        //ros::Subscriber stop_subscriber = n.subscribe("/manual_control/stop", 1, Callback_stop);

        ros::Rate loop(20);
        std_msgs::Int16 msg_steering;
        std_msgs::Int16 msg_speed;

	butter_A.resize(filter_order + 1);
	butter_B.resize(filter_order + 1);
	butter_A[5] = 1;
        for (int i=0; i<=filter_order; i++)
	{
	    input_buffer.push_back(0.0);
	    output_buffer.push_back(0.0);
	}

	switch(cutoff)
	{
	case 1:
	    butter_A[4] = BUTTER_N5_0_1_A1;
	    butter_A[3] = BUTTER_N5_0_1_A2;
	    butter_A[2] = BUTTER_N5_0_1_A3;
	    butter_A[1] = BUTTER_N5_0_1_A4;
	    butter_A[0] = BUTTER_N5_0_1_A5;

	    butter_B[5] = BUTTER_N5_0_1_B0;
	    butter_B[4] = BUTTER_N5_0_1_B1;
	    butter_B[3] = BUTTER_N5_0_1_B2;
	    butter_B[2] = BUTTER_N5_0_1_B3;
	    butter_B[1] = BUTTER_N5_0_1_B4;
	    butter_B[0] = BUTTER_N5_0_1_B5;
	    break;
	case 2:
	    butter_A[4] = BUTTER_N5_0_2_A1;
	    butter_A[3] = BUTTER_N5_0_2_A2;
	    butter_A[2] = BUTTER_N5_0_2_A3;
	    butter_A[1] = BUTTER_N5_0_2_A4;
	    butter_A[0] = BUTTER_N5_0_2_A5;

	    butter_B[5] = BUTTER_N5_0_2_B0;
	    butter_B[4] = BUTTER_N5_0_2_B1;
	    butter_B[3] = BUTTER_N5_0_2_B2;
	    butter_B[2] = BUTTER_N5_0_2_B3;
	    butter_B[1] = BUTTER_N5_0_2_B4;
	    butter_B[0] = BUTTER_N5_0_2_B5;
	    break;
	case 3:
	    butter_A[4] = BUTTER_N5_0_3_A1;
	    butter_A[3] = BUTTER_N5_0_3_A2;
	    butter_A[2] = BUTTER_N5_0_3_A3;
	    butter_A[1] = BUTTER_N5_0_3_A4;
	    butter_A[0] = BUTTER_N5_0_3_A5;

	    butter_B[5] = BUTTER_N5_0_3_B0;
	    butter_B[4] = BUTTER_N5_0_3_B1;
	    butter_B[3] = BUTTER_N5_0_3_B2;
	    butter_B[2] = BUTTER_N5_0_3_B3;
	    butter_B[1] = BUTTER_N5_0_3_B4;
	    butter_B[0] = BUTTER_N5_0_3_B5;
	    break;
	case 4:
	    butter_A[4] = BUTTER_N5_0_4_A1;
	    butter_A[3] = BUTTER_N5_0_4_A2;
	    butter_A[2] = BUTTER_N5_0_4_A3;
	    butter_A[1] = BUTTER_N5_0_4_A4;
	    butter_A[0] = BUTTER_N5_0_4_A5;

	    butter_B[5] = BUTTER_N5_0_4_B0;
	    butter_B[4] = BUTTER_N5_0_4_B1;
	    butter_B[3] = BUTTER_N5_0_4_B2;
	    butter_B[2] = BUTTER_N5_0_4_B3;
	    butter_B[1] = BUTTER_N5_0_4_B4;
	    butter_B[0] = BUTTER_N5_0_4_B5;
	    break;
	}


        while(ros::ok())
        {
                // if(shutdown)
                // {
                //         msg_steering.data = 90;
                //         msg_speed.data    = 0;
                //         pub_steering.publish(msg_steering);
                //         pub_speed.publish(msg_speed);
                //         continue;
                //
                // }
                // printf("Debuggin 1\n");

                msg_steering.data = steering;
                msg_speed.data    = speed;
                // printf("Debuggin 2\n");

                pub_steering.publish(msg_steering);
                pub_speed.publish(msg_speed);
                // printf("Debuggin 3\n");



                ros::spinOnce();
                loop.sleep();


        }
        return 0;
}
