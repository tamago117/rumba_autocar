#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <string>

const std::string recovery = "recovery";
const std::string run = "run";

std_msgs::String mode;
void mode_callback(const std_msgs::String& mode_message)
{
    mode = mode_message;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_recovery");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    double linear_vel, angular_vel;
    double duration;
    pnh.param<double>("linear_vel", linear_vel, -0.3);
    pnh.param<double>("angular_vel", angular_vel, 0);
    pnh.param<double>("duration", duration, 0.5);

    ros::Subscriber mode_sub = nh.subscribe("safety_limit/mode", 10, mode_callback);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("simple_recovery/cmd_vel", 10);
    ros::Publisher mode_pub = nh.advertise<std_msgs::String>("simple_recovery/mode", 10);

    ros::Rate loop_rate(10);

    std_msgs::String mode_out;
    geometry_msgs::Twist cmd_vel;
    ros::Time preT = ros::Time::now();
    while(ros::ok())
    {
        if(mode_out.data == recovery){
            mode_out.data = recovery;
            cmd_vel.linear.x = linear_vel;
            cmd_vel.angular.z = angular_vel;

            if((ros::Time::now() - preT) >= ros::Duration(duration)){
                mode_out.data = run;
            }

        }else{
            mode_out = mode;
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;

            preT = ros::Time::now();
        }

        cmd_pub.publish(cmd_vel);
        mode_pub.publish(mode_out);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}