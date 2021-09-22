#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include "rumba_autocar/tf_position.h"

double poseStampDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
{
    double diffX = pose1.pose.position.x - pose2.pose.position.x;
    double diffY = pose1.pose.position.y - pose2.pose.position.y;
    double diffZ = pose1.pose.position.z - pose2.pose.position.z;

    return sqrt(pow(diffX, 2) + pow(diffY, 2) + pow(diffZ, 2));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_check_test");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string base_link_id, map_id;
    pnh.param<std::string>("base_link_frame_id", base_link_id, "base_link");
    pnh.param<std::string>("map_frame_id", map_id, "odom");
    double rate;
    pnh.param<double>("loop_rate", rate, 20);

    ros::Publisher vel_pub = nh.advertise<std_msgs::Float32>("velocity", 10);

    tf_position nowPosition(map_id, base_link_id, rate);

    ros::Rate loop_rate(rate);

    geometry_msgs::PoseStamped prePos;
    std_msgs::Float32 speed_rviz;
    prePos = nowPosition.getPoseStamped();

    ros::Time preT = ros::Time::now();
    while(ros::ok())
    {
        double distance = poseStampDistance(nowPosition.getPoseStamped(), prePos);
        ros::Duration dt = ros::Time::now() - preT;
        std::cout<<distance<<" "<<dt.nsec*pow(10, -9);
        double speed = distance/(dt.nsec*pow(10, -9));
        speed_rviz.data = speed;
        prePos = nowPosition.getPoseStamped();
        preT = ros::Time::now();

        ROS_INFO("%f",speed);
        vel_pub.publish(speed_rviz);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}