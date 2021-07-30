/**
* @file wpControll.cpp
* @brief select target way point
* @author Michikuni Eguchi
* @date 2021.7.29
* @details 位置情報やインタフェースからpublishするtarget way point を選ぶ
*/
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include <string>
#include <rumba_autocar/tf_position.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpControll");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string map_id, base_link_id;
    pnh.param<std::string>("map_frame_id", map_id, "map");
    pnh.param<std::string>("base_link_frame_id", base_link_id, "base_link");
    double wp_pitch;
    pnh.param<double>("waypoint_pitch", wp_pitch, 0.1);
    double rate;
    pnh.param<double>("loop_rate", rate, 10);

    tf_position nowPosition(map_id, base_link_id, rate);

    ros::Publisher wp_pub = nh.advertise<std_msgs::Int32>("targetWp", 10);

    ros::Rate loop_rate(rate);

    std_msgs::Int32 targetWp;
    while(ros::ok())
    {

        wp_pub.publish(targetWp);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}