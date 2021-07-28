/**
* @file wp_load.cpp
* @brief csv to path node
* @author Michikuni Eguchi
* @date 2021.7.28
* @details wpをcsvから読み込んでpathとmarkerarrayで配信する
*          waypoint/nowにあわせてmarkerの色を変える
*/

#include <ros/ros.h>
#include <std_msgs/PoseStamped>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include "rumba_autocar/csv_input.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpLoad");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    //csv file nameの読み込み
    std::string filePath;
    pnh.getParam("filePath", filePath);
    pnh.param<std::string>("map_frame_id", map_id, "map");
    //csv読み込み
    csv::csv_input(filePath);

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("wayPoint/path", 10);

    std_msgs::PoseStamped pose;
    nav_msgs::Path path;

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        pub.publish(ps_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
}