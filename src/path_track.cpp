#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include "rumba_autocar/purePursuit.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_track");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");


}