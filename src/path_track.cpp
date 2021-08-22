/**
* @file path_track.cpp
* @brief tracking target way point
* @author Michikuni Eguchi
* @date 2021.7.29
* @details 受け取ったtarget way point に追従するようにcmd_velをpublishする
*/
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <rumba_autocar/PurePursuit.h>
#include <rumba_autocar/tf_position.h>
#include <string>
#include <iostream>

geometry_msgs::Twist cmd_vel;
void cmd_callback(const geometry_msgs::Twist cmd_message)
{
    //前進だけ外部入力に委ねる
    cmd_vel.linear.x = cmd_message.linear.x;
}

int targetWp = 0;
void targetWp_callback(const std_msgs::Int32& targetWp_num)
{
    targetWp = targetWp_num.data;
}

nav_msgs::Path path;
void path_callback(const nav_msgs::Path& path_message)
{
    path = path_message;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_track");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string base_link_id, map_id;
    pnh.param<std::string>("base_link_frame_id", base_link_id, "base_link");
    pnh.param<std::string>("map_frame_id", map_id, "map");
    double rate;
    pnh.param<double>("loop_rate", rate, 100);
    double max_angular_vel;
    pnh.param<double>("max_angular_vel", max_angular_vel, 1);

    ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 10, cmd_callback);
    ros::Subscriber targetWp_sub = nh.subscribe("targetWp", 50, targetWp_callback);
    ros::Subscriber path_sub = nh.subscribe("path", 50, path_callback);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("roomba/cmd_vel", 10);

    ctr::PurePursuit pure_pursuit(max_angular_vel);
    tf_position nowPosition(map_id, base_link_id, rate);

    ros::Rate loop_rate(rate);

    
    while(ros::ok())
    {
        if(path.poses.size()>0){
            cmd_vel.angular.z = pure_pursuit.getYawVel(nowPosition.getPoseStamped(), path.poses[targetWp] , cmd_vel.linear.x);
            cmd_pub.publish(cmd_vel);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}