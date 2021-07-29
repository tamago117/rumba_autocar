/**
* @file tf_position.h
* @brief  read frame position from tf
* @author Michikuni Eguchi
* @date 2021.7.29
* @details Read position from transformations in tf
*/

#pragma once
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <turtlesim/Spawn.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>

class tf_position
{
public:
    tf_position(std::string base_id, std::string child_id, double rate);
    geometry_msgs::PoseStamped getPoseStamped();
    geometry_msgs::Pose getPose();
private:
    geometry_msgs::TransformStamped tfStamp;
    geometry_msgs::PoseStamped poseStamp;

};

tf_position::tf_position(std::string base_id, std::string child_id, double rate) : nh() ,tfBuffer() ,tfListener(tfBuffer)
{
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Timer timer = nh.createTimer(ros::Duration(1.0/rate), [&](const ros::TimerEvent& e)
    {
        try
        {
            tfStamp = tfBuffer.lookupTransform(base_id, child_id, ros::Time(0));
            poseStamp.header = tfStamp.header;
            poseStamp.pose.position.x = tfStamp.transform.translation.x;
            poseStamp.pose.position.y = tfStamp.transform.translation.y;
            poseStamp.pose.position.z = tfStamp.transform.translation.z;
            poseStamp.pose.orientation = tfStamp.transform.rotation;
        }
        catch(tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }
    });

}

geometry_msgs::PoseStamped tf_position::getPoseStamped()
{
    return poseStamp;
}

geometry_msgs::Pose tf_position::getPose()
{
    return poseStamp.pose;
}