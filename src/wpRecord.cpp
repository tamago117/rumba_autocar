/**
* @file wpRecord.cpp
* @brief record way point
* @author Michikuni Eguchi
* @date 2021.7.27
* @details robotの移動経路からrobotのnavigationコース用ポイント(way point)をcsv形式で記憶
           移動経路には/base_linkから/mapまでの座標変換の計算結果を用いる
*/

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpRecord");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("path2csv", 10);

    std::string map_id, base_id;
    pn.param<std::string>("map_frame_id",map_id,"map");
    pn.param<std::string>("base_link_frame_id",base_id,"base_link");
    int rate;
    pn.param<int>("loop_rate", rate, 50);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped tfStamped;
    ros::Timer timer = nh.createTimer(ros::Duration(1.0/rate), [&](const ros::TimerEvent& e)
    {
        try
        {
            tfStamped = tfBuffer.lookupTransform(map_id, base_id, ros::Time(0));
        }
        catch(tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }
        
        
    });

    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
        /*geometry_msgs::Pose pose;
        pose.position.x = tfStamped.transform.translation.x;
        pose.position.y = tfStamped.transform.translation.y;
        pose.position.z = tfStamped.transform.translation.z;
        pose.orientation = tfStamped.transform.rotation;
        ROS_INFO("position x:%f,y:%f,z:%f orientation x:%f,y:%f,z:%f,w:%f\n", pose.position.x, pose.position.y, pose.position, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);*/

        //csvdata : [x, y, z, q_x, q_y, q_z, q_w]
        std_msgs::Float32MultiArray array;
        array.data.resize(7);
        array.data[0] = tfStamped.transform.translation.x;
        array.data[1] = tfStamped.transform.translation.y;
        array.data[2] = tfStamped.transform.translation.z;
        array.data[3] = tfStamped.transform.rotation.x;
        array.data[4] = tfStamped.transform.rotation.y;
        array.data[5] = tfStamped.transform.rotation.z;
        array.data[6] = tfStamped.transform.rotation.w;

        pub.publish(array);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}
