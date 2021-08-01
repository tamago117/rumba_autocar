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
#include <rumba_autocar/tf_position.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpRecord");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("path2csv", 10);

    std::string map_id, base_link_id;
    pnh.param<std::string>("map_frame_id",map_id,"map");
    pnh.param<std::string>("base_link_frame_id",base_link_id,"base_link");
    int rate;
    pnh.param<int>("loop_rate", rate, 50);

    tf_position nowPosition(map_id, base_link_id, rate);

    ros::Rate loop_rate(rate);

    geometry_msgs::Pose pose;
    while (ros::ok())
    {
        //csvdata : [x, y, z, q_x, q_y, q_z, q_w]
        std_msgs::Float32MultiArray array;
        array.data.resize(7);
        pose = nowPosition.getPose();
        array.data[0] = pose.position.x;
        array.data[1] = pose.position.y;
        array.data[2] = pose.position.z;
        array.data[3] = pose.orientation.x;
        array.data[4] = pose.orientation.y;
        array.data[5] = pose.orientation.z;
        array.data[6] = pose.orientation.w;

        pub.publish(array);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
