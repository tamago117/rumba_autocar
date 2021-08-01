/**
* @file wpControll.cpp
* @brief select target way point
* @author Michikuni Eguchi
* @date 2021.7.29
* @details 位置情報やインタフェースからpublishするtarget way point を選ぶ
*/
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include <string>
#include <rumba_autocar/tf_position.h>

//poseStamp間の距離
double poseStampDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
{
    double diffX = pose1.pose.position.x - pose2.pose.position.x;
    double diffY = pose1.pose.position.y - pose2.pose.position.y;
    double diffZ = pose1.pose.position.z - pose2.pose.position.z;

    return sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);
}

nav_msgs::Path path;
void path_callback(const nav_msgs::Path path_message)
{
    path = path_message;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpControll");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string map_id, base_link_id;
    pnh.param<std::string>("map_frame_id", map_id, "map");
    pnh.param<std::string>("base_link_frame_id", base_link_id, "base_link");
    double wp_pitch, tar_deviation;
    pnh.param<double>("waypoint_pitch", wp_pitch, 0.1);
    pnh.param<double>("target_deviation", tar_deviation, 0.05);
    double rate;
    pnh.param<double>("loop_rate", rate, 100);

    tf_position nowPosition(map_id, base_link_id, rate);

    ros::Subscriber path_sub = nh.subscribe("path", 50, path_callback);
    ros::Publisher wp_pub = nh.advertise<std_msgs::Int32>("targetWp", 10);

    ros::Rate loop_rate(rate);

    bool trace_wp_mode = true;
    std_msgs::Int32 targetWp;
    while(ros::ok())
    {
        if(path.poses.size()>0){
            if(trace_wp_mode){
                //waypoint_pitchになるよう target way pointの更新
                while(!(poseStampDistance(path.poses[targetWp.data], nowPosition.getPoseStamped()) >= wp_pitch))
                {
                    std::cout<<"dis  "<<poseStampDistance(path.poses[targetWp.data], nowPosition.getPoseStamped())<<std::endl;
                    //終端
                    if(targetWp.data > (path.poses.size()-1)){
                        std::cout<<path.poses.size()<<" "<<targetWp.data<<std::endl;
                        break;
                    }
                    targetWp.data++;
                    std::cout<<"targetWp"<<targetWp.data<<std::endl;
                }
            }
        }

        wp_pub.publish(targetWp);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}