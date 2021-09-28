/**
* @file wpSelect.cpp
* @brief select target way point
* @author Michikuni Eguchi
* @date 2021.9.21
* @details 位置情報からpublishするtarget way pointとなるpose を選ぶ
*          (等間隔で設定されてない5〜10mの少し離れた間隔のwp pointをもとにする)
*/
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <string>
#include "rumba_autocar/tf_position.h"
#include "rumba_autocar/robot_status.h"

//poseStamp間の距離
double poseStampDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
{
    double diffX = pose1.pose.position.x - pose2.pose.position.x;
    double diffY = pose1.pose.position.y - pose2.pose.position.y;
    double diffZ = pose1.pose.position.z - pose2.pose.position.z;

    return sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);
}

template<class T> T constrain(T num, double minVal, double maxVal)
{
    if(num > maxVal){
        num = maxVal;
    }
    if(num < minVal){
        num = minVal;
    }

    return num;
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
    double target_deviation, fin_tar_deviation;
    pnh.param<double>("target_deviation", target_deviation, 0.5);
    pnh.param<double>("final_target_deviation", fin_tar_deviation, 0.1);
    double rate;
    pnh.param<double>("loop_rate", rate, 100);
    double maxVelocity;
    pnh.param<double>("maxVelocity", maxVelocity, 1.0);

    tf_position nowPosition(map_id, base_link_id, rate);

    ros::Subscriber path_sub = nh.subscribe("path", 50, path_callback);
    ros::Publisher tarWp_pub = nh.advertise<std_msgs::Int32>("targetWp", 10);
    ros::Publisher tarPos_pub = nh.advertise<geometry_msgs::PoseStamped>("targetWpPose", 10);
    ros::Publisher mode_pub = nh.advertise<std_msgs::String>("mode_select/mode", 10);

    ros::Rate loop_rate(rate);

    bool trace_wp_mode = true;
    std_msgs::Int32 targetWp;

    std_msgs::String mode;
    mode.data = robot_status_str(robot_status::angleAdjust);

    bool isReach = false;

    targetWp.data = 0;

    geometry_msgs::PoseStamped tarPos;
    while(ros::ok())
    {
        if(path.poses.size()>0){
            if(trace_wp_mode){
                //target_deviationになるよう target way pointの更新
                while(!(poseStampDistance(path.poses[targetWp.data], nowPosition.getPoseStamped()) >= target_deviation))
                {
                    //end point
                    if(targetWp.data >= (path.poses.size()-1)){
                        break;
                    }
                    targetWp.data++;
                }
            }

            if(targetWp.data >= (path.poses.size()-1)){
                //distance
                if(!isReach){
                    if(poseStampDistance(path.poses[targetWp.data], nowPosition.getPoseStamped()) <= fin_tar_deviation){
                        //isReach = true;
                        mode_pub.publish(mode);
                    }
                }
            }

            tarPos.header.frame_id = path.header.frame_id;
            tarPos.header.stamp = ros::Time::now();
            tarPos.pose = path.poses[targetWp.data].pose;

            tarPos_pub.publish(tarPos);
            tarWp_pub.publish(targetWp);

        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}