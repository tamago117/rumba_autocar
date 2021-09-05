#include <ros/ros.h>
#include <math.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include "rumba_autocar/tf_position.h"

geometry_msgs::Twist cmd_vel;
void cmd_callback(const geometry_msgs::Twist& cmd_message)
{
    cmd_vel = cmd_message;
}

geometry_msgs::Twist recovery_cmd_vel;
void recovery_cmd_callback(const geometry_msgs::Twist& cmd_message)
{
    recovery_cmd_vel = cmd_message;
}

std::string stop = "stop";
std_msgs::String mode;
void mode_callback(const std_msgs::String& mode_message)
{
    mode = mode_message;
}

std_msgs::String recovery_mode;
void recovery_mode_callback(const std_msgs::String& mode_message)
{
    recovery_mode = mode_message;
}

nav_msgs::Path path;
void path_callback(const nav_msgs::Path& path_message)
{
    path = path_message;
}

int targetWp = 0;
void targetWp_callback(const std_msgs::Int32& targetWp_num)
{
    targetWp = targetWp_num.data;
}

int nowWp = 0;
void nowWp_callback(const std_msgs::Int32& nowWp_num)
{
    nowWp = nowWp_num.data;
}

double quat2yaw(geometry_msgs::Quaternion orientation)
{
    double roll, pitch, yaw;
    tf::Quaternion quat;
    quaternionMsgToTF(orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference

    return yaw;
}

double arrangeAngle(double angle)
{
    while(angle>M_PI)
    {
        angle -= 2*M_PI;
    }
    while(angle<-M_PI)
    {
        angle += 2*M_PI;
    }

    return angle;
}

template<class T> T constrain(T num, T minVal, T maxVal)
{
    if(num > maxVal){
        num = maxVal;
    }
    if(num < minVal){
        num = minVal;
    }

    return num;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "twist_maneger");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string map_id, base_link_id;
    pnh.param<std::string>("map_frame_id", map_id, "map");
    pnh.param<std::string>("base_link_frame_id", base_link_id, "base_link");
    double rate, maxYaw_rate;
    pnh.param<double>("loop_rate", rate, 100);
    pnh.param<double>("maxYaw_rate", maxYaw_rate, 2);

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("twist_maneger/cmd_vel", 10);
    ros::Publisher mode_pub = nh.advertise<std_msgs::String>("mode", 10);
    ros::Publisher vel_rviz_pub = nh.advertise<std_msgs::Float32>("dwa_path_plan/vel_rviz", 10);
    ros::Publisher yawVel_rviz_pub = nh.advertise<std_msgs::Float32>("dwa_path_plan/yawVel_rviz", 10);
    ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 10 , cmd_callback);
    ros::Subscriber recovery_cmd_sub = nh.subscribe("recovery/cmd_vel", 10, recovery_cmd_callback);
    ros::Subscriber mode_sub = nh.subscribe("mode_select/mode", 10 , mode_callback);
    ros::Subscriber recovery_mode_sub = nh.subscribe("recovery/mode", 10 , recovery_mode_callback);
    ros::Subscriber path_sub = nh.subscribe("path", 50, path_callback);
    ros::Subscriber targetWp_sub = nh.subscribe("targetWp", 50, targetWp_callback);
    ros::Subscriber nowWp_sub = nh.subscribe("nowWp", 50, nowWp_callback);

    ros::Rate loop_rate(rate);

    tf_position nowPosition(map_id, base_link_id, rate);
    mode.data = stop;

    bool run_init = true;
    bool recovery_init = false;

    std_msgs::Float32 vel, yawVel;
    while(ros::ok())
    {
        if(path.poses.size()>0){
            if(run_init){
                //run mode
                if(mode.data == "run"){
                    double diffAngle = arrangeAngle(quat2yaw(path.poses[nowWp+(targetWp - nowWp)/2].pose.orientation) - nowPosition.getYaw());

                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = constrain(diffAngle * 1.5, -maxYaw_rate, maxYaw_rate);
                    if(abs(diffAngle) < 1*M_PI/180){
                        run_init = false;
                    }
                }
            }
            //stop mode
            if(mode.data == "stop"){
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                run_init = true;
            }
            //angle adjust
            if(mode.data == "adjust"){
                double diffAngle = arrangeAngle(quat2yaw(path.poses[targetWp].pose.orientation) - nowPosition.getYaw());

                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = constrain(diffAngle * 1.5, -maxYaw_rate, maxYaw_rate);
                if(abs(diffAngle) < 1*M_PI/180){
                    mode.data = "stop";
                }
            }
            if(recovery_mode.data == "safety_stop"){
                mode.data = "safety_stop";
            }
            //recovery mode
            if(recovery_mode.data == "recovery"){
                recovery_init = true;
                mode.data = "recovery";
                cmd_vel = recovery_cmd_vel;
            }
            if(recovery_init){
                if(!(recovery_mode.data == "recovery")){
                    recovery_init = false;

                    run_init = true;
                    mode.data = "run";
                }
            }


            cmd_vel_pub.publish(cmd_vel);
            mode_pub.publish(mode);

            vel.data = cmd_vel.linear.x;
            yawVel.data = cmd_vel.angular.z;
            vel_rviz_pub.publish(vel);
            yawVel_rviz_pub.publish(yawVel);
        }
        

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}