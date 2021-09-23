#include <ros/ros.h>
#include <math.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
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

geometry_msgs::PoseStamped targetWpPose;
void targetWpPose_callback(const geometry_msgs::PoseStamped& poseStamp_message)
{
    targetWpPose = poseStamp_message;
}

geometry_msgs::Pose targetPose;
void targetPose_callback(const geometry_msgs::Pose& pose_message)
{
    targetPose = pose_message;
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
    double rate, max_angular_vel;
    pnh.param<double>("loop_rate", rate, 100);
    pnh.param<double>("max_angular_vel", max_angular_vel, 2);

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("twist_maneger/cmd_vel", 10);
    ros::Publisher mode_pub = nh.advertise<std_msgs::String>("mode", 10);
    ros::Publisher vel_rviz_pub = nh.advertise<std_msgs::Float32>("dwa_path_plan/vel_rviz", 10);
    ros::Publisher yawVel_rviz_pub = nh.advertise<std_msgs::Float32>("dwa_path_plan/yawVel_rviz", 10);
    ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 10 , cmd_callback);
    ros::Subscriber recovery_cmd_sub = nh.subscribe("recovery/cmd_vel", 10, recovery_cmd_callback);
    ros::Subscriber mode_sub = nh.subscribe("mode_select/mode", 10 , mode_callback);
    ros::Subscriber recovery_mode_sub = nh.subscribe("recovery/mode", 10 , recovery_mode_callback);
    ros::Subscriber targetWpPose_sub = nh.subscribe("twist_maneger/targetWpPose_in", 50, targetWpPose_callback);
    ros::Subscriber targetPose_sub = nh.subscribe("twist_maneger/targetPose_in", 10 , targetPose_callback);

    ros::Rate loop_rate(rate);

    tf_position nowPosition(map_id, base_link_id, rate);
    mode.data = stop;

    bool run_init = true;
    bool recovery_init = false;

    std_msgs::Float32 vel, yawVel;
    while(ros::ok())
    {
        if(run_init){
            //run mode
            if(mode.data == "run"){
                double dx = targetPose.position.x - nowPosition.getPose().position.x;
                double dy = targetPose.position.y - nowPosition.getPose().position.y;
                double targetAngle = atan2(dy, dx);
                double diffAngle = arrangeAngle(targetAngle - nowPosition.getYaw());

                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = constrain(diffAngle * 1.5, -max_angular_vel, max_angular_vel);
                if(abs(diffAngle) < 10*M_PI/180){
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
            double diffAngle = arrangeAngle(quat2yaw(targetWpPose.pose.orientation) - nowPosition.getYaw());

            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = constrain(diffAngle * 1.5, -max_angular_vel, max_angular_vel);
            if(abs(diffAngle) < 1*M_PI/180){
                mode.data = "stop";
            }
        }
        if(recovery_mode.data == "safety_stop"){
            mode.data = "safety_stop";
        }
        if(recovery_mode.data == "run" && mode.data == "safety_stop"){
            mode.data = "run";
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
        

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}