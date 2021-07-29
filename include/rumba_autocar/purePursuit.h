#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <iostream>
#include <math.h>

namespace pp{
class PurePursuit
{
    enum class car_model
    {
        steer,
        wheel_2
    };

private:
    car_model model;
    double wheel_tred;
    double quat2yaw(geometry_msgs::Quaternion geometry_quat);
public:
    PurePursuit();
    PurePursuit(double wheel_tred_);
    double getYawVel(const geometry_msgs::PoseStamped& nowPos, const geometry_msgs::PoseStamped& tarPos, double forwardV);
    double getYawVel(const geometry_msgs::Pose& nowPos, const geometry_msgs::Pose& tarPos, double forwardV);
};

PurePursuit::PurePursuit()
{
    model = car_model::wheel_2;
}

PurePursuit::PurePursuit(double wheel_tred_):wheel_tred(wheel_tred_)
{
    model = car_model::steer
}

double PurePursuit::getYawVel(const geometry_msgs::PoseStamped& nowPos, const geometry_msgs::PoseStamped& tarPos, double forwardV)
{
    double nowX = nowPos.pose.position.x;//m
    double nowY = nowPos.pose.position.y;//m
    double nowYaw = quat2yaw(nowpos.pose.orientation);//rad
    double tarX = tarPos.pose.position.x;
    double tarY = tarPos.pose.position.y;
    double tarYaw = quat2yaw(tarPos.pose.orientation);

    double L = sqrt(pow(nowX, 2)+pow(nowY, 2)) - sqrt(pow(tarX, 2)+pow(tarY, 2));
    double angle = atan2(tarY-nowY, tarX-nowX) - nowYaw;
    double alfa = angle;

    if(model == car_model::wheel_2){
        return 2.0*forwardV*sin(alfa)/L;
    }else if(model == car_model::steer){
        return atan2(2*wheel_tred*sin(alfa), L);
    }
    

}

double PurePursuit::getYawVel(const geometry_msgs::Pose& nowPos, const geometry_msgs::Pose& tarPos, double forwardV)
{
    double nowX = nowPos.position.x;//m
    double nowY = nowPos.position.y;//m
    double nowYaw = quat2yaw(nowpos.orientation);//rad
    double tarX = tarPos.position.x;
    double tarY = tarPos.position.y;
    double tarYaw = quat2yaw(tarPos.orientation);

    double L = sqrt(pow(nowX, 2)+pow(nowY, 2)) - sqrt(pow(tarX, 2)+pow(tarY, 2));
    double angle = atan2(tarY-nowY, tarX-nowX) - nowYaw;
    double alfa = angle;

    if(model == car_model::wheel_2){
        return 2.0*forwardV*sin(alfa)/L;
    }else if(model == car_model::steer){
        return atan2(2*wheel_tred*sin(alfa), L);
    }

}

double quat2yaw(geometry_msgs::Quaternion geometry_quat)
{
    double roll, pitch, yaw;
    tf::Quaternion quat;

    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference

    return yaw;
}

}//end namespace pp