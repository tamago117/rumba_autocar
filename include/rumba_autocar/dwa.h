/**
* @file DWA.h
* @brief dynamic window approach
* @author Michikuni Eguchi
* @date 2021.8.17
* @details reference : https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/DynamicWindowApproach/dynamic_window_approach.py
*                      https://qiita.com/MENDY/items/16343a00d37d14234437
**/

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>

namespace ctr{

class DWA
{
public:
    struct motionState{
        double x;
        double y;
        double yawAngle;
        double vel;
        double yawVel;
    };

    DWA();
    //set now state of robot, goal posion value, object position
    void dwa_controll(const motionState& motion_state, const geometry_msgs::Pose& goal_, const nav_msgs::OccupancyGrid& costmap_);
    void get_result(geometry_msgs::Twist& cmd_vel, nav_msgs::Path& resultTrajectory);
private:

    ///////dwa config///////
    double maxSpeed;
    double minSpeed;
    double maxYaw_rate;
    double maxAccel;
    double maxYawVel_rate;
    double v_resolution;
    double w_resolution;
    double dt;
    double predictTime;
    double angle_gain;
    double distance_gain;
    double obstacleCost_gain;
    double speedCost_gain;
    double robot_stuck_value;
    double robot_radius;
    /////////////////////////
    std::string map_id;

    motionState state;
    geometry_msgs::Pose goal;
    nav_msgs::OccupancyGrid costmap;

    geometry_msgs::Twist best_cmd_vel;
    nav_msgs::Path bestTrajectory;

    //set dynamic window
    std::vector<double> calc_dynamicWindow();
    //calc best velocity and trajectory
    void calc_controll_and_trajectory(std::vector<double> dw);
    //predict trajectory with velocity and w
    std::vector<motionState> predict_trajectory(double v, double w);
    //calculate position in trajectory from velocity, angular velocity and dt
    motionState motion(motionState predict_state, double v, double w);
    //calculate goal angle cost from goal position and robot position
    double goalAngleCost(const std::vector<motionState> trajectory);
    //calculate goal distance cost from goal position and robot position
    double goalDistanceCost(const std::vector<motionState> trajectory);
    //calculate obstacle cost from object position
    double obstacleCost(const std::vector<motionState> trajectory);
    //calculate speed cost from robot velocity
    double speedCost(const std::vector<motionState> trajectory);
    //get cost from costmap(occupancy grid)
    double getCost(double x, double y);
    //arrange angle -pi ~ +pi
    double arrangeAngle(double angle);
    //normalize (0~1)
    template <class T> std::vector<T> normalize(std::vector<T>& num);
    //rpy2quaternion
    geometry_msgs::Quaternion rpy_to_geometry_quat(double roll, double pitch, double yaw);

};

DWA::DWA()
{
    ros::NodeHandle pnh("~");
    //set config value
    pnh.param<double>("maxSpeed", maxSpeed, 2.0);
    pnh.param<double>("minSpeed", maxSpeed, -0.5);
    pnh.param<double>("maxYaw_rate", maxYaw_rate, 40);
    maxYaw_rate *= M_PI/180;
    pnh.param<double>("maxAccel", maxAccel, 0.4);
    pnh.param<double>("maxYawVel_rate", maxYawVel_rate, 40);
    maxYawVel_rate *= M_PI/180;
    pnh.param<double>("v_resolution", v_resolution, 0.1);
    pnh.param<double>("w_resolution", w_resolution, 1);
    w_resolution *= M_PI/180;
    pnh.param<double>("dt", dt, 0.3);
    pnh.param<double>("predictTime", predictTime, 3.0);
    pnh.param<double>("angle_gain", angle_gain, 0.15);
    pnh.param<double>("distance_gain", distance_gain, 0.2);
    pnh.param<double>("obstacleCost_gain", obstacleCost_gain, 0.5);
    pnh.param<double>("speedCost_gain", speedCost_gain, 0.2);
    pnh.param<double>("robot_stuck_value", robot_stuck_value, 0.001);
    pnh.param<double>("robot_radius", robot_radius, 1.0);
    pnh.param<std::string>("map_frame_id", map_id, "map");
}

void DWA::dwa_controll(const motionState& motion_state, const geometry_msgs::Pose& goal_, const nav_msgs::OccupancyGrid& costmap_)
{
    state = motion_state;
    goal = goal_;
    costmap = costmap_;

    std::vector<double> dynamic_window = calc_dynamicWindow();
    calc_controll_and_trajectory(dynamic_window);

}

void DWA::get_result(geometry_msgs::Twist& cmd_vel, nav_msgs::Path& resultTrajectory)
{
    cmd_vel = best_cmd_vel;
    resultTrajectory = bestTrajectory;
}

std::vector<double> DWA::calc_dynamicWindow()
{
    std::vector<double> vs{minSpeed, maxSpeed, -maxYaw_rate, maxYaw_rate};

    std::vector<double> vd{state.vel - maxAccel*dt,
                           state.vel + maxAccel*dt,
                           state.yawVel - maxYawVel_rate*dt,
                           state.yawVel + maxYawVel_rate*dt};

    std::vector<double> dw{std::max(vs[0], vs[0]),
                           std::min(vs[1], vs[1]),
                           std::max(vs[2], vs[2]),
                           std::min(vs[3], vs[3])};

    return dw;
}

void DWA::calc_controll_and_trajectory(std::vector<double> dw)
{
    double goalAngle_cost, goalDistance_cost, obstacle_cost, final_cost;
    std::vector<double> goalAngle_costs, goalDistance_costs, obstacle_costs;
    std::vector<std::vector<double>> velocity_array; //[v,w],[v,w].....
    std::vector<double> tmp_array(2); //[v, w]
    std::vector<motionState> trajectory; //[x, y, angle, v, w],....
    geometry_msgs::PoseStamped pose;


    double minCost = INFINITY;

    //search all trajectory in dynamic window
    for(double v = dw[0]; v <= dw[1]; v += v_resolution){
        for(double w = dw[2]; w <= dw[3]; w += w_resolution){
            trajectory = predict_trajectory(v, w);

            tmp_array[0] = v;
            tmp_array[1] = w;
            velocity_array.push_back(tmp_array);

            //calculate each cost
            goalAngle_costs.push_back(goalAngleCost(trajectory));
            goalDistance_costs.push_back(goalDistanceCost(trajectory));
            obstacle_costs.push_back(obstacleCost(trajectory));
        }
    }

    //cost normalize
    goalAngle_costs = normalize(goalAngle_costs);
    goalDistance_costs = normalize(goalDistance_costs);
    obstacle_costs = normalize(obstacle_costs);

    //select largest cost trajectory
    for(int i=0; i<goalAngle_costs.size(); i++){
        goalAngle_cost = angle_gain * goalAngle_costs[i];
        goalDistance_cost = distance_gain * goalDistance_costs[i];
        obstacle_cost = obstacleCost_gain * obstacle_costs[i];

        final_cost = goalAngle_cost + goalDistance_cost + obstacle_cost;

        if(final_cost < minCost){
            minCost = final_cost;

            best_cmd_vel.linear.x = velocity_array[i][0];
            best_cmd_vel.angular.z = velocity_array[i][1];

            trajectory = predict_trajectory(velocity_array[i][0], velocity_array[i][1]);
            bestTrajectory.header.frame_id = map_id;
            bestTrajectory.header.stamp = ros::Time::now();

            for(const auto& pos : trajectory)
            {
                pose.header.frame_id = map_id;
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = pos.x;
                pose.pose.position.y = pos.y;
                pose.pose.position.z = 0;
                pose.pose.orientation = rpy_to_geometry_quat(0, 0, pos.yawAngle);

                bestTrajectory.poses.push_back(pose);
            }

            //if robot stuck, avoid stuck by big turn
            if(best_cmd_vel.linear.x < robot_stuck_value && state.vel < robot_stuck_value){
                best_cmd_vel.angular.z = maxYawVel_rate;
            }
        }
    }



}

std::vector<DWA::motionState> DWA::predict_trajectory(double v, double w)
{
    std::vector<motionState> trajectory;
    motionState predict_state = state;

    for(double t = 0; t <= predictTime; t += dt){
        predict_state = motion(predict_state, v, w);
        trajectory.push_back(predict_state);
    }

    return trajectory;

}

DWA::motionState DWA::motion(motionState predict_state, double v, double w)
{
    predict_state.yawAngle += w*dt;
    predict_state.x += v*cos(predict_state.yawAngle)*dt;
    predict_state.y += v*sin(predict_state.yawAngle)*dt;
    predict_state.vel = v;
    predict_state.yawVel = w;

    return predict_state;
}

double DWA::goalAngleCost(const std::vector<motionState> trajectory)
{
    double dx = goal.position.x - trajectory.back().x;
    double dy = goal.position.y - trajectory.back().y;
    double cost_angle = atan2(dy, dx) - trajectory.back().yawAngle;
    cost_angle = abs(arrangeAngle(cost_angle));

    return cost_angle;
}

double DWA::goalDistanceCost(const std::vector<motionState> trajectory)
{
    double dx = goal.position.x - trajectory.back().x;
    double dy = goal.position.y - trajectory.back().y;
    double distance = sqrt(pow(dx, 2) + pow(dy, 2));

    return distance;
}

double DWA::obstacleCost(const std::vector<motionState> trajectory)
{
    double cost;
    double maxCost = -INFINITY;

    for(int i = trajectory.size()-1; i >= 0; i--){
        cost = getCost(trajectory[i].x, trajectory[i].y);

        if(cost > maxCost){
            maxCost = cost;
            if(maxCost == 100){
                break;
            }
        }
    }

    return maxCost;
}

double DWA::speedCost(const std::vector<motionState> trajectory)
{
    return maxSpeed - trajectory.back().vel;
}

double DWA::getCost(double x, double y)
{
    //x = costmap.info.origin.position.x + x_grid*costmap.info.resolution
    double x_grid = (x-costmap.info.origin.position.x)/costmap.info.resolution;
    double y_grid = (y-costmap.info.origin.position.y)/costmap.info.resolution;

    double data_pos = (costmap.info.width/costmap.info.resolution)*y_grid + x_grid;
    double cost = costmap.data[data_pos];

    return cost;
}

double DWA::arrangeAngle(double angle)
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

template <class T> std::vector<T> DWA::normalize(std::vector<T>& num)
{
    T xmin = *std::min_element(std::begin(num), std::end(num));
    T xmax = *std::max_element(std::begin(num), std::end(num));

    for(auto& n : num){
        n = (n - xmin) / (xmax - xmin);
    }

    return num;
}

geometry_msgs::Quaternion DWA::rpy_to_geometry_quat(double roll, double pitch, double yaw){
    tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    return geometry_quat;
}

} //namespace ctr