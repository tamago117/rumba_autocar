#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <string>
#include <vector>
#include <math.h>
#include "rumba_autocar/astar_planning.h"
#include "rumba_autocar/bezier_curve.h"
#include "rumba_autocar/tf_position.h"

geometry_msgs::PoseStamped goalPose;
void poseStamp_callback(const geometry_msgs::PoseStamped& poseStamp_message)
{
    goalPose = poseStamp_message;
}

nav_msgs::OccupancyGrid costmap;
void cost_callback(const nav_msgs::OccupancyGrid& costmap_message)
{
    costmap = costmap_message;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "astar_planning_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    int costmapThreshold;
    pnh.param<int>("costmapThreshold", costmapThreshold, 90);
    double heuristic_gain, resolution;
    pnh.param<double>("heuristic_gain", heuristic_gain, 1.0);
    pnh.param<double>("resolution", resolution, 0.1);
    int approNode;
    double approResolution;
    pnh.param<int>("approximationNode", approNode, 15);
    pnh.param<double>("approximationResolution", approResolution, 10);
    double rate;
    pnh.param<double>("loop_rate", rate, 10);
    std::string map_id, base_link_id;
    pnh.param<std::string>("map_frame_id", map_id, "map");
    pnh.param<std::string>("base_link_frame_id", base_link_id, "base_link");

    ros::Subscriber goalPose_sub = nh.subscribe("astar_plannnig_node/goal", 50, poseStamp_callback);
    ros::Subscriber cost_sub = nh.subscribe("astar_plannnig_node/costmap", 10, cost_callback);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("astar_plannnig_node/path", 10);

    ctr::a_star star(resolution, costmapThreshold, heuristic_gain);
    bezier_curve bezier;
    tf_position nowPosition(map_id, base_link_id, rate);

    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
        if(costmap.data.size()>0){

            std::vector<double> path_x, path_y;
            star.planning(path_x, path_y, nowPosition.getPose(), goalPose.pose, costmap);

            //approximate bezier curve
            int node;
            std::vector<double> r_x, r_y;
            for(int i=0; i<path_x.size()/approNode+1; i++){
                node = i*approNode;
                std::vector<double> p_x, p_y;
                if(i<path_x.size()/approNode){
                    for(int j=0; j<approNode; j++){
                        p_x.push_back(path_x[node+j]);
                        p_y.push_back(path_y[node+j]);
                    }
                }else{
                    if(path_x.size() - node == 0){
                        break;
                    }
                    for(int j=0; j<path_x.size() - node; j++){
                        p_x.push_back(path_x[node+j]);
                        p_y.push_back(path_y[node+j]);
                    }
                }

                for (double t = 0; t <= 1.0; t += (1/approResolution)){
                    std::array<double, 2> point = bezier.getPos(t, p_x, p_y);
                    r_x.push_back(point[0]);
                    r_y.push_back(point[1]);
                }
            }

            //convert path message
            nav_msgs::Path plan_path;
            for(int i=0; i<r_x.size(); i++){
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = map_id;
                pose.pose.position.x = r_x[i];
                pose.pose.position.y = r_y[i];

                plan_path.poses.push_back(pose);
            }
            plan_path.header.frame_id = map_id;
            plan_path.header.stamp = ros::Time::now();

            path_pub.publish(plan_path);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
