#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <string>
#include <std_msgs/Int32.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_broadcaster.h>

int targetWp = 0;
void targetWp_callback(const std_msgs::Int32& targetWp_num)
{
    targetWp = targetWp_num.data;
}

nav_msgs::Path path;
void path_callback(const nav_msgs::Path& path_message)
{
    path = path_message;

}

nav_msgs::OccupancyGrid costmap;
void cost_callback(const nav_msgs::OccupancyGrid& costmap_message)
{
    costmap = costmap_message;

}

std_msgs::ColorRGBA set_color(double r, double g, double b, double a)
{
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    return color;
}
std_msgs::ColorRGBA green = set_color(0.0, 1.0, 0.0, 1.0);
std_msgs::ColorRGBA red = set_color(1.0, 0.0, 0.0, 1.0);

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

double getCost(double x, double y)
{
    //x = costmap.info.origin.position.x + x_grid*costmap.info.resolution
    int x_grid = abs((x - costmap.info.origin.position.x)/costmap.info.resolution);
    int y_grid = abs((y - costmap.info.origin.position.y)/costmap.info.resolution);

    int data_pos = costmap.info.width * y_grid + x_grid;

    if(data_pos > 0 && data_pos < costmap.data.size()){
        return costmap.data[data_pos];
    }else{
        return -INFINITY;
    }
}

template <class T> std::vector<T> normalize(std::vector<T>& num, T amin, T amax)
{
    if(num.empty()){
        return num;
    }

    T xmin = *std::min_element(std::begin(num), std::end(num));
    T xmax = *std::max_element(std::begin(num), std::end(num));

    //0 division
    if(xmin == xmax){
        if(xmin == 0){
            return num;
        }else{
            for(auto& n : num){
                n = n/n; //1
            }
            return num;
        }
    }

    for(auto& n : num){
        n = (amax - amin)*(n - xmin) / (xmax - xmin);
    }
    return num;
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "state_lattice_planner");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~/state_lattice_planner");

    std::string base_link_id, map_id;
    //pnh.param<std::string>("base_link_frame_id", base_link_id, "base_link");
    pnh.param<std::string>("map_frame_id", map_id, "map");
    double l_width, v_width;
    pnh.param<double>("lane_width", l_width, 2.5);
    pnh.param<double>("vehicle_width", v_width, 0.8);
    int sampling_num;
    pnh.param<int>("sampling_number", sampling_num, 7);
    double distanceCost_gain, obstacleCost_gain;
    pnh.param<double>("distanceCost_gain", distanceCost_gain, 1.0);
    pnh.param<double>("obstacleCost_gain", obstacleCost_gain, 1.0);
    double rate;
    pnh.param<double>("loop_rate", rate, 30);

    ros::Subscriber targetWp_sub = nh.subscribe("targetWp", 50, targetWp_callback);
    ros::Subscriber path_sub = nh.subscribe("wayPoint/path", 50, path_callback);
    ros::Subscriber cost_sub = nh.subscribe("state_lattice_planner/costmap", 10, cost_callback);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("state_lattice_planner/path", 10);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("state_lattice_planner/marker_array", 10);

    ros::Rate loop_rate(rate);

    const int repeatNum = 3;
    while(ros::ok())
    {
        if(path.poses.size() > 0){
            std::vector<double> lane_center_x;
            std::vector<double> lane_center_y;
            std::vector<double> lane_heading;
            //calculate cost using 3 way points
            for(int i=-1; i<repeatNum-1; i++){
                int index = constrain(targetWp+i, 0, path.poses.size());

                lane_center_x.push_back(path.poses[index].pose.position.x);
                lane_center_y.push_back(path.poses[index].pose.position.y);
                lane_heading.push_back(arrangeAngle(quat2yaw(path.poses[index].pose.orientation)));
            }
            /*double lane_center_x = path.poses[targetWp].pose.position.x;
            double lane_center_y = path.poses[targetWp].pose.position.y;
            double lane_heading = quat2yaw(path.poses[targetWp].pose.orientation);
            lane_heading = arrangeAngle(lane_heading);*/

            //set target point and calculate each costs
            std::vector<std::vector<double>> targetPoint;
            std::vector<double> distances;
            std::vector<double> costs;

            for(int i=0; i<sampling_num; i++){
                //calculate cost using 3 way points
                double distance = 0;
                double cost = 0;
                for(int j=0; j<repeatNum; j++){
                    //set target point
                    double delta = (l_width - v_width)*i/(sampling_num-1) - 0.5*(l_width -v_width);
                    double xf = lane_center_x[j] - delta*sin(lane_heading[j]);
                    double yf = lane_center_y[j] + delta*cos(lane_heading[j]);
                    std::vector<double> temp{xf, yf, lane_heading[j]};

                    //set array target way point
                    if(j==1){
                        targetPoint.push_back(temp);
                    }

                    //calculate cost
                    distance += sqrt(pow(xf - lane_center_x[j], 2)+pow(yf - lane_center_y[j], 2));
                    cost += getCost(xf, yf);
                }
                
                distances.push_back(distance);
                costs.push_back(cost);
            }

            //-> 0~100*3
            distances = normalize(distances, 0.0, 100.0*repeatNum);

            //select best path
            double minCost = INFINITY;
            int bestPathNum;
            for(int i=0; i<sampling_num; i++){
                double final_cost = distanceCost_gain * distances[i] + obstacleCost_gain * costs[i];

                if(final_cost < minCost){
                    minCost = final_cost;
                    bestPathNum = i;
                }
            }


            //create marker array
            // config arrow shape
            geometry_msgs::Vector3 arrow;
            arrow.x = 0.02;
            arrow.y = 0.04;
            arrow.z = 0.1;

            visualization_msgs::MarkerArray marker_array;
            marker_array.markers.resize(sampling_num);
            for(int i=0; i<sampling_num; i++){
                geometry_msgs::Point linear_start;
                linear_start.x = targetPoint[i][0];
                linear_start.y = targetPoint[i][1];
                linear_start.z = 0.1;
                geometry_msgs::Point linear_end;
                linear_end.x = 0.2 * cos(lane_heading[1]) + targetPoint[i][0];
                linear_end.y = 0.2 * sin(lane_heading[1]) + targetPoint[i][1];
                linear_end.z = 0.1;


                marker_array.markers[i].header.frame_id = map_id;
                marker_array.markers[i].header.stamp = ros::Time::now();
                marker_array.markers[i].ns = "plannig_point";
                marker_array.markers[i].id = i;
                marker_array.markers[i].lifetime = ros::Duration();

                marker_array.markers[i].type = visualization_msgs::Marker::ARROW;
                marker_array.markers[i].action = visualization_msgs::Marker::ADD;
                marker_array.markers[i].scale = arrow;

                marker_array.markers[i].points.resize(2);
                marker_array.markers[i].points[0] = linear_start;
                marker_array.markers[i].points[1] = linear_end;

                if(i == bestPathNum){
                    marker_array.markers[i].color = red;
                }else{
                    marker_array.markers[i].color = green;
                }
            }

            //best path head to target point
            nav_msgs::Path plan_path;
            plan_path.header = path.header;
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = targetPoint[bestPathNum][0];
            pose.pose.position.y = targetPoint[bestPathNum][1];
            pose.pose.position.z = 0;
            pose.pose.orientation = path.poses[targetWp].pose.orientation;

            plan_path.poses.push_back(pose);

            path_pub.publish(plan_path);
            marker_pub.publish(marker_array);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}