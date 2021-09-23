/**
 * @brief
 * @author Michikuni Eguchi
 * @date 2021.9.4
 * @ref https://ppdr.softether.net/ros-pointcloud-zatxy
 */
#include <numeric> 		//  to calculate average
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <string>
#include <limits>
#include "rumba_autocar/tf_position.h"

const std::string safety_stop = "safety_stop";
const std::string recovery = "recovery";

class safety_limit
{
public:
    safety_limit();
private:
    ros::NodeHandle nh;

    // publisher
    ros::Publisher cmd_pub;
    ros::Publisher mode_pub;
    // subscribe
    ros::Subscriber cmd_sub;
    ros::Subscriber pc2_sub;
    ros::Subscriber cost_sub;

    //function
    pcl::PointCloud<pcl::PointXYZ>::Ptr pass_through(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string iaxis, double imin, double imax);
    void cmd_callback(const geometry_msgs::Twist& cmd_message);
    void callback_knn(const sensor_msgs::PointCloud2ConstPtr& pc2);
    void cost_callback(const nav_msgs::OccupancyGrid& costmap_message);
    int getCost(double x, double y);
    bool check_around_obstacle(const geometry_msgs::Pose& nowPos);

    double g_xmin, g_xmax, g_ymin, g_ymax;
    double robotRadius;
    int g_max_nn; //何点見つかったら探索を打ち切るか。0にすると打ち切らない
    int rate;
    double dt;
    double recovery_start_time;

    double lowMode_speedRatio;
    bool dangerous_potential;
    std::string map_id, base_link_id;

    int stop_count;
    geometry_msgs::Twist cmd_vel_limit;
    std_msgs::String mode;

    pcl::PointCloud<pcl::PointXYZ>::Ptr gfiltered;
    geometry_msgs::Twist cmd_vel;
    nav_msgs::OccupancyGrid costmap;
};

safety_limit::safety_limit() :  stop_count(0)
{
    //publisher
    cmd_pub = nh.advertise<geometry_msgs::Twist>("safety_limit/cmd_vel_out", 10);
    mode_pub = nh.advertise<std_msgs::String>("safety_limit/mode", 10);
    // subscriber
    cmd_sub = nh.subscribe("safety_limit/cmd_vel_in", 10, &safety_limit::cmd_callback, this);
    pc2_sub = nh.subscribe<sensor_msgs::PointCloud2>("laser2pc/pc2", 1, &safety_limit::callback_knn, this);
    cost_sub = nh.subscribe("safety_limit/costmap", 10, &safety_limit::cost_callback, this);

    //set parameter from ros command
    ros::NodeHandle pnh("~");
    pnh.param<double>("x_max", g_xmax, 5.0);
    pnh.param<double>("x_min", g_xmin, -0.5);
    pnh.param<double>("y_max", g_ymax, 0.4);
    pnh.param<double>("y_min", g_ymin, -0.4);
    pnh.param<int>("max_nn", g_max_nn, 100);
    pnh.param<double>("robot_radius", robotRadius, 0.5);
    pnh.param<int>("loop_rate", rate, 50);
    pnh.param<double>("dt", dt, 0.3);
    pnh.param<double>("recovery_start_time", recovery_start_time, 2.0);
    pnh.param<double>("lowMode_speedRatio", lowMode_speedRatio, 0.5);
    pnh.param<bool>("dangerous_potential", dangerous_potential, true);
    pnh.param<std::string>("map_frame_id", map_id, "map");
    pnh.param<std::string>("base_link_frame_id", base_link_id, "base_link");

}

// pass through filter
pcl::PointCloud<pcl::PointXYZ>::Ptr safety_limit::pass_through(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string iaxis, double imin, double imax)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName (iaxis);
    // pass.setFilterFieldName ("x");
    pass.setFilterLimits (imin, imax);
    pass.filter(*cloud_filtered);
    // printf("%f\n",cloud->points.size());
    return cloud_filtered;
}

void safety_limit::cmd_callback(const geometry_msgs::Twist& cmd_message)
{
    cmd_vel = cmd_message;
}

// callback
void safety_limit::callback_knn(const sensor_msgs::PointCloud2ConstPtr& pc2){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nan (new pcl::PointCloud<pcl::PointXYZ>); // NaN値あり
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); // NaN値なし

    pcl::fromROSMsg(*pc2, *cloud_nan);

    // NaN値が入ってるといろいろ面倒なので除去
    std::vector<int> nan_index;
    pcl::removeNaNFromPointCloud(*cloud_nan, *cloud, nan_index);

    // Filtered by each axis"
    cloud = pass_through(cloud, "x", g_xmin, g_xmax);
    cloud = pass_through(cloud, "y", g_ymin, g_ymax);
    gfiltered = cloud;

    cmd_vel_limit = cmd_vel;
    //近傍点探索 and get average distance
    if(cloud->size()>0){
        //x座標とy座標だけをコピーしたPointCloudを作る
        pcl::PointCloud<pcl::PointXY>::Ptr cloud2d (new pcl::PointCloud<pcl::PointXY>); // NaN値なし
        cloud2d->points.resize(cloud->size());
        for(int i=0; i<cloud->points.size(); i++){
            cloud2d->points[i].x = cloud->points[i].x;
            cloud2d->points[i].y = cloud->points[i].y;
        }

        //treeも2Dで作る
        pcl::KdTreeFLANN<pcl::PointXY>::Ptr tree2d (new pcl::KdTreeFLANN<pcl::PointXY>);
        tree2d->setInputCloud(cloud2d);

        //近傍点探索に使うパラメータと結果が入る変数
        double radius = g_xmax; //半径r
        unsigned int max_nn = g_max_nn; //何点見つかったら探索を打ち切るか。0にすると打ち切らない
        std::vector<int> k_indices; //範囲内の点のインデックスが入る
        std::vector<float> k_distances; //範囲内の点の距離が入る

        //中心座標
        pcl::PointXY p;
        p.x = 0.0;
        p.y = 0.0;

        //2Dで近傍点探索
        tree2d->radiusSearch(p, radius, k_indices, k_distances, max_nn);

        // error handling
        if(k_indices.size() == 0) {
            return;
        }

        // get center of each nearest neighbors
        if(!(max_nn > 1)){
            ROS_ERROR("max_nn must be setted 2 more over");
        }
        ROS_ASSERT(max_nn > 1);

        double next_robot_yaw = cmd_vel.angular.z * dt;
        double next_robot_x = cmd_vel.linear.x * cos(next_robot_yaw) * dt;
        double next_robot_y = cmd_vel.linear.y * sin(next_robot_yaw) * dt;

        mode.data = "run";
        for(int i=0; i<k_indices.size(); i++){
            double point_x = cloud->points[k_indices[i]].x;
            double point_y = cloud->points[k_indices[i]].y;
            double r = sqrt(pow(point_x - next_robot_x, 2) + pow(point_y - next_robot_y, 2));

            //次点のロボットのエリアに点群が入るようならストップ
            if(r < robotRadius){
                cmd_vel_limit.linear.x = 0;
                cmd_vel_limit.angular.z = 0;

                ROS_INFO("robot safety stop");
                mode.data = safety_stop;

                //一定のカウントでrecoveryに入る
                stop_count++;
                if(stop_count > (int)(recovery_start_time*rate)){
                    ROS_INFO("robot recovery behaviour");
                    mode.data = recovery;
                    if(stop_count > (int)((recovery_start_time+dt)*rate)){
                        stop_count = 0;
                    }
                }

                cmd_pub.publish(cmd_vel_limit);
                mode_pub.publish(mode);
                return;
            }
        }
        //衝突判定に引っかからなければcountを初期化
        stop_count = 0;

        //potential check
        static tf_position nowPosition(map_id, base_link_id, rate);
        if(costmap.data.size()>0){
            if(dangerous_potential){
                if(check_around_obstacle(nowPosition.getPose())){
                    cmd_vel_limit.linear.x *= lowMode_speedRatio;
                    cmd_vel_limit.angular.z *= lowMode_speedRatio;
                }
            }
        }

        cmd_pub.publish(cmd_vel_limit);
        mode_pub.publish(mode);

    }
}

void safety_limit::cost_callback(const nav_msgs::OccupancyGrid& costmap_message)
{
    costmap = costmap_message;
}

int safety_limit::getCost(double x, double y)
{
    //x = costmap.info.origin.position.x + x_grid*costmap.info.resolution
    int x_grid = abs((x - costmap.info.origin.position.x)/costmap.info.resolution);
    int y_grid = abs((y - costmap.info.origin.position.y)/costmap.info.resolution);

    int data_pos = costmap.info.width * y_grid + x_grid;

    if(data_pos > 0 && data_pos < costmap.data.size()){
        return costmap.data[data_pos];
    }else{
        return std::numeric_limits<int>::min();
    }
}

bool safety_limit::check_around_obstacle(const geometry_msgs::Pose& nowPos)
{
    if(getCost(nowPos.position.x, nowPos.position.y)>10){
        return true;
    }else{
        return false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "safety_limit");
    safety_limit sl;;

    ros::spin();
    return 0;

}