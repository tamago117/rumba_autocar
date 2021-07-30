#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <rumba_autocar/PurePursuit.h>
#include <rumba_autocar/tf_position.h>
#include <string>

int targetWp = 0;
void nowWp_callback(const std_msgs::Int32& targetWp_num)
{
    targetWp = targetWp_num.data;
}

nav_msgs::Path path;
void path_callback(const nav_msgs::Path& path_message)
{
    path = path_message;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_track");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string base_link_id, map_id;
    pnh.param<std::string>("base_link_frame_id", base_link_id, "base_link");
    pnh.param<std::string>("map_frame_id", map_id, "map");
    double rate;
    pnh.param<double>("rate", rate, 100);
    double max_angular_vel;
    pnh.param<double>("max_angular_vel", max_angular_vel, 1);

    ros::Subscriber nowWp_sub = nh.subscribe("nowWp", 50, nowWp_callback);
    ros::Subscriber path_sub = nh.subscribe("path", 50, path_callback);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("roomba/cmd_vel", 10);

    pp::PurePursuit pure_pursuit(max_angular_vel);
    tf_position nowPosition(map_id, base_link_id, rate);

    ros::Rate loop_rate(rate);

    geometry_msgs::Twist cmd_vel;
    while(ros::ok())
    {

        cmd_vel.linear.x = 0.5;
        cmd_vel.angular.z = pure_pursuit.getYawVel(nowPosition.getPoseStamped(), path.poses[targetWp] , cmd_vel.linear.x);

        cmd_pub.publish(cmd_vel);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}