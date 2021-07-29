#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <rumba_autocar/PurePursuit.h>

int nowWp_num = 0;
void nowWp_callback(const std_msgs::Int32& now_wp)
{
    nowWp_num = nowWp.data;
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

    ros::Subscriber nowWp_sub = nh.subscriber("nowWp", 50, nowWp_callback);
    ros::Subscriber path_sub = nh.subscribe("path", 50, path_callback);
    ros::Publisher cmd_pub = nh.advertise("roomba/cmd_vel", 10);

    pp::PurePursuit pure_pursuit;

    ros::Rate loop_rate(100);

    geometry_msgs::Twist cmd_vel;
    while(ros::ok())
    {
        cmd_vel.linear.x = 0.5;
        cmd_vel.angular.z = pure_pursuit.getYawVel(path.poses[nowWp_num], )
    }
    return 0;

}