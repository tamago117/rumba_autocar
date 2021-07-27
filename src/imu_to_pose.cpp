#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"

geometry_msgs::PoseStamped ps_msg;
void imu_callback(const sensor_msgs::Imu& imu_msg)
{
    ps_msg.header = imu_msg.header;
    //ps_msg.pose.orientation = imu_msg.orientation;
    ps_msg.pose.orientation.w = imu_msg.orientation.x;
    //ps_msg.pose.orientation.x = imu_msg.orientation.y;
    //ps_msg.pose.orientation.y = imu_msg.orientation.z;
    ps_msg.pose.orientation.x = 0;
    ps_msg.pose.orientation.y = 0;
    ps_msg.pose.orientation.z = imu_msg.orientation.w;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_to_pose");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("imu_pose", 10);
    ros::Subscriber sub = nh.subscribe("imu", 10, imu_callback);

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        pub.publish(ps_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
}