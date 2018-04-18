#include <ros/ros.h>
#include <ros/package.h>
#include "ros_opencl/ros_opencl.hpp"
#include <sensor_msgs/LaserScan.h>

using namespace std;

ros::Publisher pub;
ros_opencl::ROS_OpenCL roscl;

void scanCallback (const sensor_msgs::LaserScan& msg){
    sensor_msgs::LaserScan ls = sensor_msgs::LaserScan(msg);
    ls.header.stamp = ros::Time::now();
    ls.header.frame_id = "laser";
    ls.ranges = roscl.process(msg.ranges);
    pub.publish(ls);
}

int main (int argc, char** argv){
    ros::init (argc, argv, "ros_opencl_float_vector_test1");
    ros::NodeHandle nh;
    string kernel_filename;
    string scan_topic;
    string result_topic;

    nh.param("ros_opencl_float_vector_test1/kernel_filename", kernel_filename, string("test_kernels.cl"));
    nh.param("ros_opencl_float_vector_test1/scan_topic", scan_topic, string("/scan"));
    nh.param("ros_opencl_float_vector_test1/result_topic", result_topic, string("ros_opencl_float_vector_test1/result"));

    string full_kernel_path = ros::package::getPath("ros_opencl") + "/tests/kernels/" + kernel_filename;

    roscl = ros_opencl::ROS_OpenCL(full_kernel_path, "closerLaserScan");

    pub = nh.advertise<sensor_msgs::LaserScan>(result_topic, 1);
    ros::Subscriber s = nh.subscribe (scan_topic, 1, scanCallback);

    while(ros::ok()){
        ros::spin();
    }

    roscl.clean();
}