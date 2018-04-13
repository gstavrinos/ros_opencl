#include <ros/ros.h>
#include <ros/package.h>
#include "ros_opencl/ros_opencl.hpp"
#include <sensor_msgs/PointCloud2.h>

using namespace std;

ros::Publisher pub;
ros_opencl::ROS_OpenCL roscl;

void cloudCallback (sensor_msgs::PointCloud2::Ptr msg){
    roscl.process(msg);
    pub.publish(msg);
}

int main (int argc, char** argv){
    ros::init (argc, argv, "ros_opencl_pointcloud_test2");
    ros::NodeHandle nh;
    string kernel_filename;
    string cloud_topic;
    string result_topic;

    nh.param("ros_opencl_pointcloud_test2/kernel_filename", kernel_filename, string("test_kernels.cl"));
    nh.param("ros_opencl_pointcloud_test2/cloud_topic", cloud_topic, string("/zed/point_cloud/cloud_registered"));
    nh.param("ros_opencl_pointcloud_test2/result_topic", result_topic, string("ros_opencl_pointcloud_test2/result"));

    string full_kernel_path = ros::package::getPath("ros_opencl") + "/tests/kernels/" + kernel_filename;

    roscl = ros_opencl::ROS_OpenCL(full_kernel_path, "cubedPointcloud");

    pub = nh.advertise<sensor_msgs::PointCloud2>(result_topic, 1);
    ros::Subscriber s = nh.subscribe (cloud_topic, 1, cloudCallback);

    while(ros::ok()){
        ros::spin();
    }

    roscl.clean();
}