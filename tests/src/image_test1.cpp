#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include "ros_opencl/ros_opencl.hpp"

using namespace std;

ros::Publisher pub;
ros_opencl::ROS_OpenCL roscl;

void imageCallback (const sensor_msgs::Image& msg){
    pub.publish(roscl.process(msg));
}

int main (int argc, char** argv){
    ros::init (argc, argv, "ros_opencl_image_test1");
    ros::NodeHandle nh;
    string kernel_filename;
    string image_topic;
    string result_topic;

    nh.param("ros_opencl_image_test1/kernel_filename", kernel_filename, string("test_kernels.cl"));
    nh.param("ros_opencl_image_test1/image_topic", image_topic, string("/usb_cam/image_raw"));
    nh.param("ros_opencl_image_test1/result_topic", result_topic, string("ros_opencl_image_test1/result"));

    string full_kernel_path = ros::package::getPath("ros_opencl") + "/tests/kernels/" + kernel_filename;

    roscl = ros_opencl::ROS_OpenCL(full_kernel_path, "grayScale");

    pub = nh.advertise<sensor_msgs::Image>(result_topic, 1);
    ros::Subscriber s = nh.subscribe (image_topic, 1, imageCallback);

    while(ros::ok()){
        ros::spin();
    }

    roscl.clean();
}