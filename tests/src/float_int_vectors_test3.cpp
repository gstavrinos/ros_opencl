#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include "ros_opencl/ros_opencl.hpp"

using namespace std;

ros::Publisher pub;
ros::Publisher pub2;
ros_opencl::ROS_OpenCL roscl;

void imageCallback (const sensor_msgs::Image& msg){
    sensor_msgs::Image im = sensor_msgs::Image(msg);
    sensor_msgs::Image im2 = sensor_msgs::Image(msg);
    std::vector<float> v;
    std::vector<int> v2;
    for (unsigned i = 0; i<msg.data.size(); i++){
        v.push_back(msg.data[i]);
        v2.push_back(msg.data[i]);
    }
    roscl.process(&v, &v2);
    im.data.clear();
    im2.data.clear();
    for (unsigned i = 0; i<msg.data.size(); i++){
        im.data.push_back(v[i]);
        im2.data.push_back(v2[i]);
    }
    pub.publish(im);
    pub2.publish(im2);
}

int main (int argc, char** argv){
    ros::init (argc, argv, "ros_opencl_float_int_vectors_test3");
    ros::NodeHandle nh;
    string kernel_filename;
    string image_topic;
    string result_topic;
    string result_topic2;

    nh.param("ros_opencl_float_int_vectors_test3/kernel_filename", kernel_filename, string("test_kernels.cl"));
    nh.param("ros_opencl_float_int_vectors_test3/image_topic", image_topic, string("/usb_cam/image_raw"));
    nh.param("ros_opencl_float_int_vectors_test3/result_topic", result_topic, string("ros_opencl_float_int_vectors_test3/result"));
    nh.param("ros_opencl_float_int_vectors_test3/result_topic", result_topic2, string("ros_opencl_float_int_vectors_test3/result2"));

    string full_kernel_path = ros::package::getPath("ros_opencl") + "/tests/kernels/" + kernel_filename;

    roscl = new ros_opencl::ROS_OpenCL(full_kernel_path, "doubleGrayScaleFloatInt");

    pub = nh.advertise<sensor_msgs::Image>(result_topic, 1);
    pub2 = nh.advertise<sensor_msgs::Image>(result_topic2, 1);
    ros::Subscriber s = nh.subscribe (image_topic, 1, imageCallback);

    while(ros::ok()){
        ros::spin();
    }
}