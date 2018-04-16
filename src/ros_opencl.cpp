#include "ros_opencl/ros_opencl.hpp"

namespace ros_opencl {

    // Private Methods

    std::string ROS_OpenCL::getPlatformName (const cl_platform_id id){
        size_t size = 0;
        clGetPlatformInfo (id, CL_PLATFORM_NAME, 0, NULL, &size);

        std::string result;
        result.resize (size);
        clGetPlatformInfo (id, CL_PLATFORM_NAME, size, const_cast<char*> (result.data ()), NULL);

        return result;
    }

    std::string ROS_OpenCL::getDeviceName (const cl_device_id id){
        size_t size = 0;
        clGetDeviceInfo (id, CL_DEVICE_NAME, 0, NULL, &size);

        std::string result;
        result.resize (size);
        clGetDeviceInfo (id, CL_DEVICE_NAME, size, const_cast<char*> (result.data ()), NULL);

        return result;
    }

    void ROS_OpenCL::checkError (const cl_int error){
        if (error != CL_SUCCESS) {
            ROS_ERROR("OpenCL call failed with error: %d", error);
            // TODO error handling
            //exit (1);
        }
    }

    std::string ROS_OpenCL::LoadKernel (const char* name){
        std::ifstream in (name);
        std::string result((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        return result;
    }

    cl_program ROS_OpenCL::createProgram (const std::string& source, const cl_context context){
        size_t lengths [1] = { source.size () };
        const char* sources [1] = { source.data () };

        cl_int error = 0;
        cl_program program_ = clCreateProgramWithSource(context, 1, sources, lengths, &error);
        checkError (error);

        return program_;
    }

    // Public Methods

    ROS_OpenCL::ROS_OpenCL(const std::string full_kernel_path, const std::string kernel_function){
        cl_uint platformIdCount = 0;
        clGetPlatformIDs (0, NULL, &platformIdCount);

        if (platformIdCount == 0) {
            ROS_ERROR("No OpenCL platform found");
            exit (1);
        }
        else {
            ROS_INFO("Found %d platform(s)", platformIdCount);
        }

        std::vector<cl_platform_id> platformIds (platformIdCount);
        clGetPlatformIDs (platformIdCount, platformIds.data (), NULL);

        for (cl_uint i = 0; i < platformIdCount; ++i) {
            ROS_INFO("\t (%d) : %s", i+1, getPlatformName (platformIds [i]).c_str());
        }

        cl_uint deviceIdCount = 0;
        clGetDeviceIDs (platformIds [0], CL_DEVICE_TYPE_ALL, 0, NULL, &deviceIdCount);

        if (deviceIdCount == 0) {
            ROS_ERROR("No OpenCL devices found");
            exit (1);
        }
        else {
            ROS_INFO("Found %d device(s)", deviceIdCount);
        }

        deviceIds  = std::vector<cl_device_id>(deviceIdCount);
        clGetDeviceIDs (platformIds [0], CL_DEVICE_TYPE_ALL, deviceIdCount, deviceIds.data(), NULL);

        for (cl_uint i = 0; i < deviceIdCount; ++i) {
            ROS_INFO("\t (%d) : %s", i+1, getDeviceName (deviceIds [i]).c_str());
        }

        const cl_context_properties contextProperties [] = {CL_CONTEXT_PLATFORM, reinterpret_cast<cl_context_properties> (platformIds [0]), 0, 0};

        cl_int error = CL_SUCCESS;
        context = clCreateContext (contextProperties, deviceIdCount, deviceIds.data (), NULL, NULL, &error);
        checkError (error);

        ROS_INFO("Context created");

        program = createProgram (LoadKernel (full_kernel_path.c_str()), context);

        checkError (clBuildProgram (program, deviceIdCount, deviceIds.data (), "-D FILTER_SIZE=1", NULL, NULL));

        ROS_INFO("Program built");

        kernel = clCreateKernel (program, kernel_function.c_str(), &error);
        checkError (error);

        ROS_INFO("Kernel created");
    }

    sensor_msgs::PointCloud2 ROS_OpenCL::process(const sensor_msgs::PointCloud2& msg){
        cl_int sz = msg.data.size();
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sz, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, sz, &msg.data[0], 0, NULL, NULL);
        checkError (error);

        size_t size = sz;

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, 1, NULL, &size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        uint8_t *result = (uint8_t *) malloc(sz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, sz, result, 0, NULL, NULL));

        sensor_msgs::PointCloud2 res = sensor_msgs::PointCloud2(msg);
        res.data.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(sensor_msgs::PointCloud2::Ptr msg){
        cl_int sz = msg->data.size();
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sz, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, sz, &msg->data[0], 0, NULL, NULL);
        checkError (error);

        size_t size = sz;

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, 1, NULL, &size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        uint8_t *result = (uint8_t *) malloc(sz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, sz, result, 0, NULL, NULL));

        msg->data.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        free(result);
    }

    sensor_msgs::LaserScan ROS_OpenCL::process(const sensor_msgs::LaserScan& msg){
        cl_int sz = msg.ranges.size();
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sz, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, sz, &msg.ranges[0], 0, NULL, NULL);
        checkError (error);

        size_t size = sz;

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, 1, NULL, &size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        float *result = (float *) malloc(sz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, sz, result, 0, NULL, NULL));

        sensor_msgs::LaserScan res = sensor_msgs::LaserScan(msg);
        res.ranges.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(sensor_msgs::LaserScan::Ptr msg){
        cl_int sz = msg->ranges.size();
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(float) * sz, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, sz, &msg->ranges[0], 0, NULL, NULL);
        checkError (error);

        size_t size = sz;

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, 1, NULL, &size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        float *result = (float *) malloc(sz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, sz, result, 0, NULL, NULL));

        msg->ranges.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        free(result);
    }

    sensor_msgs::Image ROS_OpenCL::process(const sensor_msgs::Image& msg){
        cl_int sz = msg.data.size();
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sz, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, sz, &msg.data[0], 0, NULL, NULL);
        checkError (error);

        size_t size = sz;

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, 1, NULL, &size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        uint8_t *result = (uint8_t *) malloc(sz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, sz, result, 0, NULL, NULL));

        sensor_msgs::Image res = sensor_msgs::Image(msg);
        res.data.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(sensor_msgs::Image::Ptr msg){
        cl_int sz = msg->data.size();
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sz, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, sz, &msg->data[0], 0, NULL, NULL);
        checkError (error);

        size_t size = sz;

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, 1, NULL, &size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        uint8_t *result = (uint8_t *) malloc(sz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, sz, result, 0, NULL, NULL));

        msg->data.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        free(result);
    }

    void ROS_OpenCL::clean(){
        clReleaseKernel (kernel);
        clReleaseProgram (program);
        clReleaseContext (context);
    }

}
