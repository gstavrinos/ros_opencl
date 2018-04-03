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
            exit (1);
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
        cl_uint8 *in = (cl_uint8 *) malloc(sizeof(cl_uint8)*sz);
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(cl_uint8) * sz, NULL, &error);
        checkError(error);
        // TODO bug around here, msg.data is not written in the buffer!
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, sizeof(cl_uint8) * sz, in, 0, NULL, NULL);
        checkError (error);

        size_t size = sz;

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, 1, NULL, &size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        uint8_t *result = (uint8_t *) malloc(sizeof(uint8_t) * sz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, sizeof(uint8_t) * sz, result, 0, NULL, NULL));

        sensor_msgs::PointCloud2 res = sensor_msgs::PointCloud2(msg);
        res.data.clear();
        res.data.insert(res.data.end(), &result[0], &result[sz]);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        free(in);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(sensor_msgs::PointCloud2::Ptr msg){
        cl_int sz = msg->data.size();
        cl_uint8 *in = (cl_uint8 *) malloc(sizeof(cl_uint8)*sz);
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(cl_uint8) * sz, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, sizeof(cl_uint8) * sz, in, 0, NULL, NULL);
        checkError (error);

        size_t size = sz;

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, 1, NULL, &size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        uint8_t *result = (uint8_t *) malloc(sizeof(uint8_t) * sz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, sizeof(uint8_t) * sz, result, 0, NULL, NULL));

        msg->data.insert(msg->data.end(), &result[0], &result[sz]);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        free(in);
        free(result);
    }

    void ROS_OpenCL::clean(){
        clReleaseKernel (kernel);
        clReleaseProgram (program);
        clReleaseContext (context);
    }

}
