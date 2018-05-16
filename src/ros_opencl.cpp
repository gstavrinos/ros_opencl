#include "ros_opencl/ros_opencl.hpp"

namespace ros_opencl {

    // Private Methods

    void ROS_OpenCL::clean(){
        clReleaseKernel (kernel);
        clReleaseProgram (program);
        clReleaseContext (context);
    }

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

    ROS_OpenCL::~ROS_OpenCL(){
        if(!deviceIds.empty()){
            clean();
        }
    }

    ROS_OpenCL ROS_OpenCL::operator=(ROS_OpenCL* s){
        std::swap(kernel, s->kernel);
        std::swap(context, s->context);
        std::swap(program, s->program);
        std::swap(deviceIds, s->deviceIds);
        ROS_INFO("=====");
        return *this;
    }

    sensor_msgs::PointCloud2 ROS_OpenCL::process(const sensor_msgs::PointCloud2& msg){
        size_t sz = msg.data.size();
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
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(sensor_msgs::PointCloud2::Ptr msg){
        size_t sz = msg->data.size();
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
        clReleaseEvent(gpuExec);
        free(result);
    }

    sensor_msgs::LaserScan ROS_OpenCL::process(const sensor_msgs::LaserScan& msg){
        size_t sz = msg.ranges.size();
        cl_int typesz = sizeof(float) * sz;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &msg.ranges[0], 0, NULL, NULL);
        checkError (error);

        size_t size = sz;

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, 1, NULL, &size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        float *result = (float *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        sensor_msgs::LaserScan res = sensor_msgs::LaserScan(msg);
        res.ranges.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(sensor_msgs::LaserScan::Ptr msg){
        size_t sz = msg->ranges.size();
        cl_int typesz = sizeof(float) * sz;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &msg->ranges[0], 0, NULL, NULL);
        checkError (error);

        size_t size = sz;

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, 1, NULL, &size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        float *result = (float *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        msg->ranges.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseEvent(gpuExec);
        free(result);
    }

    sensor_msgs::Image ROS_OpenCL::process(const sensor_msgs::Image& msg){
        size_t sz = msg.data.size();
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
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(sensor_msgs::Image::Ptr msg){
        size_t sz = msg->data.size();
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
        clReleaseEvent(gpuExec);
        free(result);
    }

    std::vector<float> ROS_OpenCL::process(const std::vector<float> v){
        size_t sz = v.size();
        cl_int typesz = sizeof(float) * sz;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v[0], 0, NULL, NULL);
        checkError (error);

        size_t size = typesz;

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, 1, NULL, &size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        float *result = (float *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        std::vector<float> res = std::vector<float>();
        res.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(std::vector<float>* v){
        size_t sz = v->size();
        cl_int typesz = sizeof(float) * sz;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);

        size_t size = typesz;

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, 1, NULL, &size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        float *result = (float *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseEvent(gpuExec);
        free(result);
    }

    std::vector<double> ROS_OpenCL::process(const std::vector<double> v){
        size_t sz = v.size();
        cl_int typesz = sizeof(double) * sz;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v[0], 0, NULL, NULL);
        checkError (error);

        size_t size = typesz;

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, 1, NULL, &size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        double *result = (double *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        std::vector<double> res = std::vector<double>();
        res.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(std::vector<double>* v){
        size_t sz = v->size();
        cl_int typesz = sizeof(double) * sz;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);

        size_t size = typesz;

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, 1, NULL, &size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        double *result = (double *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseEvent(gpuExec);
        free(result);
    }

    std::vector<int> ROS_OpenCL::process(const std::vector<int> v){
        size_t sz = v.size();
        cl_int typesz = sizeof(int) * sz;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v[0], 0, NULL, NULL);
        checkError (error);

        size_t size = typesz;

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, 1, NULL, &size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        int *result = (int *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        std::vector<int> res = std::vector<int>();
        res.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(std::vector<int>* v){
        size_t sz = v->size();
        cl_int typesz = sizeof(int) * sz;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);

        size_t size = typesz;

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, 1, NULL, &size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        int *result = (int *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseEvent(gpuExec);
        free(result);
    }

    std::vector<char> ROS_OpenCL::process(const std::vector<char> v){
        size_t sz = v.size();
        cl_int typesz = sizeof(char) * sz;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v[0], 0, NULL, NULL);
        checkError (error);

        size_t size = typesz;

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, 1, NULL, &size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        char *result = (char *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        std::vector<char> res = std::vector<char>();
        res.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(std::vector<char>* v){
        size_t sz = v->size();
        cl_int typesz = sizeof(char) * sz;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);

        size_t size = typesz;

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, 1, NULL, &size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        char *result = (char *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseEvent(gpuExec);
        free(result);
    }

    std::vector<char> ROS_OpenCL::process(const std::vector<char> v, const std::vector<char> v2, bool two_dimensional){
        size_t sz = v.size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(char) * sz;
        size_t typesz2 = sizeof(char) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v[0], 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        char *result = (char *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        std::vector<char> res = std::vector<char>();
        res.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(std::vector<char>* v, const std::vector<char> v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(char) * sz;
        size_t typesz2 = sizeof(char) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        char *result = (char *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
    }

    void ROS_OpenCL::process(std::vector<char>* v, std::vector<char>* v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2->size();
        size_t typesz = sizeof(char) * sz;
        size_t typesz2 = sizeof(char) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2->at(0), 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        char *result = (char *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        char *result2 = (char *) malloc(typesz2);
        checkError(clEnqueueReadBuffer(queue, buffer2, CL_TRUE, 0, typesz2, result2, 0, NULL, NULL));

        v2->assign(result2, result2+sz2);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
        free(result2);
    }

    std::vector<char> ROS_OpenCL::process(const std::vector<char> v, const std::vector<int> v2, bool two_dimensional){
        size_t sz = v.size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(char) * sz;
        size_t typesz2 = sizeof(int) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v[0], 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        char *result = (char *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        std::vector<char> res = std::vector<char>();
        res.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(std::vector<char>* v, const std::vector<int> v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(char) * sz;
        size_t typesz2 = sizeof(int) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        char *result = (char *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
    }

    void ROS_OpenCL::process(std::vector<char>* v, std::vector<int>* v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2->size();
        size_t typesz = sizeof(char) * sz;
        size_t typesz2 = sizeof(int) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2->at(0), 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;
        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        char *result = (char *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        int *result2 = (int *) malloc(typesz2);
        checkError(clEnqueueReadBuffer(queue, buffer2, CL_TRUE, 0, typesz2, result2, 0, NULL, NULL));

        v2->assign(result2, result2+sz2);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
        free(result2);
    }

    std::vector<char> ROS_OpenCL::process(const std::vector<char> v, const std::vector<float> v2, bool two_dimensional){
        size_t sz = v.size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(char) * sz;
        size_t typesz2 = sizeof(float) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v[0], 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        char *result = (char *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        std::vector<char> res = std::vector<char>();
        res.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(std::vector<char>* v, const std::vector<float> v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(char) * sz;
        size_t typesz2 = sizeof(float) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        char *result = (char *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
    }

    void ROS_OpenCL::process(std::vector<char>* v, std::vector<float>* v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2->size();
        size_t typesz = sizeof(char) * sz;
        size_t typesz2 = sizeof(float) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2->at(0), 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;
        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        char *result = (char *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        float *result2 = (float *) malloc(typesz2);
        checkError(clEnqueueReadBuffer(queue, buffer2, CL_TRUE, 0, typesz2, result2, 0, NULL, NULL));

        v2->assign(result2, result2+sz2);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
        free(result2);
    }

    std::vector<char> ROS_OpenCL::process(const std::vector<char> v, const std::vector<double> v2, bool two_dimensional){
        size_t sz = v.size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(char) * sz;
        size_t typesz2 = sizeof(double) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v[0], 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        char *result = (char *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        std::vector<char> res = std::vector<char>();
        res.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(std::vector<char>* v, const std::vector<double> v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(char) * sz;
        size_t typesz2 = sizeof(double) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        char *result = (char *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
    }

    void ROS_OpenCL::process(std::vector<char>* v, std::vector<double>* v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2->size();
        size_t typesz = sizeof(char) * sz;
        size_t typesz2 = sizeof(double) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2->at(0), 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;
        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        char *result = (char *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        double *result2 = (double *) malloc(typesz2);
        checkError(clEnqueueReadBuffer(queue, buffer2, CL_TRUE, 0, typesz2, result2, 0, NULL, NULL));

        v2->assign(result2, result2+sz2);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
        free(result2);
    }

    std::vector<int> ROS_OpenCL::process(const std::vector<int> v, const std::vector<char> v2, bool two_dimensional){
        size_t sz = v.size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(int) * sz;
        size_t typesz2 = sizeof(char) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v[0], 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        int *result = (int *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        std::vector<int> res = std::vector<int>();
        res.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(std::vector<int>* v, const std::vector<char> v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(int) * sz;
        size_t typesz2 = sizeof(char) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        int *result = (int *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
    }

    void ROS_OpenCL::process(std::vector<int>* v, std::vector<char>* v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2->size();
        size_t typesz = sizeof(int) * sz;
        size_t typesz2 = sizeof(char) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2->at(0), 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;
        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        int *result = (int *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        char *result2 = (char *) malloc(typesz2);
        checkError(clEnqueueReadBuffer(queue, buffer2, CL_TRUE, 0, typesz2, result2, 0, NULL, NULL));

        v2->assign(result2, result2+sz2);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
        free(result2);
    }

    std::vector<int> ROS_OpenCL::process(const std::vector<int> v, const std::vector<int> v2, bool two_dimensional){
        size_t sz = v.size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(int) * sz;
        size_t typesz2 = sizeof(int) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v[0], 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        int *result = (int *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        std::vector<int> res = std::vector<int>();
        res.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(std::vector<int>* v, const std::vector<int> v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(int) * sz;
        size_t typesz2 = sizeof(int) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        int *result = (int *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
    }

    void ROS_OpenCL::process(std::vector<int>* v, std::vector<int>* v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2->size();
        size_t typesz = sizeof(int) * sz;
        size_t typesz2 = sizeof(int) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2->at(0), 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;
        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        int *result = (int *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        int *result2 = (int *) malloc(typesz2);
        checkError(clEnqueueReadBuffer(queue, buffer2, CL_TRUE, 0, typesz2, result2, 0, NULL, NULL));

        v2->assign(result2, result2+sz2);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
        free(result2);
    }

    std::vector<int> ROS_OpenCL::process(const std::vector<int> v, const std::vector<float> v2, bool two_dimensional){
        size_t sz = v.size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(int) * sz;
        size_t typesz2 = sizeof(float) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v[0], 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        int *result = (int *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        std::vector<int> res = std::vector<int>();
        res.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(std::vector<int>* v, const std::vector<float> v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(int) * sz;
        size_t typesz2 = sizeof(float) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        int *result = (int *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
    }

    void ROS_OpenCL::process(std::vector<int>* v, std::vector<float>* v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2->size();
        size_t typesz = sizeof(int) * sz;
        size_t typesz2 = sizeof(float) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2->at(0), 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;
        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        int *result = (int *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        float *result2 = (float *) malloc(typesz2);
        checkError(clEnqueueReadBuffer(queue, buffer2, CL_TRUE, 0, typesz2, result2, 0, NULL, NULL));

        v2->assign(result2, result2+sz2);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
        free(result2);
    }

    std::vector<int> ROS_OpenCL::process(const std::vector<int> v, const std::vector<double> v2, bool two_dimensional){
        size_t sz = v.size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(int) * sz;
        size_t typesz2 = sizeof(double) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v[0], 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        int *result = (int *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        std::vector<int> res = std::vector<int>();
        res.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(std::vector<int>* v, const std::vector<double> v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(int) * sz;
        size_t typesz2 = sizeof(double) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        int *result = (int *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
    }

    void ROS_OpenCL::process(std::vector<int>* v, std::vector<double>* v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2->size();
        size_t typesz = sizeof(int) * sz;
        size_t typesz2 = sizeof(double) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2->at(0), 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;
        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        int *result = (int *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        double *result2 = (double *) malloc(typesz2);
        checkError(clEnqueueReadBuffer(queue, buffer2, CL_TRUE, 0, typesz2, result2, 0, NULL, NULL));

        v2->assign(result2, result2+sz2);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
        free(result2);
    }

    std::vector<float> ROS_OpenCL::process(const std::vector<float> v, const std::vector<char> v2, bool two_dimensional){
        size_t sz = v.size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(float) * sz;
        size_t typesz2 = sizeof(char) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v[0], 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        float *result = (float *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        std::vector<float> res = std::vector<float>();
        res.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(std::vector<float>* v, const std::vector<char> v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(float) * sz;
        size_t typesz2 = sizeof(char) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        float *result = (float *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
    }

    void ROS_OpenCL::process(std::vector<float>* v, std::vector<char>* v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2->size();
        size_t typesz = sizeof(float) * sz;
        size_t typesz2 = sizeof(char) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2->at(0), 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;
        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        float *result = (float *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        char *result2 = (char *) malloc(typesz2);
        checkError(clEnqueueReadBuffer(queue, buffer2, CL_TRUE, 0, typesz2, result2, 0, NULL, NULL));

        v2->assign(result2, result2+sz2);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
        free(result2);
    }

    std::vector<float> ROS_OpenCL::process(const std::vector<float> v, const std::vector<int> v2, bool two_dimensional){
        size_t sz = v.size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(float) * sz;
        size_t typesz2 = sizeof(int) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v[0], 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        float *result = (float *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        std::vector<float> res = std::vector<float>();
        res.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(std::vector<float>* v, const std::vector<int> v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(float) * sz;
        size_t typesz2 = sizeof(int) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        float *result = (float *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
    }

    void ROS_OpenCL::process(std::vector<float>* v, std::vector<int>* v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2->size();
        size_t typesz = sizeof(float) * sz;
        size_t typesz2 = sizeof(int) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2->at(0), 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {typesz, typesz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;
        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        float *result = (float *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        int *result2 = (int *) malloc(typesz2);
        checkError(clEnqueueReadBuffer(queue, buffer2, CL_TRUE, 0, typesz2, result2, 0, NULL, NULL));

        v2->assign(result2, result2+sz2);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
        free(result2);
    }

    std::vector<float> ROS_OpenCL::process(const std::vector<float> v, const std::vector<float> v2, bool two_dimensional){
        size_t sz = v.size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(float) * sz;
        size_t typesz2 = sizeof(float) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v[0], 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {sz, sz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        float *result = (float *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        std::vector<float> res = std::vector<float>();
        res.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(std::vector<float>* v, const std::vector<float> v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(float) * sz;
        size_t typesz2 = sizeof(float) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {sz, sz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        float *result = (float *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
    }

    void ROS_OpenCL::process(std::vector<float>* v, std::vector<float>* v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2->size();
        size_t typesz = sizeof(float) * sz;
        size_t typesz2 = sizeof(float) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2->at(0), 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {sz, sz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;
        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        float *result = (float *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        float *result2 = (float *) malloc(typesz2);
        checkError(clEnqueueReadBuffer(queue, buffer2, CL_TRUE, 0, typesz2, result2, 0, NULL, NULL));

        v2->assign(result2, result2+sz2);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
        free(result2);
    }

    std::vector<float> ROS_OpenCL::process(const std::vector<float> v, const std::vector<double> v2, bool two_dimensional){
        size_t sz = v.size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(float) * sz;
        size_t typesz2 = sizeof(double) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v[0], 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {sz, sz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        float *result = (float *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        std::vector<float> res = std::vector<float>();
        res.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(std::vector<float>* v, const std::vector<double> v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(float) * sz;
        size_t typesz2 = sizeof(double) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {sz, sz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        float *result = (float *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
    }

    void ROS_OpenCL::process(std::vector<float>* v, std::vector<double>* v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2->size();
        size_t typesz = sizeof(float) * sz;
        size_t typesz2 = sizeof(double) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2->at(0), 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {sz, sz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;
        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        float *result = (float *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        double *result2 = (double *) malloc(typesz2);
        checkError(clEnqueueReadBuffer(queue, buffer2, CL_TRUE, 0, typesz2, result2, 0, NULL, NULL));

        v2->assign(result2, result2+sz2);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
        free(result2);
    }

    std::vector<double> ROS_OpenCL::process(const std::vector<double> v, const std::vector<char> v2, bool two_dimensional){
        size_t sz = v.size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(double) * sz;
        size_t typesz2 = sizeof(char) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v[0], 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {sz, sz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        double *result = (double *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        std::vector<double> res = std::vector<double>();
        res.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(std::vector<double>* v, const std::vector<char> v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(double) * sz;
        size_t typesz2 = sizeof(char) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {sz, sz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        double *result = (double *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
    }

    void ROS_OpenCL::process(std::vector<double>* v, std::vector<char>* v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2->size();
        size_t typesz = sizeof(double) * sz;
        size_t typesz2 = sizeof(char) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2->at(0), 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {sz, sz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;
        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        double *result = (double *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        char *result2 = (char *) malloc(typesz2);
        checkError(clEnqueueReadBuffer(queue, buffer2, CL_TRUE, 0, typesz2, result2, 0, NULL, NULL));

        v2->assign(result2, result2+sz2);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
        free(result2);
    }

    std::vector<double> ROS_OpenCL::process(const std::vector<double> v, const std::vector<int> v2, bool two_dimensional){
        size_t sz = v.size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(double) * sz;
        size_t typesz2 = sizeof(int) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v[0], 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {sz, sz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        double *result = (double *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        std::vector<double> res = std::vector<double>();
        res.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(std::vector<double>* v, const std::vector<int> v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(double) * sz;
        size_t typesz2 = sizeof(int) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {sz, sz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        double *result = (double *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
    }

    void ROS_OpenCL::process(std::vector<double>* v, std::vector<int>* v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2->size();
        size_t typesz = sizeof(double) * sz;
        size_t typesz2 = sizeof(int) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2->at(0), 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {sz, sz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;
        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        double *result = (double *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        int *result2 = (int *) malloc(typesz2);
        checkError(clEnqueueReadBuffer(queue, buffer2, CL_TRUE, 0, typesz2, result2, 0, NULL, NULL));

        v2->assign(result2, result2+sz2);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
        free(result2);
    }

    std::vector<double> ROS_OpenCL::process(const std::vector<double> v, const std::vector<float> v2, bool two_dimensional){
        size_t sz = v.size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(double) * sz;
        size_t typesz2 = sizeof(float) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v[0], 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {sz, sz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        double *result = (double *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        std::vector<double> res = std::vector<double>();
        res.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(std::vector<double>* v, const std::vector<float> v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(double) * sz;
        size_t typesz2 = sizeof(float) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {sz, sz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        double *result = (double *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
    }

    void ROS_OpenCL::process(std::vector<double>* v, std::vector<float>* v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2->size();
        size_t typesz = sizeof(double) * sz;
        size_t typesz2 = sizeof(float) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2->at(0), 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {sz, sz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;
        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        double *result = (double *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        float *result2 = (float *) malloc(typesz2);
        checkError(clEnqueueReadBuffer(queue, buffer2, CL_TRUE, 0, typesz2, result2, 0, NULL, NULL));

        v2->assign(result2, result2+sz2);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
        free(result2);
    }

    std::vector<double> ROS_OpenCL::process(const std::vector<double> v, const std::vector<double> v2, bool two_dimensional){
        size_t sz = v.size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(double) * sz;
        size_t typesz2 = sizeof(double) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v[0], 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {sz, sz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        double *result = (double *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        std::vector<double> res = std::vector<double>();
        res.assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);

        return res;
    }

    void ROS_OpenCL::process(std::vector<double>* v, const std::vector<double> v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2.size();
        size_t typesz = sizeof(double) * sz;
        size_t typesz2 = sizeof(double) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_WRITE_ONLY, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2[0], 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {sz, sz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;

        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        double *result = (double *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
    }

    void ROS_OpenCL::process(std::vector<double>* v, std::vector<double>* v2, bool two_dimensional){
        size_t sz = v->size();
        size_t sz2 = v2->size();
        size_t typesz = sizeof(double) * sz;
        size_t typesz2 = sizeof(double) * sz2;
        cl_int error = 0;
        cl_mem buffer = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz, NULL, &error);
        checkError(error);
        cl_mem buffer2 = clCreateBuffer(context, CL_MEM_READ_WRITE, typesz2, NULL, &error);
        checkError(error);
        clSetKernelArg (kernel, 0, sizeof (cl_mem), &buffer);
        clSetKernelArg (kernel, 1, sizeof (cl_mem), &buffer2);
        cl_command_queue queue = clCreateCommandQueueWithProperties (context, deviceIds [0], NULL, &error);

        clEnqueueWriteBuffer(queue, buffer, CL_TRUE, 0, typesz, &v->at(0), 0, NULL, NULL);
        checkError (error);
        clEnqueueWriteBuffer(queue, buffer2, CL_TRUE, 0, typesz2, &v2->at(0), 0, NULL, NULL);
        checkError (error);

        size_t size[2] = {sz, sz2};
        size_t work_dimension = 2;

        if (sz == sz2 and not two_dimensional){
            work_dimension--;
        }

        cl_event gpuExec;
        checkError (clEnqueueNDRangeKernel (queue, kernel, work_dimension, NULL, size, NULL, 0, NULL, &gpuExec));

        clWaitForEvents(1, &gpuExec);

        double *result = (double *) malloc(typesz);
        checkError(clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0, typesz, result, 0, NULL, NULL));

        v->assign(result, result+sz);

        double *result2 = (double *) malloc(typesz2);
        checkError(clEnqueueReadBuffer(queue, buffer2, CL_TRUE, 0, typesz2, result2, 0, NULL, NULL));

        v2->assign(result2, result2+sz2);

        clReleaseCommandQueue (queue);
        clReleaseMemObject(buffer);
        clReleaseMemObject(buffer2);
        clReleaseEvent(gpuExec);
        free(result);
        free(result2);
    }

}
