#pragma once

#include <vector>

namespace ros_opencl_params {

/**
 * @brief      The ROS_OpenCL_Params helper class.
 */
class ROS_OpenCL_Params{

    public:

        bool two_dimensional;                 /**< True if the global work size should
                                               * be two dimensional with the sizes of the two buffers. This is 
                                               * equivalent to using the buffers_size parameter, and pushing back
                                               * the sizes of the two input vectors. Defaults to false, and is not 
                                               * used in functions with one input vector.
                                               */
        std::vector<size_t> global_work_size; /**< The global work size value(s).*/
        std::vector<size_t> buffers_size;     /**< The size for each of the
                                               * buffers used to communicate with the kernel.
                                               */

        /**
         * @brief      Constructs a ROS_OpenCL_Params object.
         */
        ROS_OpenCL_Params(){two_dimensional=false;}

        /**
         * @brief      Destroys a ROS_OpenCL_Params object (without doing anything).
         */
        ~ROS_OpenCL_Params(){}
};

}
