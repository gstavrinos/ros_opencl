#pragma once

#include <vector>

namespace ros_opencl_params {

/**
 * @brief      The ROS_OpenCL_Params helper class.
 */
class ROS_OpenCL_Params{

    public:

        bool two_dimensional;                 /**< The parameter that states if the global work size should be two dimensional.*/
        std::vector<size_t> global_work_size; /**< The parameter that will hold the global work size value(s).*/
        std::vector<size_t> buffers_size;     /**< The parameter that will hold the size for each of the
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
