#ifndef CUSTOM_TYPES_H
#define CUSTOM_TYPES_H



#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <algorithm>



#include "open3d/Open3D.h"
#include <open3d/pipelines/registration/RobustKernel.h>
#include <open3d/pipelines/registration/ColoredICP.h>
#include <open3d/pipelines/registration/FastGlobalRegistration.h>




using PC_o3d = open3d::geometry::PointCloud;
using PC_o3d_ptr = std::shared_ptr < open3d::geometry::PointCloud>;



















#endif