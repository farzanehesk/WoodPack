#ifndef UTILS_H
#define UTILS_H

#include <open3d/Open3D.h>
#include <vector>

Eigen::Vector3d hslToRgb(double h, double s, double l);
std::vector<Eigen::Vector3d> generateColorGradient(size_t num_rows);


















#endif // UTILS_H