#include "../include/utils.hpp"
#include <cmath>

Eigen::Vector3d hslToRgb(double h, double s, double l) {
    h = std::fmod(h, 360.0);
    if (h < 0) h += 360.0;

    double c = (1.0 - std::abs(2.0 * l - 1.0)) * s;
    double hp = h / 60.0;
    double x = c * (1.0 - std::abs(std::fmod(hp, 2.0) - 1.0));
    double m = l - c / 2.0;

    double r = 0.0, g = 0.0, b = 0.0;
    if (hp >= 0 && hp < 1) { r = c; g = x; b = 0; }
    else if (hp >= 1 && hp < 2) { r = x; g = c; b = 0; }
    else if (hp >= 2 && hp < 3) { r = 0; g = c; b = x; }
    else if (hp >= 3 && hp < 4) { r = 0; g = x; b = c; }
    else if (hp >= 4 && hp < 5) { r = x; g = 0; b = c; }
    else if (hp >= 5 && hp < 6) { r = c; g = 0; b = x; }

    return Eigen::Vector3d(r + m, g + m, b + m);
}

std::vector<Eigen::Vector3d> generateColorGradient(size_t num_rows) {
    std::vector<Eigen::Vector3d> colors;
    colors.reserve(num_rows);

    if (num_rows == 0) return colors;

    const double start_hue = 120.0;
    const double saturation = 0.8;
    const double lightness = 0.5;
    const double hue_step = 360.0 / num_rows;

    for (size_t i = 0; i < num_rows; ++i) {
        double hue = start_hue + i * hue_step;
        colors.push_back(hslToRgb(hue, saturation, lightness));
    }

    return colors;
}