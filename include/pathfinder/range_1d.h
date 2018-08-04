//
// Created by chaoz on 2/08/18.
//

#ifndef PATHFINDER_RANGE_1D_H
#define PATHFINDER_RANGE_1D_H

#include <cstdlib>
#include <vector>
#include <sensor_msgs/LaserScan.h>

class range_1d {
private:
    std::vector<double> ranges;
    double max_range;
    double min_angle;
    double max_angle;
    double resolution;

public:
    const std::vector<double> &getRanges() const;

    double get_max_range() const;

    double get_min_angle() const;

    double get_max_angle() const;

    double get_resolution() const;

    double &operator[](double angle);

    void set_ranges(const std::vector<double> &ranges);

    void set_max_range(double max_range);

    void set_min_angle(double min_angle);

    void set_max_angle(double max_angle);

    void set_resolution(double resolution);

    void median_filter(int size);

    range_1d() = default;

    range_1d(sensor_msgs::LaserScanConstPtr ls_ptr);

    range_1d(sensor_msgs::LaserScanConstPtr ls_ptr, double desired_angle_max, double desired_angle_min);
};


#endif //PATHFINDER_RANGE_1D_H
