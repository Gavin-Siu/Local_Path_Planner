//
// Created by chaoz on 2/08/18.
//

#include <pathfinder/steering_range.h>
#include <cassert>
#include <cstdlib>

steering_range::steering_range(double min, double max, double front_rear_distance):
        min(min),
        max(max),
        front_rear_distance(front_rear_distance)
{
    min_curve_center = point_2d(0.0, front_rear_distance / sin(min));
    max_curve_center = point_2d(0.0, front_rear_distance / sin(max));
}

double steering_range::get_size() {
    return max - min;
}

double steering_range::get_mean() {
    return (max + min) / 2.0;
}

double steering_range::get_min() const {
    return min;
}

void steering_range::set_min(double min) {
    steering_range::min = min;
}

double steering_range::get_max() const {
    return max;
}

void steering_range::set_max(double max) {
    steering_range::max = max;
}

const point_2d &steering_range::get_min_curve_center() const {
    return min_curve_center;
}

void steering_range::set_min_curve_center(const point_2d &min_curve_center) {
    steering_range::min_curve_center = min_curve_center;
}

const point_2d &steering_range::get_max_curve_center() const {
    return max_curve_center;
}

void steering_range::set_max_curve_center(const point_2d &max_curve_center) {
    steering_range::max_curve_center = max_curve_center;
}

double steering_range::get_front_rear_distance() const {
    return front_rear_distance;
}

void steering_range::set_front_rear_distance(double front_rear_distance) {
    steering_range::front_rear_distance = front_rear_distance;
}

bool steering_range::operator<(const steering_range &a) const {
    return min < a.get_min();
}