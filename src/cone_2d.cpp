//
// Created by chaoz on 2/08/18.
//

#include <cassert>
#include <cstdlib>
#include "pathfinder/cone_2d.h"



/**
 * point_2d::operator -
 *
 * @param a - target point_2d
 * @return the distance between current point and target point
 */
double point_2d::operator-(const point_2d &a) const {
    return sqrt(pow(x - a.x, 2) + pow(y - a.y, 2));
}

/**
 * point_2d::operator /
 *
 * @param a - target point
 * @return the orientation from current point toward target point
 */
double point_2d::operator/(const point_2d &a) const {
    return atan2(a.y - y, a.x - x);
}

point_2d::point_2d(double x, double y):x(x),y(y) {}



cone_2d::cone_2d(double x, double y, double half_diameter):half_diameter(half_diameter) {
    point = new point_2d(x,y);
    dist_to_car = *point - point_2d(0,0);
    dist_to_left = *point - point_2d(0,4.0);
    orientation = *point / point_2d(0,0);
}

bool cone_2d::operator<(const cone_2d &a) const {
    return dist_to_left < a.dist_to_left;
}

double cone_2d::operator-(const cone_2d &a) const {
    return *point - *(a.point);
}

double cone_2d::operator/(const cone_2d &a) const {
    return *point / *(a.point);
}

cone_2d::~cone_2d() {
//    delete point;
}

double cone_2d::get_orientation() {
    return orientation;
}

void cone_2d::set_x(double value) {
    point->x=value;
}

void cone_2d::set_y(double value) {
    point->y=value;
}

void cone_2d::set_orientation(double value) {
    orientation = value;
}

double cone_2d::get_x() {
    return point->x;
}

double cone_2d::get_y() {
    return point->y;
}

double cone_2d::get_dist_to_car() const {
    return dist_to_car;
}

void cone_2d::set_dist_to_car(double dist_to_car) {
    cone_2d::dist_to_car = dist_to_car;
}

double cone_2d::get_dist_to_left() const {
    return dist_to_left;
}

void cone_2d::set_dist_to_left(double dist_to_left) {
    cone_2d::dist_to_left = dist_to_left;
}

double cone_2d::get_half_diameter() const {
    return half_diameter;
}

void cone_2d::set_half_diameter(double half_diameter) {
    cone_2d::half_diameter = half_diameter;
}

double cone_2d::get_dist_to_point(point_2d tar_point) {
    return *(point) - tar_point;
}

double cone_2d::get_min_collision_steering(double front_to_rear_distance, double clearance_radius) {
    if (point->y > clearance_radius)
    return 0;
}

double cone_2d::get_max_collision_steering(double front_to_rear_distance, double clearance_radius) {
    return 0;
}
