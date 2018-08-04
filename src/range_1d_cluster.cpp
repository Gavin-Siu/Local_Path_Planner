//
// Created by chaoz on 2/08/18.
//

#include "pathfinder/range_1d_cluster.h"
#include <cmath>


/**
 * range_1d_cluster::range_1d_cluster - constructor
 *
 * @param angle - angle of first element in this cluster
 * @param distance - distance of first element in this cluster
 */
range_1d_cluster::range_1d_cluster(double angle, double distance) {
    start_angle = angle;
    end_angle = angle;
    farthest_distance = distance;
    closest_distance = distance;
}

/**
 * range_1d_cluster::push
 *
 * @param angle - angle of the new element
 * @param distance - distance of the new element
 * @param threshold - threshold for judging whether to adopt this new element or not
 * @return true if the new element is accepted; false otherwise.
 */
bool range_1d_cluster::push(double angle, double distance, double threshold) {
    if (distance < (closest_distance - threshold)) {
        return false;
    }

    if (distance > (farthest_distance + threshold)) {
        return false;
    }

    if (angle < start_angle) {
        start_angle = angle;
    } else if (angle > end_angle) {
        end_angle = angle;
    }

    if (distance > farthest_distance) {
        farthest_distance = distance;
    } else if (distance < closest_distance) {
        closest_distance = distance;
    }

    return true;
}

/**
 * range_1d_cluster::mean_angle
 *
 * @return mean of the start and end angle
 */
double range_1d_cluster::mean_angle() {
    return (start_angle + end_angle) / 2.0;
}

/**
 * range_1d_cluster::size
 *
 * @return size of the cluster
 */
double range_1d_cluster::size() {
    return farthest_distance * sin(end_angle - start_angle);
}

/**
 * range_1d_cluster::get_x
 *
 * @return x value of the center of the cluster
 */
double range_1d_cluster::get_x() {
    return cos(mean_angle()) * closest_distance;
}

/**
 * range_1d_cluster::get_y
 *
 * @return y value of the center of the cluster
 */
double range_1d_cluster::get_y() {
    return sin(mean_angle()) * closest_distance;
}

double range_1d_cluster::getStart_angle() const {
    return start_angle;
}

double range_1d_cluster::getEnd_angle() const {
    return end_angle;
}

double range_1d_cluster::getClosest_distance() const {
    return closest_distance;
}

double range_1d_cluster::getFarthest_distance() const {
    return farthest_distance;
}
