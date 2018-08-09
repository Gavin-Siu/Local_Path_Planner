//
// Created by chaoz on 2/08/18.
//

#ifndef PATHFINDER_RANGE_1D_CLUSTER_H
#define PATHFINDER_RANGE_1D_CLUSTER_H


#include "cone_2d.h"
#include <list>

#define DIST_LIDAR_TO_CAR 1.7

class range_1d_cluster {
private:
    double start_angle;
    double end_angle;
    double closest_angle;
    double closest_distance;
    double farthest_distance;
    double last_distance;

public:
    range_1d_cluster() {};

    range_1d_cluster(double angle, double distance);

    bool push(double angle, double distance, double threshold);

    double mean_angle();

    double size();

    double get_x();

    double get_y();

    bool load_cones(std::list<cone_2d> cones, double clearance_radius);

    double getStart_angle() const;

    double getEnd_angle() const;

    double getClosest_distance() const;

    double getFarthest_distance() const;
};


#endif //PATHFINDER_RANGE_1D_CLUSTER_H
