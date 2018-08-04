//
// Created by chaoz on 2/08/18.
//

#ifndef PATHFINDER_CONE_2D_H
#define PATHFINDER_CONE_2D_H
#include <cmath>

struct point_2d {
    double x;
    double y;
    point_2d() = default;
    ~point_2d() = default;
    point_2d(double x, double y);
    double operator-(const point_2d &a) const;
    double operator/(const point_2d &a) const;
};

class cone_2d {
private:
    point_2d *point;
    double dist_to_car;
    double dist_to_left;
    double half_diameter;
    double orientation;
public:
    double get_dist_to_car() const;

    void set_dist_to_car(double dist_to_car);

    double get_dist_to_left() const;

    void set_dist_to_left(double dist_to_left);

    double get_half_diameter() const;

    void set_half_diameter(double half_diameter);

    int get_mode() const;

    void set_mode(int mode);

    double get_dist_to_point(point_2d tar_point);

    double get_min_collision_steering(double front_to_rear_distance, double clearance_radius);

    double get_max_collision_steering(double front_to_rear_distance, double clearance_radius);

private:

    int mode;

public:
    cone_2d() = default;
    cone_2d(double x, double y, double half_diameter, int mode = 2);
    ~cone_2d();

    void set_x(double value);

    void set_y(double value);

    double get_x();

    double get_y();

    void set_orientation(double value);

    double get_orientation();

    bool operator<(const cone_2d &a) const;

    double operator-(const cone_2d &a) const;

    double operator/(const cone_2d &a) const;
};


#endif //PATHFINDER_CONE_2D_H
