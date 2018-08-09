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
    double half_diameter;
public:
    double get_dist_to_car() const;

    void set_dist_to_car(double dist_to_car);

    double get_half_diameter() const;

    void set_half_diameter(double half_diameter);

    double get_dist_to_point(point_2d tar_point);

    cone_2d() = default;

    cone_2d(double x, double y, double half_diameter);

    ~cone_2d();

    void set_x(double value);

    void set_y(double value);

    double get_x();

    double get_y();

    double operator-(const cone_2d &a) const;

    double operator/(const cone_2d &a) const;
};


#endif //PATHFINDER_CONE_2D_H
