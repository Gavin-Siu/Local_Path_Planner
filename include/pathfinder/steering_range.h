//
// Created by chaoz on 2/08/18.
//

#ifndef PATHFINDER_STEERING_RANGE_H
#define PATHFINDER_STEERING_RANGE_H

#include <list>
#include <pathfinder/cone_2d.h>

class steering_range{
private:
    double min;
    double max;
    point_2d min_curve_center;
    point_2d max_curve_center;
    double front_rear_distance;
public:
    steering_range()=default;
    steering_range(double min,
                   double max,
                   double front_rear_distance);
    double get_size();
    double get_mean();

    double get_min() const;

    void set_min(double min);

    double get_max() const;

    void set_max(double max);

    const point_2d &get_min_curve_center() const;

    void set_min_curve_center(const point_2d &min_curve_center);

    const point_2d &get_max_curve_center() const;

    void set_max_curve_center(const point_2d &max_curve_center);

    double get_front_rear_distance() const;

    void set_front_rear_distance(double front_rear_distance);

    bool operator < (const steering_range &a) const;

};


#endif //PATHFINDER_STEERING_RANGE_H
