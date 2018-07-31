//
// Created by chaoz on 26/07/18.
//

#ifndef PROJECT_PATHFINDER_H
#define PROJECT_PATHFINDER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <exception>

#define DIST_LIDAR_TO_CAR 1.8
#define DIST_FRONT_TO_REAR 1.8

struct point_2d {
    double x;
    double y;
    double dist;
    double half_diameter;
    double orientation;

    point_2d() = default;

    point_2d(double _x, double _y, double _half_diameter);

    bool operator<(const point_2d &a) const;

    double operator-(const point_2d &a) const;

    double operator/(const point_2d &a) const;
};

struct range_1d {
    std::vector<double> ranges;
    double max_range;
    double min_angle;
    double max_angle;
    double resolution;

    double &operator[](double angle);

    range_1d() = default;

    range_1d(sensor_msgs::LaserScanConstPtr ls_ptr);

    range_1d(sensor_msgs::LaserScanConstPtr ls_ptr, double desired_angle_max, double desired_angle_min);
};

struct range_1d_cluster {
    double start_angle;
    double end_angle;
    double closest_distance;
    double farthest_distance;

    range_1d_cluster() {};

    range_1d_cluster(double angle, double distance);

    bool push(double angle, double distance, double threshold);

    double mean_angle();

    double size();

    double get_x();

    double get_y();
};

enum class sort_type {
    left,
    right
};

class pathfinder {
public:
    pathfinder(ros::NodeHandle);

    ~pathfinder() = default;

    bool Config();

    bool PrintConfig();

    bool PrintStatus();

    bool Start();

    bool Stop();

private:
    bool read_parameters();

    bool setup_subscribers();

    bool setup_publishers();

    bool drive();

    void stop();

    void curve(double angle, int sign);

    void odom_cb(nav_msgs::OdometryConstPtr msg);

    void cones_cb(visualization_msgs::MarkerArrayConstPtr msg);

    void laser_cb(sensor_msgs::LaserScanConstPtr msg);

    bool find_cones();

    bool sort_cones(sort_type st);

    double check_outer_track(int sign);

    double check_inner_track(int sign);

    bool visualise_path(double time_interval, int num_of_points);

    ros::NodeHandle n;
    int spin_rate;
    ros::Rate rate;

    double cur_x;
    double cur_y;
    double orientation;
    double cur_linear_velocity;
    double cur_rotate_velocity;

    double tar_linear_velocity;
    double tar_rotate_velocity;
    double tar_steering_angle;

    std::vector<point_2d> cones;
    std::vector<point_2d> left_cones;
    std::vector<point_2d> right_cones;
    double cones_max_distance;

    bool has_new_cone;
    bool has_new_laserscan;
    bool use_laserscan;
    range_1d laserscan;
    double clustering_threshold;

	std::vector<double> r_meter;
	std::vector<double> angle_degree;
    bool publish_path;
    int num_of_path_points;
    double time_interval;

    ros::Subscriber odom_sub;
    std::string odom_topic_name;

    ros::Subscriber cones_sub;
    std::string cones_topic_name;

    ros::Subscriber laser_sub;
    std::string laser_topic_name;

    ros::Publisher cmdvel_pub;
    std::string cmdvel_topic_name;

    ros::Publisher path_pub;
    std::string path_topic_name;

};

#endif //PROJECT_PATHFINDER_H
