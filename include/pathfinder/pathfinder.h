//
// Created by chaoz on 26/07/18.
//

#ifndef PROJECT_PATHFINDER_H
#define PROJECT_PATHFINDER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <exception>

#include <pathfinder/cone_2d.h>
#include <pathfinder/range_1d_cluster.h>
#include <pathfinder/range_1d.h>
#include <pathfinder/enum_def.h>
#include <pathfinder/steering_range.h>

#define DIST_FRONT_TO_REAR 1.8

class pathfinder {
public:
    pathfinder(ros::NodeHandle);

    ~pathfinder() = default;

    bool Config();

    bool PrintConfig();

    bool PrintStatus();

    bool Start();

private:
    bool read_parameters();

    bool setup_subscribers();

    bool setup_publishers();

	bool drive();

    bool update_steering_ranges(cone_2d& cone);

    double evaluate_boundary_steering(double x, double y, double shift);

    void stop();

    void curve(double angle);

    void odom_cb(nav_msgs::OdometryConstPtr msg);

    void cones_cb(visualization_msgs::MarkerArrayConstPtr msg);

    void laser_cb(sensor_msgs::LaserScanConstPtr msg);

    void ctrl_cb(std_msgs::BoolConstPtr msg);

    bool find_cones();

	bool visualise_path();

	bool visualise_cones(std::vector<cone_2d> &tar_cones,
                         std::string name,
                         MARKER_COLOR color);

	bool visualise_boundary();

    visualization_msgs::MarkerArray& get_clear_markers(std::string name);

	visualization_msgs::MarkerArray _get_clear_markers(std::string frame_id);

	bool get_steering_marker(std::string name,
							 double speed,
                             double steering,
                             visualization_msgs::MarkerArray& markers,
                             std_msgs::ColorRGBA color);

	bool visualise_ranges();

    ros::NodeHandle n;
    int spin_rate;
    ros::Rate rate;

    double desired_speed;
    double applied_speed;
    double cur_x;
    double cur_y;
    double orientation;
    double cur_linear_velocity;
    double cur_rotate_velocity;

    double tar_linear_velocity;
    double tar_rotate_velocity;
    double tar_steering_angle;
    
    double fix_driving_speed;

    std::list<cone_2d> cones;
    std::vector<cone_2d> valid_cones;

    steering_range initial_range;
    std::list<steering_range> valid_ranges;
    double clearance_radius;
    double max_steering;

    bool apply_median_filter;
    int filter_size;
    bool has_new_cone;
    bool has_new_laserscan;
    bool use_laserscan;
    range_1d laserscan;
    double clustering_threshold;

	std::vector<double> r_meter;
	std::vector<double> angle_degree;

	bool publish_cones;

    bool publish_path;
    int num_of_path_points;
    double time_interval;

    double processing_range;

    std::string frame_id;

    ros::Subscriber odom_sub;
    std::string odom_topic_name;

    ros::Subscriber cones_sub;
    ros::Publisher cones_pub;
    std::string cones_topic_name;

    ros::Subscriber laser_sub;
    std::string laser_topic_name;

    ros::Publisher cmdvel_pub;
    std::string cmdvel_topic_name;

    ros::Publisher path_pub;
    std::string path_topic_name;

    ros::Publisher ranges_pub;
    std::string ranges_topic_name;
    bool publish_ranges;

    ros::Publisher boundary_pub;
    std::string boundary_topic_name;
    bool publish_boundary;

    ros::Subscriber ctrl_sub;
    std::string ctrl_topic_name;
    bool ctrl_by_topic;
};

#endif //PROJECT_PATHFINDER_H
