//
// Created by chaoz on 26/07/18.
//

#include <pathfinder/pathfinder.h>
#include <functional>
#include <algorithm>


std::vector<double> r_meter = {4.25916285, 5.26284792, 6.954665949, 10.36578687, 20.65268384, 41.26605413,
                                     51.57667503,
                                     103.1376393};
std::vector<double> angle_degree = {25.0, 20.0, 15.0, 10.0, 5.0, 2.5, 2.0, 1.0};

/**
 * point_2d::point_2d - constructor
 *
 * @param _x - x value for the point in 2d planar
 * @param _y - y value for the point in 2d planar
 * @param _half_diameter - radius for the point
 */
point_2d::point_2d(double _x, double _y, double _half_diameter) : x(_x), y(_y), half_diameter(_half_diameter) {

    dist = sqrt(x * x + y * y);
    orientation = atan2(y, x);
}

/**
 * point_2d::operator <
 *
 * @param a - target point_2d for comparison
 * @return whether current point is smaller than target point
 */
bool point_2d::operator<(const point_2d &a) const {
    return (dist < a.dist);
}

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

/**
 * range_1d::operator[]
 *
 * @param angle - desired angle
 * @return the laserscan reading at desired angle
 */
double &range_1d::operator[](double angle) {
    assert(angle <= max_angle && angle >= min_angle);
    return ranges[(int) ((angle - min_angle) / resolution)];
}

/**
 * range_1d::range_1d - contrustor
 *
 * @param ls_ptr - ros msg type pointer for laserscan
 */
range_1d::range_1d(sensor_msgs::LaserScanConstPtr ls_ptr) {
    max_range = ls_ptr->range_max;
    resolution = ls_ptr->angle_increment;
    if (resolution > 0) {
        max_angle = ls_ptr->angle_max * M_1_PI * 180.0;
        min_angle = ls_ptr->angle_min * M_1_PI * 180.0;
        int num_data = (int) ((max_angle - min_angle) / resolution) + 1;
        ranges.reserve(num_data);
        for (int i = 0; i < num_data; i++) {
            ranges.push_back(ls_ptr->ranges[i]);
        }

    } else if (resolution < 0) {
        resolution = -resolution;
        max_angle = ls_ptr->angle_min * M_1_PI * 180.0;
        min_angle = ls_ptr->angle_max * M_1_PI * 180.0;
        int num_data = (int) ((max_angle - min_angle) / resolution) + 1;
        ranges.reserve(num_data);
        for (int i = num_data - 1; i >= 0; i--) {
            ranges.push_back(ls_ptr->ranges[i]);
        }
    } else {
        throw std::runtime_error("failed to construct: resolution can not be zero!");
    }
}

/**
 * range_1d::range_1d - contrustor
 *
 * @param ls_ptr - ros msg type pointer for laserscan
 * @param desired_angle_max - maximum constrains in angle
 * @param desired_angle_min - minimum constrains in angle
 */
range_1d::range_1d(sensor_msgs::LaserScanConstPtr ls_ptr, double desired_angle_max, double desired_angle_min) {
    max_range = ls_ptr->range_max;
    resolution = ls_ptr->angle_increment;
    if (resolution > 0) {
        max_angle = ls_ptr->angle_max * M_1_PI * 180.0;
        min_angle = ls_ptr->angle_min * M_1_PI * 180.0;
        assert(max_angle < desired_angle_max);
        assert(min_angle > desired_angle_min);
        int begin_index = (int) ((desired_angle_min - min_angle) / resolution);
        int end_index = (int) ((desired_angle_max - min_angle) / resolution) + 1;
        ranges.reserve((size_t) end_index - begin_index);
        for (int i = begin_index; i != end_index; i++) {
            ranges.push_back(ls_ptr->ranges[i]);
        }
        max_angle = desired_angle_max;
        min_angle = desired_angle_min;
    } else if (resolution < 0) {
        resolution = -resolution;
        max_angle = ls_ptr->angle_min * M_1_PI * 180.0;
        min_angle = ls_ptr->angle_max * M_1_PI * 180.0;
        assert(max_angle < desired_angle_max);
        assert(min_angle > desired_angle_min);
        int rbegin_index = (int) ((max_angle - desired_angle_min) / resolution);
        int rend_index = (int) ((max_angle - desired_angle_max) / resolution) - 1;
        ranges.reserve((size_t) rbegin_index - rend_index);
        for (int i = rbegin_index; i > rend_index; i--) {
            ranges.push_back(ls_ptr->ranges[i]);
        }
        max_angle = desired_angle_max;
        min_angle = desired_angle_min;
    } else {
        throw std::runtime_error("failed to construct: resolution can not be zero!");
    }
}

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
    return farthest_distance * sin((end_angle - start_angle) / 360.0 * M_PI);
}

/**
 * range_1d_cluster::get_x
 *
 * @return x value of the center of the cluster
 */
double range_1d_cluster::get_x() {
    return cos(mean_angle() / 180.0 * M_PI) * closest_distance + DIST_LIDAR_TO_CAR;
}

/**
 * range_1d_cluster::get_y
 *
 * @return y value of the center of the cluster
 */
double range_1d_cluster::get_y() {
    return sin(mean_angle() / 180.0 * M_PI) * closest_distance;
}

/**
 * pathfinder::pathfinder - constructor
 *
 * @param n - ros node handle
 */
pathfinder::pathfinder(ros::NodeHandle n) : n(n), rate(15) {
    cones.clear();
    cur_rotate_velocity = 0.0;
    tar_linear_velocity = 0.0;
    tar_rotate_velocity = 0.0;
    cur_linear_velocity = 0.0;
    cur_x = 0.0;
    cur_y = 0.0;
    orientation = 0.0;
    spin_rate = 15; //default
    has_new_cone = false;
    has_new_laserscan = false;
    use_laserscan = false;
}

/**
 * pathfinder::Config
 *
 * @return true if success; false otherwise.
 */
bool pathfinder::Config() {
    bool status = true;
    status = read_parameters();
    status = setup_subscribers();
    status = setup_publishers();
    rate = ros::Rate(spin_rate);

    return status;
}

/**
 * pathfinder::PrintConfig
 *
 * @return true if success; false otherwise.
 */
bool pathfinder::PrintConfig() {
    std::string s_tab = "    ";
    ROS_INFO_STREAM("\nodom_sub:\n" << s_tab << "topic name:\n" << s_tab + s_tab << odom_topic_name <<
                                    "\ncones_sub:\n" << s_tab << "topic name:\n" << s_tab + s_tab << cones_topic_name <<
                                    "\nlaser_sub:\n" << s_tab << "topic name:\n" << s_tab + s_tab << laser_topic_name <<
                                    "\ncmdvel_pub:\n" << s_tab << "topic name:\n" << s_tab + s_tab << cmdvel_topic_name
                                    <<
                                    "\npath_pub:\n" << s_tab << "topic name:\n" << s_tab + s_tab << path_topic_name <<
                                    "\n\nspin rate:\n" << s_tab << spin_rate << '\n');
    return true;
}

/**
 * pathfinder::PrintStatus
 *
 * @return true if success; false otherwise.
 */
bool pathfinder::PrintStatus() {
    std::string s_tab = "    ";
    ROS_INFO_STREAM("\npose:\n" << s_tab << "x:\n" << s_tab + s_tab << cur_x <<
                                s_tab << "y:\n" << s_tab + s_tab << cur_y <<
                                s_tab << "yaw:\n" << s_tab + s_tab << orientation <<
                                "\nvelocity:\n" << s_tab << "current:\n" << s_tab + s_tab << "linear:\n"
                                << s_tab + s_tab + s_tab << cur_linear_velocity <<
                                s_tab + s_tab << "angular:\n" << s_tab + s_tab + s_tab << cur_rotate_velocity <<
                                s_tab << "target:\n" << s_tab + s_tab << "linear:\n" << s_tab + s_tab + s_tab
                                << tar_linear_velocity <<
                                s_tab + s_tab << "angular:\n" << s_tab + s_tab + s_tab << tar_rotate_velocity <<
                                "\ncones:\n" << s_tab << "number:\n" << s_tab + s_tab << cones.size() << '\n');
    return true;
}

/**
 * pathfinder::Start
 *
 * @return true if success; false otherwise.
 */
bool pathfinder::Start() {
    // follow the path defined by cones until no more valid track is detected.
    bool status = true;
    int count = 0;
    while (status && ros::ok()) {
        // attempt to read newest message
        ros::spinOnce();

        // print status every 1 s
        if (++count == spin_rate) {
            status &= PrintStatus();
            count = 0;
        }

        // if laserscan is used, find cones
        if (use_laserscan) {
            if (has_new_laserscan) {
                has_new_laserscan = false;
                if (find_cones()) {
                    has_new_cone = true;
                }
            }
        }

//        sort cones into left and right side
        if (has_new_cone) {
            has_new_cone = false;

            status &= (sort_cones(sort_type::left) | sort_cones(sort_type::right));

//            valid cones found
            if (status) {
//                start driving
                status &= drive();
            }
        }


        rate.sleep();
    }

    return status;
}

/**
 * pathfinder::Stop
 *
 * @return true if success; false otherwise.
 */
bool pathfinder::Stop() {
    return false;
}

/**
 * pathfinder::sort_cones
 *
 * @param st - left or right
 * @return true if success; false otherwise.
 */
bool pathfinder::sort_cones(sort_type st) {
//    sort cones into either left or right
//    return false if less than 2 cones are sorted as valid.

    std::vector<point_2d> *cone_vec_ptr = NULL;
    double compare_orientation;
    // based on sort type select correct vector and orientation to work with
    switch (st) {
        case sort_type::left:
            cone_vec_ptr = &left_cones;
            compare_orientation = orientation;
            break;

        case sort_type::right:
            cone_vec_ptr = &right_cones;
            compare_orientation = -orientation;
            break;
    }

    // get closest cone in left or right side as first element
    cone_vec_ptr->clear();
    for (auto iter = cones.begin(); iter != cones.end(); iter++) {
        if ((*iter).orientation > compare_orientation) {
            cone_vec_ptr->push_back(*iter);
            cones.erase(iter);
            break;
        }
    }

    // take the closest cone to the previous cone as next element
    // until exceed threshold distance
    double min_dist = cones_max_distance;
    do {
        min_dist = cones_max_distance;
        std::vector<point_2d>::iterator min_iter;
        for (auto iter = cones.begin(); iter != cones.end(); iter++) {
            double this_dist = (*iter) - cone_vec_ptr->front();
            if (this_dist < min_dist) {
                min_dist = this_dist;
                min_iter = iter;
            }
        }
        if (min_dist < cones_max_distance) {
            cone_vec_ptr->back().orientation = cone_vec_ptr->back() / (*min_iter);
            cone_vec_ptr->push_back(*min_iter);
            cones.erase(min_iter);
        }
    } while (min_dist >= cones_max_distance);

    // check number of valid cones
    if (cone_vec_ptr->size() < 2) {
        return false;
    }

    return true;
}

/**
 * pathfinder::read_parameters
 *
 * @return true if success; false otherwise.
 */
bool pathfinder::read_parameters() {
    std::string package_name = "pathfinder";
    std::string parameter_name;
    std::string *str_ptr = NULL;
    int *int_ptr = NULL;
    double *double_ptr = NULL;
    bool *bool_ptr = NULL;

    bool status = true;

    parameter_name = "use_laserscan";
    bool_ptr = &use_laserscan;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *bool_ptr);

    parameter_name = "spin_rate";
    int_ptr = &spin_rate;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *int_ptr);

    parameter_name = "cones_max_distance";
    double_ptr = &cones_max_distance;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *double_ptr);

    parameter_name = "odom_topic";
    str_ptr = &odom_topic_name;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *str_ptr);

    parameter_name = "cones_topic";
    str_ptr = &cones_topic_name;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *str_ptr);

    parameter_name = "laser_topic";
    str_ptr = &laser_topic_name;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *str_ptr);

    parameter_name = "cmdvel_topic";
    str_ptr = &cmdvel_topic_name;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *str_ptr);

    parameter_name = "path_topic";
    str_ptr = &path_topic_name;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *str_ptr);

    parameter_name = "clustering_threshold";
    double_ptr = &clustering_threshold;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *double_ptr);

    parameter_name = "publish_path";
    bool_ptr = &publish_path;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *bool_ptr);

    parameter_name = "num_of_path_points";
    int_ptr = &num_of_path_points;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *int_ptr);

    parameter_name = "time_interval";
    double_ptr = &time_interval;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *double_ptr);

    

    return status;
}

/**
 * pathfinder::setup_subscribers
 *
 * @return true if success; false otherwise.
 */
bool pathfinder::setup_subscribers() {
    bool status = true;

    // odom_sub
    status = status && !odom_topic_name.empty();
    assert(!odom_topic_name.empty());
    odom_sub = n.subscribe(odom_topic_name, 1, &pathfinder::odom_cb, this);

    // cones_sub
    if (!use_laserscan) {
        status = status && !cones_topic_name.empty();
        assert(!cones_topic_name.empty());
        cones_sub = n.subscribe(cones_topic_name, 1, &pathfinder::cones_cb, this);
    }

    // laser_sub
    if (use_laserscan) {
        status = status && !laser_topic_name.empty();
        assert(!laser_topic_name.empty());
        laser_sub = n.subscribe(laser_topic_name, 1, &pathfinder::laser_cb, this);
    }

    return status;
}

/**
 * pathfinder::setup_publishers
 *
 * @return true if success; false otherwise.
 */
bool pathfinder::setup_publishers() {
    bool status = true;

    // cmdvel_pub
    status = status && !cmdvel_topic_name.empty();
    assert(!cmdvel_topic_name.empty());
    cmdvel_pub = n.advertise<geometry_msgs::Twist>(cmdvel_topic_name, 1);

    // path_pub
    status = status && !path_topic_name.empty();
    if (status) {
        path_pub = n.advertise<visualization_msgs::MarkerArray>(path_topic_name, 1);
    }

    return status;
}

/**
 * pathfinder::odom_cb
 *
 * @param msg - nav_msgs::OdometryConstPtr
 */
void pathfinder::odom_cb(nav_msgs::OdometryConstPtr msg) {
    assert(msg != NULL);

    // get x & y
    cur_x = msg->pose.pose.position.x;
    cur_y = msg->pose.pose.position.y;

    // get orientation
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    orientation = tf::getYaw(q);

    // get velocity
    cur_linear_velocity = msg->twist.twist.linear.x;
    cur_rotate_velocity = msg->twist.twist.angular.z;
}

/**
 * pathfinder::cones_cb
 *
 * @param msg - visualization_msgs::MarkerArrayConstPtr
 */
void pathfinder::cones_cb(visualization_msgs::MarkerArrayConstPtr msg) {
    assert(msg != NULL);
    assert(!msg->markers.empty());
    has_new_cone = true;
    cones.clear();
    auto iter = msg->markers.begin();
    for (; iter != msg->markers.end(); iter++) {
//        TODO: find proper size of objects
        assert((*iter).header.frame_id == "base_link");
        cones.emplace_back((*iter).pose.position.x, (*iter).pose.position.y, 0.5);
    }
    std::sort(cones.begin(), cones.end());
}

/**
 * pathfinder::laser_cb
 *
 * @param msg - sensor_msgs::LaserScanConstPtr msg
 */
void pathfinder::laser_cb(sensor_msgs::LaserScanConstPtr msg) {
    assert(msg != NULL);
    assert(!msg->ranges.empty());

    // load ros sensor msg into range_1d
    has_new_laserscan = true;
    try {
        laserscan = range_1d(msg, 90.0, -90.0);
    } catch (const std::exception &e) {
        std::cerr << e.what();
        exit(-1);
    }
}

/**
 * pathfinder::drive
 *
 * @return true if success; false otherwise.
 */
bool pathfinder::drive() {
    double mean_ori_left = 0;
    double mean_ori_right = 0;
    int sign = 0;

    // get mean orientation of left track
    for (auto iter = left_cones.begin(); iter != left_cones.end() - 1; iter++) {
        mean_ori_left += (*iter).orientation;
    }
    mean_ori_left /= (left_cones.size() - 1);

    // get mean orientation of right track
    for (auto iter = right_cones.begin(); iter != right_cones.end() - 1; iter++) {
        mean_ori_right += (*iter).orientation;
    }
    mean_ori_right /= (right_cones.size() - 1);

    // get sign based on mean orientation
    if ((mean_ori_right + mean_ori_left) / 2.0 > 0) {
        sign = -1;    // curving to right

    } else {
        sign = 1;     // curving to left
    }

    // get minimum allowable steering angle
    double min_angle = check_outer_track(sign);

    // get maximum allowable steering angle
    double max_angle = check_inner_track(sign);

    // invalid steering angle
    if (min_angle > max_angle) {
        stop();
        return false;
    }

    // control car using the mean of the allowable range
    double mean_angle = (min_angle + max_angle) / 2.0;
    curve(mean_angle, sign);
    return true;

}

/**
 * pathfinder::check_outer_track
 *
 * @param sign - 1 for driving left; -1 for driving right
 * @return minimum allowable steering angle
 */
double pathfinder::check_outer_track(int sign) {
    std::vector<point_2d> *cone_vec_ptr = NULL;
    point_2d center_point;

    // select vector based on sign
    switch (sign) {
        case 1:
            cone_vec_ptr = &left_cones;
            break;

        case -1:
            cone_vec_ptr = &right_cones;
            break;

        default:
            exit(-1);
    }

    // get minimum allowable angle following outer track
    double min_angle = 361.0;
    {
        std::vector<double>::iterator r_iter, a_iter;
        for (r_iter = r_meter.begin(), a_iter = angle_degree.begin(); r_iter != r_meter.end(); r_iter++, a_iter++) {
            bool got_desired = false;
            center_point.x = 0;
            center_point.y = (double) sign * (*r_iter);
            for (auto p_iter = cone_vec_ptr->begin(); p_iter != cone_vec_ptr->end(); p_iter++) {
                if (center_point - (*p_iter) <= (*r_iter + cones_max_distance)) {
                    got_desired = true;
                    break;
                }
            }
            if (got_desired) {
                min_angle = *(r_iter - 1);
            }
        }
    }

    return min_angle;
}

/**
 * pathfinder::check_inner_track
 *
 * @param sign - 1 for driving left; -1 for driving right
 * @return maximum allowable steering angle
 */
double pathfinder::check_inner_track(int sign) {
    std::vector<point_2d> *cone_vec_ptr = NULL;
    point_2d center_point;

    // select vector based on sign
    switch (sign) {
        case 1:
            cone_vec_ptr = &right_cones;
            break;

        case -1:
            cone_vec_ptr = &left_cones;
            break;

        default:
            exit(-1);
    }

    // get maximum allowable steering angle following inner track
    double max_angle = 0.0;
    {
        std::vector<double>::reverse_iterator r_iter, a_iter;
        for (r_iter = r_meter.rbegin(), a_iter = angle_degree.rbegin(); r_iter != r_meter.rend(); r_iter--, a_iter--) {
            bool got_desired = false;
            center_point.x = 0;
            center_point.y = (double) sign * (*r_iter);
            for (auto p_iter = cone_vec_ptr->begin(); p_iter != cone_vec_ptr->end(); p_iter++) {
                if (center_point - (*p_iter) <= (*r_iter + cones_max_distance)) {
                    got_desired = true;
                    break;
                }
            }

            if (got_desired) {
                max_angle = *(r_iter + 1);
            }
        }
    }

    return max_angle;
}

/**
 * pathfinder::stop
 *
 */
void pathfinder::stop() {
    // stop
    geometry_msgs::Twist cmdvel;
    cmdvel.linear.x = 0.0;
    cmdvel.angular.z = tar_steering_angle;
    tar_linear_velocity = cmdvel.linear.x;
    cmdvel_pub.publish(cmdvel);
}

/**
 * pathfinder::curve
 *
 * @param angle - steering angle
 * @param sign - steering direction: 1 for left; -1 for right
 */
void pathfinder::curve(double angle, int sign) {
    // curve
    geometry_msgs::Twist cmdvel;
    cmdvel.linear.x = 5.0;
    cmdvel.angular.z = (double) sign * angle / 180.0 * M_PI;
    tar_linear_velocity = cmdvel.linear.x;
    tar_steering_angle = cmdvel.angular.z;
    tar_rotate_velocity = tar_linear_velocity * sin(cmdvel.angular.z) / DIST_FRONT_TO_REAR;
    cmdvel_pub.publish(cmdvel);
}

/**
 * pathfinder::find_cones
 *
 * @return true if success; false otherwise.
 */
bool pathfinder::find_cones() {

    // clustering
    std::vector<range_1d_cluster> clusters;
    bool new_cluster = true;
    range_1d_cluster cluster;
    for (double current_angle = laserscan.min_angle;
         current_angle <= laserscan.max_angle; current_angle += laserscan.resolution) {
        if (new_cluster) {
            cluster = range_1d_cluster(current_angle, laserscan[current_angle]);
            new_cluster = false;
        } else {
            if (!cluster.push(current_angle, laserscan[current_angle], clustering_threshold)) {
                if (cluster.size() < clustering_threshold) {
                    clusters.push_back(cluster);
                }
                cluster = range_1d_cluster(current_angle, laserscan[current_angle]);
            }
        }
    }
    clusters.push_back(cluster);

    // put clusters into point_2d vector
    cones.clear();
    for (auto iter = clusters.begin(); iter != clusters.end(); iter++) {
        cones.emplace_back((*iter).get_x(), (*iter).get_y(), (*iter).size());
    }

    return !cones.empty();
}

bool pathfinder::visualise_path(double time_interval, int num_of_points) {
    visualization_msgs::MarkerArray path_points;
    for(int i = 0; i < num_of_points; i++) {
        double time_spent = time_interval * (i + 1.0);
        visualization_msgs::Marker path_point;
        path_point.header.frame_id = "base_link";
        path_point.action = visualization_msgs::Marker::ADD;
        path_point.type = visualization_msgs::Marker::ARROW;
        path_point.color.a = 1.0;
        path_point.color.r = 1.0;
        path_point.color.g = 1.0;
        path_point.color.b = 0.0;
        path_point.scale.x = 0.5;
        path_point.scale.y = 0.1;
        path_point.scale.z = 0.1;
        double yaw_change = time_spent * tar_rotate_velocity;
        double turning_radius = DIST_LIDAR_TO_CAR / sin(tar_steering_angle);
        tf::Quaternion q = tf::createQuaternionFromYaw(yaw_change);
        tf::quaternionTFToMsg(q, path_point.pose.orientation);
        path_point.pose.position.z = 0.0;
        path_point.pose.position.x = sin(yaw_change) * turning_radius;
        path_point.pose.position.y = cos(yaw_change) * turning_radius;
        path_point.id = i;
        path_points.markers.push_back(path_point);
    }
    path_pub.publish(path_points);
}