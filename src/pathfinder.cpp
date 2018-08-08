//
// Created by chaoz on 26/07/18.
//

#include <pathfinder/pathfinder.h>
#include <functional>
#include <algorithm>
#include <cerrno>


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
    r_meter = {4.25916285, 5.26284792, 6.954665949, 10.36578687, 20.65268384, 41.26605413,
               51.57667503,
               103.1376393};
    angle_degree = {25.0, 20.0, 15.0, 10.0, 5.0, 2.5, 2.0, 1.0};
//    std::cout << r_meter.size() << " | " << angle_degree.size() << std::endl;
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
    static std::string s_tab = "    ";
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
    char key = 0;
    while (status && ros::ok()) {
        // attempt to read newest message
        ros::spinOnce();

        // if laserscan is used, find cones
        if (use_laserscan) {
            if (has_new_laserscan) {
                has_new_laserscan = false;
//                std::cout<<"has new laserscan\n"<<"continue to find cones.\n";
//                std::cin>>key;
                if (find_cones()) {
                    has_new_cone = true;
//                    std::cout << "found cones: " << cones.size() << std::endl;
                }
            }
        }

        // print status every 1 s
        if (++count == spin_rate) {
            //status &= PrintStatus();
            count = 0;
        }

        if(has_new_cone) {
            has_new_cone = false;
//            std::cout<<"has new cones\n"<<"continue to publish cones\n";
//            std::cin>>key;
            if (publish_cones) {
//                std::cout << "start publishing cones" << std::endl;
//                visualise_cones(cones, "cones", MARKER_COLOR::GREEN);
//                visualise_cones(left_cones, "left_cones", MARKER_COLOR::RED);
//                visualise_cones(right_cones, "right_cones", MARKER_COLOR::AUTO);
//                visualise_cones(invalid_cones, "invalid_cones", MARKER_COLOR::AUTO);
            }
        }

//          valid cones found
//          start driving
//        std::cout << "start driving mode: " << mode << std::endl;
//        std::cin>>key;
        switch (mode) {
            case 1:
                assert(false);
                break;

            case 2:
                drive2();
                visualise_cones(left_cones, "left_cones", MARKER_COLOR::RED);
                if(publish_boundary) {
                    visualise_boundary();
                }
                break;

            default:
                assert(false);
                exit(-1);
        }

        if (publish_path) {
            visualise_path(time_interval, num_of_path_points);
        }

        if (publish_ranges) {
            visualise_ranges();
        }

        cones.clear();
        valid_ranges.clear();

        rate.sleep();
//        std::cout << "sleep done" << std::endl;
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
 * pathfinder::read_parameters
 *
 * @return true if success; false otherwise.
 */
bool pathfinder::read_parameters() {
    std::string package_name = "pathfinder";
    std::string parameter_name;
    std::string *str_ptr = nullptr;
    int *int_ptr = nullptr;
    double *double_ptr = nullptr;
    bool *bool_ptr = nullptr;

    bool status = true;

    parameter_name = "mode";
    int_ptr = &mode;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *int_ptr);

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

    parameter_name = "publish_cones";
    bool_ptr = &publish_cones;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *bool_ptr);

    parameter_name = "frame_id";
    str_ptr = &frame_id;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *str_ptr);

    parameter_name = "max_steering";
    max_steering = 24.0;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, max_steering);
    max_steering = max_steering / 180.0 * M_PI;
    initial_range = steering_range(-2.0*max_steering, 2.0*max_steering, DIST_FRONT_TO_REAR);

    parameter_name = "processing_range";
    double_ptr = &processing_range;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *double_ptr);

    parameter_name = "clearance_radius";
    double_ptr = &clearance_radius;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *double_ptr);

    parameter_name = "ranges_topic";
    str_ptr = &ranges_topic_name;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *str_ptr);

    parameter_name = "publish_ranges";
    bool_ptr = &publish_ranges;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *bool_ptr);

    parameter_name = "boundary_topic";
    str_ptr = &boundary_topic_name;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *str_ptr);

    parameter_name = "publish_boundary";
    bool_ptr = &publish_boundary;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *bool_ptr);

    parameter_name = "apply_median_filter";
    bool_ptr = &apply_median_filter;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *bool_ptr);

    parameter_name = "filter_size";
    int_ptr = &filter_size;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *int_ptr);
    
    parameter_name = "driving_speed";
    double_ptr = &fix_driving_speed;
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
    if (publish_path) {
        status = status && !path_topic_name.empty();
        assert(!path_topic_name.empty());
        path_pub = n.advertise<visualization_msgs::MarkerArray>(path_topic_name, 1);
    }

    if (publish_cones) {
        status = status && !cones_topic_name.empty();
        assert(!cones_topic_name.empty());
        cones_pub = n.advertise<visualization_msgs::MarkerArray>(cones_topic_name, 1);
    }

    if (publish_ranges) {
        status = status && !ranges_topic_name.empty();
        assert(!ranges_topic_name.empty());
        ranges_pub = n.advertise<visualization_msgs::MarkerArray>(ranges_topic_name, 1);
    }

    if (publish_boundary) {
        status = status && !boundary_topic_name.empty();
        assert(!boundary_topic_name.empty());
        boundary_pub = n.advertise<visualization_msgs::MarkerArray>(boundary_topic_name, 1);
    }

    return status;
}

/**
 * pathfinder::odom_cb
 *
 * @param msg - nav_msgs::OdometryConstPtr
 */
void pathfinder::odom_cb(nav_msgs::OdometryConstPtr msg) {
    assert(msg != nullptr);

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
    assert(msg != nullptr);
    assert(!msg->markers.empty());
    has_new_cone = true;
    cones.clear();
    auto iter = msg->markers.begin();
    for (; iter != msg->markers.end(); iter++) {
//        TODO: find proper size of objects
        //assert((*iter).header.frame_id == "base_link");
        cones.emplace_back((*iter).pose.position.x, (*iter).pose.position.y, 0.5, mode);
    }
    cones.sort();
    ROS_INFO_STREAM("cones_size:" << cones.size());
}

/**
 * pathfinder::laser_cb
 *
 * @param msg - sensor_msgs::LaserScanConstPtr msg
 */
void pathfinder::laser_cb(sensor_msgs::LaserScanConstPtr msg) {
    assert(msg != nullptr);
    assert(!msg->ranges.empty());

    // load ros sensor msg into range_1d
    has_new_laserscan = true;
    try {
        laserscan = range_1d(msg, 90.0, -90.0);
    } catch (const std::exception &e) {
        std::cerr << e.what();
        exit(-1);
    }

    if(apply_median_filter) {
        laserscan.median_filter(filter_size);
    }
//    std::cout << "get new laserscan" << std::endl;
}

bool pathfinder::drive2() {
    // init range list
//    std::cout<<"init ranges"<<std::endl;
    valid_ranges.clear();
    valid_ranges.push_back(initial_range);
    left_cones.clear();
    double key;
//    std::cout<<"iterating cones"<<std::endl;
//    std::cin>>key;
    int count = 0;
    // iterate through the cones and identify the collision free steering ranges
    for (auto cones_iter = cones.begin(); cones_iter != cones.end();) {
//        std::cout<<"cone "<< count++ <<":\n";
//        std::cout<<"    dist:"<<cones_iter->get_dist_to_car()<<"\n";
        // check if the cone is outside of the processing range
//        std::cout<<"check ranges"<<std::endl;
        if (cones_iter->get_dist_to_car() < processing_range) {
            left_cones.push_back(*cones_iter);
            // if the cone is inside the processing range
            // calculate the boundary of collision steering at given cone location
            // check existence of path
//            std::cout<<"update ranges"<<std::endl;
            if (!update_steering_ranges(*cones_iter)) {
                // no available path
                // stop and return false
//                std::cout<<"stop"<<std::endl;
                stop();
                return false;
            }
        }
        visualise_cones(left_cones, "left_cones", MARKER_COLOR::RED);
//        double cc = 0;
//        for(auto iter = valid_ranges.begin(); iter != valid_ranges.end(); iter++) {
//            std::cout<<"ranges "<<cc++<<":\n";
//            std::cout<<"    max:"<<iter->get_max()<<"\n";
//            std::cout<<"    min:"<<iter->get_min()<<"\n";
//        }
//        std::cin>> key;
        // remove from list
//        std::cout<<"remove cones"<<std::endl;
        cones_iter = cones.erase(cones_iter);
    }

    // select the largest steering range (prefer as small change as possible to current steering)
    auto desired_range_iter = valid_ranges.begin();
    double max_size = desired_range_iter->get_size();
    double min_change = fabs(desired_range_iter->get_mean() - tar_steering_angle);
    count = 0;
    for (auto range_iter = valid_ranges.begin(); range_iter != valid_ranges.end(); range_iter++) {
        if(range_iter->get_max() < -max_steering) {
            continue;
        }
        if(range_iter->get_min() > max_steering) {
            continue;
        }
        double this_size = range_iter->get_size();
//        std::cout<<"range "<<count++<<std::endl;
//        std::cout<<"    max: "<< range_iter->get_max()<<std::endl;
//        std::cout<<"    min: "<< range_iter->get_min()<<std::endl;
//        std::cout<<"    size: "<< this_size<<std::endl;
//        std::cout<<"    mean: "<< range_iter->get_mean()<<std::endl;
        // check size first
        if (this_size > max_size) {
            // update with larger range
            max_size = this_size;
            desired_range_iter = range_iter;
        } else if (this_size == max_size) {
            // if range is same
            double this_change = fabs(range_iter->get_mean() - tar_steering_angle);
            // check change in steering
            if (this_change < min_change) {
                // update with smaller change option
                min_change = this_change;
                desired_range_iter = range_iter;
            }
        }
    }


//    std::cout<<"range used:"<<std::endl;
//    std::cout<<"    max: "<< desired_range_iter->get_max()<<std::endl;
//    std::cout<<"    min: "<< desired_range_iter->get_min()<<std::endl;
//    std::cout<<"    size: "<< desired_range_iter->get_size()<<std::endl;
//    std::cout<<"    mean: "<< desired_range_iter->get_mean()<<std::endl;
    double steering_angle = desired_range_iter->get_mean();
    if(steering_angle < -max_steering) {
        steering_angle = -max_steering;
    }
    if(steering_angle > max_steering) {
        steering_angle = max_steering;
    }
    // drive using mean steering for that range
    curve(steering_angle);
    return true;
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
void pathfinder::curve(double angle) {
    // curve
    geometry_msgs::Twist cmdvel;
    cmdvel.linear.x = fix_driving_speed;
    cmdvel.angular.z = angle;
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
    for (double current_angle = laserscan.get_min_angle();
         current_angle <= laserscan.get_max_angle();) {
        if (new_cluster) {
            cluster = range_1d_cluster(current_angle, laserscan[current_angle]);
            new_cluster = false;
        } else {
            if (!cluster.push(current_angle, laserscan[current_angle], clustering_threshold)) {
                if (cluster.size() < clustering_threshold && cluster.size() > 0.05) {
//                    std::cout << "cluster size: " << cluster.size() << std::endl;
//                    std::cout << "x: " << cluster.get_x() << " y: " << cluster.get_y() << std::endl;
//                    std::cout << "from " << cluster.getClosest_distance() << " to " << cluster.getFarthest_distance()
//                              << std::endl;
                    clusters.push_back(cluster);
                }
                cluster = range_1d_cluster(current_angle, laserscan[current_angle]);
            }
        }
        current_angle += laserscan.get_resolution();
    }

    // put clusters into point_2d vector
    cones.clear();
    for (auto iter = clusters.begin(); iter != clusters.end(); iter++) {
        cones.emplace_back((*iter).get_x(), (*iter).get_y(), (*iter).size(), mode);
    }

    return !cones.empty();
}

bool pathfinder::visualise_path(double time_interval, int num_of_points) {
    static std::string ns = "/pathfinder/path";

    // clear previous path
    path_pub.publish(get_clear_markers(ns));

    // generate msg fro new path
    visualization_msgs::MarkerArray markers;
    double turning_radius = DIST_FRONT_TO_REAR / sin(tar_steering_angle);
//    turning_radius = fabs(turning_radius);
    for (int i = 0; i < num_of_points; i++) {
        double time_spent = time_interval * (i + 1.0);
        visualization_msgs::Marker marker;
        marker.ns = ns;
        marker.header.frame_id = "base_link";
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.lifetime = ros::Duration(rate);
        double yaw_change = time_spent * tar_rotate_velocity;
        tf::Quaternion q = tf::createQuaternionFromYaw(yaw_change);
        tf::quaternionTFToMsg(q, marker.pose.orientation);
        marker.pose.position.z = 0.0;
//        if(yaw_change > 0.0){
//            marker.pose.position.x = sin(yaw_change) * turning_radius;
//            marker.pose.position.y = turning_radius - cos(yaw_change) * turning_radius;
//        }
//        if(yaw_change < 0.0) {
//            marker.pose.position.x = - sin(yaw_change) * turning_radius;
//            marker.pose.position.y = - turning_radius + cos(yaw_change) * turning_radius;
//        }
        if(yaw_change == 0.0) {
            marker.pose.position.x = time_spent*tar_linear_velocity;
            marker.pose.position.y = 0.0;
        } else {
            marker.pose.position.x = sin(yaw_change) * turning_radius;
            marker.pose.position.y = turning_radius - cos(yaw_change) * turning_radius;
        }
        marker.id = i;
        markers.markers.push_back(marker);
//        std::cout<<"yaw_change:"<<yaw_change<<" | turning_radius:"<<turning_radius<<std::endl;
    }

    // publish
    path_pub.publish(markers);
}

bool pathfinder::visualise_cones(std::vector<cone_2d> &tar_cones, std::string name, MARKER_COLOR color) {
    static std::deque<std::string> ns_list;

    // check empty of input vector
    if (tar_cones.empty()) {
        return false;
    }

    int ns_id = ns_list.size();
    for (int i = 0; i < ns_list.size(); i++) {
        if (ns_list[i] == name) {
            ns_id = i;
        }
    }
    if (ns_id == ns_list.size()) {
        ns_list.push_back(name);
    }

    // clear any previous markers published
    cones_pub.publish(get_clear_markers(name));

    std_msgs::ColorRGBA colorRGBA;
    colorRGBA.a = 1.0;

    // auto color code
    if (color == MARKER_COLOR::AUTO) {
        color = static_cast<MARKER_COLOR>(ns_id % static_cast<int>(MARKER_COLOR::NUM_OF_COLOR));
    }

    // decode color code into ColorRGBA
    switch (color) {
        case MARKER_COLOR::BLACK:
            break;

        case MARKER_COLOR::BLUE:
            colorRGBA.b = 1.0;
            break;

        case MARKER_COLOR::GREEN:
            colorRGBA.g = 1.0;
            break;

        case MARKER_COLOR::RED:
            colorRGBA.r = 1.0;
            break;

        case MARKER_COLOR::YELLOW:
            colorRGBA.r = 1.0;
            colorRGBA.g = 1.0;
            break;

        case MARKER_COLOR::WHITE:
            colorRGBA.b = 1.0;
            colorRGBA.g = 1.0;
            colorRGBA.r = 1.0;
            break;

        default:
            assert(false);
            break;
    }



    // generate markers from input vector
    visualization_msgs::MarkerArray markers;
    for (int i = 0; i < tar_cones.size(); i++) {
        double time_spent = time_interval * (i + 1.0);
        visualization_msgs::Marker marker;
        marker.ns = name;
        marker.header.frame_id = "laser";
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.color = colorRGBA;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.5;
        //marker.lifetime = ros::Duration(rate);
        tf::poseTFToMsg(tf::Transform(tf::createQuaternionFromRPY(0.0, 0.0, 0.0),
                                      tf::Vector3(tar_cones[i].get_x(), tar_cones[i].get_y(), 0.0)),
                        marker.pose);
        marker.id = i;
        markers.markers.push_back(marker);
    }

    // publish msg
    cones_pub.publish(markers);
}

bool pathfinder::visualise_cones(std::list<cone_2d> &tar_cones, std::string name, MARKER_COLOR color) {
    static std::deque<std::string> ns_list;

    // check empty of input vector
    if (tar_cones.empty()) {
        return false;
    }

    int ns_id = ns_list.size();
    for (int i = 0; i < ns_list.size(); i++) {
        if (ns_list[i] == name) {
            ns_id = i;
        }
    }
    if (ns_id == ns_list.size()) {
        ns_list.push_back(name);
    }

    // clear any previous markers published
    cones_pub.publish(get_clear_markers(name));

    std_msgs::ColorRGBA colorRGBA;
    colorRGBA.a = 1.0;

    // auto color code
    if (color == MARKER_COLOR::AUTO) {
        color = static_cast<MARKER_COLOR>(ns_id % static_cast<int>(MARKER_COLOR::NUM_OF_COLOR));
    }

    // decode color code into ColorRGBA
    switch (color) {
        case MARKER_COLOR::BLACK:
            break;

        case MARKER_COLOR::BLUE:
            colorRGBA.b = 1.0;
            break;

        case MARKER_COLOR::GREEN:
            colorRGBA.g = 1.0;
            break;

        case MARKER_COLOR::RED:
            colorRGBA.r = 1.0;
            break;

        case MARKER_COLOR::YELLOW:
            colorRGBA.r = 1.0;
            colorRGBA.g = 1.0;
            break;

        case MARKER_COLOR::WHITE:
            colorRGBA.b = 1.0;
            colorRGBA.g = 1.0;
            colorRGBA.r = 1.0;
            break;

        default:
            assert(false);
            break;
    }


    int i = 0;
    // generate markers from input vector
    visualization_msgs::MarkerArray markers;
    for (auto iter = tar_cones.begin(); iter != tar_cones.end(); iter++) {
        double time_spent = time_interval * (i + 1.0);
        visualization_msgs::Marker marker;
        marker.ns = name;
        marker.header.frame_id = "laser";
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.color = colorRGBA;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.5;
        marker.lifetime = ros::Duration(rate);
        tf::poseTFToMsg(tf::Transform(tf::createQuaternionFromRPY(0.0, 0.0, 0.0),
                                      tf::Vector3(iter->get_x(), iter->get_y(), 0.0)),
                        marker.pose);
        marker.id = i;
        markers.markers.push_back(marker);
        i++;
    }

    // publish msg
    cones_pub.publish(markers);
}

visualization_msgs::MarkerArray &pathfinder::get_clear_markers(std::string name) {
    static visualization_msgs::MarkerArray markers = _get_clear_markers(frame_id);
    markers.markers.back().ns = name;
    return markers;
}

visualization_msgs::MarkerArray pathfinder::_get_clear_markers(std::string frame_id) {
    visualization_msgs::MarkerArray markers;
    markers.markers.resize(1);
    markers.markers[0].header.frame_id = frame_id;
    markers.markers[0].action = visualization_msgs::Marker::DELETEALL;
    return markers;
}

bool pathfinder::update_steering_ranges(cone_2d &cone) {
    double min_steering = evaluate_boundary_steering(cone.get_x(), cone.get_y(), -clearance_radius);
    double max_steering = evaluate_boundary_steering(cone.get_x(), cone.get_y(), clearance_radius);
//    std::cout<<"\nx:"<<cone.get_x()<<" y:"<<cone.get_y()<<std::endl;
//    std::cout<<"max steering:"<<max_steering<<std::endl;
//    std::cout<<"min steering:"<<min_steering<<std::endl;
//    std::cout<<"check and combine ranges:"<<std::endl;
    int count = 0;
    for (auto ranges_iter = valid_ranges.begin(); ranges_iter != valid_ranges.end();) {
//        std::cout<<"ranges no."<<count++<<std::endl;
//        std::cout<<"max:"<<ranges_iter->get_max()<<"\n";
//        std::cout<<"min:"<<ranges_iter->get_min()<<"\n";
        // all good
        if (min_steering >= ranges_iter->get_max()) {
//            std::cout<<"all good min: "<<min_steering<<">="<<ranges_iter->get_max()<<std::endl;
            ranges_iter++;
            continue;
        }

        // all good
        if (max_steering <= ranges_iter->get_min()) {
//            std::cout<<"all good max: "<<max_steering<<"<="<<ranges_iter->get_min()<<std::endl;
            ranges_iter++;
            continue;
        }

        // no common interval
        if (min_steering < ranges_iter->get_min() && max_steering > ranges_iter->get_max()) {
//            std::cout<<"no common:\n";
//            std::cout<<min_steering<<"<"<<ranges_iter->get_min()<<"&&"<<max_steering<<">"<<ranges_iter->get_max()<<std::endl;
            ranges_iter = valid_ranges.erase(ranges_iter);
            if (valid_ranges.empty()) {
//                std::cout<<"no available ranges\n";
                return false;
            }
        }

        // split into two range
        if (min_steering >= ranges_iter->get_min() && max_steering <= ranges_iter->get_max()) {
//            std::cout<<"split into two\n";
//            std::cout<<"range:"<<min_steering<<" | "<<ranges_iter->get_min()<<'\n';
//            std::cout<<"range:"<<ranges_iter->get_max()<<" | "<<max_steering<<'\n';
            steering_range smaller_range(ranges_iter->get_min(), min_steering, ranges_iter->get_front_rear_distance());
            ranges_iter->set_min(max_steering);
            valid_ranges.insert(ranges_iter, smaller_range);
            return true;
        }

        // update steering
        if (max_steering <= ranges_iter->get_max()) {
//            std::cout<<"update min\n";
            ranges_iter->set_min(max_steering);
        }

        if (min_steering >= ranges_iter->get_min()) {
//            std::cout<<"update max\n";
            ranges_iter->set_max(min_steering);
        }
        ranges_iter++;
    }

    return true;

}

double pathfinder::evaluate_boundary_steering(double x, double y, double shift) {
    double _y = y + shift;
//    int sign;
//    if(_y<0.0) {
//        sign = -1;
//        _y = -_y;
//    } else {
//        sign = 1;
//    }
    x = x + DIST_LIDAR_TO_CAR;
    if (_y == 0.0) {
        return 0.0;
    }
    double len = x / sin(2 * atan2(x, _y)) - shift;
    if(len == 0.0) {
        return 0.0;
    }
    double rad = DIST_FRONT_TO_REAR / len;
    if (rad > 1.0) {
        rad = 1.0;
    }
    else if (rad < -1.0) {
        rad = -1.0;
    }

    return asin(rad);
}

bool pathfinder::visualise_ranges() {
    // clear previous path
    ranges_pub.publish(get_clear_markers(" "));
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    int count = 0;
    // generate msg fro new path
    visualization_msgs::MarkerArray markers;
    for(auto iter = valid_ranges.begin(); iter != valid_ranges.end(); iter++) {
        color.g = 0.0;
        color.b = 1.0;
        get_steering_marker(std::to_string(count++), iter->get_max(), markers, color);
        get_steering_marker(std::to_string(count++), iter->get_min(), markers, color);

        color.b = 0.0;
        color.g = 1.0;
        get_steering_marker(std::to_string(count++), iter->get_mean(), markers, color);
    }

    ranges_pub.publish(markers);

    return false;
}

bool pathfinder::get_steering_marker(std::string name,
                                     double steering,
                                     visualization_msgs::MarkerArray& markers,
                                     std_msgs::ColorRGBA color) {
    double turning_radius = DIST_FRONT_TO_REAR / sin(steering);
    double rot_vel = 5.0 / turning_radius;


//    turning_radius = fabs(turning_radius);
    for (int i = 0; i < num_of_path_points; i++) {
        double time_spent = time_interval * (i + 1.0);
        visualization_msgs::Marker marker;
        marker.ns = name;
        marker.header.frame_id = "base_link";
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.color = color;
        marker.scale.x = 0.5;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.lifetime = ros::Duration(rate);
        double yaw_change = time_spent * rot_vel;
        tf::Quaternion q = tf::createQuaternionFromYaw(yaw_change);
        tf::quaternionTFToMsg(q, marker.pose.orientation);
//            marker.pose.position.z = 0.0;
//        if(yaw_change > 0.0){
//            marker.pose.position.x = sin(yaw_change) * turning_radius;
//            marker.pose.position.y = turning_radius - cos(yaw_change) * turning_radius;
//        }
//        if(yaw_change < 0.0) {
//            marker.pose.position.x = - sin(yaw_change) * turning_radius;
//            marker.pose.position.y = - turning_radius + cos(yaw_change) * turning_radius;
//        }
        if(yaw_change == 0.0) {
            marker.pose.position.x = time_spent*tar_linear_velocity;
//                marker.pose.position.y = 0.0;
        } else {
            marker.pose.position.x = sin(yaw_change) * turning_radius;
            marker.pose.position.y = turning_radius - cos(yaw_change) * turning_radius;
        }
        marker.id = i;
        markers.markers.push_back(marker);
    }

    return false;
}

bool pathfinder::visualise_boundary() {
    int count = 0;
    double steering;
    visualization_msgs::MarkerArray markers;
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    for(auto iter = left_cones.begin(); iter != left_cones.end(); iter++) {
        steering = evaluate_boundary_steering(iter->get_x(), iter->get_y(), -clearance_radius);
        color.b = 1.0;
        color.g = 1.0;
        color.r = 0.0;
        get_steering_marker(std::to_string(count++),steering,markers, color);


        steering = evaluate_boundary_steering(iter->get_x(), iter->get_y(), clearance_radius);
        color.b = 0.0;
        color.g = 0.0;
        color.r = 1.0;
        get_steering_marker(std::to_string(count++),steering,markers, color);
    }

    boundary_pub.publish(markers);
    return false;
}
