// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

// standard
#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <random>

#include <nav_msgs/OccupancyGrid.h>
// Struct defining the Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
using namespace std;
typedef struct Node {
    double x, y;
    double cost; // only used for RRT*
    int id;
    int parent; // index of parent node in the tree vector
    bool is_root = false;
} Node;


class RRT {
public:
    RRT(ros::NodeHandle &nh);
    virtual ~RRT();
private:
    ros::NodeHandle nh_;
    // ros pub/sub
    // TODO: add the publishers and subscribers you need

    ros::Subscriber pf_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber map_sub_;
    ros::Publisher marker_pub;
    ros::Publisher drive_pub;
    // tf stuff
    tf::TransformListener listener;

    visualization_msgs::Marker marker;
    visualization_msgs::Marker path_line;
    visualization_msgs::Marker velo_points;

    // TODO: create RRT params

    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;

    bool RRT_star;

    double car_pos_x;
    double car_pos_y;
    int flag = 0;
    double L = 0.6 ;  //0.8 works 1.4   1.6
    double P = 0.22;  //0.21   0.22
    double understeer_gain = 10;  //0.21   0.22
    double velocity_gamma = 1;
    vector<float> L_velocity = {0.5,1,1.5,2}; //velocity lookahead distances


    double resol;
    int wid;
    int hei;
    double origin_x;
    double origin_y;
    double L_waypoints;
    double rrt_radius;
    int rrt_iteration;
    double prop_gain;
    double global_L_follow;
    std::string waypoint_path;
    vector<int8_t> map_data;
    vector<float> range_data;

    double sample_lx;
    double sample_ly;

    double rrt_goal_dist;
    double inflation;

    double steer1;
    double vel_1;
    double steer2;
    double vel_2;
    double vel_3;

    bool show_obstacles;
    bool show_rrt_path;
    bool show_tree;
    bool show_ball;

    double steer_param;
    double steer_param_2;

    double angle_min;
    double angle_max;
    double angle_incre;

    bool waypoint_load_flag = true;
    bool load_map_flag = true;


    std::vector<std::vector<std::string> > dataList;
    vector<vector<float>> data_int;

    ackermann_msgs::AckermannDriveStamped drive_msg;
    // callbacks
    // where rrt actually happens
    void pf_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);


    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
    // RRT methods
    std::vector<double> sample(double start_x,double start_y, double goal_x, double goal_y);
  //  std::vector<double> sample();
    int nearest(std::vector<Node> &tree, std::vector<double> &sampled_point);
    Node steer(Node &nearest_node, std::vector<double> &sampled_point);
    bool check_collision(Node &nearest_node, Node &new_node,vector<int8_t> &map_update);
    bool is_goal(Node &latest_added_node, double goal_x, double goal_y);
    std::vector<Node> find_path(std::vector<Node> &tree);
    // RRT* methods
    double cost(std::vector<Node> &tree, Node &node);
    double line_cost(Node &n1, Node &n2);
    int xy_to_grid(vector<double> coo);
    std::vector<double> grid_to_xy(int it);
    std::vector<int> near(std::vector<Node> &tree, Node &node,double r);

};
