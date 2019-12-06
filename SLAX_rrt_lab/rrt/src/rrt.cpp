// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <iostream>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <visualization_msgs/Marker.h>
#include <time.h>

using namespace std;
// Destructor of the RRT class

RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    ROS_INFO("RRT shutting down");
}


// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen((std::random_device())()) {

    // TODO: Load parameters from yaml file, you could add your own parameters to the rrt_params.yaml fill
    std::string pose_topic, scan_topic;
    srand(time(NULL));
    nh_.getParam("pose_topic", pose_topic);
    nh_.getParam("scan_topic", scan_topic);
    nh_.getParam("L_waypoints", L_waypoints);
    nh_.getParam("rrt_radius", rrt_radius);
    nh_.getParam("rrt_iteration", rrt_iteration);
    nh_.getParam("prop_gain", prop_gain);
    nh_.getParam("global_L_follow", global_L_follow);
    nh_.getParam("waypoint_path", waypoint_path);

    nh_.getParam("sample_lx", sample_lx);
    nh_.getParam("sample_ly", sample_ly);

    nh_.getParam("rrt_goal_dist", rrt_goal_dist);
    nh_.getParam("inflation", inflation);
    nh_.getParam("steer_param", steer_param);

    nh_.getParam("RRT_star", RRT_star);

    nh_.getParam("steer1", steer1);
    nh_.getParam("steer2", steer2);
    nh_.getParam("vel_1", vel_1);
    nh_.getParam("vel_2", vel_2);
    nh_.getParam("vel_3", vel_3);

    nh_.getParam("show_obstacles", show_obstacles);
    nh_.getParam("show_rrt_path", show_rrt_path);
    nh_.getParam("show_tree", show_tree);
    nh_.getParam("show_ball", show_ball);
    nh_.getParam("steer_param_2", steer_param_2);

    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need

    // ROS subscribers
    // TODO: create subscribers as you need
    pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
    map_sub_ = nh_.subscribe("/map",10, &RRT::map_callback, this);
    marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
    drive_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 10);

    // TODO: create a occupancy grid

    ROS_INFO("Created new RRT Object.");
}


int RRT::xy_to_grid(vector<double> coo) {
    double x =  coo[0];
    double y = coo[1];
    int res;
    res = int((y-origin_y)/resol)*wid + int((x-origin_x)/resol);

    return res;
}

std::vector<double> RRT::grid_to_xy(int it) {
    vector<double> res;
    double x;
    double y;
    x = (it%wid)*resol + origin_x;
    y = (it/wid)*resol + origin_y;
    res.push_back(x);
    res.push_back(y);
    return res;
}

void RRT::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {


  // vector<int8_t> map_data;
  if(load_map_flag){
    map_data = map_msg -> data;
    origin_x = (map_msg -> info).origin.position.x;
    origin_y = (map_msg -> info).origin.position.y;
    wid = (map_msg -> info).width;
    hei = (map_msg -> info).height;
    resol = (map_msg -> info).resolution;
    load_map_flag = false;
  }
   // int total = 0;
   // int positive =0;
   // for(auto it = 0; it != map_data.size(); it++){
   //   if(map_data[it] >0){
   //     int row = it%2048;
   //     int col = it/2048;
   //     geometry_msgs::Point p;
   //     p.x =  (row)*0.05 -51.225;
   //     p.y =  (col)*0.05 -51.225;
   //     p.z = 0;
   //     grid_point.points.push_back(p);
   //
   //   }
   // }
   // marker_pub.publish(grid_point);
   //
   //  cout <<"width" << (map_msg -> info).origin << endl;
   //
   //  cout << total << endl;
   //  cout << positive <<endl;
   //  cout <<"width" << (map_msg -> info).resolution << endl;
   //   cout <<"width" << (map_msg -> info).width << endl;
   //   cout <<"width" << (map_msg -> info).height << endl;
   //   cout <<"width" << (map_msg -> info).origin << endl;
   //   cout << map_data.size() << endl;


    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //

    // TODO: update your occupancy grid
}

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {

    range_data = (scan_msg->ranges);
    angle_min = (scan_msg->angle_min);
    angle_max = (scan_msg->angle_max);
    angle_incre = (scan_msg->angle_increment);

}

void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {

  // Waypoints load only in the beginning
    if(waypoint_load_flag){
      std::ifstream myfile;
      myfile.open(waypoint_path);
      string line;
      string delimeter = ",";

      if (myfile.is_open())
      {
        while ( getline (myfile,line) )
        {
          //  cout << "du le ma" << endl;
          std::vector<std::string> vec;
          boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
          dataList.push_back(vec);
        }
        myfile.close();
      }

      unsigned long size = dataList.size();

      data_int = vector<vector<float>> (size, vector<float> (4, 0));

      for(unsigned int i = 0; i < size; i ++){
        for (unsigned int r = 0; r<4; r++){
          // cout<<(stof(dataList[i][r]));
          data_int[i][r] = stof(dataList[i][r]);
        }
      }
      waypoint_load_flag=false;
   }

// check if obstacle is present
  car_pos_x = (pose_msg->pose).position.x;
  car_pos_y = (pose_msg->pose).position.y;
  tf::Quaternion quat((pose_msg->pose).orientation.x,
  (pose_msg->pose).orientation.y,
  (pose_msg->pose).orientation.z,
  (pose_msg->pose).orientation.w);
  tf::Matrix3x3 m(quat);
  double roll,pitch,yaw;
  m.getRPY(roll,pitch,yaw);


  vector<int8_t>  map_update =  map_data;
  int obstacle_count=0;
  vector<double> point_pos_xw;
  vector<double> point_pos_yw;
  for(int i =0; i<range_data.size(); i++){
  //  cout << i << "of" << range_data.size()<<endl;
    double point_len = range_data[i];
    double point_ang = angle_min + i*angle_incre;
    double point_x_car = point_len*cos(point_ang);
    double point_y_car = point_len*sin(point_ang);
    vector<double> point_pos_w;
    double point_x_w = cos(yaw)*point_x_car - sin(yaw)*point_y_car + car_pos_x;
    double point_y_w = sin(yaw)*point_x_car + cos(yaw)*point_y_car + car_pos_y;
    bool is_obstacle = true;
    for(double m=-inflation; m<=inflation; m=m+resol){
      for(double n=-inflation; n<=inflation ;n=n+resol){
        point_pos_w.clear();
        point_pos_w.push_back(point_x_w+m);
        point_pos_w.push_back(point_y_w+n);
        point_pos_xw.push_back(point_x_w+m);  //0.8
        point_pos_yw.push_back(point_y_w+n);
        int idx = xy_to_grid(point_pos_w);
        if(map_update[idx]>0){
          is_obstacle = false;
        }
      }
    }
    if(is_obstacle){
      obstacle_count=obstacle_count+1;
    }
  }
  cout<<"                obstacle count  : "<<obstacle_count <<"\n";

      vector<double> point_pos_w;
    for(int i=0; i<point_pos_xw.size();i++){
      point_pos_w.clear();
      point_pos_w.push_back(point_pos_xw[i]);
      point_pos_w.push_back(point_pos_yw[i]);
      int idx = xy_to_grid(point_pos_w);
      map_update[idx] = 1;
    }

    visualization_msgs::Marker grid_point;
    grid_point.header.frame_id = "/map";
    grid_point.header.stamp = ros::Time();
    grid_point.ns = "grids";
    grid_point.id = 0;
    grid_point.type = visualization_msgs::Marker::POINTS;
    grid_point.action = visualization_msgs::Marker::ADD;

    grid_point.pose.orientation.x = 0.0;
    grid_point.pose.orientation.y = 0.0;
    grid_point.pose.orientation.z = 0.0;
    grid_point.pose.orientation.w = 1.0;
    grid_point.scale.x = 0.03;
    grid_point.scale.y = 0.03;
    grid_point.scale.z = 0.03;
    grid_point.color.a = 1.0; // Don't forget to set the alpha!
    grid_point.color.r = 1.0;
    grid_point.color.g = 0.0;
    grid_point.color.b = 0.0;
    geometry_msgs::Point p;
    for(auto it = 0; it != map_update.size(); it++){
      if(int(map_update[it]) >0){
        vector<double> xy = grid_to_xy(it);
    //    cout << xy[0] << "and " << xy[1] << endl;

        p.x =  xy[0];
        p.y =  xy[1];
        p.z = 0;

        grid_point.points.push_back(p);
    //    grid_point.points.push_back(p);
      }
    }

    if(obstacle_count>5){
      RRT_star = true;
      cout<<"RRT star\n";
    }else{
      RRT_star = false;
      cout<<"pure pursuit\n";
    }
  // end of obstacle detection

  if(RRT_star){
    // for(auto it = 0; it != map_update.size(); it++){
    //   map_update[it]=0;
    // }

    // for(int i =0; i<range_data.size(); i++){
    // //  cout << i << "of" << range_data.size()<<endl;
    //    double point_len = range_data[i];
    //    double point_ang = angle_min + i*angle_incre;
    //    double point_x_car = point_len*cos(point_ang);
    //    double point_y_car = point_len*sin(point_ang);
    //    vector<double> point_pos_w;
    //    double point_x_w = cos(yaw)*point_x_car - sin(yaw)*point_y_car + car_pos_x;
    //    double point_y_w = sin(yaw)*point_x_car + cos(yaw)*point_y_car + car_pos_y;
    //    for(double m=-inflation; m<=inflation; m=m+resol){
    //      for(double n=-inflation; n<=inflation ;n=n+resol){
    //        point_pos_w.clear();
    //        point_pos_w.push_back(point_x_w+m+0.8);
    //        point_pos_w.push_back(point_y_w+n);
    //        int idx = xy_to_grid(point_pos_w);
    //        map_update[idx] = 1;
    //      }
    //    }
    // }
    // vector<double> point_pos_w;
    // for(int i=0; i<point_pos_xw.size();i++){
    //   point_pos_w.clear();
    //   point_pos_w.push_back(point_pos_xw[i]);
    //   point_pos_w.push_back(point_pos_yw[i]);
    //   int idx = xy_to_grid(point_pos_w);
    //   map_update[idx] = 1;
    // }

    // visualization_msgs::Marker grid_point;
    // grid_point.header.frame_id = "/map";
    // grid_point.header.stamp = ros::Time();
    // grid_point.ns = "grids";
    // grid_point.id = 0;
    // grid_point.type = visualization_msgs::Marker::POINTS;
    // grid_point.action = visualization_msgs::Marker::ADD;

    // grid_point.pose.orientation.x = 0.0;
    // grid_point.pose.orientation.y = 0.0;
    // grid_point.pose.orientation.z = 0.0;
    // grid_point.pose.orientation.w = 1.0;
    // grid_point.scale.x = 0.03;
    // grid_point.scale.y = 0.03;
    // grid_point.scale.z = 0.03;
    // grid_point.color.a = 1.0; // Don't forget to set the alpha!
    // grid_point.color.r = 1.0;
    // grid_point.color.g = 0.0;
    // grid_point.color.b = 0.0;
    // geometry_msgs::Point p;

    visualization_msgs::Marker best_point;
    best_point.header.frame_id = "/map";
    best_point.header.stamp = ros::Time();
    best_point.ns = "best points";
    best_point.id = 1;
    best_point.type = visualization_msgs::Marker::POINTS;
    best_point.action = visualization_msgs::Marker::ADD;

    best_point.pose.orientation.x = 0.0;
    best_point.pose.orientation.y = 0.0;
    best_point.pose.orientation.z = 0.0;
    best_point.pose.orientation.w = 0.0;
    best_point.scale.x = 0.4;
    best_point.scale.y = 0.4;
    best_point.scale.z = 0.4;
    best_point.color.a = 1.0; // Don't forget to set the alpha!
    best_point.color.r = 1.0;
    best_point.color.g = 1.0;
    best_point.color.b = 0.0;
    geometry_msgs::Point best_p;

    visualization_msgs::Marker path_point;
    path_point.header.frame_id = "/map";
    path_point.header.stamp = ros::Time();
    path_point.ns = "waypoints";
    path_point.id = 2;
    path_point.type = visualization_msgs::Marker::LINE_STRIP;
    path_point.action = visualization_msgs::Marker::ADD;

    path_point.pose.orientation.x = 0.0;
    path_point.pose.orientation.y = 0.0;
    path_point.pose.orientation.z = 0.0;
    path_point.pose.orientation.w = 1.0;
    path_point.scale.x = 0.08;
    path_point.scale.y = 0.08;
    path_point.scale.z = 0.08;
    path_point.color.a = 1.0; // Don't forget to set the alpha!
    path_point.color.r = 0.0;
    path_point.color.g = 1.0;
    path_point.color.b = 0.0;
    geometry_msgs::Point path_p;



    // for(auto it = 0; it != map_update.size(); it++){
    //   if(int(map_update[it]) >0){
    //     vector<double> xy = grid_to_xy(it);
    // //    cout << xy[0] << "and " << xy[1] << endl;

    //     p.x =  xy[0];
    //     p.y =  xy[1];
    //     p.z = 0;

    //     grid_point.points.push_back(p);
    // //    grid_point.points.push_back(p);
    //   }
    // }
  //  marker_pub.publish(grid_point);
    //waypoint_path="/home/samarth/rcws/logs/test.csv";
    double shortest = DBL_MAX;
    double best_x;
    double best_y;
    double dis;
    double x_car_frame;
    double mark_x;
    double mark_y;
    double flo_x;
    double flo_y;
    string waypoint_x;
    string waypoint_y;
    string rot;
    string emp;
    double y_car_frame;
    double L = L_waypoints;
    int count = 0;


    for(int i=0; i<data_int.size(); i++){

      flo_x = data_int[i][0];
      flo_y = data_int[i][1];

      p.x =  flo_x;
      p.y =  flo_y;
      p.z = 0;

      grid_point.points.push_back(p);

      x_car_frame = (flo_x-car_pos_x)*cos(yaw) + (flo_y-car_pos_y)*sin(yaw);
      y_car_frame = -(flo_x-car_pos_x)*sin(yaw) + (flo_y-car_pos_y)*cos(yaw);
      dis = abs(sqrt(x_car_frame*x_car_frame + y_car_frame*y_car_frame)-L);

      if (x_car_frame>L/1000 && dis < shortest){
        shortest = dis;
        mark_x = flo_x;
        mark_y = flo_y;
        best_x = x_car_frame;
        best_y = y_car_frame;
      }
    }

    double goal_point_x = mark_x;
    double goal_point_y = mark_y;

    cout << "goal_point_x" << goal_point_x << endl;
    cout << "goal_point_y" << goal_point_y << endl;
    best_p.x =  goal_point_x;
    best_p.y =  goal_point_y;
    best_p.z = 0;

    best_point.points.push_back(best_p);

  //  cout << mark_x << "and " << mark_y << endl;
    // tree as std::vector
    std::vector<Node> tree;
    Node start_point;
    start_point.x = car_pos_x;
    start_point.y = car_pos_y;
    start_point.cost = 0;
    start_point.id = 0;
    start_point.is_root = true;
    tree.push_back(start_point);
    cout << "car_x" << car_pos_x << endl;
    cout << "car_y" << car_pos_y << endl;


    double radius = rrt_radius;
    int iteration = rrt_iteration;
    for(int m =0; m<iteration ;m++){
      vector<double> sample_point = sample(car_pos_x,car_pos_y, goal_point_x, goal_point_y);
      int it = xy_to_grid(sample_point);
      if(map_update[it] >0 ){
        continue;
      }
      int nearest_count = nearest(tree,sample_point);
      Node nearest_node = tree[nearest_count];
      Node new_node = steer(nearest_node,sample_point);
      //Node new_node = sample_point;

      if(check_collision(nearest_node,new_node,map_update) == false){
          new_node.id = int(tree.size());
          tree.push_back(new_node);
        //  cout << "omg" << (tree[int(tree.size()-1)].parent) << endl;
          vector<int> neig = near(tree, new_node,radius);
      //    cout << "howmanyfk" << neig.size() << endl;
          if(neig.size() > 1){
             for(int g =0; g<neig.size();g++){
              Node node_i = tree[neig[g]];
              double new_cost = node_i.cost + sqrt((node_i.x-new_node.x)*(node_i.x-new_node.x) + (node_i.y-new_node.y)*(node_i.y-new_node.y));
              if(new_cost < new_node.cost && check_collision(node_i,new_node,map_update) == false ){
                new_node.parent = node_i.id;
                new_node.cost = new_cost;
            //    cout << "gg" << new_node.parent << endl;
              }

             }
             tree[int(tree.size()-1)] = new_node;
          //   cout << "wtf" << (tree[int(tree.size()-1)].parent) << endl;
             for(int g =0; g<neig.size();g++){
              Node node_i = tree[neig[g]];
              double cost_i = new_node.cost + sqrt((node_i.x-new_node.x)*(node_i.x-new_node.x) + (node_i.y-new_node.y)*(node_i.y-new_node.y));
              if(cost_i < node_i.cost && check_collision(node_i,new_node,map_update) == false ){
                node_i.parent = new_node.id;
                node_i.cost = cost_i;
                tree[neig[g]] = node_i;
              }
             }

          }

      if( is_goal(new_node, goal_point_x, goal_point_y) == true){
        continue;
      }
    }
    }

   // vector<visualization_msgs::Marker> tree_vec(int(tree.size())-1);
    geometry_msgs::Point tree_p;
    visualization_msgs::Marker tree_line;
  //   for(int i =1 ; i<iteration;i++){

  //   //visualization_msgs::Marker tree_line;

  //   tree_line.points.clear();

  //   tree_line.header.frame_id = "/map";
  //   tree_line.header.stamp = ros::Time();
  //   tree_line.ns = "trees";
  //   tree_line.type = visualization_msgs::Marker::LINE_STRIP;
  //   tree_line.action = visualization_msgs::Marker::ADD;
  //   tree_line.id = i+2;
  //   tree_line.pose.orientation.x = 0.0;
  //   tree_line.pose.orientation.y = 0.0;
  //   tree_line.pose.orientation.z = 0.0;
  //   tree_line.pose.orientation.w = 1.0;
  //   tree_line.scale.x = 0.01;
  //   tree_line.scale.y = 0.01;
  //   tree_line.scale.z = 0.01;
  //   tree_line.color.a = 1.0; // Don't forget to set the alpha!
  //   tree_line.color.r = 0.0;
  //   tree_line.color.g = 0.0;
  //   tree_line.color.b = 1.0;
  //   //tree_line.lifetime = ros::Duration(0.4);
  //  if (i < tree.size()){
  //   tree_p.x =  tree[i].x;
  //   tree_p.y =  tree[i].y;
  //   tree_p.z = 0;
  //   tree_line.points.push_back(tree_p);
  //   int par1 = tree[i].parent;
  //   tree_p.x =  tree[par1].x;
  //   tree_p.y =  tree[par1].y;
  //   tree_p.z = 0;
  //   tree_line.points.push_back(tree_p);
  // }
  //   else{
  //   tree_p.x =  tree[1].x;
  //   tree_p.y =  tree[1].y;
  //   tree_p.z = 0;
  //   tree_line.points.push_back(tree_p);
  //   int par1 = tree[1].parent;
  //   tree_p.x =  tree[par1].x;
  //   tree_p.y =  tree[par1].y;
  //   tree_p.z = 0;
  //   tree_line.points.push_back(tree_p);

  //   }
  //   if(show_tree){
  //       marker_pub.publish(tree_line);
  //      }
  //   }

    for(int i =1 ; i<tree.size();i++){

    //visualization_msgs::Marker tree_line;

    tree_line.points.clear();

    tree_line.header.frame_id = "/map";
    tree_line.header.stamp = ros::Time();
    tree_line.ns = "trees";
    tree_line.type = visualization_msgs::Marker::POINTS;
    tree_line.action = visualization_msgs::Marker::ADD;
    tree_line.id = i+2;
    tree_line.pose.orientation.x = 0.0;
    tree_line.pose.orientation.y = 0.0;
    tree_line.pose.orientation.z = 0.0;
    tree_line.pose.orientation.w = 1.0;
    tree_line.scale.x = 0.05;
    tree_line.scale.y = 0.05;
    tree_line.scale.z = 0.05;
    tree_line.color.a = 1.0; // Don't forget to set the alpha!
    tree_line.color.r = 0.0;
    tree_line.color.g = 0.0;
    tree_line.color.b = 1.0;
    tree_line.lifetime = ros::Duration(0.02);

    tree_p.x =  tree[i].x;
    tree_p.y =  tree[i].y;
    tree_p.z = 0;
    tree_line.points.push_back(tree_p);
    int par1 = tree[i].parent;
    tree_p.x =  tree[par1].x;
    tree_p.y =  tree[par1].y;
    tree_p.z = 0;
    tree_line.points.push_back(tree_p);
    if(show_tree){
        marker_pub.publish(tree_line);
       }
    }



    double L_follow = global_L_follow;
    shortest = DBL_MAX;

    vector<Node> path = find_path(tree,goal_point_x,goal_point_y);
    for(int i=0; i < path.size(); i++){
     flo_x = path[i].x;
     flo_y = path[i].y;
     x_car_frame = (flo_x-car_pos_x)*cos(yaw) + (flo_y-car_pos_y)*sin(yaw);
     y_car_frame = -(flo_x-car_pos_x)*sin(yaw) + (flo_y-car_pos_y)*cos(yaw);
     dis = abs(sqrt(x_car_frame*x_car_frame + y_car_frame*y_car_frame)-L_follow);

    //if (x_car_frame>L/2 && dis < shortest){
    if (dis < shortest && check_collision(path[0],path[i],map_update)==false){
      shortest = dis;
      mark_x = flo_x;
      mark_y = flo_y;
      best_x = x_car_frame;
      best_y = y_car_frame;
    }

    }
    goal_point_x = mark_x;
    goal_point_y = mark_y;
    best_p.x =  goal_point_x;
    best_p.y =  goal_point_y;
    best_p.z = 0;

    best_point.points.push_back(best_p);

    //cout<< "path:" <<waypoint_path<<endl;

    double P = prop_gain;
    double L_square = best_x*best_x+best_y*best_y;
    double arc =  2*best_y/L_square;
    double angle = P*arc;

    //cout << "x" << x_car_frame << endl;
    //cout << "y" << y_car_frame << endl;
    // cout << "angle" << angle << endl;
   // cout << angle <<endl;

    double velocity = 5;
    if(abs(angle*180/M_PI) <steer1){
      velocity = vel_1;
    }else if(abs(angle*180/M_PI) <steer2){
     velocity = vel_2;
   }else{
     velocity = vel_3;
   }

   if(angle >0.42)
   angle = 0.42;
   if(angle <-0.42)
   angle = -0.42;
   drive_msg.header.stamp = ros::Time::now();
   drive_msg.header.frame_id = "laser";
   drive_msg.drive.steering_angle = angle;
   drive_msg.drive.speed = velocity;

   drive_pub.publish(drive_msg);



    for(int q=0; q<path.size(); q++){
     Node node_i = path[q];
     path_p.x =  node_i.x;
     path_p.y =  node_i.y;
     path_p.z = 0;

     path_point.points.push_back(path_p);

    }

    if(show_ball){
      marker_pub.publish(best_point);
     }
     // marker_pub.publish(tree_line);

    if(show_obstacles){
      marker_pub.publish(grid_point);
    }
    if(show_rrt_path){
     marker_pub.publish(path_point);
    }
  }


  //bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb
  //bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb
  //bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb
  //bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb

  if(!RRT_star){


  double closest_x;
  double closest_y;

  double shortest = DBL_MAX;
  double closest_wp_dis = DBL_MAX;
  double best_x;
  double best_y;
  double dis;
  double x_car_frame;
  double mark_x;
  double mark_y;
  double mark_x1;
  double mark_y1;
  double y_car_frame;
  double temp_i;

  vector<double> mark_xx(L_velocity.size());
  vector<double> mark_yy(L_velocity.size());

  vector<double> shortest_vel;
  for(int i=0; i<L_velocity.size(); i++){
    shortest_vel.push_back(DBL_MAX);
  }
  vector<double> future_velocities(L_velocity.size());


  int count = 0;
  if(flag == 0){
    path_line.header.frame_id = "/map";
    path_line.header.stamp = ros::Time();
    path_line.ns = "waypoints";
    path_line.id = 1;
    path_line.type = visualization_msgs::Marker::LINE_STRIP;
    path_line.action = visualization_msgs::Marker::ADD;

    path_line.pose.orientation.x = 0.0;
    path_line.pose.orientation.y = 0.0;
    path_line.pose.orientation.z = 0.0;
    path_line.pose.orientation.w = 1.0;
    path_line.scale.x = 0.05;
    path_line.scale.y = 0.05;
    path_line.scale.z = 0.05;
    path_line.color.a = 1.0; // Don't forget to set the alpha!
    path_line.color.r = 0.0;
    path_line.color.g = 0.0;
    path_line.color.b = 1.0;
  }



  for(int i=0; i<data_int.size(); i++){
    x_car_frame = (data_int[i][0]-car_pos_x)*cos(yaw) + (data_int[i][1]-car_pos_y)*sin(yaw);
    y_car_frame = -(data_int[i][0]-car_pos_x)*sin(yaw) + (data_int[i][1]-car_pos_y)*cos(yaw);

    dis = sqrt(x_car_frame*x_car_frame + y_car_frame*y_car_frame);
    if (dis < closest_wp_dis){
      closest_wp_dis = abs(dis);
      closest_x= data_int[i][0];
      closest_y= data_int[i][1];
    }

    for(int m=0; m<L_velocity.size(); m++){
      dis = abs(sqrt(x_car_frame*x_car_frame + y_car_frame*y_car_frame)-L_velocity[m]);
      if (x_car_frame>L_velocity[m]/2 && dis < shortest_vel[m]){
        shortest_vel[m] = dis;
        future_velocities[m] = data_int[i][3];
        mark_xx[m] = data_int[i][0];
        mark_yy[m] = data_int[i][1];
      }
    }

    dis = abs(sqrt(x_car_frame*x_car_frame + y_car_frame*y_car_frame)-L);
    if (x_car_frame>L/2 && dis < shortest){
      shortest = dis;
      mark_x = data_int[i][0];
      mark_y = data_int[i][1];
      best_x = x_car_frame;
      best_y = y_car_frame;
    }
    if ( flag==0){
      geometry_msgs::Point p;
      p.x = data_int[i][0];
      p.y = data_int[i][1];
      p.z = 0;
      path_line.points.push_back(p);
      //  marker.lifetime = ros::Duration(0.01);
    }
  }

  if (flag ==0)
  marker_pub.publish(path_line);
  flag = 1;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "chase_ball";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = closest_x;  marker.pose.position.y = closest_y;  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;  marker.pose.orientation.y = 0.0;  marker.pose.orientation.z = 0.0;  marker.pose.orientation.w = 0.0;
  marker.scale.x = 0.2;  marker.scale.y = 0.2;  marker.scale.z = 0.2;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;  marker.color.g = 0.0;  marker.color.b = 0.0;
  marker_pub.publish( marker );
  //  marker.lifetime = ros::Duration(0.01);




  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "velo_ball";
  marker.id = 3;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = mark_xx[0];  marker.pose.position.y = mark_yy[0];  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;  marker.pose.orientation.y = 0.0;  marker.pose.orientation.z = 0.0;  marker.pose.orientation.w = 0.0;
  marker.scale.x = 0.4;  marker.scale.y = 0.4;  marker.scale.z = 0.4;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;  marker.color.g = 1.0;  marker.color.b = 0.0;
  marker_pub.publish( marker );

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "velo_ball";
  marker.id = 4;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = mark_xx[1];  marker.pose.position.y = mark_yy[1];  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;  marker.pose.orientation.y = 0.0;  marker.pose.orientation.z = 0.0;  marker.pose.orientation.w = 0.0;
  marker.scale.x = 0.4;  marker.scale.y = 0.4;  marker.scale.z = 0.4;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;  marker.color.g = 1.0;  marker.color.b = 0.0;
  marker_pub.publish( marker );

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "velo_ball";
  marker.id = 5;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = mark_xx[2];  marker.pose.position.y = mark_yy[2];  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;  marker.pose.orientation.y = 0.0;  marker.pose.orientation.z = 0.0;  marker.pose.orientation.w = 0.0;
  marker.scale.x = 0.4;  marker.scale.y = 0.4;  marker.scale.z = 0.4;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;  marker.color.g = 1.0;  marker.color.b = 0.0;
  marker_pub.publish( marker );

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "velo_ball";
  marker.id = 6;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = mark_xx[3];  marker.pose.position.y = mark_yy[3];  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;  marker.pose.orientation.y = 0.0;  marker.pose.orientation.z = 0.0;  marker.pose.orientation.w = 0.0;
  marker.scale.x = 0.4;  marker.scale.y = 0.4;  marker.scale.z = 0.4;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;  marker.color.g = 1.0;  marker.color.b = 0.0;
  marker_pub.publish( marker );

  double L_square = best_x*best_x+best_y*best_y;
  double arc =  2*best_y/L_square;
  double angle = P*arc;
  // cout << "x" << x_car_frame << endl;
  // for(int m=0; m<L_velocity.size(); m++){
  //   cout<<future_velocities[m]<<" ,";

  // }
  //cout<<"\n";
  // cout << "y" << y_car_frame << endl;
  // cout << "angle" << angle << endl;
  // cout << angle <<endl;
  double velocity = 0;
  // if(abs(angle*180/M_PI) <5){
  //   velocity = 4.5;
  // }else if(abs(angle*180/M_PI) <8){
  //   velocity = 1.3;
  // }else{
  //   velocity = 0.5; //1
  // }
  for(int m=0; m<future_velocities.size(); m++){
    velocity=velocity + pow(velocity_gamma,m)*future_velocities[m];
  }
  velocity=velocity/future_velocities.size();
  // cout<<velocity<<" .....    ";

  // understeer aware

  velocity = (velocity - understeer_gain*closest_wp_dis )*0.5;
  if(velocity<0.5){
    velocity = 0.5;
  }

  // cout<<velocity<<"\n";
  // cout<<"                              "<<closest_wp_dis<<"\n";


  if(angle >0.42)
    angle = 0.42;
  if(angle <-0.42)
    angle = -0.42;
  drive_msg.header.stamp = ros::Time::now();
  drive_msg.header.frame_id = "laser";
  drive_msg.drive.steering_angle = angle;
  drive_msg.drive.speed = velocity;

  drive_pub.publish(drive_msg);
  if(show_obstacles){
      marker_pub.publish(grid_point);
    }
  }




  // path found as Path message

}

std::vector<double> RRT::sample(double start_x,double start_y, double goal_x, double goal_y) {
  //  srand(time(NULL));

    double x_rand = ((double)rand()/(RAND_MAX));
    double y_rand = ((double)rand()/(RAND_MAX));
  //  cout << x_rand << "and" << y_rand << endl;
    vector<double> y_dir;
    vector<double> x_dir;
    double L_x = sample_lx;
    double L_y = sample_ly;
    y_dir.push_back((goal_x - start_x)/sqrt((goal_x - start_x)*(goal_x - start_x)+(goal_y - start_y)*(goal_y - start_y)));
    y_dir.push_back((goal_y - start_y)/sqrt((goal_x - start_x)*(goal_x - start_x)+(goal_y - start_y)*(goal_y - start_y)));
    x_dir.push_back(-y_dir[1]);
    x_dir.push_back(y_dir[0]);
    double bx = start_x - L_x/2*x_dir[0];
    double by = start_y - L_x/2*x_dir[1];

    double res_x =  bx+ x_rand*L_x*x_dir[0] + y_rand*L_y*y_dir[0];
    double res_y =  by+ x_rand*L_x*x_dir[1] + y_rand*L_y*y_dir[1];

    std::vector<double> sampled_point;
    sampled_point.push_back(res_x);
    sampled_point.push_back(res_y);
    // TODO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)

    return sampled_point;
}


int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point) {
     double shorest_dis = DBL_MAX;
     int res;
     for(int i=0 ; i<tree.size(); i++){
       double node_x = tree[i].x;
       double node_y = tree[i].y;
       double dis =sqrt((sampled_point[0] - node_x)* (sampled_point[0] - node_x) + (sampled_point[1] - node_y)* (sampled_point[1] - node_y));
       if(dis < shorest_dis){
          shorest_dis = dis;
          res =i;
       }
     }
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    // TODO: fill in this method

    return res;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point) {
    Node new_node;
    new_node.x = sampled_point[0];
    new_node.y = sampled_point[1];
    new_node.cost =nearest_node.cost + sqrt((nearest_node.x - sampled_point[0])*(nearest_node.x - sampled_point[0]) +(nearest_node.y - sampled_point[1]*(nearest_node.y - sampled_point[1])));
    new_node.parent = nearest_node.id;

    double p = steer_param;
    if(sqrt((nearest_node.x - sampled_point[0])*(nearest_node.x - sampled_point[0]) +(nearest_node.y - sampled_point[1]*(nearest_node.y - sampled_point[1])))>steer_param_2){
      new_node.x =   sampled_point[0] + (nearest_node.x - sampled_point[0])*p;
      new_node.y =   sampled_point[1] + (nearest_node.y - sampled_point[1])*p;
      new_node.cost =nearest_node.cost + p*sqrt((nearest_node.x - sampled_point[0])*(nearest_node.x - sampled_point[0]) +(nearest_node.y - sampled_point[1]*(nearest_node.y - sampled_point[1])));
    }
    // The function steer:(x,y)->z returns a point such that z is “closer”
    // to y than x is. The point z returned by the function steer will be
    // such that z minimizes ||z−y|| while at the same time maintaining
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:&& check_collision(node_i,new_node,map_update) == false
    //    new_node (Node): new node created from steering

    // TODO: fill in this method

    return new_node;
}

bool RRT::check_collision(Node &nearest_node, Node &new_node, vector<int8_t> &map_update) {
     bool collision = false;
     double x1 = nearest_node.x;
     double y1 = nearest_node.y;
     double x2 = new_node.x;
     double y2 = new_node.y;
     double dis = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
     int count = int(dis/resol);
     for(int m =0; m<count-1;m++){
      vector<double> xy;
      xy.push_back(x1+(x2-x1)/count*(m+1));
      xy.push_back(y1+(y2-y1)/count*(m+1));
      int it = xy_to_grid(xy);
      if(map_update[it]>0){
        collision = true;
        break;
      }
     }
    // This method returns a boolean indicating if the path between the
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    new_node (Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise


    // TODO: fill in this method

    return collision;
}

bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y) {

    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
     if(sqrt((latest_added_node.x-goal_x)*(latest_added_node.x-goal_x)+(latest_added_node.y-goal_y)*(latest_added_node.y-goal_y))<rrt_goal_dist){
       close_enough = true;
     }


    // TODO: fill in this method

    return close_enough;
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree,double goal_point_x, double goal_point_y) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    //vector<Node>* itr;
    double dis = DBL_MAX;
    int best;
    for(int lit = 0; lit < tree.size(); lit++){
     double dis_now = sqrt((tree[lit].x - goal_point_x)*(tree[lit].x - goal_point_x) + (tree[lit].y - goal_point_y)*(tree[lit].y - goal_point_y));
     if (dis_now < dis){
       dis = dis_now;
       best = lit;
     }
    }


    Node end_node = tree[best];
    cout << "tree_x" << end_node.x << endl;
    cout << "tree_y" << end_node.y << endl;
    cout << "goal_x" << goal_point_x << endl;
    cout << "goal_y" << goal_point_y << endl;


    std::vector<Node> found_path;
    while(end_node.is_root != true){
        found_path.push_back(end_node);
        end_node = tree[end_node.parent];
    }
    found_path.push_back(tree[0]);
    reverse(found_path.begin(),found_path.end());
    // TODO: fill in this method

    return found_path;
}

// RRT* methods
// double RRT::cost(std::vector<Node> &tree, Node &node) {
//     // This method returns the cost associated with a node
//     // Args:
//     //    tree (std::vector<Node>): the current tree
//     //    node (Node): the node the cost is calculated for
//     // Returns:
//     //    cost (double): the cost value associated with the node
//
//     double cost = 0;
//     // TODO: fill in this method
//
//     return cost;
// }
//
// double RRT::line_cost(Node &n1, Node &n2) {
//     // This method returns the cost of the straight line path between two nodes
//     // Args:
//     //    n1 (Node): the Node at one end of the path
//     //    n2 (Node): the Node at the other end of the path
//     // Returns:
//     //    cost (double): the cost value associated with the path
//
//     double cost = 0;
//     // TODO: fill in this method
//
//     return cost;
// }

std::vector<int> RRT::near(std::vector<Node> &tree, Node &node,double r) {
      std::vector<int> neighborhood;
      for(int i = 0; i<int(tree.size())-1 ; i++){
       Node node_i = tree[i];
       if(sqrt((node_i.x-node.x)*(node_i.x-node.x) + (node_i.y-node.y)*(node_i.y-node.y))<r){
       neighborhood.push_back(i);
     }
      }
    // This method returns the set of Nodes in the neighborhood of a
    // node.
    // Args:
    //   tree (std::vector<Node>): the current tree
    //   node (Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    // TODO:: fill in this method

    return neighborhood;
}
