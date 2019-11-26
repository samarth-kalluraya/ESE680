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
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <visualization_msgs/Marker.h>


using namespace std;
// TODO: include ROS msg type headers and libraries you need

class PurePursuit {
private:
    ros::NodeHandle n;
    ros::Subscriber pose_sub;
    ros::Publisher drive_pub;
    ros::Publisher marker_pub;
    ackermann_msgs::AckermannDriveStamped drive_msg;

    visualization_msgs::Marker marker;
    visualization_msgs::Marker path_line;


    double car_pos_x;
    double car_pos_y;
    tf::Quaternion quat;


    string waypoint_x;
    string waypoint_y;
    string rot;
    string emp;
    double flo_x;
    double flo_y;
    double L = 1.6;  //0.8 works 1.4   1.6
    double P = 0.22;  //0.21   0.22
    int k = 0;
    int flag = 0;

    // TODO: create ROS subscribers and publishers

public:
    PurePursuit() {
        n = ros::NodeHandle();
        pose_sub = n.subscribe("/pf/pose/odom", 1000, &PurePursuit::pose_callback,this);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_1", 10);
        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100);
        // TODO: create ROS subscribers and publishers
    }

    void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg) {

       car_pos_x = (pose_msg->pose).pose.position.x;
       car_pos_y = (pose_msg->pose).pose.position.y;
       tf::Quaternion quat((pose_msg->pose).pose.orientation.x,
     (pose_msg->pose).pose.orientation.y,
   (pose_msg->pose).pose.orientation.z,
 (pose_msg->pose).pose.orientation.w);
   tf::Matrix3x3 m(quat);
   double roll,pitch,yaw;
   m.getRPY(roll,pitch,yaw);


   ifstream waypoints("/home/samarth/samarth_ws/src/f110-fall2019-skeletons/SLAX_pure_pursuit/test.csv");

   double shortest = DBL_MAX;
   double best_x;
   double best_y;
   double dis;
   double x_car_frame;
   double mark_x;
   double mark_y;
   double y_car_frame;
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
   path_line.scale.x = 0.2;
   path_line.scale.y = 0.2;
   path_line.scale.z = 0.2;
   path_line.color.a = 1.0; // Don't forget to set the alpha!
   path_line.color.r = 0.0;
   path_line.color.g = 0.0;
   path_line.color.b = 1.0;
 }
   while(waypoints.good()){
     count++;
     getline(waypoints,waypoint_x,',');
     getline(waypoints,waypoint_y,',');
     getline(waypoints,rot,',');
     getline(waypoints,emp,'\n');
     stringstream xx(waypoint_x);
     stringstream yy(waypoint_y);
     xx >> flo_x;
     yy >> flo_y;
     x_car_frame = (flo_x-car_pos_x)*cos(yaw) + (flo_y-car_pos_y)*sin(yaw);
     y_car_frame = -(flo_x-car_pos_x)*sin(yaw) + (flo_y-car_pos_y)*cos(yaw);
     dis = abs(sqrt(x_car_frame*x_car_frame + y_car_frame*y_car_frame)-L);

     if (x_car_frame>L/2 && dis < shortest){
       shortest = dis;
       mark_x = flo_x;
       mark_y = flo_y;
       best_x = x_car_frame;
       best_y = y_car_frame;
     }
    if ( flag==0){
       geometry_msgs::Point p;
       p.x = flo_x;
       p.y = flo_y;
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
    marker.pose.position.x = mark_x;
    marker.pose.position.y = mark_y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    marker.scale.x = 0.6;
    marker.scale.y = 0.6;
    marker.scale.z = 0.6;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  //  marker.lifetime = ros::Duration(0.01);
    marker_pub.publish( marker );

   double L_square = best_x*best_x+best_y*best_y;
   double arc =  2*best_y/L_square;
   double angle = P*arc;
   cout << "x" << x_car_frame << endl;
   cout << "y" << y_car_frame << endl;
   cout << "angle" << angle << endl;
  // cout << angle <<endl;
   double velocity = 5;
   if(abs(angle*180/M_PI) <5){
     velocity = 4.5;
   }else if(abs(angle*180/M_PI) <8){
    velocity = 1.3;
  }else{
    velocity = 0.5; //1
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

   waypoints.close();




        // TODO: find the current waypoint to track using methods mentioned in lecture

        // TODO: transform goal point to vehicle frame of reference

        // TODO: calculate curvature/steering angle

        // TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit pp;
    ros::spin();
    return 0;
}
