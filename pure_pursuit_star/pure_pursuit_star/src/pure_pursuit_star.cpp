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
#include <boost/algorithm/string.hpp>


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
  visualization_msgs::Marker velo_points;
  
  std::vector<std::vector<std::string> > dataList;
  vector<vector<float>> data_int;

  double car_pos_x;
  double car_pos_y;
  tf::Quaternion quat;

  // int k = 0;

  string waypoint_x;
  string waypoint_y;
  string rot;
  string emp;
  double flo_x;
  double flo_y;
  int flag = 0;

  double L = 0.6 ;  //0.8 works 1.4   1.6
  double P = 0.22;  //0.21   0.22
  double understeer_gain = 10;  //0.21   0.22
  double velocity_gamma = 1;
  vector<float> L_velocity = {0.5,1,1.5,2}; //velocity lookahead distances
  
  // double L;
  // double P;  
  // double understeer_gain;  
  // double velocity_gamma;
  // vector<float> L_velocity; //velocity lookahead distances


  // TODO: create ROS subscribers and publishers

public:
PurePursuit() {
  n = ros::NodeHandle();
  srand(time(NULL));
  n.getParam("L", L);
  n.getParam("P", P);
  n.getParam("understeer_gain", understeer_gain);
  n.getParam("velocity_gamma", velocity_gamma);
  cout<<L<<"\n";
  pose_sub = n.subscribe("/pf/pose/odom", 1000, &PurePursuit::pose_callback,this);
  drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 10);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100);
  // TODO: create ROS subscribers and publishers

}

void GetWaypoints(){
  // File pointer
  std::ifstream myfile;
  myfile.open("/home/samarth/samarth_ws/src/f110-fall2019-skeletons/final_project/ESE680/pure_pursuit_star/final_processed.csv");

  ///home/xinlong/XinlongZheng_ws/src/vision/src/vehicle_tracker_prediction_skeleton/waypoints
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
  cout<<velocity<<" .....    ";

  // understeer aware
  
  velocity = (velocity - understeer_gain*closest_wp_dis )*0.7;
  if(velocity<0.5){
    velocity = 0.5;
  }
  
  cout<<velocity<<"\n";
  cout<<"                              "<<closest_wp_dis<<"\n";


  if(angle >0.42)
    angle = 0.42;
  if(angle <-0.42)
    angle = -0.42;
  drive_msg.header.stamp = ros::Time::now();
  drive_msg.header.frame_id = "laser";
  drive_msg.drive.steering_angle = angle;
  drive_msg.drive.speed = velocity;

  drive_pub.publish(drive_msg);





  // TODO: find the current waypoint to track using methods mentioned in lecture

  // TODO: transform goal point to vehicle frame of reference

  // TODO: calculate curvature/steering angle

  // TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
}

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit pp;
    pp.GetWaypoints();
    ros::spin();
    return 0;
}
