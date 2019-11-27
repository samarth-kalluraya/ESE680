#include <visualization_msgs/MarkerArray.h>
#include "ros/ros.h"
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



class PurePursuitStar{

private:

  std::vector<std::vector<std::string> > dataList;
  vector<vector<float>> data_int;
	void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg);
	void PublishMarkers();
  ros::Subscriber pose_sub;
  ros::Publisher drive_pub;
  ros::Publisher marker_pub;

	vector<vector<float>> waypoints;
	//float theta;
	//float pos_x;
	//float pos_y;

  ackermann_msgs::AckermannDriveStamped drive_msg;


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

  visualization_msgs::Marker marker;
  visualization_msgs::Marker path_line;



public:
		PurePursuitStar(ros::NodeHandle& nh);
		void GetWaypoints();
		


};

#endif