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


PurePursuitStar::PurePursuitStar(ros::NodeHandle& nh){
	pose_sub = n.subscribe("/pf/pose/odom", 1000, &PurePursuit::pose_callback,this);
	drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_1", 10);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100);
}

void Pose_Estimator::GetWaypoints(){
	// File pointer
	std::ifstream myfile;
	myfile.open("/home/samarth/samarth_ws/src/final_proje/ESE680/pure_pursuit_star/test.csv");

	///home/xinlong/XinlongZheng_ws/src/vision/src/vehicle_tracker_prediction_skeleton/waypoints
	string line;
	string delimeter = ",";

	if (myfile.is_open())
	{
		while ( getline (myfile,line) )
		{
			//	cout << "du le ma" << endl;
		  	std::vector<std::string> vec;
			boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
			dataList.push_back(vec);
		}
		myfile.close();
	}

	unsigned long size = dataList.size();

	data_int = vector<vector<float>> (size, vector<float> (3, 0));

	for(unsigned int i = 0; i < size; i ++){
		for (unsigned int r = 0; r<3; r++){
			// cout<<(stof(dataList[i][r]));
			data_int[i][r] = stof(dataList[i][r]);
		}
 }

 	// ROS_INFO("reached here with size : %lu", data_int.size());
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
	for(int i=0; i<data_int.size(); i++){
		x_car_frame = (data_int[i][0]-x)*cos(theta) + (data_int[i][1]-y)*sin(theta);
		y_car_frame = -(data_int[i][0]-x)*sin(theta) + (data_int[i][1]-y)*cos(theta);
		dis = abs(sqrt(x_car_frame*x_car_frame + y_car_frame*y_car_frame)-L);
		//cout << "bestx" << x_car_frame<< endl;
		//cout << "besty" << y_car_frame<< endl;
		if (x_car_frame>L/2 && dis < shortest){
			shortest = dis;
			best_x = x_car_frame;
			best_y = y_car_frame;
			// cout << "bestx" << best_x<< endl;
			// cout << "besty" << best_y<< endl;
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
    ros::init(argc, argv, "pure_pursuit_star_node");
    ros::NodeHandle nh;
    PurePursuit pp(nh);
    pp.GetWaypoints();
    ros::spin();
    return 0;
}