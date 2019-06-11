// Include C++ Library
#include <iostream>
#include <fstream>
#include <cmath>
#include <math.h>
#include <boost/regex.hpp>
// Include ROS Library
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
//#include <visualization_msgs>
//#include <std_msgs/String.h>
// ROS-OpenCV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
// Node's Header File
#include "apriltag_ekf_localization/EKFLocalizationWrapper.h"


#define DEG2RAD M_PI/180
#define RAD2DEG 180/M_PI

namespace ekf_localization_node_ns{
EKFLocalizationNode::EKFLocalizationNode(ros::NodeHandle node, ros::NodeHandle private_nh)
: node_(node), priv_node_(private_nh){
	// Reading the Parameters and Config File From the Launch File
	get_all_params_ = true;
	get_all_params_ = get_all_params_ && priv_node_.getParam("duration_time", duration_time_);
	get_all_params_ = get_all_params_ && priv_node_.getParam("road_sign_config_file", road_sign_config_file_);
}

bool EKFLocalizationNode::Spin()
{
	if(init() == false)
		return false;
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::waitForShutdown();

	return true;
}

bool EKFLocalizationNode::init()
{
	// Initialize the ekf localizer
	if(ekf_localizer_.Init(road_sign_config_file_.c_str()) == false)
		return false;
	//debug
	std::cout << "ekf_localizer_ initialization succeed ~~!!!" << std::endl;

	// Setup the Subscriber
        wheel_speed_sub_ = node_.subscribe("/chassis_data", 1, &EKFLocalizationNode::onWheelSpeed, this);
	tag_pose_sub_ = node_.subscribe("/tag_detections", 1, &EKFLocalizationNode::onTagPoseArray, this);
	imu_sub_ = node_.subscribe("/sensor/imu", 1, &EKFLocalizationNode::onIMU, this);
	steering_angle_sub_ = node_.subscribe("/command/steering_angle", 1, &EKFLocalizationNode::onSteeringAngle, this);

	// Setup the Publisher
	location_pub_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/localization/pose", 1);
	vehicle_visualization_pub_ = node_.advertise<visualization_msgs::Marker>("/localization/vehicle_location_vis", 1);
	tag_locations_visualization_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/localization/tag_locations_vis", 1);

	// Setup the Timer, which Trigger the Locate() process ~~~!!!
	// Localizing Frequency = 20 Hz || Period = 0.05s
	timer_ = node_.createTimer(ros::Duration(duration_time_), &EKFLocalizationNode::onTimer, this);

	return true;
}

// May add Serialization Later ~~~
void EKFLocalizationNode::onWheelSpeed(const apriltag_ekf_localization::Chassis_data::ConstPtr &msg)
{
	std::lock_guard<std::mutex> guard(wheel_speed_mutex_);
        wheel_speed_ = msg->speed;
	// debug
	std::cout << "Wheel Speed Received  ~~~" << std::endl;
}

void EKFLocalizationNode::onIMU(const std_msgs::Float32::ConstPtr &msg)
{
	std::lock_guard<std::mutex> guard(imu_mutex_);
        yaw_rate_ = msg->data; // in deg/s
	// debug
	std::cout << "IMU Yaw Rate Received ~~~" << std::endl;
}

void EKFLocalizationNode::onSteeringAngle(const apriltag_ekf_localization::Chassis_cmd::ConstPtr &msg)
{
	std::lock_guard<std::mutex> guard(steering_angle_mutex_);
        steering_angle_ = msg->servo_turn;  // in degree
        steering_angle_ = steering_angle_ * static_cast<double>(DEG2RAD); // In radian
	// debug
	std::cout << "Steering Angle Command Received ~~~~~" << std::endl;
}

void EKFLocalizationNode::onTagPoseArray(const apriltag_ekf_localization::AprilTagDetectionArray::ConstPtr &msg)
{
	std::lock_guard<std::mutex> guard(tag_detections_mutex_);
	for (int i = 0; i < msg->detections.size(); ++i)
	{
		ekf_localization_node_ns::AprilTagDetection temp;
		temp.id = msg->detections[i].id[0];
		temp.size = msg->detections[i].size[0];
		temp.pose.position.x = msg->detections[i].pose.pose.pose.position.x;
		temp.pose.position.y = msg->detections[i].pose.pose.pose.position.y;
		temp.pose.position.z = msg->detections[i].pose.pose.pose.position.z;
		temp.pose.orientation.x = msg->detections[i].pose.pose.pose.orientation.x;
		temp.pose.orientation.y = msg->detections[i].pose.pose.pose.orientation.y;
		temp.pose.orientation.z = msg->detections[i].pose.pose.pose.orientation.z;
		temp.pose.orientation.w = msg->detections[i].pose.pose.pose.orientation.w;
		tag_detections_.push_back(temp);
	}
	// debug
	std::cout << "Tag Detections Received ~~~" << std::endl;
}

void EKFLocalizationNode::onTimer(const ros::TimerEvent& event)
{
	// create the local copies for holding the latest global copy of the latest subscription
	std::vector<ekf_localization_node_ns::AprilTagDetection> tag_detections;
	double wheel_speed;
	double yaw_rate;
	double steering_angle;
	// Multi-threading Reading :: Avoid conflicts between local copies and latest global copies
	{
		std::lock_guard<std::mutex> guard(wheel_speed_mutex_);
		wheel_speed = wheel_speed_;
	}
	{
		std::lock_guard<std::mutex> guard(imu_mutex_);
		yaw_rate = yaw_rate_;
	}
	{
		std::lock_guard<std::mutex> guard(steering_angle_mutex_);
		steering_angle = steering_angle_;
	}
	{
		std::lock_guard<std::mutex> guard(tag_detections_mutex_);
		tag_detections = tag_detections_;
	}

	// Update the ekf_localizer with the latest copy of input data
	if(ekf_localizer_.UpdateWheelSpeed(wheel_speed) == false)
	{
		std::cout << "Not receiving Wheel Speed, Cannot Locate the vehicle" << std::endl;
		return;
	}
	if(ekf_localizer_.UpdateYawRate(yaw_rate) == false)
	{
		std::cout << "Not receiving Yaw Rate from IMU, Cannot Locate the vehicle" << std::endl;
		return;
	}
	if(ekf_localizer_.UpdateSteeringAngle(steering_angle) == false)
	{
		std::cout << "Not receiving steer command, Cannot Locate the vehicle" << std::endl;
		return;
	}
	if(ekf_localizer_.UpdateTagDetections(tag_detections) == false)
	{
		std::cout << "Not receiving the tag detections, EKF only locate pose with Wheel Encoder and IMU" << std::endl;
	}

	// Create a variable holding the PoseWithCovariance Data Structures
	ekf_localization_node_ns::PoseWithConvariance vehicle_position;
	// Call the ekf_localizer->Locate(vehicle_position)
	if(ekf_localizer_.Locate(&vehicle_position) == true)
	{
		geometry_msgs::PoseWithCovarianceStamped vehicle_position_msg;
		vehicle_position_msg.header.frame_id = "world";
		vehicle_position_msg.header.stamp = ros::Time::now();
		vehicle_position_msg.pose.pose.position.x = vehicle_position.pose.position.x;
		vehicle_position_msg.pose.pose.position.y = vehicle_position.pose.position.y;
		vehicle_position_msg.pose.pose.position.z = vehicle_position.pose.position.z;
		vehicle_position_msg.pose.pose.orientation.x = vehicle_position.pose.orientation.x;
		vehicle_position_msg.pose.pose.orientation.y = vehicle_position.pose.orientation.y;
		vehicle_position_msg.pose.pose.orientation.z = vehicle_position.pose.orientation.z;
		vehicle_position_msg.pose.pose.orientation.w = vehicle_position.pose.orientation.w;
		vehicle_position_msg.pose.covariance.at(0) = vehicle_position.covariance(0,0);
		vehicle_position_msg.pose.covariance.at(7) = vehicle_position.covariance(1,1);
		vehicle_position_msg.pose.covariance.at(14) = vehicle_position.covariance(2,2);
		vehicle_position_msg.pose.covariance.at(21) = vehicle_position.covariance(3,3);
		vehicle_position_msg.pose.covariance.at(28) = vehicle_position.covariance(4,4);
		vehicle_position_msg.pose.covariance.at(35) = vehicle_position.covariance(5,5);

		// Publish the vehicle_location here ~~~!!!!
		location_pub_.publish(vehicle_position_msg); 
		std::cout << "vehicle_position published ~~" << std::endl;

		// Display the vehicle_positon and all tag locations here tooo ~~~~~
		visualization_msgs::Marker display_vehicle = ekf_localizer_.VisualizeVehicle();
		vehicle_visualization_pub_.publish(display_vehicle);

		visualization_msgs::MarkerArray display_tag_locations = ekf_localizer_.VisualizeTagLocations();
		tag_locations_visualization_pub_.publish(display_tag_locations);

	}
	else
	{
		std::cout << "vehicle localization failed ~~!!!" << std::endl;
	}
}	













} // end of namespace
