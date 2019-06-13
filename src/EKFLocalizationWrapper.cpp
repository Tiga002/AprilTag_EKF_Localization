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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
        steering_angle_sub_ = node_.subscribe("/chassis_cmd", 1, &EKFLocalizationNode::onSteeringAngle, this);

	// Setup the Publisher
	location_pub_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/localization/pose", 1);
	vehicle_visualization_pub_ = node_.advertise<visualization_msgs::Marker>("/localization/vehicle_location_vis", 1);
	tag_locations_visualization_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/localization/tag_locations_vis", 1);
        moved_path_visualization_pub_ = node_.advertise<visualization_msgs::Marker>("/localization/moved_path", 1);
        //detected_tag_locations_visualization_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/localization/detected_tag_locations_vis", 1);

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

}

void EKFLocalizationNode::onIMU(const std_msgs::Float32::ConstPtr &msg)
{
	std::lock_guard<std::mutex> guard(imu_mutex_);
        yaw_rate_ = msg->data; // in deg/s

}

void EKFLocalizationNode::onSteeringAngle(const apriltag_ekf_localization::Chassis_cmd::ConstPtr &msg)
{
	std::lock_guard<std::mutex> guard(steering_angle_mutex_);
        steering_angle_ = msg->servo_turn;  // in degree
        steering_angle_ = steering_angle_ * static_cast<double>(DEG2RAD); // In radian

}

void EKFLocalizationNode::onTagPoseArray(const apriltag_ekf_localization::AprilTagDetectionArray::ConstPtr &msg)
{
	std::lock_guard<std::mutex> guard(tag_detections_mutex_);
	for (int i = 0; i < msg->detections.size(); ++i)
	{
		ekf_localization_node_ns::AprilTagDetection temp;
                temp.id = msg->detections[i].id[0];
                temp.size = msg->detections[i].size[0];
                temp.pose.position.x = msg->detections[i].pose.pose.pose.position.z;
                temp.pose.position.y = msg->detections[i].pose.pose.pose.position.x * -1.0;
                temp.pose.position.z = msg->detections[i].pose.pose.pose.position.y * -1.0;

                tf2::Quaternion q_orig, q_rot, q_new;
                tf2::convert(msg->detections[i].pose.pose.pose.orientation, q_orig);
                double roll = 0.0;
                double pitch = 90 * DEG2RAD;
                double yaw = 90 * DEG2RAD;
                q_rot.setRPY(roll, pitch, yaw);
                q_new = q_rot*q_orig;
                q_new.normalize();
                //tf2::convert(q_new, msg->detections[i].pose.pose.pose.orientation);
//		temp.pose.orientation.x = msg->detections[i].pose.pose.pose.orientation.x;
//		temp.pose.orientation.y = msg->detections[i].pose.pose.pose.orientation.y;
//		temp.pose.orientation.z = msg->detections[i].pose.pose.pose.orientation.z;
//		temp.pose.orientation.w = msg->detections[i].pose.pose.pose.orientation.w;
                temp.pose.orientation.x = q_new[0];
                temp.pose.orientation.y = q_new[1];
                temp.pose.orientation.z = q_new[2];
                temp.pose.orientation.w = q_new[3];
		tag_detections_.push_back(temp);

                // For Debug
                std::cout << "Number of Tag Bundle seen = " << msg->detections.size() << std::endl;
                std::cout << "Detected Master Tag ID: " << temp.id << "Position X = " << temp.pose.position.x << std::endl;
                std::cout << "Detected Master Tag ID: " << temp.id << "Position Y = " << temp.pose.position.y << std::endl;
                std::cout << "Detected Master Tag ID: " << temp.id << "Position Z = " << temp.pose.position.z << std::endl;
                std::cout << "Detected Master Tag ID: " << temp.id << "quad = [" << temp.pose.orientation.x << ","
                          << temp.pose.orientation.y << "," << temp.pose.orientation.z << "," << temp.pose.orientation.w << "]" << std::endl;
                std::cout << "================================================================================" << std::endl;


        }

//        // rviz debug
//        visualization_msgs::MarkerArray detected_tag_pose_msg;
//        // Display all the taf location on the map as reference
//        for (int i = 0; i < msg->detections.size(); ++i)
//        {
//                visualization_msgs::Marker tag_marker;
//                tag_marker.header.frame_id = "world";
//                tag_marker.header.stamp = ros::Time();
//                tag_marker.ns = "apriltag_ekf_localization";
//                tag_marker.id = i;
//                tag_marker.type = visualization_msgs::Marker::CUBE;
//                tag_marker.action = visualization_msgs::Marker::ADD;
//                tag_marker.pose.position.x = msg->detections[i].pose.pose.pose.position.z;
//                tag_marker.pose.position.y = msg->detections[i].pose.pose.pose.position.x * -1.0;
//                tag_marker.pose.position.z = msg->detections[i].pose.pose.pose.position.y * -1.0;
//                tag_marker.pose.orientation = msg->detections[i].pose.pose.pose.orientation;
//                tag_marker.scale.x = 0.25;
//                tag_marker.scale.y = 0.25;
//                tag_marker.scale.z = 0.25;
//                tag_marker.color.a = 1;
//                tag_marker.color.r = 0.0;
//                tag_marker.color.g = 1.0;
//                tag_marker.color.b = 0.0;

//               detected_tag_pose_msg.markers.push_back(tag_marker);
        //}

        //detected_tag_locations_visualization_pub_.publish(detected_tag_pose_msg);

}

visualization_msgs::Marker EKFLocalizationNode::ConverterMarker(
    const std::vector<cv::Point2f> &data) {
    visualization_msgs::Marker marker_msg;
    marker_msg.header.frame_id ="world";
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.ns = "ns";
    marker_msg.action = visualization_msgs::Marker::MODIFY;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.id = 0;
    marker_msg.lifetime = ros::Duration(0.1);
    marker_msg.type = visualization_msgs::Marker::SPHERE_LIST;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale,
    // for the line width
    marker_msg.scale.x = 0.01;
    // Line list is red
    marker_msg.color.r = 1.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 0.0;
    marker_msg.text = "marker";
    // must be non-zero. otherwise is transparency
    marker_msg.color.a = 1.0;
    // Create the vertices for the points and lines
    geometry_msgs::Point base_link;
    base_link.x = 0;
    base_link.y = 0;
    marker_msg.points.push_back(base_link);
    for (auto point : data) {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0;
        marker_msg.points.push_back(p);
    }
    return marker_msg;
}


void EKFLocalizationNode::onTimer(const ros::TimerEvent& event)
{
	// create the local copies for holding the latest global copy of the latest subscription
	std::vector<ekf_localization_node_ns::AprilTagDetection> tag_detections;
	double wheel_speed;
	double yaw_rate;
	double steering_angle;
        // debug
        std::cout << "Start ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
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
                tag_detections_.clear();
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

                // Push the location to the tracking vector
                cv::Point2f moved_point;
                moved_point.x = vehicle_position.pose.position.x;
                moved_point.y = vehicle_position.pose.position.y;
                moved_path.push_back(moved_point);
                std::cout << "Pushed ~ " << std::endl;

	}
	else
	{
		std::cout << "vehicle localization failed ~~!!!" << std::endl;
                return;
	}

        // Visualize the Moved Path
       visualization_msgs::Marker moved_path_msg = ConverterMarker(moved_path);
       moved_path_visualization_pub_.publish(moved_path_msg);
//       for(int i; i < moved_path.size(); ++i)
//       {
//           std::cout << "Moved_Path Size: " << moved_path.size() << std::endl;
//           std::cout << "Point #" << i <<"Pos X: " << moved_path.at(i).x << std::endl;
//           std::cout << "Point #" << i <<"Pos Y: " << moved_path.at(i).y << std::endl;
//       }
//       std::cout << "==================================================" << std::endl;

}	












} // end of namespace
