#ifndef APRILTAG_EKF_LOCALIZATION_NODE_H_
#define APRILTAG_EKF_LOCALIZATION_NODE_H_

#include <mutex>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

//#include <std_msgs/string.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

#include "apriltag_ekf_localization/AprilTagDetection.h"
#include "apriltag_ekf_localization/AprilTagDetectionArray.h"
#include "apriltag_ekf_localization/Chassis_cmd.h"
#include "apriltag_ekf_localization/Chassis_data.h"

#include "EKFLocalizer.h"
#include "data_type.h"

namespace ekf_localization_node_ns {
class EKFLocalizationNode {

public:
	/**
	* @brief: Constructor :: Getting the Parameters, ConfigPath from Launch
	**/
	EKFLocalizationNode(ros::NodeHandle node, ros::NodeHandle private_nh);

	/**
	* @brief: Spinning - Call the init()
	*/
	bool Spin();

private:

	/**
	* @brief: 1.Call the EKFLocalizer->init()
   	 	 	  2.Setting up the Subscriber, Publisher and Timer for running the EKFLocalizer
   	**/
   	bool init();

   	/**
   	* @brief: CallBack Function for WheelEncoder
        **/
        void onWheelSpeed(const apriltag_ekf_localization::Chassis_data::ConstPtr &msg);

        /**
        * @brief: CallBack Function for IMU
        **/
        void onIMU(const std_msgs::Float32::ConstPtr &msg);

        /**
        * @brief: CallBack function for SteeringAngle Command
        **/
        void onSteeringAngle(const apriltag_ekf_localization::Chassis_cmd::ConstPtr &msg);

        /**
        * @brief: CallBack Function for TagPoseArray
	**/
	void onTagPoseArray(const apriltag_ekf_localization::AprilTagDetectionArray::ConstPtr &msg);

	/**
	* @brief: Perfrom the Following action in a timer-based of 20Hz Frequency (Period = 0.05s)
			  1. Call the EKFLocalizer->Locate()
			  2. Publish the current Pose
	**/
	void onTimer(const ros::TimerEvent& event);


        visualization_msgs::Marker ConverterMarker(
            const std::vector<cv::Point2f> &data);

	/*****************************************************************
								Private Attributes
	*****************************************************************/
	// Property Components
	bool get_all_params_;
	double duration_time_;
	std::string road_sign_config_file_;

	//std::vector <
	// ROS Components
	ros::NodeHandle node_;
	ros::NodeHandle priv_node_;

	ros::Subscriber wheel_speed_sub_;
	ros::Subscriber imu_sub_;
	ros::Subscriber steering_angle_sub_;
	ros::Subscriber tag_pose_sub_;
	
	ros::Publisher location_pub_;
	ros::Publisher vehicle_visualization_pub_;
	ros::Publisher tag_locations_visualization_pub_;
        ros::Publisher moved_path_visualization_pub_;
        ros::Publisher detected_tag_locations_visualization_pub_;
	
	ros::Timer timer_;

	// EKFLocalizer Instance
	EKFLocalizer ekf_localizer_;

	// Receiver
	std::mutex tag_detections_mutex_; 
	std::vector<ekf_localization_node_ns::AprilTagDetection> tag_detections_;
	
	std::mutex wheel_speed_mutex_;
	double wheel_speed_;

	std::mutex imu_mutex_;
	double yaw_rate_;

	std::mutex steering_angle_mutex_;
	double steering_angle_;

        // Create a vector for tracking the moved path
        std::vector<cv::Point2f> moved_path;
};


} // end of namespace

#endif
