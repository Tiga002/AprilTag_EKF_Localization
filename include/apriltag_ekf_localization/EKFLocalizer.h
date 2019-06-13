#ifndef EKF_LOCALIZER_H_
#define EKF_LOCALIZER_H_

#include <string>
#include <cmath>

#include "json11/json11.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

#include <apriltag_ekf_localization/data_type.h>

namespace ekf_localization_node_ns
{	
	class Measurement_Z{
	
	public:
		Measurement_Z(){};
		Measurement_Z(const int& ID, const double& r, const double& phi): ID_(ID), r_(r), phi_(phi){}
		int ID_;
		double r_;
		double phi_;
	};

	class EKFLocalizer{
	
	public:
		
		/**
		* @brief: Constructor 
		*/
		EKFLocalizer() = default;

		/**
		* @brief: Destructor
		**/
		~EKFLocalizer() = default;

		/**
		* @brief: Initialze the EKFLocalizer with several things:
				  1. Load the Road Sign Position @ the map
				  2. Reset all the system Matrixes
		**/
		bool Init(const std::string& config_file);

		/**
		* @brief: Update the EKFLocalizer's wheel speed
		**/
		bool UpdateWheelSpeed(double wheel_speed);

		/**
		* @brief: Update the EKFLocalizer's Yaw Rate from IMU
		**/
		bool UpdateYawRate(double yaw_rate);

		/**
		* @brief: Update the EKFLocalizer's Steer Command
		**/
		bool UpdateSteeringAngle(double steering_angle);

		/**
		* @brief: Update the EKFLocalizer's Tag Detections
		**/
		bool UpdateTagDetections(std::vector<ekf_localization_node_ns::AprilTagDetection> tag_detections);

		/**
		*@ brief: Locate the current Vehicle Position !!!
		**/
		bool Locate(ekf_localization_node_ns::PoseWithConvariance* vehicle_position);

		/**
		* @brief: Visualize the Vehicle Position
		**/
		visualization_msgs::Marker VisualizeVehicle();

		/**
		* @brief: Visualize the Tag Locations
		**/
		visualization_msgs::MarkerArray VisualizeTagLocations();

	private:
		// EKFLocalizer's Internal Private Function
		
		/**
		* @brief: Prediction ~~
		**/
		void Predict(const double& wheel_speed, const double& yaw_rate, const double& steering_angle);

		/**
		@ brief: Update the Prediction with Non-Linear LandMark Measurement Model ~~~~!!!
		**/
		void Correct(const std::vector<ekf_localization_node_ns::AprilTagDetection>& tag_detections, const std::vector<ekf_localization_node_ns::AprilTagGroundTruth>& tag_ground_truth);

		/**
		@ brief: Get the Measurement with using the landmark measurement model ~~
		**/
		int Measure(std::vector<Measurement_Z>& measurement_z, const int& num_of_tags_detected, const std::vector<ekf_localization_node_ns::AprilTagDetection>& tag_detections);

		/**
		@ brief : Bound the angle between -pi and pi
		**/
		void normalizeAngle(double &angle);
		
		/**
		@ brief: Get the corresponding Index @Ground Truth Config File{The nth tag in tag_ground_truth}
		@ input: Tag's Specific ID
		@ return: the nth tag in the tag_ground_truth, so we can acquire its pre-defined location
		**/
		int checkTagIndex(int tag_id, const std::vector<ekf_localization_node_ns::AprilTagGroundTruth>& tag_ground_truth);

		/**
		@ brief: Read the Tag Ground Truth from the xml
		**/
		bool GetJsonObjectFromConfigFile(const std::string& config_file, json11::Json* json);
		bool ReadTagGroundTruth(const std::string& config_file);

		// Local Holder for the inputs
		double wheel_speed_;
		double yaw_rate_;
		double steering_angle_;
		std::vector<ekf_localization_node_ns::AprilTagDetection> tag_detections_;

		// Boolean for checking information received
		bool wheel_speed_received_ = false;
		bool yaw_rate_received_ = false;
                bool tag_detections_received_ = false;
                bool steering_angle_received_ = false;

		//config file path
		const std::string name_;
		// Holder for the Road Sign Ground Truth Location
		std::vector<ekf_localization_node_ns::AprilTagGroundTruth> tag_ground_truth_;
		int total_number_of_tags_;

		// EKF System Matrixes
		//  Eigen::Matrix<double, 3, 2> Gup;
		// Prediction State Xt
		Eigen::Matrix<double, 3, 1> Xt_;
		// Prediction Covariance Pt
		Eigen::Matrix<double, 3, 3>Pt_;

		// Process Covariance Parameters (a1, .., a9) >= 0
		double a1, a2, a3, a4, a5, a6, a7, a8, a9;

		// Offset between the camera and base frame (from base frame to camera frame)
		double camera_base_x_offset_;
		double camera_base_y_offset_;
		double camera_base_z_offset_;

		// Searching radius of tag (1m)
		double searching_radius_ = 1.;

		// Measurement Covariance Parameters 
		double variance_on_r_;
		double variance_on_phi_;
		double variance_on_s_;

		// Transition Matrix Ft-1
		//Eigen::Matrix<double, 3, 3> Ft_;
		// Process Covariance Matrix Qt-1
		//Eigen::Matrix<double, 3, 3> Qt_;
		// Mapping Jacobian Matrix Lt-1
		//Eigen::Matrix<double, 3, 3>Lt_;
	


	};




















}

#endif
