// Include C++ Core Libraries
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>
// Include ROS Libraries
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
// EKFLocalizer's Header File
#include "apriltag_ekf_localization/EKFLocalizer.h"

#define DEG2RAD M_PI/180
#define RAD2DEG 180/M_PI

namespace ekf_localization_node_ns{

bool EKFLocalizer::Init(const std::string& config_file)
{	
	// 1. Clear all the states and inputs
	wheel_speed_ = 0.0;
	yaw_rate_ = 0.0;
	steering_angle_ = 0.0;
	tag_detections_.clear();
	tag_ground_truth_.clear();
	total_number_of_tags_ = 0;

	// 2. Read the Tag Ground Truth Location from the .json config file
	std::cout << "Config File Path : " << config_file << std::endl; // for debug
	if (ReadTagGroundTruth(config_file) != true)
	{
		std::cout << "Failed to read the Tag Ground Truth Config File ~!!" << std::endl;
		return false;
	}
	else // for debug
		std::cout << "Successfully read the Tag Ground Truth Config File ~~~!" << std::endl;

	// 3.Reset all the system Matrixes
	Xt_.setZero(); // Xt = [0,0,0]^T
	Pt_.setZero();
        Pt_(0,0) = 1.0;
        Pt_(1,1) = 1.0;
        Pt_(2,2) = 1.0;

	return true;
}

bool EKFLocalizer::UpdateWheelSpeed(double wheel_speed)
{
	wheel_speed_ = wheel_speed;
	wheel_speed_received_ = true;
        // debug
        std::cout << "Wheel Speed Received  ~~~" << std::endl;
	return true;
}

bool EKFLocalizer::UpdateYawRate(double yaw_rate)
{
        yaw_rate_ = yaw_rate * DEG2RAD;
        // Revert the Rotation as --> Right Thumb Rule --> AntiClockwise(+)
        yaw_rate_ = yaw_rate_ * -1.0;
	yaw_rate_received_ = true;
        // debug
        std::cout << "IMU Yaw Rate Received ~~~" << std::endl;
	return true;
}

bool EKFLocalizer::UpdateSteeringAngle(double steering_angle)
{
	steering_angle_ = steering_angle;
        //Steering angle Low-PassFilter
                if(steering_angle_* static_cast<double>(RAD2DEG) <= 1)
            if(steering_angle_* static_cast<double>(RAD2DEG) >= -1)
                steering_angle_ = 0;
        steering_angle_received_ = true;
        // debug
        std::cout << "Steering Angle Command Received ~~~~~" << std::endl;
	return true;
}

bool EKFLocalizer::UpdateTagDetections(std::vector<ekf_localization_node_ns::AprilTagDetection> tag_detections)
{
	tag_detections_ = tag_detections;
        if(tag_detections_.empty())
            tag_detections_received_ = false;
        else
            tag_detections_received_ = true;
        // debug
        std::cout << "Tag Detections Received ~~~" << std::endl;
        std::cout << "Tag Detections size: " << tag_detections_.size() << std::endl;
        return tag_detections_received_;
}

bool EKFLocalizer::GetJsonObjectFromConfigFile(const std::string& config_file, json11::Json* json)
{
	if(json == nullptr)
	{
		return false;
	}
	
	std::filebuf input_file;
    
    if(!input_file.open(config_file, std::ios::in))
    {
    	std::cout << config_file << std::endl;
		return false;
    }
    std::istream iss(&input_file);
    std::istreambuf_iterator<char> eos;
    std::string config_json(std::istreambuf_iterator<char>(iss), eos);
    input_file.close();
    std::string error;
    *json = json11::Json::parse(config_json, error);  
    if(error.empty() == false)
    {
    	return false;
    }
    
    return true;
}

bool EKFLocalizer::ReadTagGroundTruth(const std::string& config_file)
{
	json11::Json json_obj;
    if(GetJsonObjectFromConfigFile(config_file, &json_obj) == false)
    {
    	return false;
    }
    // Start Reading from .json
    // Number of Tags in total
   	if(json_obj["map_setting"]["num_of_tags"].is_null())
   	{
        std::cout << "Cannot determine total number of Tags on the Map" << std::endl;
        return false;
    }
    total_number_of_tags_= json_obj["map_setting"]["num_of_tags"].number_value();

    for (int i = 0; i < total_number_of_tags_; ++i)
    {
    	ekf_localization_node_ns::AprilTagGroundTruth TempTagGroundTruth;

    	std::string counter_str;
    	std::stringstream ss;
    	ss << i;
    	counter_str = ss.str();
    	// ID
    	if(json_obj["tag_"+counter_str]["ID"].is_null())
   		{
        	std::cout << "Cannot Retrieve Tag #" << i <<"'s ID ~" << std::endl;
        	return false;
    	}
    	TempTagGroundTruth.ID = json_obj["tag_"+counter_str]["ID"].number_value();

    	// Pos_X
    	if(json_obj["tag_"+counter_str]["Pos_X"].is_null())
   		{
        	std::cout << "Cannot Retrieve Tag #" << i <<"'s Pos_X ~" << std::endl;
        	return false;
    	}
    	TempTagGroundTruth.Pos_X = json_obj["tag_"+counter_str]["Pos_X"].number_value();

    	// Pos_Y
    	if(json_obj["tag_"+counter_str]["Pos_Y"].is_null())
   		{
        	std::cout << "Cannot Retrieve Tag #" << i <<"'s Pos_Y ~" << std::endl;
        	return false;
    	}
    	TempTagGroundTruth.Pos_Y = json_obj["tag_"+counter_str]["Pos_Y"].number_value();

    	// Direction
    	if(json_obj["tag_"+counter_str]["Direction"].is_null())
   		{
        	std::cout << "Cannot Retrieve Tag #" << i <<"'s Direction ~" << std::endl;
        	return false;
    	}
    	TempTagGroundTruth.Direction = json_obj["tag_"+counter_str]["Direction"].number_value();

    	tag_ground_truth_.push_back(TempTagGroundTruth);
    }

    // Read the Process Covariance Parameters (a1, .., a9)
    // a1
    if(json_obj["process_covariance"]["a1"].is_null())
   	{
        std::cout << "Cannot retrieve process covariance a1" << std::endl;
        return false;
    }
    a1= json_obj["process_covariance"]["a1"].number_value();

    //a2 
    if(json_obj["process_covariance"]["a2"].is_null())
   	{
        std::cout << "Cannot retrieve process covariance a2" << std::endl;
        return false;
    }
    a2= json_obj["process_covariance"]["a2"].number_value();

    //a3
    if(json_obj["process_covariance"]["a3"].is_null())
   	{
        std::cout << "Cannot retrieve process covariance a3" << std::endl;
        return false;
    }
    a3= json_obj["process_covariance"]["a3"].number_value();

    //a4
    if(json_obj["process_covariance"]["a4"].is_null())
   	{
        std::cout << "Cannot retrieve process covariance a4" << std::endl;
        return false;
    }
    a4= json_obj["process_covariance"]["a4"].number_value();

    //a5
    if(json_obj["process_covariance"]["a5"].is_null())
   	{
        std::cout << "Cannot retrieve process covariance a5" << std::endl;
        return false;
    }
    a5= json_obj["process_covariance"]["a5"].number_value();

    //a6 
    if(json_obj["process_covariance"]["a6"].is_null())
   	{
        std::cout << "Cannot retrieve process covariance a6" << std::endl;
        return false;
    }
    a6= json_obj["process_covariance"]["a6"].number_value();

    //a7 
    if(json_obj["process_covariance"]["a7"].is_null())
   	{
        std::cout << "Cannot retrieve process covariance a7" << std::endl;
        return false;
    }
    a7= json_obj["process_covariance"]["a7"].number_value();

    //a8 
    if(json_obj["process_covariance"]["a8"].is_null())
   	{
        std::cout << "Cannot retrieve process covariance a8" << std::endl;
        return false;
    }
    a8= json_obj["process_covariance"]["a8"].number_value();

    //a9 
    if(json_obj["process_covariance"]["a9"].is_null())
   	{
        std::cout << "Cannot retrieve process covariance a9" << std::endl;
        return false;
    }
    a9= json_obj["process_covariance"]["a9"].number_value();


    // Read the Offset between the camera and base frame (from base frame to camera frame)
    // X Offset
    if(json_obj["camera_base_offset"]["x"].is_null())
   	{
        std::cout << "Cannot retrieve the X-Offset between the camera and base frame" << std::endl;
        return false;
    }
    camera_base_x_offset_= json_obj["camera_base_offset"]["x"].number_value();

    // Y Offset
    if(json_obj["camera_base_offset"]["y"].is_null())
   	{
        std::cout << "Cannot retrieve the Y-Offset between the camera and base frame" << std::endl;
        return false;
    }
    camera_base_y_offset_= json_obj["camera_base_offset"]["y"].number_value();

    // Z Offset
    if(json_obj["camera_base_offset"]["z"].is_null())
   	{
        std::cout << "Cannot retrieve the Z-Offset between the camera and base frame" << std::endl;
        return false;
    }
    camera_base_z_offset_= json_obj["camera_base_offset"]["z"].number_value();


    // Read the Measurement Covariance Parameters

    // Variance on r
    if(json_obj["measurement_covariance"]["variance_on_r"].is_null())
   	{
        std::cout << "Cannot retrieve the Measurement Covariance on r" << std::endl;
        return false;
    }
    variance_on_r_= json_obj["measurement_covariance"]["variance_on_r"].number_value();

    // Variance on phi
    if(json_obj["measurement_covariance"]["variance_on_phi"].is_null())
   	{
        std::cout << "Cannot retrieve the Measurement Covariance on phi" << std::endl;
        return false;
    }
    variance_on_phi_= json_obj["measurement_covariance"]["variance_on_phi"].number_value();

    // Variance on s
    if(json_obj["measurement_covariance"]["variance_on_s"].is_null())
   	{
        std::cout << "Cannot retrieve the Measurement Covariance on s" << std::endl;
        return false;
    }
    variance_on_s_= json_obj["measurement_covariance"]["variance_on_s"].number_value();


    return true;
}

// Main Function !!!! Triggered by the onTimer() at Node
bool EKFLocalizer::Locate(ekf_localization_node_ns::PoseWithConvariance* vehicle_position)
{
	// Prediction
        if(wheel_speed_received_ == true && yaw_rate_received_ == true && steering_angle_received_ == true)
	{
		// debug
		std::cout << "wheel_speed, yaw_rate and steering_angle received. Prediction Starts ~~~" << std::endl;
		Predict(wheel_speed_, yaw_rate_, steering_angle_);   // Xt and Pt is predicted.
	}

	// Correction (only when tag detected)
        if(tag_detections_received_ == true)
	{
		// debug
		std::cout << "Tags Detected, Correction Starts ~~~" << std::endl;
		Correct(tag_detections_, tag_ground_truth_);  // Xt and Pt is corrected.
	}
	else
		std::cout << "Tags aren't detected. Localization based on IMU/Encoder ~~~" << std::endl;

        // For Debug`
        // DEBUG
        std::cout << " ================= Current Vehicle Status ==========================" << std::endl;
        std::cout << "Vehicle Speed: " << wheel_speed_ << std::endl;
        std::cout << "Yaw Rate: " << yaw_rate_ << std::endl;
        std::cout << "Steering Angle: " << steering_angle_;
        std::cout << " " << std::endl;

        std::cout << "State X = \n" << Xt_ << std::endl;
        std::cout << "Corvariance P = \n" << Pt_ << std::endl;
        if(Xt_(0,0) < 0)
            return false;
	vehicle_position->pose.position.x = Xt_(0,0);
	vehicle_position->pose.position.y = Xt_(1,0);
	vehicle_position->pose.position.z = 0.0;

	tf2::Quaternion quad;
	quad.setRPY(0, 0, Xt_(2,0));
	vehicle_position->pose.orientation.x = quad[0];
	vehicle_position->pose.orientation.y = quad[1];
	vehicle_position->pose.orientation.z = quad[2];
	vehicle_position->pose.orientation.w = quad[3];

	vehicle_position->covariance.setZero();
	vehicle_position->covariance(0,0) = Pt_(0,0);  // Covariance on X
	vehicle_position->covariance(1,1) = Pt_(1,1); // Covariance on Y
	vehicle_position->covariance(2,2) = 1;         // Covariance on Z
	vehicle_position->covariance(3,3) = 1;		   // Covariance on rotation about x 
	vehicle_position->covariance(4,4) = 1;		   // Covariance on rotation about y
	vehicle_position->covariance(5,5) = Pt_(2,2);   // Covariance on rotation about z

	return true;
}

// Prediction: Incorporatoing the wheelspeed, yawrate and steerangle with the equation of motion, to predict the current state
void EKFLocalizer::Predict(const double& wheel_speed, const double& yaw_rate, const double& steering_angle)
{	
	double dt = 0.05;  // same as the frequency of the timer
	// Heading = prev.heading + yawrate*dt
	double heading = static_cast<double>(Xt_(2,0)) + yaw_rate*static_cast<double>(DEG2RAD)*dt; 
	normalizeAngle(heading);

	// 1. Setup the Transition Matrix Ft-1
	Eigen::Matrix<double, 3, 3> Ft_;
	Ft_.setZero();
	Ft_(0,0) = 1;
	Ft_(0,1) = 0;
        Ft_(0,2) = -wheel_speed*sin(heading + steering_angle)*dt;

	Ft_(1,0) = 0;
	Ft_(1,1) = 1;
        Ft_(1,2) = wheel_speed*cos(heading + steering_angle)*dt;

	Ft_(2,0) = 0;
	Ft_(2,1) = 0;
	Ft_(2,2) = 1;

	// 2. Prediction :: directly using the equation of motion
	// X
        Xt_(0,0) += wheel_speed*dt * cos(heading + steering_angle);
	// Y
        Xt_(1,0) += wheel_speed*dt * sin(heading + steering_angle);
	// Heading :: the current heading
	Xt_(2,0) = heading;


	// 3. Setup the Process Covariance Matrix Qt-1
	Eigen::Matrix<double, 3, 3> Qt_;
	Qt_.setZero();
	Qt_(0,0) = a1*wheel_speed*wheel_speed + a2*steering_angle*steering_angle + a3*yaw_rate*yaw_rate;
	Qt_(1,1) = a6*wheel_speed*wheel_speed + a5*steering_angle*steering_angle + a6*yaw_rate*yaw_rate;
	Qt_(2,2) = a7*wheel_speed*wheel_speed + a8*steering_angle*steering_angle + a9*yaw_rate*yaw_rate;

	// 4. Define the Mapping Jacobian Matrix Lt 
	Eigen::Matrix<double, 3, 3> Lt_;
	Lt_.setZero();
        Lt_(0,0) = cos(heading + steering_angle) * dt;
        Lt_(0,1) = -sin(heading + steering_angle) * dt;
	Lt_(0,2) = 0;

        Lt_(1,0) = sin(heading + steering_angle) * dt;
        Lt_(1,1) = cos(heading + steering_angle) * dt;
	Lt_(1,2) = 0;

	Lt_(2,0) = 0;
	Lt_(2,1) = 0;
	Lt_(2,2) = dt;

	// 5. Propagate the Covariance Matrix Pt !!!
	Pt_ = Ft_ * Pt_ * Ft_.transpose() + Lt_ * Qt_ * Lt_.transpose();

}

// Correction :: Correct the Position by observing the landmarks/markers
void EKFLocalizer::Correct(const std::vector<ekf_localization_node_ns::AprilTagDetection>& tag_detections, const std::vector<ekf_localization_node_ns::AprilTagGroundTruth>& tag_ground_truth)
{
	// 1. Fetch the Measurement_Z from the tag_detections vector subscribed from apriltag_ros 
	
	// Create a vector holding the measurements
	std::vector<ekf_localization_node_ns::Measurement_Z> measurements_z;

	// Check how many tag detected
	int num_of_tags_detected = tag_detections.size();
	int num_of_valid_measurement = Measure(measurements_z, num_of_tags_detected, tag_detections);
	
	// Correction Main Process ~~~!!!!
	// Correct N times, where N = number of valid measurements
	for(Measurement_Z measured_z : measurements_z)
	{
		// a. Check the corresponding Index @Ground Truth Config File{The nth tag in tag_ground_truth}
		int index_at_ground_truth_list = checkTagIndex(measured_z.ID_, tag_ground_truth);

		// b. Define the Expected Measurement
		double mx = tag_ground_truth.at(index_at_ground_truth_list).Pos_X;
		double my = tag_ground_truth.at(index_at_ground_truth_list).Pos_Y;
		double predicted_x = Xt_(0,0);
		double predicted_y = Xt_(1,0);
		double predicted_heading = Xt_(2,0);

		double expected_r = sqrt( pow((mx - predicted_x), 2) + pow((my - predicted_y), 2) );
		double expected_phi = atan2( (my - predicted_y), (mx - predicted_x)) - predicted_heading;
		normalizeAngle(expected_phi);

		// Real Measurement
		Eigen::Vector3d measurement_z (measured_z.r_, measured_z.phi_, 1);
		// Expected Measurement
		Eigen::Vector3d expected_z (expected_r, expected_phi, 1);

		// c. Define the Measurement Matrix Ht
		Eigen::Matrix<double, 3, 3> Ht_;
		Ht_.setZero();
		Ht_(0,0) = -(mx - predicted_x) / expected_r;
		Ht_(0,1) = -(my - predicted_y) / expected_r;
		Ht_(0,2) = 0.;

		Ht_(1,0) = (my - predicted_y) / pow(expected_r, 2) ;
		Ht_(1,1) = -(mx - predicted_x) / pow(expected_r, 2);
		Ht_(1,2) = -1.;

		Ht_(2,0) = 0.;
		Ht_(2,1) = 0.;
		Ht_(2,2) = 0.;

		// d. Define the Measurement Covariance Rt
		Eigen::Matrix<double, 3, 3> Rt_;
		Rt_.setZero();
		Rt_(0,0) = pow(variance_on_r_, 2);
		Rt_(1,1) = pow(variance_on_phi_, 2);
		Rt_(2,2) = pow(variance_on_s_, 2);

		// e! Find the Matrix S !!!
		Eigen::Matrix<double, 3, 3>St_;
		St_ = Ht_ * Pt_ * Ht_.transpose() + Rt_;

		// f! Find the KALMAN GAIN ~~~!!!!!
		Eigen::Matrix<double, 3, 3>Kt_;
		Kt_ = Pt_ * Ht_.transpose() * St_.inverse();

		// g!!!! Correct the Prediction Here ~~~~!!!!!
		Xt_ = Xt_ + Kt_*(measurement_z - expected_z);

		// h!!!!! Propagate the Covariance also as well
		Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);  // identity matrix of 1
		Pt_ = (I - Kt_*Ht_) * Pt_;

	}


}

int EKFLocalizer::checkTagIndex(int tag_id, const std::vector<ekf_localization_node_ns::AprilTagGroundTruth>& tag_ground_truth)
{
	bool found = false;
	int index_at_ground_truth_list = 0;
	while(!found)
	{
		if (tag_id == tag_ground_truth.at(index_at_ground_truth_list).ID)
			found = true;
		else
			index_at_ground_truth_list = index_at_ground_truth_list + 1;
	}

	return index_at_ground_truth_list;
}

int EKFLocalizer::Measure(std::vector<Measurement_Z>& measurements_z, const int& num_of_tags_detected, const std::vector<ekf_localization_node_ns::AprilTagDetection>& tag_detections)
{
	// Loop through the tag_detections --> Fetch the Pose and Orientation of each tags --> Choose the tags within 1m --> Express it with Transformation matrix  --> save each as (r, phi, index) :: Landmark Measurement Model Format
	for (int i = 0; i < num_of_tags_detected; ++i)
	{
		//Fetch the Pose and Orientation of each tags and Express it with Transformation matrix
		// Pose
		cv::Vec3d tvec;
		tvec[0] = tag_detections.at(i).pose.position.x;
		tvec[1] = tag_detections.at(i).pose.position.y;
		tvec[2] = tag_detections.at(i).pose.position.z;

		// Only consider the tags within 1m
		double distance = cv::norm<double>(tvec);
		if (distance > searching_radius_)
			continue;

		// Orientation
		tf2::Quaternion quad;
		quad[0] = tag_detections.at(i).pose.orientation.x;
		quad[1] = tag_detections.at(i).pose.orientation.y;
		quad[2] = tag_detections.at(i).pose.orientation.z;
		quad[3] = tag_detections.at(i).pose.orientation.w;

		tf2::Matrix3x3 R(quad);

		// Transformation Matrix of TagPosition with respect to Camera Frame
		Eigen::Matrix<double, 4, 4> T_camera_tag;
		T_camera_tag << 
		R[0][0], R[0][1], R[0][2], tvec[0],
		R[1][0], R[1][1], R[1][2], tvec[1],
		R[2][0], R[2][1], R[2][2], tvec[2],
		0., 0., 0., 1;

		// Transformation Matrix of Camera Frame with respect to Robot Base Link(at the center of gravity of the robot)
		Eigen::Matrix<double, 4, 4> T_base_camera;
		T_base_camera <<
		1., 0., 0., camera_base_x_offset_,
		0., 1., 0., camera_base_y_offset_,
		0., 0., 1., camera_base_z_offset_;

		// Transformation Matrix from BaseLink to Marker/Tag ~~~!!!!
		Eigen::Matrix<double, 4, 4> T_base_tag;
		T_base_tag = T_base_camera * T_camera_tag;

		//save each as (r, phi, index) :: Landmark Measurement Model Format
		double measured_x = T_base_tag(0,3);
		double measured_y = T_base_tag(1,3);
		
		double r = sqrt(measured_x*measured_x + measured_y*measured_y);
		double phi = atan2(measured_y, measured_x);
                int id = tag_detections.at(i).id;  // id of the tag

		// Push Back the Valid Measurement to the vector holding the measurements
		measurements_z.push_back(Measurement_Z(id, r, phi));
	}
	// Return number of valid measurement_z
	return measurements_z.size();
}


void EKFLocalizer::normalizeAngle(double &angle)
{
	if(angle >= M_PI)
		angle = angle - 2*M_PI;
	if(angle < -M_PI)
		angle = angle + 2*M_PI;
}



visualization_msgs::Marker EKFLocalizer::VisualizeVehicle()
{
	
	// Display the Vehicle Position
	visualization_msgs::Marker vehicle_marker;
	vehicle_marker.header.frame_id = "world";
	vehicle_marker.header.stamp = ros::Time();
	vehicle_marker.ns = "apriltag_ekf_localization";
	vehicle_marker.id = 0;
	vehicle_marker.type = visualization_msgs::Marker::SPHERE;
	vehicle_marker.action = visualization_msgs::Marker::ADD;
	vehicle_marker.pose.position.x = Xt_(0,0);
	vehicle_marker.pose.position.y = Xt_(1,0);
	vehicle_marker.pose.position.z = 0.0;
	vehicle_marker.pose.orientation = tf::createQuaternionMsgFromYaw(Xt_(2,0));
        vehicle_marker.scale.x = 0.25;
        vehicle_marker.scale.y = 0.25;
        vehicle_marker.scale.z = 0.25;
	vehicle_marker.color.a = 0.8;
	vehicle_marker.color.r = 1.0;
	vehicle_marker.color.g = 0.0;
	vehicle_marker.color.b = 0.0;
    
    return vehicle_marker;
}

visualization_msgs::MarkerArray EKFLocalizer::VisualizeTagLocations()
{
	
	visualization_msgs::MarkerArray all_markers;
	// Display all the taf location on the map as reference
	for (int i = 0; i < tag_ground_truth_.size(); ++i)
	{
		visualization_msgs::Marker tag_marker;
		tag_marker.header.frame_id = "world";
		tag_marker.header.stamp = ros::Time();
		tag_marker.ns = "apriltag_ekf_localization";
		tag_marker.id = i;
		tag_marker.type = visualization_msgs::Marker::CUBE;
		tag_marker.action = visualization_msgs::Marker::ADD;
		tag_marker.pose.position.x = tag_ground_truth_.at(i).Pos_X;
		tag_marker.pose.position.y = tag_ground_truth_.at(i).Pos_Y;
		tag_marker.pose.position.z = 0.0;
		tag_marker.pose.orientation = tf::createQuaternionMsgFromYaw(tag_ground_truth_.at(i).Direction);
                tag_marker.scale.x = 0.25;
                tag_marker.scale.y = 0.25;
                tag_marker.scale.z = 0.25;
		tag_marker.color.a = 1;
		tag_marker.color.r = 0.0;
		tag_marker.color.g = 0.0;
		tag_marker.color.b = 1.0;

		all_markers.markers.push_back(tag_marker);
	}

	return all_markers;
}


} // end of namespace ~ 
