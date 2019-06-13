#ifndef DATA_TYPE_H_
#define DATA_TYPE_H_
#include <string>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace ekf_localization_node_ns{

struct Point
{
	double x;
	double y;
	double z;
};

struct Quaternion
{
	double x;
	double y;
	double z;
	double w;
};

struct Pose
{
	Point position;
	Quaternion orientation;
};

struct PoseWithConvariance
{
	Pose pose;
	Eigen::Matrix<double, 6, 6> covariance;
};


struct AprilTagDetection
{
        int id;
        double size;
	Pose pose;
};

struct AprilTagGroundTruth
{
	int ID;
	double Pos_X;
	double Pos_Y;
	double Direction;
};

}

#endif
