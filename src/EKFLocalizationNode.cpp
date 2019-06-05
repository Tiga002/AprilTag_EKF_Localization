#include <ros/ros.h>

#include "apriltag_ekf_localization/EKFLocalizationWrapper.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "ekf_localization_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");
    ekf_localization_node_ns::EKFLocalizationNode localization(node, private_nh);
    localization.Spin();
    return 0;
}