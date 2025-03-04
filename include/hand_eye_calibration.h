#ifndef HAND_EYE_CALIBRATION_H
#define HAND_EYE_CALIBRATION_H

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h> // Include YAML-CPP for parsing
#include <opencv2/core/eigen.hpp>


#define PI 3.1415926

namespace HandEyeCalibration {

    // Function declarations
    Eigen::Matrix3d skew(Eigen::Vector3d V);
    Eigen::Matrix4d quat2rot(Eigen::Vector3d q);
    Eigen::Vector3d rot2quat(Eigen::MatrixXd R);
    Eigen::Matrix4d transl(Eigen::Vector3d x);
    Eigen::Matrix4d handEye(const std::vector<Eigen::Matrix4d>& bHg, const std::vector<Eigen::Matrix4d>& cHw);
    int handEye_calib(Eigen::Matrix4d& gHc, const std::string& path);

    // Helper functions
    void validateCalibration(const Eigen::Matrix4d& gHc, const std::vector<Eigen::Matrix4d>& bHg, const std::vector<Eigen::Matrix4d>& cHw);

    // YAML parsing function
    std::vector<Eigen::Matrix4d> parsePosesFromYAML(const std::string& yaml_path);
}

#endif // HAND_EYE_CALIBRATION_H