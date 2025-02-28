#include "hand_eye_calibration.h"

namespace HandEyeCalibration {

    // Skew matrix function
    Eigen::Matrix3d skew(Eigen::Vector3d V) {
        Eigen::Matrix3d S;
        S << 0, -V(2), V(1),
             V(2), 0, -V(0),
             -V(1), V(0), 0;
        return S;
    }

    // Quaternion to rotation matrix
    Eigen::Matrix4d quat2rot(Eigen::Vector3d q) {
        double p = q.transpose() * q;
        if (p > 1) std::cerr << "Warning: quat2rot: quaternion greater than 1" << std::endl;
        double w = sqrt(1 - p); // w = cos(theta/2)
        Eigen::Matrix4d R = Eigen::MatrixXd::Identity(4, 4);
        Eigen::Matrix3d res = 2 * (q * q.transpose()) + 2 * w * skew(q);
        res += Eigen::MatrixXd::Identity(3, 3) - 2 * p * Eigen::MatrixXd::Identity(3, 3);
        R.topLeftCorner(3, 3) = res;
        return R;
    }

    // Rotation matrix to quaternion
    Eigen::Vector3d rot2quat(Eigen::MatrixXd R) {
        double w4 = 2 * sqrt(1 + R.topLeftCorner(3, 3).trace());
        Eigen::Vector3d q;
        q << (R(2, 1) - R(1, 2)) / w4,
             (R(0, 2) - R(2, 0)) / w4,
             (R(1, 0) - R(0, 1)) / w4;
        return q;
    }

    // Translation transformation
    Eigen::Matrix4d transl(Eigen::Vector3d x) {
        Eigen::Matrix4d r = Eigen::MatrixXd::Identity(4, 4);
        r.topRightCorner(3, 1) = x;
        return r;
    }

    // Hand-Eye Calibration using TSAI method
    Eigen::Matrix4d handEye(const std::vector<Eigen::Matrix4d>& bHg, const std::vector<Eigen::Matrix4d>& cHw) {
        int M = bHg.size();
        int K = M - 1; // Optimized to avoid redundant computations
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3 * K, 3);
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3 * K, 1);
        int k = 0;

        for (int i = 0; i < M - 1; ++i) {
            for (int j = i + 1; j < M; ++j) {
                Eigen::Matrix3d Rg = bHg[i].topLeftCorner(3, 3).transpose() * bHg[j].topLeftCorner(3, 3);
                Eigen::Vector3d Pg = bHg[i].topRightCorner(3, 1) - bHg[j].topRightCorner(3, 1);
                Eigen::Vector3d Pc = cHw[i].topRightCorner(3, 1) - cHw[j].topRightCorner(3, 1);

                A.row(k) = skew(Rg * Pc + Pc).transpose();
                B.row(k) = Pc - Pg;
                k++;
            }
        }

        // Solve for gHc using least squares
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Vector3d t = svd.solve(B);

        Eigen::Matrix4d gHc = Eigen::MatrixXd::Identity(4, 4);
        gHc.topRightCorner(3, 1) = t;
        return gHc;
    }

    // Parse poses from the YAML file
    std::vector<Eigen::Matrix4d> parsePosesFromYAML(const std::string& yaml_path) {
        YAML::Node config = YAML::LoadFile(yaml_path); // Load the YAML file
        int num_poses = config["num_poses"].as<int>(); // Number of poses
        std::vector<Eigen::Matrix4d> poses;

        for (int i = 0; i < num_poses; ++i) {
            YAML::Node pose_node = config["poses"][i]; // Access each pose
            std::vector<double> position = pose_node["position"].as<std::vector<double>>();
            std::vector<double> orientation = pose_node["orientation"].as<std::vector<double>>();

            // Convert roll-pitch-yaw to a rotation matrix
            double roll = orientation[0] * M_PI / 180.0;
            double pitch = orientation[1] * M_PI / 180.0;
            double yaw = orientation[2] * M_PI / 180.0;

            Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

            // Create the homogeneous transformation matrix
            Eigen::Matrix4d pose = Eigen::MatrixXd::Identity(4, 4);
            pose.topLeftCorner(3, 3) = q.toRotationMatrix();
            pose.topRightCorner(3, 1) << position[0], position[1], position[2];

            poses.push_back(pose.inverse()); // Store the inverse pose (bHg)
        }

        return poses;
    }

    // Main calibration function
    int handEye_calib(Eigen::Matrix4d& gHc, const std::string& path) {
        // Parse poses from the YAML file
        std::string yaml_path = path + "/robot_poses.yaml"; // Path to the YAML file
        std::vector<Eigen::Matrix4d> bHg = parsePosesFromYAML(yaml_path);

        if (bHg.empty()) {
            std::cerr << "Error: No valid poses found in the YAML file." << std::endl;
            return -1;
        }

        // Read images
        std::vector<cv::Mat> images;
        std::cout << "******************Reading images......******************" << std::endl;

        for (int i = 0; i < 10; ++i) {
            std::string image_path = path + "/" + std::to_string(i) + ".png";
            cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
            if (!image.empty()) {
                images.push_back(image);
            } else {
                std::cerr << "Error: Could not read image at " << image_path << std::endl;
                return -1;
            }
        }

        // Detect chessboard corners
        std::vector<std::vector<cv::Point2f>> image_points_seq;
        cv::Size board_size(9, 6);
        for (const auto& image : images) {
            std::vector<cv::Point2f> corners;
            bool found = cv::findChessboardCorners(image, board_size, corners);
            if (found) {
                cv::cornerSubPix(image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                                 cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
                image_points_seq.push_back(corners);
            }
        }

        if (image_points_seq.empty()) {
            std::cerr << "Error: No valid chessboard corners detected." << std::endl;
            return -1;
        }

        // Perform camera calibration
        std::vector<std::vector<cv::Point3f>> object_points;
        for (size_t i = 0; i < image_points_seq.size(); ++i) {
            std::vector<cv::Point3f> obj;
            for (int j = 0; j < board_size.height; ++j) {
                for (int k = 0; k < board_size.width; ++k) {
                    obj.emplace_back(cv::Point3f(j * 25, k * 25, 0));
                }
            }
            object_points.push_back(obj);
        }

        cv::Mat camera_matrix, dist_coeffs;
        std::vector<cv::Mat> rvecs, tvecs;
        cv::calibrateCamera(object_points, image_points_seq, images[0].size(), camera_matrix, dist_coeffs, rvecs, tvecs);

        // Convert camera poses
        std::vector<Eigen::Matrix4d> cHw;
        for (size_t i = 0; i < rvecs.size(); ++i) {
            Eigen::Matrix4d pose = Eigen::MatrixXd::Identity(4, 4);
            cv::cv2eigen(rvecs[i], pose.topLeftCorner(3, 3));
            cv::cv2eigen(tvecs[i], pose.topRightCorner(3, 1));
            cHw.push_back(pose.inverse());
        }

        // Perform hand-eye calibration
        gHc = handEye(bHg, cHw);
        return 0;
    }

    // Validation function
    void validateCalibration(const Eigen::Matrix4d& gHc, const std::vector<Eigen::Matrix4d>& bHg, const std::vector<Eigen::Matrix4d>& cHw) {
        double total_error = 0.0;
        for (size_t i = 0; i < bHg.size(); ++i) {
            Eigen::Matrix4d computed_cHw = bHg[i] * gHc;
            double error = (computed_cHw - cHw[i]).norm();
            total_error += error;
        }
        double avg_error = total_error / bHg.size();
        std::cout << "Average validation error: " << avg_error << std::endl;
    }
}