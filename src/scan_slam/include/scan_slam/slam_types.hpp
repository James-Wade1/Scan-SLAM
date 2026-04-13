#ifndef SCAN_SLAM_SLAM_TYPES_HPP
#define SCAN_SLAM_SLAM_TYPES_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <Eigen/Dense>

namespace scan_slam {

using Pose2D = Eigen::Vector3d; // x, y, theta

struct ScanPoints {
    std::vector<Eigen::Vector2d> points;
    double max_x;
    double min_x;
    double min_y;
    double max_y;

    ScanPoints(std::vector<Eigen::Vector2d> pts) : points(std::move(pts)) {
        calcBounds();
    }

    void calcBounds() {
        max_x = std::numeric_limits<double>::lowest();
        min_x = std::numeric_limits<double>::max();
        max_y = std::numeric_limits<double>::lowest();
        min_y = std::numeric_limits<double>::max();
        for (const auto& pt : points) {
            if (pt(0) > max_x) max_x = pt(0);
            if (pt(0) < min_x) min_x = pt(0);
            if (pt(1) > max_y) max_y = pt(1);
            if (pt(1) < min_y) min_y = pt(1);
        }
    }
};

struct KeyFrame {
    int id;
    rclcpp::Time timestamp;
    Pose2D pose; // x, y, theta
    ScanPoints scan_points;
    sensor_msgs::msg::LaserScan scan_msg;
    Eigen::Matrix3d information_matrix;
};

struct Constraint
{
    int from_id;
    int to_id;
    Pose2D relative_pose; // x, y, theta
    Eigen::Matrix3d information_matrix;
    bool is_loop_closure = false;
};

struct PoseGraph {
    std::vector<KeyFrame> keyframes;
    std::vector<Constraint> constraints;

    void addKeyFrame(KeyFrame frame) {
        keyframes.push_back(frame);
    }

    void addConstraint(Constraint constraint) {
        constraints.push_back(constraint);
    }

    bool isEmpty() const {
        return (keyframes.empty() && constraints.empty());
    }

    void clear() {
        keyframes.clear();
        constraints.clear();
    }
};

struct MatchResult {
    Pose2D delta_pose;
    Pose2D new_pose;
    Eigen::Matrix3d information_matrix;
    double score;
};

struct OptimizationResult {
    std::unordered_map<int, Pose2D> optimized_poses;
    Eigen::MatrixXd information_matrix;
};

} // namespace scan_slam

#endif // SCAN_SLAM_SLAM_TYPES_HPP