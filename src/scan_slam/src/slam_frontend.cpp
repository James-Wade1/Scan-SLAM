#include "scan_slam/slam_frontend.hpp"
#include "scan_slam/scan_matcher_interface.hpp"
#include "scan_slam/correlative_scan_matcher.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/utils.h>
#include "scan_slam_msgs/msg/constraint.hpp"
#include "scan_slam_msgs/msg/key_frame.hpp"

#include <iostream>
#include <Eigen/Eigenvalues>
#include <chrono>

namespace scan_slam {

FrontEndNode::FrontEndNode() : Node{"slam_frontend_node"}, pose_graph_{}, current_pose_{0.0, 0.0, 0.0} {

    this->declare_parameter<float>("add_keyframe_xy_threshold", 0.5);
    this->declare_parameter<float>("add_keyframe_theta_threshold", 0.2);
    this->declare_parameter<float>("search_window_xy", 1.0);
    this->declare_parameter<float>("search_window_theta", 1.5708);
    this->declare_parameter<float>("resolution_xy", 0.1);
    this->declare_parameter<float>("resolution_theta", 0.05);
    this->declare_parameter<float>("sigma_xy", 0.1);
    this->declare_parameter<int>("correlative_coarse_reduction_factor", 2);
    this->declare_parameter<bool>("use_approx_matcher_covariance", true);
    this->declare_parameter<std::vector<double>>(
        "motion_covariance",
        std::vector<double>{0.1, 0.0, 0.0,
                            0.0, 0.1, 0.0,
                            0.0, 0.0, 0.05});
    this->declare_parameter<float>("score_temperature", 0.1);
    this->declare_parameter<std::string>("scan_topic", "scan");
    this->declare_parameter<std::string>("pose_graph_topic", "pose_graph");
    this->declare_parameter<float>("backend_threshold", 1e-4);
    this->declare_parameter<float>("loop_closure_score_threshold", -0.5);
    this->declare_parameter<float>("mahalanobis_threshold", 5.991); // 95% confidence for chi-squared with 2 DOF
    this->declare_parameter<float>("loop_closure_search_radius", 2.0);
    this->declare_parameter<float>("loop_closure_search_theta", 1.5708);
    this->declare_parameter<int>("loop_closure_candidate_window", 10);
    this->declare_parameter<bool>("use_sparse_solver", true);

    add_keyframe_xy_threshold_ = this->get_parameter("add_keyframe_xy_threshold").as_double();
    add_keyframe_theta_threshold_ = this->get_parameter("add_keyframe_theta_threshold").as_double();
    loop_closure_score_threshold_ = this->get_parameter("loop_closure_score_threshold").as_double();
    mahalanobis_threshold_ = this->get_parameter("mahalanobis_threshold").as_double();
    loop_closure_candidate_window_ = this->get_parameter("loop_closure_candidate_window").as_int();

    pose_graph_.clear();

    pose_graph_publisher_ = this->create_publisher<scan_slam_msgs::msg::PoseGraph>(
        this->get_parameter("pose_graph_topic").as_string(), 1
    );
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        this->get_parameter("scan_topic").as_string(), 1, std::bind(&FrontEndNode::processScan, this, std::placeholders::_1)
    );
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&FrontEndNode::processOdom, this, std::placeholders::_1));
    pose_estimate_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose_estimate", 1);
    frontend_timing_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/frontend_timing", 10);
    backend_timing_publisher_  = this->create_publisher<std_msgs::msg::Float64MultiArray>("/backend_timing",  10);
    loop_closure_timing_publisher_  = this->create_publisher<std_msgs::msg::Float64MultiArray>("/loop_closure_timing",  10);
    std::vector<double> motion_cov_vec = this->get_parameter("motion_covariance").as_double_array();
    if (motion_cov_vec.size() != 9) {
        RCLCPP_ERROR(this->get_logger(), "Motion covariance parameter must have 9 elements. Using default identity covariance.");
        motion_cov_vec = {0.1, 0.0, 0.0,
                          0.0, 0.1, 0.0,
                          0.0, 0.0, 0.05};
    }
    Eigen::Matrix3d motion_covariance = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(motion_cov_vec.data());

    CorrelativeScanMatcherOptions matcher_options(
        this->get_parameter("search_window_xy").as_double(),
        this->get_parameter("search_window_theta").as_double(),
        this->get_parameter("resolution_xy").as_double(),
        this->get_parameter("resolution_theta").as_double(),
        this->get_parameter("sigma_xy").as_double(),
        this->get_parameter("correlative_coarse_reduction_factor").as_int(),
        motion_covariance,
        this->get_parameter("use_approx_matcher_covariance").as_bool(),
        this->get_parameter("score_temperature").as_double(),
        this->get_parameter("loop_closure_search_radius").as_double(),
        this->get_parameter("loop_closure_search_theta").as_double()
    );

    scan_matcher_ = std::make_unique<CorrelativeScanMatcher>(matcher_options);
    slam_backend_ = std::make_unique<SlamBackend>(this->get_parameter("backend_threshold").as_double(), this->get_parameter("use_sparse_solver").as_bool());

    RCLCPP_INFO(this->get_logger(), "SLAM Frontend node has been started.");
}

void FrontEndNode::processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (!scan) {
        RCLCPP_WARN(this->get_logger(), "Received null scan pointer.");
        return;
    }
    if (!odom_initialized_) {
        RCLCPP_WARN(this->get_logger(), "Odometry not initialized yet. Skipping scan processing.");
        return;
    }
    ScanPoints new_scan_points = convertScanToPoints(*scan);

    if (pose_graph_.isEmpty()) {
        pose_graph_.addKeyFrame(KeyFrame{0, scan->header.stamp, current_pose_, new_scan_points, *scan});
        last_odom_ = latest_odom_;
        RCLCPP_INFO(this->get_logger(), "Added new keyframe with ID %d at time %f", pose_graph_.keyframes.back().id, scan->header.stamp.sec + scan->header.stamp.nanosec * 1e-9);
        RCLCPP_INFO(this->get_logger(), "Current pose: (%.2f, %.2f, %.2f)", current_pose_(0), current_pose_(1), current_pose_(2));
        RCLCPP_INFO(this->get_logger(), "Last odom: (%.2f, %.2f, %.2f)", last_odom_.pose.pose.position.x, last_odom_.pose.pose.position.y, tf2::getYaw(tf2::Quaternion(last_odom_.pose.pose.orientation.x, last_odom_.pose.pose.orientation.y, last_odom_.pose.pose.orientation.z, last_odom_.pose.pose.orientation.w)));
        return;
    }

    const KeyFrame& last_frame = pose_graph_.keyframes.back();
    Pose2D starting_pose = estimatePoseFromOdom(current_pose_, last_odom_, latest_odom_);
    current_pose_ = starting_pose;

    if (shouldAddKeyFrame(current_pose_, last_frame.pose)) {
        MatchResult match_result = estimateDeltaPose(last_frame, new_scan_points, starting_pose);
        int new_id = pose_graph_.keyframes.size();
        const int last_id = last_frame.id;
        pose_graph_.addKeyFrame(KeyFrame{new_id, scan->header.stamp, current_pose_, new_scan_points, *scan});
        pose_graph_.addConstraint(Constraint{last_id, new_id, match_result.delta_pose, match_result.information_matrix});
        Eigen::LLT<Eigen::MatrixXd> llt(match_result.information_matrix);
        if (shouldAddLoopClosure(current_pose_, pose_graph_.keyframes.back())) {
            // Add loop closure constraint
            optimization_result_ = optimizePoseGraph(pose_graph_);
            // Update current pose based on optimized results
            if (optimization_result_.optimized_poses.find(pose_graph_.keyframes.back().id) != optimization_result_.optimized_poses.end()) {
                Pose2D delta_pose = optimization_result_.optimized_poses[pose_graph_.keyframes.back().id] - current_pose_;

                RCLCPP_INFO(this->get_logger(), "Loop closure detected. Change in pose after optimization: (%.2f, %.2f, %.2f)", delta_pose(0), delta_pose(1), delta_pose(2));
                for (const auto& pose : pose_graph_.keyframes) {
                    if (optimization_result_.optimized_poses.find(pose.id) != optimization_result_.optimized_poses.end()) {
                        Pose2D optimized_pose = optimization_result_.optimized_poses[pose.id];
                        pose_graph_.keyframes[pose.id].pose = optimized_pose;
                    }
                }
                current_pose_ = pose_graph_.keyframes.back().pose;
                Eigen::MatrixXd H_updated = slam_backend_->calculateH(pose_graph_);
                int last_idx = pose_graph_.keyframes.size() - 1;
                Eigen::Matrix3d Sigma_updated =
                H_updated.block<3,3>(last_idx * 3, last_idx * 3).inverse();
                scan_matcher_->setPoseCovariance(Sigma_updated);
            }
        }
        publishPoses(pose_graph_);
    }
    last_odom_ = latest_odom_;
}

OptimizationResult FrontEndNode::optimizePoseGraph(PoseGraph& pose_graph) {
    auto start = std::chrono::high_resolution_clock::now();

    slam_backend_->optimizePoseGraph(pose_graph, optimization_result_);

    auto end = std::chrono::high_resolution_clock::now();
    double wall_time_ms = std::chrono::duration<double, std::milli>(end - start).count();

    std_msgs::msg::Float64MultiArray timing_msg;
    timing_msg.data = {wall_time_ms, (double)slam_backend_->getLastIterations(), (double)pose_graph_.keyframes.size()};
    backend_timing_publisher_->publish(timing_msg);

    return optimization_result_;
}

MatchResult FrontEndNode::estimateDeltaPose(const KeyFrame& last_frame, const ScanPoints& new_scan_points, const Pose2D& starting_pose, bool loop_closure) {
    auto start = std::chrono::high_resolution_clock::now();

    MatchResult result = scan_matcher_->matchScans(
        last_frame.scan_points, last_frame.pose,
        new_scan_points, starting_pose, loop_closure);

    auto end = std::chrono::high_resolution_clock::now();
    double wall_time_ms = std::chrono::duration<double, std::milli>(end - start).count();

    std_msgs::msg::Float64MultiArray timing_msg;
    timing_msg.data = {wall_time_ms, (double)scan_matcher_->getLastIterations()};
    if (loop_closure) {
        loop_closure_timing_publisher_->publish(timing_msg);
    } else {
        frontend_timing_publisher_->publish(timing_msg);
    }

    return result;
}

bool FrontEndNode::shouldAddKeyFrame(const Pose2D& new_pose, const Pose2D& last_pose) {
    if (pose_graph_.keyframes.empty()) {
        return true;
    }
    double distance = (new_pose.head<2>() - last_pose.head<2>()).norm();
    return distance > add_keyframe_xy_threshold_ || std::abs(new_pose(2) - last_pose(2)) > add_keyframe_theta_threshold_;
}

ScanPoints FrontEndNode::convertScanToPoints(const sensor_msgs::msg::LaserScan& scan) {
    std::vector<Eigen::Vector2d> points;
    points.reserve(scan.ranges.size());
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float range = scan.ranges[i];
        if (std::isfinite(range) && range >= scan.range_min && range <= scan.range_max) {
            float angle = scan.angle_min + i * scan.angle_increment;
            points.emplace_back(range * std::cos(angle), range * std::sin(angle));
        }
    }
    return ScanPoints(points);
}

Pose2D FrontEndNode::composePose(const Pose2D& base, const Pose2D& delta) {
    double x = base(0) + delta(0) * std::cos(base(2)) - delta(1) * std::sin(base(2));
    double y = base(1) + delta(0) * std::sin(base(2)) + delta(1) * std::cos(base(2));
    double theta = std::atan2(std::sin(base(2) + delta(2)), std::cos(base(2) + delta(2)));
    return Pose2D(x, y, theta);
}

bool FrontEndNode::shouldAddLoopClosure(const Pose2D& new_pose, const KeyFrame& newest_frame) {
    // Implementation for loop closure detection
    if (pose_graph_.keyframes.size() <= loop_closure_candidate_window_) {
        return false; // Not enough keyframes for loop closure
    }
    size_t max_check = pose_graph_.keyframes.size() - loop_closure_candidate_window_;
    bool loop_closure_detected = false;
    Eigen::MatrixXd H = slam_backend_->calculateH(pose_graph_);
    Eigen::Matrix2d Sigma_new =
            H.block<2,2>((pose_graph_.keyframes.size() - 1) * 3,
                        (pose_graph_.keyframes.size() - 1) * 3).inverse();
    for (size_t i = 0; i < max_check; ++i) {
        const KeyFrame& candidate_frame = pose_graph_.keyframes[i];
        Eigen::Vector2d delta_pos = new_pose.head<2>() - candidate_frame.pose.head<2>();
        Eigen::Matrix2d Sigma_i =
            H.block<2,2>(i * 3, i * 3).inverse();

        Eigen::Matrix2d Sigma_delta = Sigma_i + Sigma_new;

        double mahalanobis_distance = delta_pos.transpose() * Sigma_delta.inverse() * delta_pos;

        if (mahalanobis_distance < mahalanobis_threshold_) {
            MatchResult loop_closure_match = estimateDeltaPose(candidate_frame, newest_frame.scan_points, newest_frame.pose, true);
            RCLCPP_INFO(this->get_logger(), "Loop closure candidate found between keyframe %d and new keyframe %d with score %.2f and Mahalanobis distance %.2f", candidate_frame.id, newest_frame.id, loop_closure_match.score, mahalanobis_distance);
            if (loop_closure_match.score > loop_closure_score_threshold_) {
                Eigen::Matrix3d matcher_covariance = scan_matcher_->computeMatcherCovariance(newest_frame.scan_points, newest_frame.pose, loop_closure_match.new_pose, true);
                scan_matcher_->setPoseCovariance(matcher_covariance);
                pose_graph_.addConstraint(Constraint{candidate_frame.id, newest_frame.id, loop_closure_match.delta_pose, matcher_covariance.inverse(), true});
                loop_closure_detected = true;
                break; // Only add one loop closure per new keyframe for simplicity
            }

        }
    }
    return loop_closure_detected;
}

void FrontEndNode::publishPoses(const PoseGraph& pose_graph) {
    scan_slam_msgs::msg::PoseGraph pose_graph_msg;
    for (const auto& keyframe : pose_graph.keyframes) {
        scan_slam_msgs::msg::KeyFrame keyframe_msg;
        keyframe_msg.id = keyframe.id;
        keyframe_msg.pose = toPoseMsg(keyframe.pose);
        keyframe_msg.scan = keyframe.scan_msg;
        keyframe_msg.stamp = keyframe.timestamp;
        pose_graph_msg.keyframes.push_back(keyframe_msg);
    }

    for (const auto& constraint : pose_graph.constraints) {
        scan_slam_msgs::msg::Constraint constraint_msg;
        constraint_msg.from_id = constraint.from_id;
        constraint_msg.to_id = constraint.to_id;
        constraint_msg.relative_pose = toPoseMsg(constraint.relative_pose);
        constraint_msg.is_loop_closure = constraint.is_loop_closure;
        pose_graph_msg.constraints.push_back(constraint_msg);
    }

    if (!pose_graph_msg.keyframes.empty()) {
        pose_graph_publisher_->publish(pose_graph_msg);
    }
    geometry_msgs::msg::PoseWithCovarianceStamped pose_estimate_msg;
    pose_estimate_msg.header.stamp = pose_graph.keyframes.back().timestamp;
    pose_estimate_msg.pose.pose = toPoseMsg(pose_graph.keyframes.back().pose);
    Eigen::Matrix3d current_covariance = scan_matcher_->getPoseCovariance();
    // x-x, x-y, x-yaw
    pose_estimate_msg.pose.covariance[0]  = current_covariance(0, 0);
    pose_estimate_msg.pose.covariance[1]  = current_covariance(0, 1);
    pose_estimate_msg.pose.covariance[5]  = current_covariance(0, 2);

    // y-x, y-y, y-yaw
    pose_estimate_msg.pose.covariance[6]  = current_covariance(1, 0);
    pose_estimate_msg.pose.covariance[7]  = current_covariance(1, 1);
    pose_estimate_msg.pose.covariance[11] = current_covariance(1, 2);

    // yaw-x, yaw-y, yaw-yaw
    pose_estimate_msg.pose.covariance[30] = current_covariance(2, 0);
    pose_estimate_msg.pose.covariance[31] = current_covariance(2, 1);
    pose_estimate_msg.pose.covariance[35] = current_covariance(2, 2);
    pose_estimate_publisher_->publish(pose_estimate_msg);
}

geometry_msgs::msg::Pose FrontEndNode::toPoseMsg(const Pose2D& pose) const {
    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = pose(0);
    pose_msg.position.y = pose(1);
    pose_msg.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, pose(2));
    pose_msg.orientation.x = q.x();
    pose_msg.orientation.y = q.y();
    pose_msg.orientation.z = q.z();
    pose_msg.orientation.w = q.w();
    return pose_msg;
}

void FrontEndNode::processOdom(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    if (!odom_msg) {
        RCLCPP_WARN(this->get_logger(), "Received null odometry pointer.");
        return;
    }
    if (!odom_initialized_) {
        last_odom_ = *odom_msg;
        odom_initialized_ = true;
    }
    latest_odom_ = *odom_msg;
}

Pose2D FrontEndNode::estimatePoseFromOdom(const Pose2D& current_pose, const nav_msgs::msg::Odometry& prev_odom, const nav_msgs::msg::Odometry& latest_odom) {
    double delta_x_odom = latest_odom.pose.pose.position.x - prev_odom.pose.pose.position.x;
    double delta_y_odom = latest_odom.pose.pose.position.y - prev_odom.pose.pose.position.y;

    tf2::Quaternion q_prev(prev_odom.pose.pose.orientation.x, prev_odom.pose.pose.orientation.y,
                           prev_odom.pose.pose.orientation.z, prev_odom.pose.pose.orientation.w);
    tf2::Quaternion q_latest(latest_odom.pose.pose.orientation.x, latest_odom.pose.pose.orientation.y,
                             latest_odom.pose.pose.orientation.z, latest_odom.pose.pose.orientation.w);
    double yaw_prev   = tf2::getYaw(q_prev);
    double yaw_latest = tf2::getYaw(q_latest);
    double delta_theta = std::atan2(std::sin(yaw_latest - yaw_prev), std::cos(yaw_latest - yaw_prev));

    double local_dx =  delta_x_odom * std::cos(yaw_prev) + delta_y_odom * std::sin(yaw_prev);
    double local_dy = -delta_x_odom * std::sin(yaw_prev) + delta_y_odom * std::cos(yaw_prev);

    return composePose(current_pose, Pose2D(local_dx, local_dy, delta_theta));
}

} // namespace scan_slam
