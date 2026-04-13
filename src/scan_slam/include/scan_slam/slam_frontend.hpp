#ifndef SCAN_SLAM_SLAM_FRONTEND_HPP
#define SCAN_SLAM_SLAM_FRONTEND_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "scan_slam/slam_types.hpp"
#include "scan_slam/scan_matcher_interface.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "scan_slam/slam_backend.hpp"
#include "scan_slam_msgs/msg/pose_graph.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"

namespace scan_slam {

class FrontEndNode : public rclcpp::Node {
    public:
        FrontEndNode();
        void processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    private:
        MatchResult estimateDeltaPose(const KeyFrame& last_frame, const ScanPoints& new_scan_points, const Pose2D& starting_pose, bool loop_closure = false);
        bool shouldAddKeyFrame(const Pose2D& new_pose, const Pose2D& last_pose);
        ScanPoints convertScanToPoints(const sensor_msgs::msg::LaserScan& scan);
        Pose2D composePose(const Pose2D& base, const Pose2D& delta);
        bool shouldAddLoopClosure(const Pose2D& new_pose, const KeyFrame& newest_frame);
        OptimizationResult optimizePoseGraph(PoseGraph& pose_graph);
        void publishPoses(const PoseGraph& pose_graph);
        void processOdom(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
        Pose2D estimatePoseFromOdom(const Pose2D& current_pose, const nav_msgs::msg::Odometry& prev_odom, const nav_msgs::msg::Odometry& latest_odom);
        geometry_msgs::msg::Pose toPoseMsg(const Pose2D& pose) const;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
        rclcpp::Publisher<scan_slam_msgs::msg::PoseGraph>::SharedPtr pose_graph_publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_estimate_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr frontend_timing_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr backend_timing_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr loop_closure_timing_publisher_;
        nav_msgs::msg::Odometry latest_odom_;
        nav_msgs::msg::Odometry last_odom_;
        PoseGraph pose_graph_;
        Pose2D current_pose_;
        std::unique_ptr<ScanMatcherInterface> scan_matcher_;
        std::unique_ptr<SlamBackend> slam_backend_;
        OptimizationResult optimization_result_;

        int loop_closure_candidate_window_ = 10;
        bool odom_initialized_ = false;
        float add_keyframe_xy_threshold_ = 0.1;
        float add_keyframe_theta_threshold_ = 0.1;
        float loop_closure_score_threshold_ = -0.5;
        float mahalanobis_threshold_ = 5.991; // 95% confidence for chi-squared with 2 DOF
};
} // namespace scan_slam

#endif // SCAN_SLAM_SLAM_FRONTEND_HPP