#ifndef SCAN_SLAM_SCAN_MATCHER_INTERFACE_HPP
#define SCAN_SLAM_SCAN_MATCHER_INTERFACE_HPP

#include "scan_slam/slam_types.hpp"

namespace scan_slam {
class ScanMatcherInterface {
public:
    virtual ~ScanMatcherInterface() = default;
    virtual MatchResult matchScans(const ScanPoints& old_scan, const Pose2D& old_pose, const ScanPoints& new_scan, const Pose2D& pose_initial_guess, bool loop_closure) = 0;
    virtual Eigen::Matrix3d computeMatcherCovariance(const ScanPoints& query_scan, const Pose2D& initial_guess, const Pose2D& new_pose, bool loop_closure) = 0;
    virtual void setPoseCovariance(const Eigen::Matrix3d& matcher_covariance) = 0;
    virtual Eigen::Matrix3d getPoseCovariance() const = 0;
    int getLastIterations() const { return last_iterations_; }
protected:
    int last_iterations_ = 0;
};
} // namespace scan_slam

#endif // SCAN_SLAM_SCAN_MATCHER_INTERFACE_HPP