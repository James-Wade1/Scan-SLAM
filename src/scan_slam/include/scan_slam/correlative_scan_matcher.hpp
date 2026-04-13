#ifndef CORRELATIVE_SCAN_MATCHER_HPP
#define CORRELATIVE_SCAN_MATCHER_HPP

#include "scan_slam/scan_matcher_interface.hpp"
#include "scan_slam/correlative_matcher_lookup_table.hpp"

namespace scan_slam {

struct CorrelativeScanMatcherOptions {
    float search_window_xy;
    float search_window_theta;
    float resolution_xy;
    float resolution_theta;
    float sigma_xy;
    int coarse_reduction_factor;
    Eigen::Matrix3d motion_covariance;
    bool use_approx_matcher_covariance;
    float score_temperature;
    float loop_closure_search_radius;
    float loop_closure_search_theta;

    CorrelativeScanMatcherOptions(float search_window_xy, float search_window_theta, float resolution_xy, float resolution_theta, float sigma_xy, int coarse_reduction_factor, Eigen::Matrix3d motion_covariance, bool use_approx_matcher_covariance, float score_temperature, float loop_closure_search_radius, float loop_closure_search_theta) :
        search_window_xy(search_window_xy),
        search_window_theta(search_window_theta),
        resolution_xy(resolution_xy),
        resolution_theta(resolution_theta),
        sigma_xy(sigma_xy),
        coarse_reduction_factor(coarse_reduction_factor),
        motion_covariance(motion_covariance),
        use_approx_matcher_covariance(use_approx_matcher_covariance),
        score_temperature(score_temperature),
        loop_closure_search_radius(loop_closure_search_radius),
        loop_closure_search_theta(loop_closure_search_theta) {}
};

class CorrelativeScanMatcher : public ScanMatcherInterface {
public:
    CorrelativeScanMatcher(CorrelativeScanMatcherOptions options);
    ~CorrelativeScanMatcher() = default;
    MatchResult matchScans(const ScanPoints& old_scan, const Pose2D& old_pose, const ScanPoints& new_scan, const Pose2D& pose_initial_guess, bool loop_closure) override;
    Eigen::Matrix3d computeMatcherCovariance(const ScanPoints& query_scan, const Pose2D& initial_guess, const Pose2D& new_pose, bool loop_closure) override;
    void setPoseCovariance(const Eigen::Matrix3d& matcher_covariance) override;
    Eigen::Matrix3d getPoseCovariance() const;
private:
    ScanPoints rotateScanToInitialGuessAndThetaOffset(const ScanPoints& query_scan, int dtheta, const Pose2D& initial_guess);
    Eigen::Matrix3d computeExactMatcherCovariance(const ScanPoints& query_scan, const Pose2D& initial_guess, bool loop_closure);
    Eigen::Matrix3d computeApproxMatcherCovariance(const ScanPoints& query_scan, const Pose2D& new_pose);
    double evaluatePosteriorAt(const ScanPoints& query_scan, int dx, int dy, int dtheta);
    double normalizeAngle(double angle) const;
    Pose2D composePose(const Pose2D& base, const Pose2D& delta);
    Pose2D worldDeltaToLocal(const Pose2D& base_pose, const Pose2D& delta_world);
    Eigen::Matrix3d propagatePoseCovariance(const Eigen::Matrix3d& pose_covariance, const Pose2D& delta_pose, const Pose2D& prev_pose);
    Pose2D computeRelativePose(const Pose2D& from, const Pose2D& to);

    float search_window_xy_;
    float search_window_theta_;
    float resolution_xy_;
    float resolution_theta_;
    float sigma_xy_;
    CorrelativeMatcherLookupTable lookupTable_;
    Eigen::Matrix3d motion_covariance_;
    Eigen::Matrix3d motion_information_;
    Eigen::Matrix3d pose_covariance_;
    Eigen::Matrix3d pose_information_;
    bool use_approx_matcher_covariance_;
    float loop_closure_search_radius_;
    float loop_closure_search_theta_;
    int coarse_reduction_factor_;
};

} // namespace scan_slam

#endif // CORRELATIVE_SCAN_MATCHER_HPP
