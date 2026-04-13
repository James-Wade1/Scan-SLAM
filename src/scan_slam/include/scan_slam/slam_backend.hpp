#ifndef SLAM_BACKEND_HPP
#define SLAM_BACKEND_HPP

#include "scan_slam/slam_types.hpp"

namespace scan_slam {

class SlamBackend {
public:
    SlamBackend(float threshold, bool use_sparse_solver);
    void optimizePoseGraph(PoseGraph& pose_graph, OptimizationResult& result);
    Eigen::MatrixXd calculateH(PoseGraph& pose_graph) const;
    int getLastIterations() const { return last_iterations_; }

private:
    void optimizeSparsePoseGraph(PoseGraph& pose_graph, OptimizationResult& result);
    void optimizeDensePoseGraph(PoseGraph& pose_graph, OptimizationResult& result);
    bool isConverged(const Eigen::VectorXd& delta_poses, double threshold) const;
    Eigen::Vector3d calcError(const Pose2D& x_i, const Pose2D& x_j, const Pose2D& iTj) const;
    Eigen::Matrix3d constructT(const Pose2D& pose) const;
    Eigen::Matrix3d inverseT(const Eigen::Matrix3d& T) const;
    Eigen::Vector3d deconstructT(const Eigen::Matrix3d& T) const;
    Eigen::Matrix3d computeAij(const Pose2D& x_i, const Pose2D& x_j, const Pose2D& z) const;
    Eigen::Matrix3d computeBij(const Pose2D& x_i, const Pose2D& z) const;
    Eigen::Matrix2d computeR(const Pose2D& z) const;
    Eigen::Matrix2d computeDelRTDelTheta(const Pose2D& z) const;
    double normalizeAngle(double a) const;

    float threshold_;
    int last_iterations_ = 0;
    bool use_sparse_solver_ = true;
};


} // namespace scan_slam

#endif // SLAM_BACKEND_HPP