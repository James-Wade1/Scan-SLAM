#include "scan_slam/slam_backend.hpp"
#include <Eigen/Sparse>

#include <iostream>

namespace scan_slam {

SlamBackend::SlamBackend(float threshold, bool use_sparse_solver) : threshold_(threshold), use_sparse_solver_(use_sparse_solver) {};

void SlamBackend::optimizePoseGraph(PoseGraph& pose_graph, OptimizationResult& result) {
    if (use_sparse_solver_) {
        optimizeSparsePoseGraph(pose_graph, result);
    } else {
        optimizeDensePoseGraph(pose_graph, result);
    }
}

void SlamBackend::optimizeSparsePoseGraph(PoseGraph& pose_graph, OptimizationResult& result) {
    const std::vector<KeyFrame>& frames = pose_graph.keyframes;
    const std::vector<Constraint>& constraints = pose_graph.constraints;

    if (frames.empty()) {
        result.optimized_poses.clear();
        result.information_matrix.resize(0, 0);
        return;
    }

    std::unordered_map<int, size_t> id_to_index;
    for (size_t k = 0; k < frames.size(); ++k) {
        id_to_index[frames[k].id] = k;
    }

    Eigen::VectorXd optimized_poses(frames.size() * 3);
    for (size_t i = 0; i < frames.size(); i++) {
        optimized_poses.segment<3>(i * 3) = frames[i].pose;
    }
    Eigen::VectorXd delta_poses = Eigen::VectorXd::Zero(optimized_poses.size());
    Eigen::SparseMatrix<double> H(optimized_poses.size(), optimized_poses.size());
    std::vector<Eigen::Triplet<double>> tripletList;
    tripletList.reserve(constraints.size() * 36 + 3); // Each constraint contributes up to 36 non-zero entries
    Eigen::VectorXd b = Eigen::VectorXd::Zero(optimized_poses.size());
    H.setFromTriplets(tripletList.begin(), tripletList.end());
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;

    int iteration = 0;
    do {
        tripletList.clear();
        b.setZero();
        for (const auto& z : constraints) {
            int id_i = id_to_index.at(z.from_id);
            int id_j = id_to_index.at(z.to_id);
            Eigen::Vector3d x_i = optimized_poses.segment<3>(id_i * 3);
            Eigen::Vector3d x_j = optimized_poses.segment<3>(id_j * 3);
            Eigen::Vector3d error = calcError(x_i, x_j, z.relative_pose);
            Eigen::Matrix3d Omega = z.information_matrix;
            Eigen::Matrix3d Aij = computeAij(x_i, x_j, z.relative_pose);
            Eigen::Matrix3d Bij = computeBij(x_i, z.relative_pose);
            Eigen::Matrix3d Mi = Eigen::Matrix3d::Identity();
            Eigen::Matrix3d Mj = Eigen::Matrix3d::Identity();
            Aij = Aij * Mi;
            Bij = Bij * Mj;
            auto AOmegaA = Aij.transpose() * Omega * Aij;
            auto AOmegaB = Aij.transpose() * Omega * Bij;
            auto BOmegaA = Bij.transpose() * Omega * Aij;
            auto BOmegaB = Bij.transpose() * Omega * Bij;
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    tripletList.emplace_back(id_i * 3 + r, id_i * 3 + c, AOmegaA(r, c));
                    tripletList.emplace_back(id_i * 3 + r, id_j * 3 + c, AOmegaB(r, c));
                    tripletList.emplace_back(id_j * 3 + r, id_i * 3 + c, BOmegaA(r, c));
                    tripletList.emplace_back(id_j * 3 + r, id_j * 3 + c, BOmegaB(r, c));
                }
            }
            b.segment<3>(id_i * 3) += Aij.transpose() * Omega * error;
            b.segment<3>(id_j * 3) += Bij.transpose() * Omega * error;
        }

        tripletList.emplace_back(0, 0, 1e6);
        tripletList.emplace_back(1, 1, 1e6);
        tripletList.emplace_back(2, 2, 1e6);
        H.setFromTriplets(tripletList.begin(), tripletList.end());
        solver.analyzePattern(H);

        solver.factorize(H);
        if (solver.info() != Eigen::Success) {
            std::cerr << "Decomposition failed!" << std::endl;
            break;
        }
        delta_poses = solver.solve(-b);
        if (solver.info() != Eigen::Success) {
            std::cerr << "Solving failed!" << std::endl;
            break;
        }
        optimized_poses += delta_poses;

        for (size_t i = 0; i < frames.size(); i++) {
            optimized_poses(i * 3 + 2) = normalizeAngle(optimized_poses(i * 3 + 2));
        }
        iteration++;
    } while(!isConverged(delta_poses, threshold_));

    last_iterations_ = iteration;
    result.optimized_poses.clear();
    for (size_t i = 0; i < frames.size(); i++) {
        result.optimized_poses[frames[i].id] = optimized_poses.segment<3>(i * 3);
    }
    result.information_matrix = Eigen::MatrixXd(H);
}

void SlamBackend::optimizeDensePoseGraph(PoseGraph& pose_graph, OptimizationResult& result) {
    const std::vector<KeyFrame>& frames = pose_graph.keyframes;
    const std::vector<Constraint>& constraints = pose_graph.constraints;

    if (frames.empty()) {
        result.optimized_poses.clear();
        result.information_matrix.resize(0, 0);
        return;
    }

    std::unordered_map<int, size_t> id_to_index;
    for (size_t k = 0; k < frames.size(); ++k) {
        id_to_index[frames[k].id] = k;
    }

    Eigen::VectorXd optimized_poses(frames.size() * 3);
    for (size_t i = 0; i < frames.size(); i++) {
        optimized_poses.segment<3>(i * 3) = frames[i].pose;
    }

    Eigen::VectorXd delta_poses = Eigen::VectorXd::Zero(optimized_poses.size());
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(optimized_poses.size(), optimized_poses.size());
    Eigen::VectorXd b = Eigen::VectorXd::Zero(optimized_poses.size());

    int iteration = 0;
    do {
        H.setZero();
        b.setZero();

        for (const auto& z : constraints) {
            int id_i = id_to_index.at(z.from_id);
            int id_j = id_to_index.at(z.to_id);
            Eigen::Vector3d x_i = optimized_poses.segment<3>(id_i * 3);
            Eigen::Vector3d x_j = optimized_poses.segment<3>(id_j * 3);
            Eigen::Vector3d error = calcError(x_i, x_j, z.relative_pose);
            Eigen::Matrix3d Omega = z.information_matrix;
            Eigen::Matrix3d Aij = computeAij(x_i, x_j, z.relative_pose);
            Eigen::Matrix3d Bij = computeBij(x_i, z.relative_pose);
            Eigen::Matrix3d Mi = Eigen::Matrix3d::Identity();
            Eigen::Matrix3d Mj = Eigen::Matrix3d::Identity();
            Aij = Aij * Mi;
            Bij = Bij * Mj;

            auto AOmegaA = Aij.transpose() * Omega * Aij;
            auto AOmegaB = Aij.transpose() * Omega * Bij;
            auto BOmegaA = Bij.transpose() * Omega * Aij;
            auto BOmegaB = Bij.transpose() * Omega * Bij;

            H.block<3, 3>(id_i * 3, id_i * 3) += AOmegaA;
            H.block<3, 3>(id_i * 3, id_j * 3) += AOmegaB;
            H.block<3, 3>(id_j * 3, id_i * 3) += BOmegaA;
            H.block<3, 3>(id_j * 3, id_j * 3) += BOmegaB;

            b.segment<3>(id_i * 3) += Aij.transpose() * Omega * error;
            b.segment<3>(id_j * 3) += Bij.transpose() * Omega * error;
        }

        H.block<3, 3>(0, 0) += Eigen::Matrix3d::Identity() * 1e6;

        Eigen::LDLT<Eigen::MatrixXd> solver;
        solver.compute(H);
        if (solver.info() != Eigen::Success) {
            std::cerr << "Decomposition failed!" << std::endl;
            break;
        }
        delta_poses = solver.solve(-b);
        if (solver.info() != Eigen::Success) {
            std::cerr << "Solving failed!" << std::endl;
            break;
        }

        optimized_poses += delta_poses;

        for (size_t i = 0; i < frames.size(); i++) {
            optimized_poses(i * 3 + 2) = normalizeAngle(optimized_poses(i * 3 + 2));
        }
        iteration++;
    } while (!isConverged(delta_poses, threshold_));

    last_iterations_ = iteration;
    result.optimized_poses.clear();
    for (size_t i = 0; i < frames.size(); i++) {
        result.optimized_poses[frames[i].id] = optimized_poses.segment<3>(i * 3);
    }
    result.information_matrix = H;
}

Eigen::MatrixXd SlamBackend::calculateH(PoseGraph& pose_graph) const {
    std::vector<KeyFrame>& frames = pose_graph.keyframes;
    std::vector<Constraint>& constraints = pose_graph.constraints;

    std::unordered_map<int, size_t> id_to_index;
    for (size_t k = 0; k < frames.size(); ++k) {
        id_to_index[frames[k].id] = k;
    }

    Eigen::VectorXd optimized_poses(frames.size() * 3);
    for (size_t i = 0; i < frames.size(); i++) {
        optimized_poses.segment<3>(i * 3) = frames[i].pose;
    }

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(optimized_poses.size(), optimized_poses.size());
    H.setZero();
    for (const auto& z : constraints) {
        int id_i = id_to_index.at(z.from_id);
        int id_j = id_to_index.at(z.to_id);
        Eigen::Vector3d x_i = optimized_poses.segment<3>(id_i * 3);
        Eigen::Vector3d x_j = optimized_poses.segment<3>(id_j * 3);
        Eigen::Vector3d error = calcError(x_i, x_j, z.relative_pose);
        Eigen::Matrix3d Omega = z.information_matrix;
        Eigen::Matrix3d Aij = computeAij(x_i, x_j, z.relative_pose);
        Eigen::Matrix3d Bij = computeBij(x_i, z.relative_pose);
        Eigen::Matrix3d Mi = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d Mj = Eigen::Matrix3d::Identity();
        Aij = Aij * Mi;
        Bij = Bij * Mj;
        H.block<3, 3>(id_i * 3, id_i * 3) += Aij.transpose() * Omega * Aij;
        H.block<3, 3>(id_i * 3, id_j * 3) += Aij.transpose() * Omega * Bij;
        H.block<3, 3>(id_j * 3, id_i * 3) += Bij.transpose() * Omega * Aij;
        H.block<3, 3>(id_j * 3, id_j * 3) += Bij.transpose() * Omega * Bij;
    }
    return H;
}

bool SlamBackend::isConverged(const Eigen::VectorXd& delta_poses, double threshold) const {
    return delta_poses.lpNorm<Eigen::Infinity>() < threshold;
}

Eigen::Vector3d SlamBackend::calcError(const Pose2D& x_i, const Pose2D& x_j, const Pose2D& z) const {
    Eigen::Matrix2d Ri = computeR(x_i);
    Eigen::Matrix2d Rz = computeR(z);

    Eigen::Vector2d ti = x_i.head<2>();
    Eigen::Vector2d tj = x_j.head<2>();
    Eigen::Vector2d tz = z.head<2>();

    Eigen::Vector2d e_translation = Rz.transpose() * (Ri.transpose() * (tj - ti) - tz);
    double e_theta = normalizeAngle((x_j(2) - x_i(2)) - z(2));

    Eigen::Vector3d e;
    e << e_translation, e_theta;
    return e;
}

Eigen::Matrix3d SlamBackend::constructT(const Pose2D& pose) const {
    double c = std::cos(pose(2));
    double s = std::sin(pose(2));
    Eigen::Matrix3d T;
    T << c, -s, pose(0),
         s,  c, pose(1),
         0,  0, 1;
    return T;
}

Eigen::Vector3d SlamBackend::deconstructT(const Eigen::Matrix3d& T) const {
    double x = T(0, 2);
    double y = T(1, 2);
    double theta = std::atan2(T(1, 0), T(0, 0));
    return Eigen::Vector3d(x, y, theta);
}

Eigen::Matrix3d SlamBackend::computeAij(const Pose2D& x_i, const Pose2D& x_j, const Pose2D& z) const {
    Eigen::Matrix3d Aij;
    Aij.setZero();
    auto Rij = computeR(z);
    auto Ri = computeR(x_i);
    auto t_i = x_i.head<2>();
    auto t_j = x_j.head<2>();
    auto dRiT = computeDelRTDelTheta(x_i);
    Aij.block(0, 0, 2, 2) = -Rij.transpose() * Ri.transpose();
    Aij.block(0, 2, 2, 1) = Rij.transpose() * dRiT * (t_j - t_i);
    Aij(2,2) = -1;
    return Aij;
}
Eigen::Matrix3d SlamBackend::computeBij(const Pose2D& x_i, const Pose2D& z) const {
    Eigen::Matrix3d Bij;
    Bij.setZero();
    auto Rij = computeR(z);
    auto Ri = computeR(x_i);
    Bij.block(0,0,2,2) = Rij.transpose() * Ri.transpose();
    Bij(2,2) = 1.0;
    return Bij;
}

Eigen::Matrix2d SlamBackend::computeR(const Pose2D& z) const {
    Eigen::Matrix2d R;
    double theta = z(2);
    double c = std::cos(theta);
    double s = std::sin(theta);
    R << c, -s,
         s,  c;
    return R;
}
Eigen::Matrix2d SlamBackend::computeDelRTDelTheta(const Pose2D& z) const {
    Eigen::Matrix2d dRT;
    double theta = z(2);
    double c = std::cos(theta);
    double s = std::sin(theta);
    dRT << -s, c,
            -c, -s;
    return dRT;
}

double SlamBackend::normalizeAngle(double a) const {
    return std::atan2(std::sin(a), std::cos(a));
}

Eigen::Matrix3d SlamBackend::inverseT(const Eigen::Matrix3d& T) const {
    Eigen::Matrix3d T_inv;
    T_inv.setZero();
    T_inv.block<2, 2>(0, 0) = T.block<2, 2>(0, 0).transpose();
    T_inv.block<2, 1>(0, 2) = -T_inv.block<2, 2>(0, 0) * T.block<2, 1>(0, 2);
    T_inv(2, 2) = 1;
    return T_inv;
}

} // namespace scan_slam