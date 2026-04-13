#include "scan_slam/correlative_scan_matcher.hpp"
#include <queue>
#include <iostream>

namespace scan_slam {

struct CandidatePose {
    int dx; // in grid cells
    int dy; // in grid cells
    int dtheta; // in grid cells
    float score;

    bool operator<(const CandidatePose& other) const {
        return score < other.score;
    }
};

CorrelativeScanMatcher::CorrelativeScanMatcher(CorrelativeScanMatcherOptions options) :
    search_window_xy_(options.search_window_xy),
    search_window_theta_(options.search_window_theta),
    resolution_xy_(options.resolution_xy),
    resolution_theta_(options.resolution_theta),
    sigma_xy_(options.sigma_xy),
    lookupTable_{options.search_window_xy, options.resolution_xy, options.sigma_xy, options.coarse_reduction_factor, options.score_temperature},
    motion_covariance_(options.motion_covariance),
    use_approx_matcher_covariance_(options.use_approx_matcher_covariance),
    loop_closure_search_radius_(options.loop_closure_search_radius),
    loop_closure_search_theta_(options.loop_closure_search_theta),
    coarse_reduction_factor_(options.coarse_reduction_factor) {
        motion_information_ = motion_covariance_.inverse();
        pose_covariance_ = motion_covariance_;
        pose_information_ = pose_covariance_.inverse();
    }

MatchResult CorrelativeScanMatcher::matchScans(const ScanPoints& old_scan, const Pose2D& old_pose, const ScanPoints& new_scan, const Pose2D& pose_initial_guess, bool loop_closure) {

    // Transform old scan to world frame using old pose
    double c = std::cos(old_pose(2));
    double s = std::sin(old_pose(2));
    std::vector<Eigen::Vector2d> transformed_points = old_scan.points;
    for (auto& pt : transformed_points) {
        double x = pt.x();
        double y = pt.y();

        pt.x() = old_pose(0) + c * x - s * y;
        pt.y() = old_pose(1) + s * x + c * y;
    }

    // Build lookup table from transformed old scan and propagate pose covariance through motion model
    ScanPoints transformed_scan(transformed_points);
    lookupTable_.build(transformed_scan);
    Pose2D delta_pose = computeRelativePose(old_pose, pose_initial_guess);
    pose_covariance_ = propagatePoseCovariance(pose_covariance_, delta_pose, old_pose);
    pose_information_ = pose_covariance_.inverse();

    // Coarse search over discretized pose space around initial guess
    std::priority_queue<CandidatePose> coarse_candidates;
    float search_radius = loop_closure ? loop_closure_search_radius_ : search_window_xy_;
    float search_theta = loop_closure ? loop_closure_search_theta_ : search_window_theta_;
    int max_dx = static_cast<int>(std::ceil(search_radius / resolution_xy_));
    int max_dy = static_cast<int>(std::ceil(search_radius / resolution_xy_));
    int max_dtheta = static_cast<int>(std::ceil(search_theta / resolution_theta_));

    int coarse_max_dx = static_cast<int>(std::ceil((float)max_dx / coarse_reduction_factor_));
    int coarse_max_dy = static_cast<int>(std::ceil((float)max_dy / coarse_reduction_factor_));

    int iterations = 0;
    for (int dtheta = -max_dtheta; dtheta <= max_dtheta; dtheta++) {
        ScanPoints rotated_scan = rotateScanToInitialGuessAndThetaOffset(new_scan, dtheta, pose_initial_guess);
        for (int dx = -coarse_max_dx; dx <= coarse_max_dx; dx++) {
            for (int dy = -coarse_max_dy; dy <= coarse_max_dy; dy++) {
                CandidatePose candidate{dx, dy, dtheta, 0.0};
                candidate.score = lookupTable_.getCoarseCorrelationWithOffsets(rotated_scan, dx * resolution_xy_ * coarse_reduction_factor_, dy * resolution_xy_ * coarse_reduction_factor_);
                coarse_candidates.push(candidate);
                iterations++;
            }
        }
    }

    CandidatePose best{0, 0, 0, -std::numeric_limits<float>::max()};

    while (!coarse_candidates.empty()) {
        CandidatePose next = coarse_candidates.top();
        coarse_candidates.pop();
        iterations++;

        if (next.score <= best.score) break; // Prune - coarse score is upper bound
        // Expand this coarse cell into its fine subcells
        for (int ddx = 0; ddx < coarse_reduction_factor_; ddx++) {
            for (int ddy = 0; ddy < coarse_reduction_factor_; ddy++) {
                int fine_dx = next.dx * coarse_reduction_factor_ + ddx;
                int fine_dy = next.dy * coarse_reduction_factor_ + ddy;
                if (std::abs(fine_dx) > max_dx || std::abs(fine_dy) > max_dy) continue;

                ScanPoints fine_scan = rotateScanToInitialGuessAndThetaOffset(new_scan, next.dtheta, pose_initial_guess);
                float fine_score = evaluatePosteriorAt(fine_scan, fine_dx, fine_dy, next.dtheta);
                if (fine_score > best.score) {
                    best = {fine_dx, fine_dy, next.dtheta, fine_score};
                }
            }
        }
    }

    last_iterations_ = iterations;

    Pose2D matched_world_pose;
    matched_world_pose(0) = pose_initial_guess(0) + best.dx * resolution_xy_;
    matched_world_pose(1) = pose_initial_guess(1) + best.dy * resolution_xy_;
    matched_world_pose(2) = pose_initial_guess(2) + best.dtheta * resolution_theta_;
    matched_world_pose(2) = normalizeAngle(matched_world_pose(2));
    Pose2D delta_from_old = computeRelativePose(old_pose, matched_world_pose);

    Pose2D new_pose = matched_world_pose;
    Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Zero();
    if (!loop_closure) {
        new_pose(2) = normalizeAngle(new_pose(2));

        Eigen::Matrix3d matcher_covariance = computeMatcherCovariance(new_scan, pose_initial_guess, new_pose, loop_closure);
        information_matrix = matcher_covariance.inverse();

        setPoseCovariance(matcher_covariance);
    }

    return MatchResult{delta_from_old, new_pose, information_matrix, best.score};
}

Pose2D CorrelativeScanMatcher::computeRelativePose(const Pose2D& from, const Pose2D& to) {
    double dx = to(0) - from(0);
    double dy = to(1) - from(1);
    double c = std::cos(from(2));
    double s = std::sin(from(2));
    return Pose2D(
         c * dx + s * dy,
        -s * dx + c * dy,
        normalizeAngle(to(2) - from(2))
    );
}

ScanPoints CorrelativeScanMatcher::rotateScanToInitialGuessAndThetaOffset(const ScanPoints& query_scan, int dtheta, const Pose2D& initial_guess) {
    std::vector<Eigen::Vector2d> rotated_points(query_scan.points.size());

    double theta = dtheta * resolution_theta_;
    double c = std::cos(theta + initial_guess(2));
    double s = std::sin(theta + initial_guess(2));

    for (size_t i = 0; i < query_scan.points.size(); i++) {
        double x = query_scan.points[i].x();
        double y = query_scan.points[i].y();

        rotated_points[i].x() = initial_guess(0) + c * x - s * y;
        rotated_points[i].y() = initial_guess(1) + s * x + c * y;
    }
    return ScanPoints(rotated_points);
}

Eigen::Matrix3d CorrelativeScanMatcher::computeMatcherCovariance(const ScanPoints& query_scan, const Pose2D& initial_guess, const Pose2D& new_pose, bool loop_closure) {
    if (!use_approx_matcher_covariance_) {
        return computeExactMatcherCovariance(query_scan, initial_guess, loop_closure);
    }
    else {
        return computeApproxMatcherCovariance(query_scan, new_pose);
    }
}

Eigen::Matrix3d CorrelativeScanMatcher::computeExactMatcherCovariance(const ScanPoints& query_scan, const Pose2D& initial_guess, bool loop_closure) {
    float search_radius = loop_closure ? loop_closure_search_radius_ : search_window_xy_;
    float search_theta = loop_closure ? loop_closure_search_theta_ : search_window_theta_;
    int max_dx = static_cast<int>(std::ceil(search_radius / resolution_xy_));
    int max_dy = static_cast<int>(std::ceil(search_radius / resolution_xy_));
    int max_dtheta = static_cast<int>(std::ceil(search_theta / resolution_theta_));

    std::vector<double> log_posteriors;
    log_posteriors.reserve((2 * max_dx + 1) * (2 * max_dy + 1) * (2 * max_dtheta + 1));

    Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
    Eigen::Vector3d u(0.0, 0.0, 0.0);

    for (int dtheta = -max_dtheta; dtheta <= max_dtheta; dtheta++) {
        ScanPoints rotated_scan = rotateScanToInitialGuessAndThetaOffset(query_scan, dtheta, initial_guess);
        for (int dx = -max_dx; dx <= max_dx; dx++) {
            for (int dy = -max_dy; dy <= max_dy; dy++) {
                log_posteriors.push_back(evaluatePosteriorAt(rotated_scan, dx, dy, dtheta));
            }
        }
    }

    double max_log = *std::max_element(log_posteriors.begin(), log_posteriors.end());

    double sum = 0.0;

    for (const auto& log_p : log_posteriors) {
        double p = std::exp(log_p - max_log);
        sum += p;
    }

    for (int dtheta = -max_dtheta; dtheta <= max_dtheta; dtheta++) {
        for (int dx = -max_dx; dx <= max_dx; dx++) {
            for (int dy = -max_dy; dy <= max_dy; dy++) {
                Pose2D delta = Pose2D(dx * resolution_xy_, dy * resolution_xy_, dtheta * resolution_theta_);
                double posterior_log = log_posteriors[(dtheta + max_dtheta) * (2 * max_dx + 1) * (2 * max_dy + 1) + (dx + max_dx) * (2 * max_dy + 1) + (dy + max_dy)];
                double p = std::exp(posterior_log - max_log) / sum;
                K += delta*delta.transpose() * p;
                u += p * delta;
            }
        }
    }

    Eigen::Matrix3d covariance = K - (u * u.transpose());
    covariance += 1e-9 * Eigen::Matrix3d::Identity();
    return covariance;
}
Eigen::Matrix3d CorrelativeScanMatcher::computeApproxMatcherCovariance(const ScanPoints& query_scan, const Pose2D& new_pose) {
    auto F = [&](const ScanPoints& scan, int dx, int dy, int dtheta) -> double {
        return evaluatePosteriorAt(scan, dx, dy, dtheta);
    };

    auto G = [&](const ScanPoints& scan, int dx, int dy, int dtheta) -> double {
        return -F(scan, dx, dy, dtheta);
    };

    const int x0 = 0;
    const int y0 = 0;
    const int theta0 = 0;

    ScanPoints scan = rotateScanToInitialGuessAndThetaOffset(query_scan, theta0, new_pose);
    double g000 = G(scan, x0, y0, theta0);
    double g100 = G(scan, x0 + 1, y0, theta0);
    double g_100 = G(scan, x0 - 1, y0, theta0);
    double g010 = G(scan, x0, y0 + 1, theta0);
    double g0_10 = G(scan, x0, y0 - 1, theta0);
    double g110 = G(scan, x0 + 1, y0 + 1, theta0);
    double g1_10 = G(scan, x0 + 1, y0 - 1, theta0);
    double g_110 = G(scan, x0 - 1, y0 + 1, theta0);
    double g_1_10 = G(scan, x0 - 1, y0 - 1, theta0);

    scan = rotateScanToInitialGuessAndThetaOffset(query_scan, theta0 + 1, new_pose);
    double g001 = G(scan, x0, y0, theta0 + 1);
    double g101 = G(scan, x0 + 1, y0, theta0 + 1);
    double g_101 = G(scan, x0 - 1, y0, theta0 + 1);
    double g011 = G(scan, x0, y0 + 1, theta0 + 1);
    double g0_11 = G(scan, x0, y0 - 1, theta0 + 1);

    scan = rotateScanToInitialGuessAndThetaOffset(query_scan, theta0 - 1, new_pose);
    double g00_1 = G(scan, x0, y0, theta0 - 1);
    double g10_1 = G(scan, x0 + 1, y0, theta0 - 1);
    double g_10_1 = G(scan, x0 - 1, y0, theta0 - 1);
    double g01_1 = G(scan, x0, y0 + 1, theta0 - 1);
    double g0_1_1 = G(scan, x0, y0 - 1, theta0 - 1);

    Eigen::Matrix3d H;
    H(0, 0) = (g100 - 2 * g000 + g_100) / (resolution_xy_ * resolution_xy_);
    H(1, 1) = (g010 - 2 * g000 + g0_10) / (resolution_xy_ * resolution_xy_);
    H(2, 2) = (g001 - 2 * g000 + g00_1) / (resolution_theta_ * resolution_theta_);
    H(0, 1) = H(1, 0) = (g110 - g1_10 - g_110 + g_1_10) / (4.0  * resolution_xy_ * resolution_xy_);
    H(0, 2) = H(2, 0) = (g101 - g10_1 - g_101 + g_10_1) / (4.0 * resolution_xy_ * resolution_theta_);
    H(1, 2) = H(2, 1) = (g011 - g01_1 - g0_11 + g0_1_1) / (4.0 * resolution_xy_ * resolution_theta_);
    H += 1e-9 * Eigen::Matrix3d::Identity();

    Eigen::LLT<Eigen::Matrix3d> llt(H);
    if (llt.info() != Eigen::Success) {
        return motion_covariance_;
    }
    return llt.solve(Eigen::Matrix3d::Identity());
}

double CorrelativeScanMatcher::evaluatePosteriorAt(const ScanPoints& query_scan, int dx, int dy, int dtheta) {
    Pose2D delta = Pose2D(dx * resolution_xy_, dy * resolution_xy_, dtheta * resolution_theta_);
    double score = lookupTable_.getFineCorrelationWithOffsets(query_scan, dx * resolution_xy_, dy * resolution_xy_);
    double prior_log = -0.5 * delta.dot(pose_information_ * delta);
    double posterior_score_log = score + prior_log;
    return posterior_score_log;
}

double CorrelativeScanMatcher::normalizeAngle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

Pose2D CorrelativeScanMatcher::composePose(const Pose2D& base, const Pose2D& delta) {
    double x = base(0) + delta(0) * std::cos(base(2)) - delta(1) * std::sin(base(2));
    double y = base(1) + delta(0) * std::sin(base(2)) + delta(1) * std::cos(base(2));
    double theta = std::atan2(std::sin(base(2) + delta(2)), std::cos(base(2) + delta(2)));
    return Pose2D(x, y, theta);
}

Pose2D CorrelativeScanMatcher::worldDeltaToLocal(const Pose2D& base_pose, const Pose2D& delta_world) {
    double c = std::cos(base_pose(2));
    double s = std::sin(base_pose(2));

    double dx_local =  c * delta_world(0) + s * delta_world(1);
    double dy_local = -s * delta_world(0) + c * delta_world(1);
    double dtheta_local = delta_world(2);

    return Pose2D(dx_local, dy_local, dtheta_local);
}

Eigen::Matrix3d CorrelativeScanMatcher::propagatePoseCovariance(const Eigen::Matrix3d& pose_covariance, const Pose2D& delta_pose, const Pose2D& prev_pose) {
    // double theta = prev_pose(2);
    // double c = std::cos(theta);
    // double s = std::sin(theta);

    // Eigen::Matrix3d F;
    // F << 1, 0, -delta_pose(0) * s - delta_pose(1) * c,
    //      0, 1,  delta_pose(0) * c - delta_pose(1) * s,
    //      0, 0, 1;

    // Eigen::Matrix3d G;
    // G << c, -s, 0,
    //      s,  c, 0,
    //      0,  0, 1;

    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d G = Eigen::Matrix3d::Identity();

    return F * pose_covariance * F.transpose() + G * motion_covariance_ * G.transpose();
}

void CorrelativeScanMatcher::setPoseCovariance(const Eigen::Matrix3d& matcher_covariance) {
    pose_covariance_ = matcher_covariance;
    pose_information_ = matcher_covariance.inverse();
}

Eigen::Matrix3d CorrelativeScanMatcher::getPoseCovariance() const {
    return pose_covariance_;
}

} // namespace scan_slam