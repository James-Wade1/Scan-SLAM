#include "scan_slam/correlative_matcher_lookup_table.hpp"
#include <iostream>

namespace scan_slam {

CorrelativeMatcherLookupTable::CorrelativeMatcherLookupTable(float search_window_xy, float resolution_xy, float sigma_xy, int coarse_reduction_factor, float score_temperature) :
    search_window_xy_{search_window_xy},
    resolution_xy_{resolution_xy},
    sigma_xy_{sigma_xy},
    score_temperature_{score_temperature},
    num_cells_x_fine_{0},
    num_cells_y_fine_{0},
    num_cells_x_coarse_{0},
    num_cells_y_coarse_{0},
    coarse_reduction_factor_{coarse_reduction_factor},
    origin_x_{0.0f},
    origin_y_{0.0f} {}

void CorrelativeMatcherLookupTable::build(const ScanPoints& reference_scan) {
    float xmin = reference_scan.min_x - search_window_xy_*1.5;
    float xmax = reference_scan.max_x + search_window_xy_*1.5;
    float ymin = reference_scan.min_y - search_window_xy_*1.5;
    float ymax = reference_scan.max_y + search_window_xy_*1.5;

    origin_x_ = xmin;
    origin_y_ = ymin;

    num_cells_x_fine_ = static_cast<int>((xmax - xmin) / resolution_xy_) + 1;
    num_cells_y_fine_ = static_cast<int>((ymax - ymin) / resolution_xy_) + 1;

    std::vector<float> dist = calcDistanceField(reference_scan, num_cells_x_fine_, num_cells_y_fine_, resolution_xy_);
    table_ = buildLookupTable(dist);
    coarse_table_ = buildReducedLookupTable(table_, coarse_reduction_factor_);
}

std::vector<float> CorrelativeMatcherLookupTable::calcDistanceField(const ScanPoints& reference_scan, int num_cells_x, int num_cells_y, float resolution_xy) {
    std::vector<float> dist(num_cells_x * num_cells_y, std::numeric_limits<float>::infinity());

    for (const auto& pt : reference_scan.points) {
        int gx = pointToFineGridX(pt.x());
        int gy = pointToFineGridY(pt.y());
        if (inBounds(gx, gy, num_cells_x, num_cells_y)) {
            dist[gy * num_cells_x + gx] = 0.0; // Distance to occupied cell is zero
        }
    }

    for (int gy = 0; gy < num_cells_y; ++gy) {
        for (int gx = 0; gx < num_cells_x; ++gx) {
            int idx = gy * num_cells_x + gx;
            float best = dist[idx];

            if (gx > 0) {
                best = std::min(best, dist[gy * num_cells_x + (gx - 1)] + 1.0f);
            }

            if (gy > 0) {
                best = std::min(best, dist[(gy - 1) * num_cells_x + gx] + 1.0f);
            }

            if (gx > 0 && gy > 0) {
                best = std::min(best, dist[(gy - 1) * num_cells_x + (gx - 1)] + std::sqrt(2.0f));
            }

            if (gx + 1 < num_cells_x && gy > 0) {
                best = std::min(best, dist[(gy - 1) * num_cells_x + (gx + 1)] + std::sqrt(2.0f));
            }

            dist[idx] = best;
        }
    }

    for (int gy = num_cells_y - 1; gy >= 0; --gy) {
        for (int gx = num_cells_x - 1; gx >= 0; --gx) {
            int idx = gy * num_cells_x + gx;
            float best = dist[idx];

            if (gx + 1 < num_cells_x) {
                best = std::min(best, dist[gy * num_cells_x + (gx + 1)] + 1.0f);
            }

            if (gy + 1 < num_cells_y) {
                best = std::min(best, dist[(gy + 1) * num_cells_x + gx] + 1.0f);
            }

            if (gx + 1 < num_cells_x && gy + 1 < num_cells_y) {
                best = std::min(best, dist[(gy + 1) * num_cells_x + (gx + 1)] + std::sqrt(2.0f));
            }

            if (gx > 0 && gy + 1 < num_cells_y) {
                best = std::min(best, dist[(gy + 1) * num_cells_x + (gx - 1)] + std::sqrt(2.0f));
            }

            dist[idx] = best;
        }
    }

    for (auto& d : dist) {
        d *= resolution_xy; // Convert to actual distance in meters
    }

    return dist;
}

std::vector<float> CorrelativeMatcherLookupTable::buildLookupTable(const std::vector<float>& dist) {
    std::vector<float> lookup_table(dist.size());
    for (size_t i = 0; i < dist.size(); ++i) {
        lookup_table[i] = -std::pow(dist[i], 2) / (2 * std::pow(sigma_xy_, 2));
        lookup_table[i] = std::max(lookup_table[i], std::log(1e-6f)); // Avoid log(0)
    }

    return lookup_table;
}

std::vector<float> CorrelativeMatcherLookupTable::buildReducedLookupTable(const std::vector<float>& lookup_table, int reduction_factor) {
    num_cells_x_coarse_ = (num_cells_x_fine_ + reduction_factor - 1) / reduction_factor;
    num_cells_y_coarse_ = (num_cells_y_fine_ + reduction_factor - 1) / reduction_factor;
    coarse_reduction_factor_ = reduction_factor;

    std::vector<float> table;
    table.resize(num_cells_x_coarse_ * num_cells_y_coarse_);

    for (int gy = 0; gy < num_cells_y_coarse_; ++gy) {
        for (int gx = 0; gx < num_cells_x_coarse_; ++gx) {
            float max_val = std::numeric_limits<float>::lowest();
            for (int dy = 0; dy < reduction_factor; ++dy) {
                for (int dx = 0; dx < reduction_factor; ++dx) {
                    int fine_gx = gx * reduction_factor + dx;
                    int fine_gy = gy * reduction_factor + dy;
                    if (inBounds(fine_gx, fine_gy, num_cells_x_fine_, num_cells_y_fine_)) {
                        max_val = std::max(max_val, lookup_table[fine_gy * num_cells_x_fine_ + fine_gx]);
                    }
                }
            }
            table[gy * num_cells_x_coarse_ + gx] = max_val;
        }
    }

    return table;
}

float CorrelativeMatcherLookupTable::getFineCorrelationWithOffsets(const ScanPoints& query_scan, float offset_x, float offset_y) const {
    float score = 0.0f;
    for (const auto& pt : query_scan.points) {
        int gx = pointToFineGridX(pt.x() + offset_x);
        int gy = pointToFineGridY(pt.y() + offset_y);
        if (inBounds(gx, gy, num_cells_x_fine_, num_cells_y_fine_)) {
            score += table_[gy * num_cells_x_fine_ + gx];
        }
    }

    return score_temperature_ * score;
}

float CorrelativeMatcherLookupTable::getCoarseCorrelationWithOffsets(const ScanPoints& query_scan, float offset_x, float offset_y) const {
    float score = 0.0f;
    for (const auto& pt : query_scan.points) {
        int gx = pointToCoarseGridX(pt.x() + offset_x);
        int gy = pointToCoarseGridY(pt.y() + offset_y);
        if (inBounds(gx, gy, num_cells_x_coarse_, num_cells_y_coarse_)) {
            score += coarse_table_[gy * num_cells_x_coarse_ + gx];
        }
    }

    return score_temperature_ * score;
}

int CorrelativeMatcherLookupTable::pointToFineGridX(float x) const {
    return static_cast<int>(std::floor((x - origin_x_) / resolution_xy_));
}

int CorrelativeMatcherLookupTable::pointToFineGridY(float y) const {
    return static_cast<int>(std::floor((y - origin_y_) / resolution_xy_));
}

int CorrelativeMatcherLookupTable::pointToCoarseGridX(float x) const {
    return static_cast<int>(std::floor((x - origin_x_) / (resolution_xy_ * coarse_reduction_factor_)));
}

int CorrelativeMatcherLookupTable::pointToCoarseGridY(float y) const {
    return static_cast<int>(std::floor((y - origin_y_) / (resolution_xy_ * coarse_reduction_factor_)));
}

bool CorrelativeMatcherLookupTable::inBounds(int gx, int gy, int num_cells_x, int num_cells_y) const {
    return gx >= 0 && gx < num_cells_x && gy >= 0 && gy < num_cells_y;
}

int CorrelativeMatcherLookupTable::getCoarseReductionFactor() const {
    return coarse_reduction_factor_;
}

void CorrelativeMatcherLookupTable::debugPrint() const {
    std::cout << "=== Fine LUT ===" << std::endl;
    std::cout << "Dimensions: " << num_cells_x_fine_ << " x " << num_cells_y_fine_ << std::endl;
    std::cout << "Origin: " << origin_x_ << ", " << origin_y_ << std::endl;
    float fine_min = *std::min_element(table_.begin(), table_.end());
    float fine_max = *std::max_element(table_.begin(), table_.end());
    std::cout << "Fine LUT min: " << fine_min << ", max: " << fine_max << std::endl;

    std::cout << "=== Coarse LUT ===" << std::endl;
    std::cout << "Dimensions: " << num_cells_x_coarse_ << " x " << num_cells_y_coarse_ << std::endl;
    float coarse_min = *std::min_element(coarse_table_.begin(), coarse_table_.end());
    float coarse_max = *std::max_element(coarse_table_.begin(), coarse_table_.end());
    std::cout << "Coarse LUT min: " << coarse_min << ", max: " << coarse_max << std::endl;

    // For each coarse cell, print its value and the max of its fine subcells
    std::cout << "=== Coarse vs Fine subcell max (first 10 coarse cells) ===" << std::endl;
    int count = 0;
    for (int gy = 0; gy < num_cells_y_coarse_ && count < 10; ++gy) {
        for (int gx = 0; gx < num_cells_x_coarse_ && count < 10; ++gx) {
            float coarse_val = coarse_table_[gy * num_cells_x_coarse_ + gx];
            float fine_max_val = std::numeric_limits<float>::lowest();
            for (int dy = 0; dy < coarse_reduction_factor_; ++dy) {
                for (int dx = 0; dx < coarse_reduction_factor_; ++dx) {
                    int fine_gx = gx * coarse_reduction_factor_ + dx;
                    int fine_gy = gy * coarse_reduction_factor_ + dy;
                    if (inBounds(fine_gx, fine_gy, num_cells_x_fine_, num_cells_y_fine_)) {
                        fine_max_val = std::max(fine_max_val, table_[fine_gy * num_cells_x_fine_ + fine_gx]);
                    }
                }
            }
            std::cout << "Coarse[" << gx << "," << gy << "] = " << coarse_val
                      << ", fine subcell max = " << fine_max_val
                      << ", match: " << (std::abs(coarse_val - fine_max_val) < 1e-6f ? "YES" : "NO")
                      << std::endl;
            count++;
        }
    }
}

} //scan_slam