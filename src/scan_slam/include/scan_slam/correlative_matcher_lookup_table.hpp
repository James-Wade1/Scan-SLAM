#ifndef CORRELATIVE_MATCHER_LOOKUP_TABLE_HPP
#define CORRELATIVE_MATCHER_LOOKUP_TABLE_HPP

#include "scan_slam/slam_types.hpp"

namespace scan_slam {

class CorrelativeMatcherLookupTable {
public:
    CorrelativeMatcherLookupTable(float search_window_xy, float resolution_xy, float sigma_xy, int coarse_reduction_factor, float score_temperature);
    ~CorrelativeMatcherLookupTable() = default;
    void build(const ScanPoints& reference_scan);
    float getFineCorrelationWithOffsets(const ScanPoints& query_scan, float offset_x, float offset_y) const;
    float getCoarseCorrelationWithOffsets(const ScanPoints& query_scan, float offset_x, float offset_y) const;
    int getCoarseReductionFactor() const;
private:
    int pointToFineGridX(float x) const;
    int pointToFineGridY(float y) const;
    int pointToCoarseGridX(float x) const;
    int pointToCoarseGridY(float y) const;
    bool inBounds(int gx, int gy, int num_cells_x, int num_cells_y) const;
    void debugPrint() const;
    std::vector<float> buildLookupTable(const std::vector<float>& dist);
    std::vector<float> buildReducedLookupTable(const std::vector<float>& lookup_table, int reduction_factor);
    std::vector<float> calcDistanceField(const ScanPoints& reference_scan, int num_cells_x, int num_cells_y, float resolution_xy);

    std::vector<float> table_;
    std::vector<float> coarse_table_;
    float search_window_xy_;
    float resolution_xy_;
    float sigma_xy_;
    float score_temperature_;
    int num_cells_x_fine_;
    int num_cells_y_fine_;

    int num_cells_x_coarse_;
    int num_cells_y_coarse_;
    int coarse_reduction_factor_;

    float origin_x_;
    float origin_y_;
};

} // scan_slam

#endif