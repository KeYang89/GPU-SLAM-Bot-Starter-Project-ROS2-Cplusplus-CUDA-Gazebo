#pragma once
#include <vector>
#include <cstdint>

namespace gpu_slam_bot {

struct GridParams {
  int width;          // cells
  int height;         // cells
  float resolution;   // m per cell
  float origin_x;     // world coords of cell (0,0)
  float origin_y;
  float l_free = -0.4f;  // log-odds decrement
  float l_occ  =  0.85f; // log-odds increment
  float l_min  = -5.0f;  // clamp
  float l_max  =  5.0f;
};

// Device-backed occupancy grid (log-odds float array)
class GpuGrid {
public:
  GpuGrid(const GridParams &params);
  ~GpuGrid();

  // Integrate a single scan on GPU
  // angles.size() must equal ranges.size()
  void integrateScan(const std::vector<float> &ranges,
                     const std::vector<float> &angles,
                     float robot_x, float robot_y, float robot_yaw,
                     float range_max);

  // Copy grid from GPU to host as int8 occupancy (0..100, -1 unknown)
  void downloadToOcc(std::vector<int8_t> &occ_out) const;

  const GridParams &params() const { return params_; }

private:
  GridParams params_;
  float *d_grid_ = nullptr; // device log-odds buffer (width*height)
};

} // namespace gpu_slam_bot