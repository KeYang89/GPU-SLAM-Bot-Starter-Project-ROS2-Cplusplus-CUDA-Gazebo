#include "gpu_slam_bot/gpu_grid.hpp"
#include <cuda_runtime.h>
#include <cmath>
#include <stdexcept>

using namespace gpu_slam_bot;

static inline void checkCuda(cudaError_t err, const char* msg){
  if (err != cudaSuccess) throw std::runtime_error(std::string(msg)+": "+cudaGetErrorString(err));
}

__device__ __forceinline__
int idx(int x, int y, int W){ return y*W + x; }

__device__ float clampf(float v, float lo, float hi){ return fminf(hi, fmaxf(lo, v)); }

// DDA ray march updating free cells; final hit cell marked occupied.
__global__ void integrate_kernel(float* grid, int W, int H,
                                 float res, float ox, float oy,
                                 float l_free, float l_occ, float lmin, float lmax,
                                 const float* ranges, const float* angles,
                                 int nbeams,
                                 float rx, float ry, float ryaw, float rmax)
{
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= nbeams) return;

  float r = ranges[i];
  float a = angles[i] + ryaw;
  bool hit = (r > 0.0f && r < rmax*0.999f);
  float use_r = hit ? r : rmax;

  // Endpoint in world
  float ex = rx + use_r * cosf(a);
  float ey = ry + use_r * sinf(a);

  // Ray start in world
  float sx = rx;
  float sy = ry;

  // Convert to grid coords
  int gx0 = (int)floorf((sx - ox) / res);
  int gy0 = (int)floorf((sy - oy) / res);
  int gx1 = (int)floorf((ex - ox) / res);
  int gy1 = (int)floorf((ey - oy) / res);

  // DDA setup
  int dx = abs(gx1 - gx0), sxg = gx0 < gx1 ? 1 : -1;
  int dy = abs(gy1 - gy0), syg = gy0 < gy1 ? 1 : -1;
  int err = (dx > dy ? dx : -dy)/2; // Bresenham-like integer DDA

  int x = gx0, y = gy0;
  while (true) {
    if (x>=0 && x<W && y>=0 && y<H) {
      int id = idx(x,y,W);
      float v = atomicAdd(&grid[id], l_free); // freespace along ray
      // clamp after add (benign race; acceptable for mapping)
      grid[id] = clampf(v + 0.0f, lmin, lmax);
    }
    if (x==gx1 && y==gy1) break;
    int e2 = err;
    if (e2 > -dx) { err -= dy; x += sxg; }
    if (e2 <  dy) { err += dx; y += syg; }
  }

  // Mark endpoint occupied if it was an actual hit
  if (hit && gx1>=0 && gx1<W && gy1>=0 && gy1<H) {
    int id = idx(gx1, gy1, W);
    float v = atomicAdd(&grid[id], l_occ);
    grid[id] = clampf(v + 0.0f, lmin, lmax);
  }
}

namespace gpu_slam_bot {

GpuGrid::GpuGrid(const GridParams &p): params_(p) {
  size_t bytes = (size_t)p.width * p.height * sizeof(float);
  checkCuda(cudaMalloc(&d_grid_, bytes), "cudaMalloc grid");
  checkCuda(cudaMemset(d_grid_, 0, bytes), "cudaMemset grid");
}

GpuGrid::~GpuGrid(){ if (d_grid_) cudaFree(d_grid_); }

void GpuGrid::integrateScan(const std::vector<float> &ranges,
                            const std::vector<float> &angles,
                            float rx, float ry, float ryaw,
                            float rmax)
{
  int n = (int)ranges.size();
  if ((int)angles.size() != n) throw std::runtime_error("ranges/angles size mismatch");

  float *d_ranges=nullptr, *d_angles=nullptr;
  checkCuda(cudaMalloc(&d_ranges, n*sizeof(float)), "malloc ranges");
  checkCuda(cudaMalloc(&d_angles, n*sizeof(float)), "malloc angles");
  checkCuda(cudaMemcpy(d_ranges, ranges.data(), n*sizeof(float), cudaMemcpyHostToDevice), "cpy ranges");
  checkCuda(cudaMemcpy(d_angles, angles.data(), n*sizeof(float), cudaMemcpyHostToDevice), "cpy angles");

  int threads = 256;
  int blocks = (n + threads - 1) / threads;

  integrate_kernel<<<blocks, threads>>>(d_grid_, params_.width, params_.height,
    params_.resolution, params_.origin_x, params_.origin_y,
    params_.l_free, params_.l_occ, params_.l_min, params_.l_max,
    d_ranges, d_angles, n, rx, ry, ryaw, rmax);

  checkCuda(cudaGetLastError(), "kernel launch");
  cudaFree(d_ranges); cudaFree(d_angles);
}

static inline int8_t logOddsToOcc(float l){
  // p = 1 - 1/(1+exp(l)) = exp(l)/(1+exp(l))
  float p = 1.0f - 1.0f/(1.0f + std::exp(l));
  int v = (int)std::round(p * 100.0f);
  if (v < 0) v = 0; if (v > 100) v = 100; return (int8_t)v;
}

void GpuGrid::downloadToOcc(std::vector<int8_t> &occ_out) const {
  size_t N = (size_t)params_.width * params_.height;
  std::vector<float> h(N);
  checkCuda(cudaMemcpy(h.data(), d_grid_, N*sizeof(float), cudaMemcpyDeviceToHost), "download grid");
  occ_out.resize(N);
  for (size_t i=0;i<N;++i) occ_out[i] = logOddsToOcc(h[i]);
}

} // namespace gpu_slam_bot