// headers in STL
#include <stdio.h>

// headers in local files
#include "common.h"
#include "boolvfe.h"

__global__ void make_pillar_histo_kernel(
    const float* dev_points, 
    float* pillar_count_histo, const int num_points,
    const int grid_x_size, const int grid_y_size, const int grid_z_size, 
    const float min_x_range, const float min_y_range, const float min_z_range, 
    const float pillar_x_size, const float pillar_y_size, const float pillar_z_size,
    const int input_point_feature) {
  int th_i = blockIdx.x * blockDim.x +  threadIdx.x ;
  if (th_i >= num_points) {
    return;
  }
  int x_coor = floor((dev_points[th_i * input_point_feature + 0] - min_x_range) / pillar_x_size);
  int y_coor = floor((dev_points[th_i * input_point_feature + 1] - min_y_range) / pillar_y_size);
  int z_coor = floor((dev_points[th_i * input_point_feature + 2] - min_z_range) / pillar_z_size);

  if (x_coor >= 0 && x_coor < grid_x_size && y_coor >= 0 &&
      y_coor < grid_y_size && z_coor >= 0 && z_coor < grid_z_size) {
    pillar_count_histo[z_coor * grid_x_size * grid_y_size + y_coor * grid_x_size + x_coor] = 1;
  }
}

BoolVFECuda::BoolVFECuda(
    const int num_threads, 
    const int input_point_feature,
    const int num_point_feature,
    const int grid_x_size, const int grid_y_size,
    const int grid_z_size, const float pillar_x_size, const float pillar_y_size,
    const float pillar_z_size, const float min_x_range, const float min_y_range,
    const float min_z_range)
    : num_threads_(num_threads),
      input_point_feature_(input_point_feature),
      num_point_feature_(num_point_feature),
      grid_x_size_(grid_x_size),
      grid_y_size_(grid_y_size),
      grid_z_size_(grid_z_size),
      pillar_x_size_(pillar_x_size),
      pillar_y_size_(pillar_y_size),
      pillar_z_size_(pillar_z_size),
      min_x_range_(min_x_range),
      min_y_range_(min_y_range),
      min_z_range_(min_z_range) {
    
  }

BoolVFECuda::~BoolVFECuda() {
  }

void BoolVFECuda::DoBoolVFECuda(
    const float* dev_points, const int in_num_points, 
    int* dev_x_coors,int* dev_y_coors, 
    float* dev_num_points_per_pillar,
    float* dev_pillar_point_feature, 
    float* dev_pillar_coors,
    int* dev_sparse_pillar_map, 
    int* host_pillar_count, 
    float* dev_pillar_count_histo) {
    // initialize paraments
    std::cout << grid_y_size_ << " " << grid_x_size_ << " " << grid_z_size_ << " " << input_point_feature_ << std::endl;
    GPU_CHECK(cudaMemset(dev_pillar_count_histo, 0 , grid_y_size_ * grid_x_size_ * grid_z_size_ * sizeof(float)));
    int num_block = DIVUP(in_num_points , num_threads_);

    make_pillar_histo_kernel<<<num_block , num_threads_>>>(
        dev_points, dev_pillar_count_histo, in_num_points, 
        grid_x_size_, grid_y_size_, grid_z_size_, 
        min_x_range_, min_y_range_, min_z_range_, 
        pillar_x_size_, pillar_y_size_, pillar_z_size_, 
        input_point_feature_);

    // float * temp_float = new float[grid_y_size_ * grid_x_size_ * grid_z_size_](); 
    // GPU_CHECK(cudaMemcpy(temp_float, dev_pillar_count_histo, grid_y_size_ * grid_x_size_ * grid_z_size_ * sizeof(float), cudaMemcpyDeviceToHost));
    // int cnt = 0;
    // for (int i = 0; i < grid_y_size_ * grid_x_size_ * grid_z_size_; i++){
    //     cnt += temp_float[i];
    // } 
    // std::cout << cnt << std::endl;
}