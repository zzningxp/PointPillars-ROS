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
    float* dev_pillar_count_histo) {
    // initialize paraments

    GPU_CHECK(cudaMemset(dev_pillar_count_histo, 0 , grid_y_size_ * grid_x_size_ * grid_z_size_ * sizeof(float)));
    int num_block = DIVUP(in_num_points , num_threads_);

    // std::cout << grid_x_size_ << " " << grid_y_size_ << " " << grid_z_size_ << " " << in_num_points << " " << input_point_feature_ << " "
    // << num_block  << " " << num_threads_ << std::endl;
    make_pillar_histo_kernel<<<num_block , num_threads_>>>(
        dev_points, dev_pillar_count_histo, in_num_points, 
        grid_x_size_, grid_y_size_, grid_z_size_, 
        min_x_range_, min_y_range_, min_z_range_, 
        pillar_x_size_, pillar_y_size_, pillar_z_size_, 
        input_point_feature_);

    // std::cout << grid_x_size_ << " " << grid_y_size_ << " " << grid_z_size_ << " " << 
    //     min_x_range_ << " " << min_y_range_ << " " << min_z_range_ << " " << 
    //     pillar_x_size_ << " " << pillar_y_size_ << " " << pillar_z_size_ << "\n";
    // float * host_point = new float[in_num_points](); 
    // GPU_CHECK(cudaMemcpy(host_point, dev_points, in_num_points * input_point_feature_ * sizeof(float), cudaMemcpyDeviceToHost));
    // for (int i = 0; i < in_num_points; i++){
    //   // int z_coor = floor((host_point[i * input_point_feature_ + 2] - min_z_range_) / pillar_z_size_);
    //   // if (z_coor == 38) 
    //   //    cout << i << " " << host_point[i * input_point_feature_ + 2] << "\n";
    // }
    // float * temp = new float[grid_y_size_ * grid_x_size_ * grid_z_size_](); 
    // GPU_CHECK(cudaMemcpy(temp, dev_pillar_count_histo, grid_y_size_ * grid_x_size_ * grid_z_size_ * sizeof(float), cudaMemcpyDeviceToHost));
    // int sum = 0;
    // for (int i = 0; i < grid_z_size_; i++){
    //     int cnt = 0;
    //     for (int j = 0; j < grid_y_size_ * grid_x_size_; j++){
    //         cnt += temp[i * grid_y_size_ * grid_x_size_ + j];
    //     }
    //     sum += cnt;
    //     std::cout << i << " " << cnt << "\n";
    // }
    // // std::cout << "total: " << sum << "\n";
    // for (int i = 0; i < 100; i++){
    //     for (int j = 80; j < 180; j++){
    //         cout << temp[0 * grid_y_size_ * grid_x_size_ + j * grid_x_size_ + i] ;
    //     }
    //     std::cout << "\n";
    // }
}