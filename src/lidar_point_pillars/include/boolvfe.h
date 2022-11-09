#pragma once

class BoolVFECuda {
 private:
    // initializer list
    const int num_threads_;
    const int input_point_feature_;
    const int num_point_feature_;
    const int grid_x_size_;
    const int grid_y_size_;
    const int grid_z_size_;
    const float pillar_x_size_;
    const float pillar_y_size_;
    const float pillar_z_size_;
    const float min_x_range_;
    const float min_y_range_;
    const float min_z_range_;
    // end initializer list

 public:
   BoolVFECuda(
    const int num_threads, 
    const int input_point_feature,
    const int num_point_feature,
    const int grid_x_size, const int grid_y_size,
    const int grid_z_size, const float pillar_x_size, const float pillar_y_size,
    const float pillar_z_size, const float min_x_range, const float min_y_range,
    const float min_z_range);
  ~BoolVFECuda();

  void DoBoolVFECuda(
    const float* dev_points, const int in_num_points, 
    float* dev_pillar_count_histo);
};