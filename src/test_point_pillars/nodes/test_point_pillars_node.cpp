// headers in STL
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
// headers in 3rd-part
// #include "lidar_point_pillars/point_pillars.h"
#include "point_pillars.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

int bin2Arrary( float* &points_array , string file_name , int num_feature)
{
  ifstream InFile;
  InFile.open(file_name.data(), std::ios::binary );
  assert(InFile.is_open());

  float f;
  std::vector<float> temp_points;
  while (InFile.read(reinterpret_cast<char*>(&f), sizeof(float)))
    temp_points.emplace_back(f);

  points_array = new float[temp_points.size()];
  for (int i = 0 ; i < temp_points.size() ; ++i) {
    points_array[i] = temp_points[i];
  }

  InFile.close();  
  std::cout << "bin size: " << temp_points.size() << " " << num_feature << std::endl;
  return temp_points.size() / num_feature;
  // printf("Done");
};

int Txt2Arrary( float* &points_array , string file_name , int num_feature = 4)
{
  ifstream InFile;
  InFile.open(file_name.data());
  assert(InFile.is_open());

  vector<float> temp_points;
  string c;

  while (!InFile.eof())
  {
      InFile >> c;

      temp_points.push_back(atof(c.c_str()));
  }
  points_array = new float[temp_points.size()];
  for (int i = 0 ; i < temp_points.size() ; ++i) {
    points_array[i] = temp_points[i];
  }

  InFile.close();  
  return temp_points.size() / num_feature;
  // printf("Done");
};

void Boxes2Txt( std::vector<float> boxes , string file_name , int num_feature = 7)
{
    ofstream ofFile;
    ofFile.open(file_name , std::ios::out );  
    if (ofFile.is_open()) {
    }
    ofFile.close();
    return ;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_point_pillars");
  std::cout << "Test_point_pillars start" << std::endl;
  std::string pfe_file, backbone_file, pp_config, input_bin, dataset_name; 
  ros::param::param<std::string>("~pfe_onnx_file", pfe_file, "");
  ros::param::param<std::string>("~backbone_file", backbone_file, "");
  ros::param::param<std::string>("~pp_config", pp_config, "");
  ros::param::param<std::string>("~input_bin", input_bin, "");
  ros::param::param<std::string>("~dataset_name", dataset_name, "");

  int points_stride;
  if (dataset_name.compare("kitti") == 0)
    points_stride = 4;
  else 
    points_stride = 5;

  float* points_array;
  int in_num_points;
  in_num_points = bin2Arrary(points_array,input_bin,points_stride);
  // std::cout << "PCL: " << input_bin << ": " << in_num_points << std::endl;

  int BoxFeature = 7;
  float ScoreThreshold = 0.1;
  float NmsOverlapThreshold = 0.2;
  bool useOnnx = true;
  std::unique_ptr<PointPillars> point_pillars_ptr_;
  point_pillars_ptr_.reset(new PointPillars(
    ScoreThreshold,
    NmsOverlapThreshold,
    useOnnx, 
    pfe_file,
    backbone_file,
    pp_config
  ));
 
  for (int _ = 0 ; _ < 4 ; _++)
  {
    std::vector<float> out_detections;
    std::vector<int> out_labels;
    std::vector<float> out_scores;

    cudaDeviceSynchronize();
    // pp.doInference(points_array, in_num_points, &out_detections, &out_labels , &out_scores);
    point_pillars_ptr_->doInference(points_array, in_num_points, &out_detections, &out_labels , &out_scores);
    cudaDeviceSynchronize();
    int BoxFeature = 7;
    int num_objects = out_detections.size() / BoxFeature;

    std::cout << num_objects << std::endl;
    // for (int i = 0 ; i < out_detections.size() / BoxFeature ; ++i) {
    //     for (int j = 0 ; j < BoxFeature ; ++j) {
    //         std::cout << out_detections.at(i * BoxFeature + j) << " ";
    //     }
    //     std::cout << "\n";
    // }
  }

  return 0;
}
