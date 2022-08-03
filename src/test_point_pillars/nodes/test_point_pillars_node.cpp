// headers in STL
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
// headers in 3rd-part
// #include "lidar_point_pillars/point_pillars.h"
#include "point_pillars.h"
using namespace std;

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
  std::string pfe_file, backbone_file, pp_config; 
  pfe_file = "/home/wheeltec/wheeltec_lidar_point_pillars_multihead/src/pretrain_models/cbgs_pp_multihead_pfe.onnx";
  backbone_file = "/home/wheeltec/wheeltec_lidar_point_pillars_multihead/src/pretrain_models/cbgs_pp_multihead_backbone.onnx"; 
  pp_config = "/home/wheeltec/wheeltec_lidar_point_pillars_multihead/src/lidar_point_pillars/cfgs/cbgs_pp_multihead.yaml";

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
  std::string file_name = "/home/wheeltec/wheeltec_lidar_point_pillars_multihead/src/test_point_pillars/data/nuscenes_10sweeps_points.txt";
  float* points_array;
  int in_num_points;
  in_num_points = Txt2Arrary(points_array,file_name,5);
 
  for (int _ = 0 ; _ < 10 ; _++)
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
