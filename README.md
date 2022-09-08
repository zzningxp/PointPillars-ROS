# PointPillars-ROS
A 3D detection Pointpillars ROS deployment on Nvidia Jetson TX1/TX2

This repo implements https://github.com/hova88/PointPillars_MultiHead_40FPS into Autoware lidar_point_pillars framework https://github.com/autowarefoundation/autoware_ai_perception/tree/master/lidar_point_pillars.

However, multihead 40FPS models is originally tested on 3080Ti. It takes about 700ms one frame on Nvidia TX1.

I use [OpenPCDet](https://github.com/hova88/OpenPCDet) to train accelerated models within 250ms on TX1.

# Requirements 
## My Environment TX1
```
Ubuntu 18.04
ROS Melodic

jetson_release
 - NVIDIA Jetson TX1
   * Jetpack 4.5.1 [L4T 32.5.1]
   * NV Power Mode: MAXN - Type: 0
   * jetson_stats.service: active
 - Libraries:
   * CUDA: 10.2.89
   * cuDNN: 8.0.0.180
   * TensorRT: 7.1.3.0
   * Visionworks: 1.6.0.501
   * OpenCV: 3.4.5 compiled CUDA: YES
   * VPI: ii libnvvpi1 1.0.15 arm64 NVIDIA Vision Programming Interface library
   * Vulkan: 1.2.70
```
## My Environment Xavier
```
 - NVIDIA Jetson AGX Xavier [16GB]
   * Jetpack 4.5.1 [L4T 32.5.1]
   * NV Power Mode: MAXN - Type: 0
   * jetson_stats.service: active
 - Libraries:
   * CUDA: 10.2.89
   * cuDNN: 8.0.0.180
   * TensorRT: 7.1.3.0
   * Visionworks: 1.6.0.501
   * OpenCV: 3.4.5 compiled CUDA: YES
   * VPI: ii libnvvpi1 1.0.12 arm64 NVIDIA Vision Programming Interface library
   * Vulkan: 1.2.70
```

## dependence
Could not find the required component 'jsk_recognition_msgs'.
```
sudo apt-get install ros-melodic-jsk-recognition-msgs 
sudo apt-get install ros-melodic-jsk-rviz-plugins
```

# Usage
## How to compile

Simply, use catkin_make build up the whole project.

## How to launch
Launch file (cuDNN and TensorRT support): 

`pfe_onnx_file, rpn_onnx_file, pp_config, input_topic` are required

```
roslaunch lidar_point_pillars lidar_point_pillars.launch pfe_onnx_file:=/PATH/TO/FILE.onnx rpn_onnx_file:=/PATH/TO/FILE.onnx pp_config:=/PATH/TO/pp_multihead.yaml input_topic:=/points_raw 
```

`score_threshold, nms_overlap_threshold, etc` are optional to change the runtime parameters.

Or, simply, use launch.sh to run.

## Test launch

```
roslaunch test_point_pillars test_point_pillars.launch
```
nuscenes test data download: [nuscenes_10sweeps_points.txt](https://drive.google.com/file/d/1KD0LT0kzcpGUysUu__dfnfYnHUW62iwN/view?usp=sharing)

From: https://github.com/hova88/PointPillars_MultiHead_40FPS

Tx1:
```
  Preprocess    7.48567  ms
  Pfe           266.516  ms
  Scatter       1.78591  ms
  Backbone      369.692  ms
  Postprocess   35.8309  ms
  Summary       681.325  ms
```

Xavier:
```
  Summary       141.752  ms
```

## Test Rosbag:

I use [nuscenes2bag](https://github.com/clynamen/nuscenes2bag) to create some test rosbag: [nu0061 all 19s 5.5G, download password: s2eh](https://pan.baidu.com/s/1vqKvJ8jRwxEZKuuFBCig2w), [nu0061 laser and tf only 19s 209M, download password: m7wh](https://pan.baidu.com/s/11geDn_kD2LuWf2R4VqdbEg).

To use this nuscenes rosbag, you shoulde change input_topic to `/lidar_top` , and use src/rviz/nuscenes.rviz for visualization.

Usually, I use `rosbag play r 0.1 ` for more play time.

More test rosbag, like kitti, carla or real data by myself, will be released recently.

## Models Files:
Faster ONNX models on TX1:
* zz0809_512_e50 model is with the same config file as cbgs model, and the evaluation data is re-tested by the same eval benchmark.
* zz0808_256_e50 model is half resolution, you should used this config file to run: `src/lidar_point_pillars/cfgs/tx1_ppmh_256x256.yaml`

|                                             | Tx1 time | Xavier time |resolution| training data | mean ap | nd score  | car ap | ped ap | truck ap| download |
|---------------------------------------------|:--------:|:-----------:|:--------:|:-------------:|:-------:|:---------:|:------:|:------:|:-------:|:--------:| 
| cbgs_pp_multihead_pfe                       | ~700ms   | ~140ms |64x512x512| unknown       |0.447    | 0.515     | 0.813  | 0.724  | 0.500   | [pfe](https://drive.google.com/file/d/1gQWtBZ4vfrSmv2nToSIarr-d7KkEWqxw/view?usp=sharing) [backbone](https://drive.google.com/file/d/1dvUkjvhE0GEWvf6GchSGg8-lwukk7bTw/view?usp=sharing) |
| zz0809_512_e50 |~700ms| ~140ms |64x512x512|nusc tr-v|0.460|0.524|0.818|0.733|0.507|[pfe](https://drive.google.com/file/d/1mLP3v0iXUG5CrT_KLi9VBbsBbByl-WeQ/view?usp=sharing) [backbone](https://drive.google.com/file/d/1bkQfxgyxYNyBbsnwgX_JWe8YgByBTSX7/view?usp=sharing)|
| zz0808_256_e50 |~250ms| ~110ms |64x256x256|nusc tr-v|0.351|0.454|0.781|0.571|0.427|[pfe](https://drive.google.com/file/d/1pxsP5fhQG0XzpU0yzJOjRcO3ru_JM5Vn/view?usp=sharing) [backbone](https://drive.google.com/file/d/1Pb8xZ_55oo95SDSzS1KHvQ_MvnS-X1Iv/view?usp=sharing)|

More models will be released recently.
