pp_path=`pwd`/src
#model_prefix=z0809_e50 ## 512x512
#model_prefix=z0808_e50 ## 256x256

#model_prefix=z0927_kitti
#model_prefix=z1009_kitti_e72
model_prefix=z1010_livox_e75

#pp_config=cbgs_pp_multihead.yaml
pp_config=pointpillar_kitti_g11.yaml

roslaunch lidar_point_pillars lidar_point_pillars.launch \
        pfe_onnx_file:=${pp_path}/pretrain_models/${model_prefix}_pfe.onnx \
        backbone_file:=${pp_path}/pretrain_models/${model_prefix}_backbone.onnx \
        pp_config:=${pp_path}/lidar_point_pillars/cfgs/${pp_config} \
        score_threshold:=0.4 \
	input_topic:=/kitti/velo/pointcloud
