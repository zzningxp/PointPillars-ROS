pp_path=`pwd`/src
#model_prefix=z0809_e50 ## 512x512
model_prefix=z0808_e50 ## 256x256

roslaunch lidar_point_pillars lidar_point_pillars.launch \
        pfe_onnx_file:=${pp_path}/pretrain_models/${model_prefix}_pfe.onnx \
        backbone_file:=${pp_path}/pretrain_models/${model_prefix}_backbone.onnx \
        pp_config:=${pp_path}/lidar_point_pillars/cfgs/cbgs_pp_multihead.yaml \
        input_topic:=/lidar_top \
        score_threshold:=0.4
