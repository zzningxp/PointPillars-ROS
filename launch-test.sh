pp_path=`pwd`/src

model_prefix=cbgs_test_e2
model_prefix=cbgs_pp_multihead
#model_prefix=z1117_boolmap_e40

pp_config=cbgs_pp_multihead.yaml
#pp_config=pointpillar_boolmap_multihead.yaml

#input_bin=n008-2018-05-21-11-06-59-0400__LIDAR_TOP__1526915243047392.pcd.bin
input_bin=000000.bin

roslaunch test_point_pillars test_point_pillars.launch \
       	pfe_onnx_file:=${pp_path}/pretrain_models/${model_prefix}_pfe.onnx \
       	backbone_file:=${pp_path}/pretrain_models/${model_prefix}_backbone.onnx \
       	pp_config:=${pp_path}/lidar_point_pillars/cfgs/${pp_config} \
	input_bin:=${pp_path}/test_point_pillars/data/${input_bin} \
	dataset_name:=nusc
	#dataset_name:=kitti
