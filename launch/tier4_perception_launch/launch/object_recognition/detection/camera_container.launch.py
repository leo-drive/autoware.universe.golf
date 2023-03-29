# Copyright 2020 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions.launch_configuration import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from launch import LaunchContext

import yaml

def generate_launch_description():
    launch_arguments = []
    context = LaunchContext()

    use_multithread = "True"
    use_intra_process = "True"
    output_topic= "rois0"

    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    # tensorrt params
    add_launch_arg("yolo_type", description="yolo model type")
    add_launch_arg("input_image", description="input camera topic")
    add_launch_arg("input_camera_info", description="input camera info topic")
    add_launch_arg("label_file", description="tensorrt node label file")
    add_launch_arg("gpu_id", description="gpu setting")
    add_launch_arg("mode", description="the inference mode:FP32,FP16,INT8")
    add_launch_arg("camera_param_path", description="camera parameter file path")

    camera_node_params=create_parameter_dict(
            "mode",
            "gpu_id",
            "yolo_type",
            "label_file",
            "input_image",
            "input_camera_info",
            "camera_param_path",

        )

    calib_image_directory= FindPackageShare("tensorrt_yolo").perform(context) + "/calib_image/"
    tensorrt_config_path = FindPackageShare('tensorrt_yolo').perform(context)+ "/config/" + str(camera_node_params["yolo_type"]) + ".param.yaml"
    #
    with open(tensorrt_config_path, "r") as f:
        tensorrt_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    print(222222)
    # camera_param_path = "/home/leodrive/projects/autoware/src/sensor_component/external/arena_camera/param/b_camera.param.yaml"

    with open(camera_node_params["camera_param_path"], "r") as f:
        camera_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]


    container = ComposableNodeContainer(
        name="front_camera_node_container",
        namespace="/perception/object_detection",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            ComposableNode(
                package="arena_camera",
                plugin="ArenaCameraNode",
                name="front_arena_camera_node",
                parameters=[{
                    "camera_name": camera_yaml_param['camera_name'],
                    "frame_id": camera_yaml_param['frame_id'],
                    "pixel_format": camera_yaml_param['pixel_format'],
                    "serial_no": camera_yaml_param['serial_no'],
                    "camera_info_url": camera_yaml_param['camera_info_url'],
                    "fps": camera_yaml_param['fps'],
                    "horizontal_binning": camera_yaml_param['horizontal_binning'],
                    "vertical_binning": camera_yaml_param['vertical_binning'],
                    "use_default_device_settings": camera_yaml_param['use_default_device_settings'],
                    "exposure_auto": camera_yaml_param['exposure_auto'],
                    "exposure_target": camera_yaml_param['exposure_target'],
                    "gain_auto": camera_yaml_param['gain_auto'],
                    "gain_target": camera_yaml_param['gain_target'],
                    "gamma_target": camera_yaml_param['gamma_target'],
                }],
                remappings=[
                ],
                extra_arguments=[
                    {"use_intra_process_comms": bool(use_intra_process)}
                ],
            ),
            ComposableNode(
                package='image_rectifier',
                plugin='image_preprocessor::ImageRectifier',
                name='rectify_front_camera_image_node',
                # Remap subscribers and publishers
                remappings=[
                    ('image', camera_node_params["input_image"]),
                    ('camera_info', camera_node_params["input_camera_info"]),
                    ('image_rect', 'image_rect_front')
                ],
                extra_arguments=[
                    {"use_intra_process_comms": bool(use_intra_process)}
                ],
            ),
            ComposableNode(
                package="tensorrt_yolo",
                plugin="object_recognition::TensorrtYoloNodelet",
                name="front_camera_tensorrt_yolo",
                parameters=[

                    {
                        "gpu_id": camera_node_params["gpu_id"],
                        "mode": camera_node_params["mode"],
                        "onnx_file": FindPackageShare("tensorrt_yolo").perform(context) +  "/data/" + str(camera_node_params["yolo_type"]) + ".onnx",
                        "engine_file":  FindPackageShare("tensorrt_yolo").perform(context) + "/data/"+ str(camera_node_params["yolo_type"]) + ".engine",
                        "label_file": FindPackageShare("tensorrt_yolo").perform(context) + "/data/" + str(camera_node_params["label_file"]),
                        "calib_image_directory": calib_image_directory,
                        "calib_cache_file": FindPackageShare("tensorrt_yolo").perform(context) + "/data/" + camera_node_params["yolo_type"] + ".cache",
                        "num_anchors": tensorrt_yaml_param['num_anchors'],
                        "anchors": tensorrt_yaml_param['anchors'],
                        "scale_x_y": tensorrt_yaml_param['scale_x_y'],
                        "score_threshold": tensorrt_yaml_param['score_threshold'],
                        "iou_thresh": tensorrt_yaml_param['iou_thresh'],
                        "detections_per_im": tensorrt_yaml_param['detections_per_im'],
                        "use_darknet_layer": tensorrt_yaml_param['use_darknet_layer'],
                        "ignore_thresh": tensorrt_yaml_param['ignore_thresh'],
                    }
                ],
                remappings=[
                    ("in/image", 'image_rect_front'),
                    ("out/objects", output_topic),
                    ("out/image", output_topic + "/debug/image"),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": bool(use_intra_process)}
                ],
            ),
        ],
        output="both",
    )

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(use_multithread),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(use_multithread),
    )

    return LaunchDescription(
        [
            *launch_arguments,
            set_container_executable,
            set_container_mt_executable,
            container,
        ]
    )