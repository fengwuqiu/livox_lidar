import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 1    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar,1-hub
publish_freq  = 10.0 # freqency of publish,1.0,2.0,5.0,10.0,etc
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
#cmdline_bd_code = 'livox0000000001'
cmdline_bd_code = '100000000000000'
cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
rviz_config_path = os.path.join(cur_config_path, 'livox_lidar.rviz')
user_config_path = os.path.join(cur_config_path, 'livox_lidar_sanitation_v7.0_two_lidars_config.json')
lidar_front_left_ip = "192.168.1.135"
lidar_front_left_frame_id = "livox_frame_front_left"
lidar_front_left_topic_name = "/livox/lidar/front_left"
imu_front_left_topic_name = "/livox/imu/front_left"
lidar_rear_right_ip = "192.168.1.146"
lidar_rear_right_frame_id = "livox_frame_rear_right"
lidar_rear_right_topic_name = "/livox/lidar/rear_right"
imu_rear_right_topic_name = "/livox/imu/rear_right"
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code},
    {"lidar_front_left_ip": lidar_front_left_ip},
    {"lidar_front_left_frame_id": lidar_front_left_frame_id},
    {"lidar_front_left_topic_name": lidar_front_left_topic_name},
    {"imu_front_left_topic_name": imu_front_left_topic_name},
    {"lidar_rear_right_ip": lidar_rear_right_ip},
    {"lidar_rear_right_frame_id": lidar_rear_right_frame_id},
    {"imu_rear_right_topic_name": imu_rear_right_topic_name}
]

def generate_launch_description():
    livox_driver = Node(
        package='livox_ros2_driver_mid360',
        executable='livox_ros2_driver_mid360_node',
        name='livox_lidar_mid360_publisher',
        output='screen',
        parameters=livox_ros2_params
        )

    return LaunchDescription([
        livox_driver
    ])
