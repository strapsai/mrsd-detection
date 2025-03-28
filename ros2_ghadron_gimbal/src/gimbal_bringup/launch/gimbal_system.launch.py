from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from datetime import datetime

def generate_launch_description():
    # 获取当前日期和时间
    now = datetime.now()
    
    # 按日创建文件夹
    day_folder = now.strftime("%Y-%m-%d")
    
    # 创建详细的时间戳文件名（年月日_时分秒）
    timestamp = now.strftime("%Y%m%d_%H%M%S")
    
    # 构建完整路径: 基础路径/日期/年月日_时分秒
    base_dir = "/home/dtc/humanflow/ros2_ghadron_gimbal/mcap_recording"
    day_dir = os.path.join(base_dir, day_folder)
    
    # 确保日期文件夹存在
    os.makedirs(day_dir, exist_ok=True)
    
    # 最终记录路径
    recording_path = os.path.join(day_dir, f"recording_{timestamp}")
    
    return LaunchDescription([
        LogInfo(msg=['启动云台系统...']),
        
        # 云台控制节点
        Node(
            package='gimbal_angle_control',
            executable='gimbal_angle_control_node',
            name='gimbal_angle_control_node',
            output='screen'
        ),
        
        # 云台状态节点
        Node(
            package='gimbal_status',
            executable='gimbal_status_node',
            name='gimbal_status_node',
            output='screen'
        ),
        
        # 视频流发布节点
        # Node(
        #     package='stream_publisher',
        #     executable='stream_node',
        #     name='stream_node',
        #     parameters=[{
        #         'rtsp_url': 'rtsp://10.3.1.124:8554/ghadron',
        #         'width': 640,
        #         'height': 360
        #     }],
        #     output='screen'
        # ),
        # Node(
        #     package='detect_and_track',
        #     executable='detect_track_node',
        #     name='detect_track_node',
        #     output='screen'
        # ),

        Node(package='inted_gimbal',
             executable='integrated_node',
             name='integrated_node',
             output='screen'),
        
        # 图像查看节点 - 不保存单独图像文件
        Node(
            package='image_viewer',
            executable='image_viewer_node',
            name='image_viewer_node',
            output='screen'
        ),

        # # YOLO检测节点
        # Node(
        #     package='yolo_detection',
        #     executable='yolo_detection_node',
        #     name='yolo_detection_node',
        #     output='screen'
        # ),
        
        # # 人物追踪节点
        # Node(
        #     package='human_tracking',
        #     executable='tracking_node',
        #     name='tracking_node',
        #     output='screen'
        # ),
        
        # 使用 bash -c 命令行格式进行录制
        ExecuteProcess(
            cmd=['bash', '-c', 
                 f"mkdir -p $(dirname {recording_path}) && " +
                 f"chmod 777 $(dirname {recording_path}) && " +
                 f"ros2 bag record " +
                 f"-o {recording_path} " +
                 f"--storage mcap " +
                 f"--max-bag-duration 60 " +
                 f"/image_raw " + 
                 f"/detection_box " +
                 f"/gimbal_attitude " +
                 f"/gimbal_angles " +
                 f"/waypoint_waiting"],
            output='screen'
        ),

        LogInfo(msg=['所有节点已启动。数据正在记录到: ' + recording_path])
    ]) 
