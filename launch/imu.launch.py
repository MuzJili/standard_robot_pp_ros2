from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # 1. IMU 坐标系转换节点
        # 此节点将 IMU 数据从 lidar 帧转换到 gimbal_yaw 帧
        Node(
            package='imu_transformer',
            executable='imu_transformer_node',
            name='imu_transformer_node',
            parameters=[{
                'target_frame': 'gimbal_yaw'  # 目标坐标系
            }],
            remappings=[
                ('/imu_in', '/livox/imu'),          # 输入话题：原始 IMU
                ('/imu_out', '/imu/transformed'),   # 输出话题：转换后的 IMU（仍无姿态）
            ]
        ),

        # 2. IMU 互补滤波器节点 (Madgwick)
        # 此节点接收转换后的 IMU 数据，并计算其姿态（四元数）
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick_node',
            parameters=[{
                # 基本参数
                'use_mag': False,                           # 不使用磁力计
                'world_frame': 'enu',                       # 世界坐标系：东-北-天
                'fixed_frame': 'imu_filter',                # 固定参考系
                'imu_frame': 'gimbal_yaw',                  # IMU 数据所在的坐标系
                
                # 滤波器参数
                'gain': 0.1,                                # 滤波器增益
                'zeta': 0.0,                                # 阻尼比
                'publish_tf': False,                        # 不发布 TF
                'publish_debug_topics': False,              # 不发布调试话题
                
                # 重力参数
                'gravity_constant': 9.81,                   # 重力常数
                'remove_gravity_acceleration': False,       # 不移除重力加速度
                
                # 初始化参数
                'orientation_stddev': 0.0,                  # 姿态协方差
                'angular_velocity_stddev': 0.02,            # 角速度协方差
                'linear_acceleration_stddev': 0.04,         # 加速度协方差
                
                # 静态检测参数
                'stat': {
                    'stationary_time': 5.0,                 # 静止检测时间
                    'acceleration_threshold': 0.1,          # 加速度阈值
                    'magnetic_threshold': 0.0               # 磁力计阈值（未使用）
                },
                
                # 频率参数（根据您的 IMU 频率调整）
                'frequency': 200.0,                         # 滤波器频率
                'constant_dt': 0.0                          # 固定时间步长（0表示自动计算）
            }],
            remappings=[
                ('/imu/data_raw', '/imu/transformed'),     # 输入：转换后无姿态的IMU
                ('/imu/data', '/imu/gimbal_yaw') # 输出：最终带姿态的IMU
            ]
        ),
    ])