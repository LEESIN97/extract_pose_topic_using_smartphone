from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pdr_tools',
            executable='pose_to_odom_path',   # ← 파이썬 노드 파일에 맞는 executable 이름
            name='pose_to_odom_path',
            output='screen',
            parameters=[{
                # --- 입력 토픽(절대경로) ---
                'pose_topic': '/mobile_sensor/pose_stamped',          # PoseStamped로 고정
                'stamp_mode': 'image',                                # /odom을 이미지 스탬프로 동기
                'stamp_image_topic': '/camera/image_raw/compressed',  # Node.js가 퍼블리시하는 곳

                # --- 좌표/프레임 ---
                'odom_frame': 'odom',
                'base_frame': 'base_link',

                # --- Unity 퍼블리시 ---
                'publish_unity_pose': True,
                'publish_unity_path': True,
                'unity_pose_topic': '/unity/pose',
                'unity_path_topic': '/unity/path',
                'path_capacity': 4000,
                'push_every_pose': True,
                'min_path_interval_sec': 0.04,
                'min_path_distance': 0.03,
                'pose_pub_hz': 5.0,

                # --- /odom & TF ---
                'publish_odom': True,
                'publish_tf': True,
                'odom_topic': '/odom',

                # --- 스케일/필터 ---
                'input_scale_m_per_unit': 1.0,
                'unity_scale': 1.0,
                'position_lowpass_alpha': 0.25,
                'orientation_slerp_alpha': 0.20,
                'clamp_y_to_zero': False,

                # --- 로깅 ---
                'log_interval_sec': 1.0,
                'log_pose_in': True,
                'log_odom_out': True,
                'log_tf_out': False,
            }],
            # 필요하면 remappings=[] 로도 절대/상대 토픽을 조정할 수 있어요.
            # remappings=[('/camera/image_raw/compressed','/your/cam/topic')]
        ),
    ])
