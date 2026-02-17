from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Kritik Plugin Listesi (Standart)
    plugin_allowlist = [
        'sys_status', 'sys_time', 'command', 
        'setpoint_position', 'setpoint_velocity', 
        'imu', 'global_position', 'local_position', 
        'param', 'mission', 'safety_area'
    ]

    return LaunchDescription([
        # --- UAV0 (Lider - Sistem ID: 2) ---
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='uav0',
            output='screen',
            remappings=[
                ('state', 'mavros/state'),
                ('cmd/arming', 'mavros/cmd/arming'),
                ('set_mode', 'mavros/set_mode'),
                ('setpoint_position/local', 'mavros/setpoint_position/local')
            ],
            parameters=[{
                'fcu_url': 'udp://:14541@127.0.0.1:14559',
                # DİKKAT: IP yok, sadece port var. (Bind Mode)
                # UAV0, 14570 portunu açıp bekleyecek.
                'gcs_url': 'udp://:14572@', 
                'target_system_id': 2,
                'target_component_id': 1,
                'fcu_protocol': 'v2.0',
                'plugin_allowlist': plugin_allowlist
            }]
        ),

        # --- UAV1 (Takipçi - Sistem ID: 3) ---
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='uav1',
            output='screen',
            remappings=[
                ('state', 'mavros/state'),
                ('cmd/arming', 'mavros/cmd/arming'),
                ('set_mode', 'mavros/set_mode'),
                ('setpoint_position/local', 'mavros/setpoint_position/local')
            ],
            parameters=[{
                'fcu_url': 'udp://:14542@127.0.0.1:14561',
                # DİKKAT: Çakışmasın diye buna 14571 dedik.
                # UAV1, 14571 portunu açıp bekleyecek.
                'gcs_url': 'udp://:14571@',
                'target_system_id': 3,
                'target_component_id': 1,
                'fcu_protocol': 'v2.0',
                'plugin_allowlist': plugin_allowlist
            }]
        )
    ])