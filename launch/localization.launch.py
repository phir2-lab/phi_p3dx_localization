from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    """
    Launch file para o nó de localização com visualização em RViz.
    
    Inicia:
    1. O nó de localização simples
    2. RViz com configuração adequada para visualizar partículas e pose estimada
    3. Transformações (map -> odom)
    """
    
    # Encontra o pacote
    pkg_share = FindPackageShare('phi_p3dx_localization')
    
    # RViz config
    rviz_config = PathJoinSubstitution([
        pkg_share,
        'rviz',
        'localization.rviz'
    ])

    # Nó de localização
    localization_node = Node(
        package='phi_p3dx_localization',
        executable='localization_node',
        name='localization_node',
        output='screen',
        parameters=[
            {'num_particles': 100},
            {'initial_x': 0.0},
            {'initial_y': 0.0},
            {'initial_theta': 0.0},
            {'use_perturbation': False},
            {'publish_freq': 10.0},
            {'frame_id': 'map'},
        ]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    # TF estático map -> odom
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    return LaunchDescription([
        localization_node,
        tf_node,
        rviz_node
    ])
