import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Путь к URDF файлу
    urdf_file_path = os.path.join(
        get_package_share_directory('hoverboard_description'),
        'urdf',
        'my_robot.urdf')
    
    # Загрузить URDF как параметр
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()
    
    # Конфигурационные параметры
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
            
        # Публикация TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': use_sim_time
            }]
        ),
        
        # Опционально: joint_state_publisher, если у вас есть подвижные соединения
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        )
    ])
