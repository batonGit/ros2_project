from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # --- Диагностическое сообщение ---
    print("\n>>> ЗАПУСК ИНДИВИДУАЛЬНОГО LAUNCH-ФАЙЛА (webcam.launch.py) <<<\n")

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='webcam_node',
            namespace='camera',
            parameters=[
                # --- Убедитесь, что эти параметры верны для вашей камеры ---
                {'video_device': '/dev/video0'},          
                {'image_width': 640},                    
                {'image_height': 480},                   
                {'pixel_format': 'yuyv'},
                # ---

                {'camera_name': 'my_webcam'},
                {'io_method': 'mmap'},
            ],
            remappings=[
                # Переименовываем топик для совместимости с yolo_node.py
                ('image_raw', 'rgb/image_raw'), 
            ]
        )
    ])