from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("custom_movit"),
            "launch",
            "gazebo.launch.py"         
        )
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("fairino_controller"),
            "launch",
            "controller.launch.py"         
        ),
        launch_arguments={"is_sim" : "True"}.items()
    )
    
    moveit = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("custom_movit"),
            "launch",
            "move_group.launch.py"         
        ),
        launch_arguments={"is_sim" : "True"}.items()
    )
    moveit_rviz = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("custom_movit"),
            "launch",
            "moveit_rviz.launch.py"         
        ),
        launch_arguments={"is_sim" : "True"}.items()
    )
    
       
    return LaunchDescription([
        gazebo,
        controller,
        moveit,
        moveit_rviz,
    ])