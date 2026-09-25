from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
   pkg = 'fs1'
  
   return LaunchDescription([

       Node(
           package=pkg,
           executable='gripper_client',
           name='gripper_client',
       ),
          
       Node(
           package=pkg,
           executable='gripper_control',
           name='gripper_control',
           output='screen',
       ),
          
       Node(
           package=pkg,
           executable='joints_control',
           name='joints_control',
           output='screen',
       ),
        Node(
           package=pkg,
           executable='kinova_api',
           name='kinova_api',
           output='screen',
       ),
       Node(
           package=pkg,
           executable='my_node',
           name='my_node',
           output='screen',
       ),
   ])
