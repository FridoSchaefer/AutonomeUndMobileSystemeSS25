# launch/autorace.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Diese Launch-Datei startet alle Nodes, die für die Autorace Challenge benötigt werden.
    """
    return LaunchDescription([
        Node(
            package='turtlebot3_test1',         
            executable='line_detector_node', 
            name='line_detector_node',
            output='screen',
            parameters=[
                
                {'calibration_y': 225},
                {'hsv.white.l_min': 90},
                {'hsv.white.l_max': 180},
                {'hsv.yellow.h_min': 15},
                {'hsv.yellow.h_max': 35},
                {'hsv.yellow.l_min': 60},
                {'hsv.yellow.l_max': 120},
                {'hsv.yellow.s_min': 200},
                {'hsv.yellow.s_max': 255},
            ]
        ),
        
        Node(
            package='turtlebot3_test1',         
            executable='line_follower_node', 
            name='line_follower_node',
            output='screen',
            parameters=[
                {'max_linear_speed': 0.06},
                {'max_angular_speed': 1.0},
                {'line_p_gain': 2.0},
                {'white_line_optimum': 280},
                {'yellow_line_optimum': 35},
            ]
        ),
        
        Node(
            package='turtlebot3_test1',        
            executable='sign_detector_node', 
            name='sign_detector_node',
            output='screen'
        ),
    ])