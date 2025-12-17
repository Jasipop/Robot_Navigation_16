import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot_name_in_model = 'yahboom_car'
    
    urdf_model_path = '/home/yahboom/yahboomcar_ws/src/yahboomcar_description/urdf/MicroROS.urdf'
    
    world_path = '/home/yahboom/maze/maze.world'

    meshes_parent_dir = '/home/yahboom/yahboomcar_ws/src'

    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path =  os.environ['GAZEBO_MODEL_PATH'] + ':' + meshes_parent_dir
    else:
        model_path =  meshes_parent_dir


    with open(urdf_model_path, 'r') as infp:
        robot_desc = infp.read()

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    set_gazebo_model_path_cmd = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', 
        value=model_path
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time, 
            'robot_description': robot_desc
        }]
    )

    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen'
    )

    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name_in_model,
            '-x', '0.5', '-y', '0.5', '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,   
        set_gazebo_model_path_cmd,  
        node_robot_state_publisher, 
        start_gazebo_cmd,           
        spawn_entity_cmd            
    ])