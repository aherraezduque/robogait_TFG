import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# For ros2_control controllers delayed startup
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.actions import TimerAction

def generate_launch_description():
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf = os.path.join(get_package_share_directory(
        'roverrobotics_description'), 'urdf', 'mini.urdf')
    world = LaunchConfiguration('world')

    robot_desc = ParameterValue(Command(['xacro ', urdf]),
                                       value_type=str)
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='mi_mundo.sdf',
        description='World file to use in Gazebo')
    
    gz_world_arg = PathJoinSubstitution([
        get_package_share_directory('roverrobotics_gazebo'), 'worlds', world])

    # Include the gz sim launch file  
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : gz_world_arg 
        }.items()
    )
    
    # Spawn Rover Robot
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "rover_mini",
            "-allow_renaming", "true",
            "-x", "0.1",
            "-y", "3.0",
            "-z", "0.1",
        ]
    )
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            #"/odometry/wheels@nav_msgs/msg/Odometry@ignition.msgs.Odometry", #no diff drive plugin ahora va con ros2_control
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',

            ## Los topic que comienzan por /world/mi_mundo son lo que genera ignition al no indicar valor al topic en 
            ## el tag sensor

            #RGB Camera
            #"/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image",
            #"/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo"

            #"/world/mi_mundo/model/rover_mini/link/base_link/sensor/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
            #"/world/mi_mundo/model/rover_mini/link/base_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",

            #Depth camera
            #"/world/mi_mundo/model/rover_mini/link/base_link/sensor/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            #"/world/mi_mundo/model/rover_mini/link/base_link/sensor/depth_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image",
            #"/world/mi_mundo/model/rover_mini/link/base_link/sensor/depth_camera/depth_image/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked"


            #RGBD camera
            "/world/mi_mundo/model/rover_mini/link/base_link/sensor/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            "/world/mi_mundo/model/rover_mini/link/base_link/sensor/rgbd_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/world/mi_mundo/model/rover_mini/link/base_link/sensor/rgbd_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/world/mi_mundo/model/rover_mini/link/base_link/sensor/rgbd_camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked"
        ],
    )


    # Robot state publisher
    params = {'use_sim_time': use_sim_time, 'robot_description': robot_desc}
    start_robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params],
            arguments=[])

    joint_broad_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],)

    diff_drive_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],)
    
    delayed_joint_broad = TimerAction(
        period=10.0,
        actions=[joint_broad_spawner]
    )

    delayed_diff_drive = TimerAction(
        period=10.0,
        actions=[diff_drive_spawner]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)

    # Launch Gazebo
    ld.add_action(gz_sim)
    ld.add_action(gz_spawn_entity)
    ld.add_action(gz_ros2_bridge)


    # Launch Robot State Publisher
    ld.add_action(start_robot_state_publisher_cmd)

    # Launch delayed joint state broadcaster ROS2_CONTROL  
    ld.add_action(delayed_joint_broad)

    # Launch delayed diff drive controller ROS2_CONTROL  
    ld.add_action(delayed_diff_drive)

    return ld
