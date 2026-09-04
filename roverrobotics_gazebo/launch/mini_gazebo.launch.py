import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    urdf = os.path.join(get_package_share_directory(
        'roverrobotics_description'), 'urdf', 'mini.urdf')
    world = LaunchConfiguration('world')

    # NAV2
    map_file_name = LaunchConfiguration('map_file_name')

    robot_desc = ParameterValue(Command(['xacro ', urdf]),
                                       value_type=str)
    
    # ROSBAG
    rosbag_record = ExecuteProcess(
    cmd=[
        'ros2', 'bag', 'record',
        #'-o', 'narx_experimentos',  
        '/cmd_vel',
        '/cmd_vel_filtered',
        '/actor_robot/distance',
        '/odometry/wheels',
        '/user_coordinates_dist'
    ],
    output='screen'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='mi_mundo.sdf',
        description='World file to use in Gazebo')
    
    # Añadido NAV2
    declare_map_file_cmd = DeclareLaunchArgument(
    'map_file_name',
    default_value='/home/alvaro/robogait_TFG/src/roverrobotics_driver/maps/my_map',
    description='Map file to use with Nav2'
    )
    
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
            "-x", "0.2561",
            "-y", "-3.5679",
            "-z", "0.1",
            "-Y", "-3.0919",     # yaw
        ]
    )

    # NAV2
    # Include Nav2 navigation_launch.py from roverrobotics_driver
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('roverrobotics_driver'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_file_name': map_file_name,
        }.items()
    )
    # Nav2 with delay
    delayed_nav2 = TimerAction(
        period=10.0,  # s
        actions=[nav2_launch]
    )

    dynamixel = Node(
        package="tower_controller",
        executable="tower_controller",
        name="tower_controller_node",
        output="screen"
    )

    # Human distance detector node
    human_distance_yolo_node = Node(
        package="human_distance_yolo",
        executable="human_distance_yolo.py",
        name="human_distance_yolo_node",
        output="screen",
        parameters=[{
            "rgb_topic": "/world/mi_mundo/model/rover_mini/link/dynamixel_top_link/sensor/rgbd_camera/image",
            "depth_topic": "/world/mi_mundo/model/rover_mini/link/dynamixel_top_link/sensor/rgbd_camera/depth_image",
            "output_topic": "/user_coordinates_dist",
            "position_topic": "/user_coordinates",
            "model": "yolo11n-seg.pt",
            "device": "cuda"
        }]
    )
    
    waypoints_file = os.path.join(
        get_package_share_directory("waypoints_pub"),
        "resource", "my_path2.xml" 
    )

    # Actor waypoint_pub 
    actor_waypoints_pub_node = Node(
        package="waypoints_pub",
        executable="waypoints_publisher",
        name="waypoint_publisher_from_xml",
        output="screen",
        parameters=[{
            'xml_file': waypoints_file
        }]
    )

    actor_with_delay = TimerAction(period=15.0, actions=[actor_waypoints_pub_node]) #15 s
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",        
            #"/cmd_vel_filtered@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/odometry/wheels@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            
            #RGBD camera
            "/world/mi_mundo/model/rover_mini/link/dynamixel_top_link/sensor/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            "/world/mi_mundo/model/rover_mini/link/dynamixel_top_link/sensor/rgbd_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/world/mi_mundo/model/rover_mini/link/dynamixel_top_link/sensor/rgbd_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/world/mi_mundo/model/rover_mini/link/dynamixel_top_link/sensor/rgbd_camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked",

            # Joint Position Controller
            "/tower_yaw/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double"

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

    # Actor waypoint pub delayed
    #ld.add_action(actor_with_delay)

    # Human distance yolo
    ld.add_action(human_distance_yolo_node)

    # Dinamixel (tower_controller)
    ld.add_action(dynamixel)

    # NAV2
    #ld.add_action(declare_map_file_cmd)
    #ld.add_action(nav2_launch)
    #ld.add_action(delayed_nav2)

    # ROS2BAG
    #ld.add_action(rosbag_record)

    
    return ld
