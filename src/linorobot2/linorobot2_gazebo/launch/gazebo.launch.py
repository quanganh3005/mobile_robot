import os
from launch import LaunchDescription
from launch.actions import (
   DeclareLaunchArgument, IncludeLaunchDescription,
   SetEnvironmentVariable, RegisterEventHandler
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
   LaunchConfiguration, PathJoinSubstitution, TextSubstitution,
   EnvironmentVariable, PythonExpression
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro




def generate_launch_description():
   use_sim_time = True


   description_share = get_package_share_directory('linorobot2_description')
   description_share_parent = os.path.dirname(description_share)
   gazebo_share = get_package_share_directory('linorobot2_gazebo')


   urdf_file = os.path.join(gazebo_share, 'urdf', 'linorobot2_gazebo.urdf.xacro')
   robot_description = xacro.process_file(urdf_file).toxml()


   ekf_config_path = PathJoinSubstitution(
       [FindPackageShare('linorobot2_base'), 'config', 'ekf.yaml']
   )


   # ─── Resolve world path ───────────────────────────────────────────────
   # Nếu world_name=empty.sdf → dùng built-in của Ignition
   # Nếu world_name=playground.sdf → ghép full path từ package
   worlds_dir = PathJoinSubstitution(
       [FindPackageShare('linorobot2_gazebo'), 'worlds',]
   )


   world_path = PathJoinSubstitution([
   worlds_dir,
   LaunchConfiguration('world_name')
])
   # ─── Định nghĩa node trước để dùng trong EventHandler ────────────────
   robot_state_publisher = Node(
       package='robot_state_publisher',
       executable='robot_state_publisher',
       name='robot_state_publisher',
       output='screen',
       parameters=[{
           'use_sim_time': use_sim_time,
           'robot_description': robot_description
       }]
   )


   spawn_robot = Node(
       package='ros_gz_sim',
       executable='create',
       name='urdf_spawner',
       output='screen',
       arguments=[
           '-name', 'linorobot2',
           '-topic', 'robot_description',
           '-x', LaunchConfiguration('spawn_x'),
           '-y', LaunchConfiguration('spawn_y'),
           '-z', LaunchConfiguration('spawn_z'),
           '-Y', LaunchConfiguration('spawn_yaw'),
       ]
   )


   return LaunchDescription([


       # ─── Fix mesh path cho Ignition ───────────────────────────────────
       SetEnvironmentVariable(
           name='IGN_GAZEBO_RESOURCE_PATH',
           value=[
               description_share_parent,
               TextSubstitution(text=':'),
               EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_value='')
           ]
       ),


       # ─── Launch arguments ─────────────────────────────────────────────
       DeclareLaunchArgument(
           name='world_name',
           default_value='empty.sdf',
           description=(
               'World file name. '
               'Mặc định "empty.sdf" chỉ hiện robot. '
               'Ví dụ: world_name:=playground.sdf'
           )
       ),
       DeclareLaunchArgument(name='odom_topic', default_value='/odom'),
       DeclareLaunchArgument(name='spawn_x', default_value='0.0'),
       DeclareLaunchArgument(name='spawn_y', default_value='0.0'),
       DeclareLaunchArgument(name='spawn_z', default_value='0.0'),
       DeclareLaunchArgument(name='spawn_yaw', default_value='0.0'),


       # ─── 1. Ignition Gazebo ───────────────────────────────────────────
       IncludeLaunchDescription(
           PythonLaunchDescriptionSource(
               PathJoinSubstitution(
                   [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
               )
           ),
           launch_arguments={
               'gz_args': [world_path, TextSubstitution(text=' -r')]
           }.items()
       ),


       # ─── 2. robot_state_publisher ─────────────────────────────────────
       robot_state_publisher,


       # ─── 3. Spawn robot sau khi robot_state_publisher start ───────────
       RegisterEventHandler(
           OnProcessStart(
               target_action=robot_state_publisher,
               on_start=[spawn_robot]
           )
       ),


       # ─── 4. Bridge Gazebo ↔ ROS 2 ────────────────────────────────────
       Node(
           package='ros_gz_bridge',
           executable='parameter_bridge',
           name='ros_gz_bridge',
           output='screen',
           parameters=[{'use_sim_time': use_sim_time}],
           arguments=[
               '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
               '/odom/unfiltered@nav_msgs/msg/Odometry@gz.msgs.Odometry',
               '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
               '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
               '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
               '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
               '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
           ]
       ),


       # ─── 5. EKF node ──────────────────────────────────────────────────
       Node(
           package='robot_localization',
           executable='ekf_node',
           name='ekf_filter_node',
           output='screen',
           parameters=[
               {'use_sim_time': use_sim_time},
               ekf_config_path
           ],
           remappings=[('odometry/filtered', LaunchConfiguration('odom_topic'))]
       ),


       # ─── 6. Command timeout ───────────────────────────────────────────
       Node(
           package='linorobot2_gazebo',
           executable='command_timeout.py',
           name='command_timeout'
       ),
   ])
