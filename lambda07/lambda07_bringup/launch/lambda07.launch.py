from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    interface_id = LaunchConfiguration('interface_id')
    interface_sn = LaunchConfiguration('interface_sn')
    effector_mass = LaunchConfiguration('effector_mass')
    use_rviz = LaunchConfiguration('use_rviz')
    use_inertia_broadcaster = LaunchConfiguration('use_inertia_broadcaster')
    use_effort_controller = LaunchConfiguration('use_effort_controller')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('lambda07_bringup'),
            'urdf',
            'lambda07.config.xacro',
        ]),
        ' robot_id:=', robot_id,
        ' use_fake_hardware:=', use_fake_hardware,
        ' interface_id:=', interface_id,
        ' interface_sn:=', interface_sn,
        ' effector_mass:=', effector_mass,
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    controllers = PathJoinSubstitution([
        FindPackageShare('lambda07_bringup'),
        'config',
        'lambda07_controllers.yaml',
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('lambda07_bringup'),
        'rviz',
        'lambda07.rviz',
    ])

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='lambda07',
        parameters=[robot_description, controllers],
        output={'stdout': 'screen', 'stderr': 'screen'},
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='lambda07',
        output='screen',
        parameters=[robot_description],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lambda07_world_tf',
        output='log',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'lambda07_base'],
    )

    spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            namespace='lambda07',
            arguments=[controller, '--controller-manager', '/lambda07/controller_manager'],
        )
        for controller in [
            'joint_state_broadcaster',
            'fd_ee_broadcaster',
            'fd_clutch_broadcaster',
        ]
    ]
    spawners.append(
        Node(
            package='controller_manager',
            executable='spawner',
            namespace='lambda07',
            arguments=['fd_controller', '--controller-manager', '/lambda07/controller_manager'],
            condition=IfCondition(use_effort_controller),
        )
    )
    spawners.append(
        Node(
            package='controller_manager',
            executable='spawner',
            namespace='lambda07',
            arguments=['fd_inertia_broadcaster', '--controller-manager', '/lambda07/controller_manager'],
            condition=IfCondition(use_inertia_broadcaster),
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='lambda07'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('interface_id', default_value='-1'),
        DeclareLaunchArgument('interface_sn', default_value='-1'),
        DeclareLaunchArgument('effector_mass', default_value='-1.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_inertia_broadcaster', default_value='false'),
        DeclareLaunchArgument('use_effort_controller', default_value='false'),
        controller_manager,
        robot_state_publisher,
        static_tf,
        rviz,
    ] + spawners)

