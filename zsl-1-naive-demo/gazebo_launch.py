import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取包路径
    pkg_share = get_package_share_directory('edu_description')

    # 2. 定位 URDF 文件
    urdf_file = os.path.join(pkg_share, 'urdf', 'edu.urdf')

    # 3. 【关键】设置环境变量
    # 3.1 让 Ignition Gazebo 能够找到您的网格文件
    # 将您的 ROS 包 share 目录添加到资源路径中
    resource_path = os.path.join(pkg_share, '..')  # 指向 ~/agibot_ws/install/share
    ign_resource_path = f"{resource_path}:/usr/share/gazebo-11/models"

    # 3.2 强制使用软件渲染（解决 OpenGL 3.3 不支持的问题）
    # 如果您的显卡驱动不支持 OpenGL 3.3，此变量将使用 CPU 进行渲染
    set_env_variables = [
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=ign_resource_path),
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=ign_resource_path),
        SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1'),
        #SetEnvironmentVariable(name='QT_QUICK_BACKEND', value='software'),
        #SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='xcb'),
        # 如果上述仍不行，可以尝试禁用 DRI
        # SetEnvironmentVariable(name='LIBGL_DRI3_DISABLE', value='1'),
    ]

    # 4. 启动 Ignition Gazebo（空世界）
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py'
            )
        ]),
        launch_arguments={'ign_args': 'empty.sdf'}.items()
    )

    # 5. 将 URDF 转换为 SDF 格式（Ignition Gazebo 需要 SDF）
    sdf_file = os.path.join(pkg_share, 'urdf', 'edu.sdf')
    urdf_to_sdf = ExecuteProcess(
        cmd=['ign', 'sdf', '-p', urdf_file, '>', sdf_file],
        shell=True,
        output='screen'
    )

    # 6. 在 Ignition Gazebo 中生成机器人模型
    spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-file', sdf_file,
            '-name', 'edu_dog',
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )

    # 7. 机器人状态发布（供 RViz 使用）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_file]
    )

    # 8. 桥接节点
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            # 格式: /topic_name@ros_type@ign_type

            # 1. 关节状态桥接 (核心)
            # ROS话题: /joint_states
            # 类型: sensor_msgs/msg/JointState
            # Ign类型: ignition.msgs.Model (注意：JointStatePublisher 在 Ign 端发出的是 Model 类型消息)
            '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',

            # 2. 时钟桥接 (必须，否则 RViz 里模型会乱跳)
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',

            # 3. TF 桥接 (可选，如果以后需要在 Ign 里看 TF)
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
        ],
        output='screen'
    )


    # 9. 按顺序启动所有动作
    return LaunchDescription([
        # 先设置环境变量
        *set_env_variables,
        # 再启动 Gazebo
        ign_gazebo,
        # 然后转换模型格式
        urdf_to_sdf,
        # 最后启动其他节点
        robot_state_publisher,
        spawn_entity,
        bridge,
    ])

