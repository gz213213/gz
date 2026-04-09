import os

import launch
import launch_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


# 解析启动参数里常见的布尔字符串。
def _parse_bool(text: str) -> bool:
    return text.strip().lower() in ('1', 'true', 'yes', 'on')


# 将启动参数安全转换为浮点数，失败时回退默认值。
def _safe_float(text: str, default: float) -> float:
    try:
        return float(text)
    except (TypeError, ValueError):
        return default


# 从评测目标文件读取出生点（约定使用 G01）。
def _load_g01_pose(goals_yaml_path: str):
    if not goals_yaml_path:
        return None
    if not os.path.exists(goals_yaml_path):
        return None

    try:
        with open(goals_yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        goals = data.get('benchmark_goals', {}).get('points', [])
        for goal in goals:
            if str(goal.get('goal_id', '')).strip() == 'G01':
                return (
                    float(goal.get('x', 0.0)),
                    float(goal.get('y', 0.0)),
                    float(goal.get('yaw_rad', 0.0)),
                )
    except Exception:
        return None

    return None


# 构建动态启动动作：
# 1) 先决定出生位姿（自动读取 G01 或手动参数兜底）；
# 2) 再在 Gazebo 中生成机器人；
# 3) 最后按顺序拉起 ros2_control 控制器。
def _spawn_with_controllers(context):
    robot_name_in_model = 'fishbot'
    spawn_x = _safe_float(
        launch.substitutions.LaunchConfiguration('spawn_x').perform(context), 0.0)
    spawn_y = _safe_float(
        launch.substitutions.LaunchConfiguration('spawn_y').perform(context), 0.0)
    spawn_z = _safe_float(
        launch.substitutions.LaunchConfiguration('spawn_z').perform(context), 0.05)
    spawn_yaw = _safe_float(
        launch.substitutions.LaunchConfiguration('spawn_yaw').perform(context), 0.0)

    auto_spawn = _parse_bool(
        launch.substitutions.LaunchConfiguration('auto_spawn_from_goals').perform(context))
    goals_yaml_path = launch.substitutions.LaunchConfiguration('goals_yaml').perform(context).strip()

    log_msgs = []
    if auto_spawn:
        g01_pose = _load_g01_pose(goals_yaml_path)
        if g01_pose is not None:
            spawn_x, spawn_y, spawn_yaw = g01_pose
            log_msgs.append(
                launch.actions.LogInfo(
                    msg=(
                        f'[gazebo_sim] 已从 G01 自动设置出生点: '
                        f'x={spawn_x:.3f}, y={spawn_y:.3f}, yaw={spawn_yaw:.3f} '
                        f'(goals_yaml={goals_yaml_path})'
                    )))
        else:
            log_msgs.append(
                launch.actions.LogInfo(
                    msg=(
                        f'[gazebo_sim] 无法从 goals_yaml={goals_yaml_path} 读取 G01，'
                        f'回退到手动 spawn_x/y/yaw'
                    )))

    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', robot_name_in_model,
            '-x', f'{spawn_x:.3f}',
            '-y', f'{spawn_y:.3f}',
            '-z', f'{spawn_z:.3f}',
            '-Y', f'{spawn_yaw:.3f}',
        ])

    # 用 controller_manager/spawner 显式激活控制器，避免“只配置未激活”导致机器人不动。
    load_joint_state_controller = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'fishbot_joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120'
        ],
        output='screen'
    )

    load_fishbot_diff_drive_controller = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'fishbot_diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120'
        ],
        output='screen'
    )

    return log_msgs + [
        spawn_entity_node,
        # 事件动作：生成机器人后加载 joint_state_broadcaster。
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[load_joint_state_controller],)
            ),
        # 事件动作：上一步完成后再加载差速控制器。
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_fishbot_diff_drive_controller],)
            ),
    ]


def generate_launch_description():
    # 获取默认路径
    urdf_tutorial_path = get_package_share_directory('fishbot_description')
    default_model_path = urdf_tutorial_path + '/urdf/fishbot/fishbot.urdf.xacro'
    default_world_path = urdf_tutorial_path + '/world/custom_room.world'
    # 声明 launch 参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF 的绝对路径')
    action_declare_arg_world_path = launch.actions.DeclareLaunchArgument(
        name='world', default_value=str(default_world_path),
        description='Gazebo world 的绝对路径')
    action_declare_arg_spawn_x = launch.actions.DeclareLaunchArgument(
        name='spawn_x', default_value='0.0',
        description='机器人初始 x 坐标（world）')
    action_declare_arg_spawn_y = launch.actions.DeclareLaunchArgument(
        name='spawn_y', default_value='0.0',
        description='机器人初始 y 坐标（world）')
    action_declare_arg_spawn_z = launch.actions.DeclareLaunchArgument(
        name='spawn_z', default_value='0.05',
        description='机器人初始 z 坐标（world）')
    action_declare_arg_spawn_yaw = launch.actions.DeclareLaunchArgument(
        name='spawn_yaw', default_value='0.0',
        description='机器人初始偏航角 yaw（rad）')
    # 评测模式：从 goals_yaml 的 G01 自动取出生点，避免与障碍重叠导致开局翻车。
    action_declare_arg_goals_yaml = launch.actions.DeclareLaunchArgument(
        name='goals_yaml', default_value='',
        description='评测目标文件路径；启用 auto_spawn_from_goals 时自动读取 G01 作为出生点')
    action_declare_arg_auto_spawn = launch.actions.DeclareLaunchArgument(
        name='auto_spawn_from_goals', default_value='true',
        description='若为 true 且 goals_yaml 有效，则用 G01 覆盖 spawn_x/spawn_y/spawn_yaw')
    # 通过 xacro 生成 robot_description 参数。
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)
  	
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # 通过 IncludeLaunchDescription 拉起 Gazebo 主 launch。
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
        # 将 world 参数传给 Gazebo 主 launch。
        launch_arguments=[
            ('world', launch.substitutions.LaunchConfiguration('world')),
            ('verbose', 'true')
        ]
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_declare_arg_world_path,
        action_declare_arg_spawn_x,
        action_declare_arg_spawn_y,
        action_declare_arg_spawn_z,
        action_declare_arg_spawn_yaw,
        action_declare_arg_goals_yaml,
        action_declare_arg_auto_spawn,
        robot_state_publisher_node,
        launch_gazebo,
        # 生成机器人与加载控制器依赖运行时参数，使用 OpaqueFunction 动态展开。
        launch.actions.OpaqueFunction(function=_spawn_with_controllers),
    ])
