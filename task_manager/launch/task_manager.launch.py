import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_tm = get_package_share_directory('task_manager')
    cfg_dir = os.path.join(pkg_tm, 'config')
    tm_params_file = os.path.join(cfg_dir, 'task_manager_params.yaml')

    if not os.path.exists(tm_params_file):
        raise FileNotFoundError(f"Parameter file not found: {tm_params_file}")
    sim_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value=EnvironmentVariable('TASK_MANAGER_SIM', default_value='true'),
        description='Run in simulation mode (true/false)'
    )
    colors_arg = DeclareLaunchArgument(
        'required_colors',
        default_value=EnvironmentVariable('TASK_MANAGER_COLORS', default_value='[green,black,blue]'),
        description='List of cube colors in order'
    )
    auto_start_arg = DeclareLaunchArgument(
        'enable_auto_start',
        default_value='false',
        description='Enable auto-start of task sequence'
    )
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug logging and tools'
    )
    test_arg = DeclareLaunchArgument(
        'enable_integrated_test',
        default_value='false',
        description='Enable the integrated test script'
    )
    log_sim_mode = LogInfo(
        msg=['Starting Task Manager with sim_mode=', LaunchConfiguration('sim_mode')]
    )
    log_colors = LogInfo(
        msg=['Required colors: ', LaunchConfiguration('required_colors')]
    )
    log_autostart = LogInfo(
        msg=['Auto-start enabled: ', LaunchConfiguration('enable_auto_start')]
    )
    task_manager_node = Node(
        package='task_manager',
        executable='task_manager_node',
        name='task_manager',
        output='screen',
        parameters=[
            tm_params_file,
            {
                'sim_mode': LaunchConfiguration('sim_mode'),
                'required_colors': LaunchConfiguration('required_colors'),
                'enable_auto_start': LaunchConfiguration('enable_auto_start'),
            }
        ],
        emulate_tty=True,
    )
    integrated_test = Node(
        package='task_manager',
        executable='integrated_test',
        condition=LaunchConfigurationEquals('enable_integrated_test', 'true')
    )
    rqt_console_node = Node(
        package='rqt_console',
        executable='rqt_console',
        name='rqt_console',
        condition=IfCondition(LaunchConfiguration('debug')),
    )
    rqt_graph_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        name='rqt_graph',
        condition=IfCondition(LaunchConfiguration('debug')),
    )
    rqt_topic_node = Node(
        package='rqt_topic',
        executable='rqt_topic',
        name='rqt_topic',
        condition=IfCondition(LaunchConfiguration('debug')),
    )
    return LaunchDescription([
        sim_arg,# Declare simulation mode argument
        colors_arg,# Declare required colors argument
        auto_start_arg,# Declare auto-start argument
        debug_arg,# Declare debug argument
        test_arg,# Declare integrated test argument
        log_sim_mode,# Log simulation mode
        log_colors,# Log required colors
        log_autostart,# Log auto-start status
        task_manager_node,# Launch Task Manager node
        integrated_test,# Launch integrated test node if enabled
        rqt_console_node,# Launch rqt_console if debug is enabled
        rqt_graph_node,# Launch rqt_graph if debug is enabled
        rqt_topic_node,#
    ])