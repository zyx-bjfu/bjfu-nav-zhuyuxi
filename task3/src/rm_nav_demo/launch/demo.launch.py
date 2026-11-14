import launch
import launch_ros.actions

def generate_launch_description():
    # 底盘节点
    chassis_node = launch_ros.actions.Node(
        package='rm_nav_demo',
        executable='chassis_node',
        name='chassis_Node',
        output='screen'
    )

    # 裁判节点
    referee_node = launch_ros.actions.Node(
        package='rm_nav_demo',
        executable='referee_node',
        name='referee_Node',
        output='screen'
    )

    # RViz2 可视化
    rviz2 = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return launch.LaunchDescription([
        chassis_node,
        referee_node,
        rviz2
    ])