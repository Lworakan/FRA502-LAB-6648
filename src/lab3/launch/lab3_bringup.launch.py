#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    eater_namespace_arg = DeclareLaunchArgument(
        'XXXX',
        default_value='eater_turtle',
        description='Namespace for the eater turtle (XXXX namespace requirement)'
    )
    
    killer_namespace_arg = DeclareLaunchArgument(
        'YYYY', 
        default_value='killer_turtle',
        description='Namespace for the killer turtle (YYYY namespace requirement)'
    )
    
    sampling_frequency_arg = DeclareLaunchArgument(
        'sampling_frequency',
        default_value='100.0',
        description='Sampling frequency for both nodes'
    )
    
    eater_turtle_ns = LaunchConfiguration('XXXX')
    killer_turtle_ns = LaunchConfiguration('YYYY')
    sampling_frequency = LaunchConfiguration('sampling_frequency')

    turtlesim_plus_node = Node(
        package='turtlesim_plus',
        executable='turtlesim_plus_node.py',
        name='turtlesim_plus',
        parameters=[{
            'enable_crazy_mode': False,
            'auto_spawn_crazy': False,
            'spawn_crazy_pizza': False,
            'spawn_crazy_turtle': False
        }],
        output='screen'
    )
    
    eater_node = Node(
        package='lab3',
        executable='eater.py',
        name='eater_node',
        namespace=eater_turtle_ns,
        parameters=[{
            'sampling_frequency': sampling_frequency
        }],
        output='screen'
    )
    
    killer_node = Node(
        package='lab3',
        executable='killer.py',
        name='killer_node',
        namespace=killer_turtle_ns,
        parameters=[{
            'sampling_frequency': sampling_frequency,
            'kill_turtle': eater_turtle_ns
        }],
        output='screen'
    )
    
    kill_turtle1 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/remove_turtle', 'turtlesim/srv/Kill', '{name: turtle1}'],
        shell=False,
        output='screen'
    )
    
    spawn_eater = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn_turtle', 'turtlesim/srv/Spawn', 
             '{x: 5.5, y: 5.5, theta: 0.0, name: eater_turtle}'],
        shell=False,
        output='screen'
    )
    
    spawn_killer = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn_turtle', 'turtlesim/srv/Spawn',
             '{x: 2.0, y: 2.0, theta: 0.0, name: killer_turtle}'],
        shell=False,
        output='screen'
    )

    pizza_on_click = Node(
        package='turtlesim_plus',
        executable='pizza_on_click.py',
        name='pizza_on_click',
        output='screen'
    )
    
    ld = LaunchDescription()
    
    ld.add_action(eater_namespace_arg)
    ld.add_action(killer_namespace_arg)
    ld.add_action(sampling_frequency_arg)
    
    ld.add_action(turtlesim_plus_node)
    ld.add_action(kill_turtle1)
    ld.add_action(spawn_eater)
    ld.add_action(spawn_killer)
    ld.add_action(eater_node)
    ld.add_action(killer_node)
    ld.add_action(pizza_on_click)

    return ld