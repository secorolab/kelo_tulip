from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()

    sl.declare_arg('use_sim_time', 'True')

    # config file
    config_file = sl.find('kelo_tulip', 'gz_controller', 'config')

    sl.node(
        'kelo_tulip',
        'kelo_gz_platform_controller',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': sl.arg('use_sim_time')}
        ]
    )

    return sl.launch_description()