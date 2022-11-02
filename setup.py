from setuptools import setup

package_name = 'mobile_robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/gazebo.launch.yaml']),
        ('share/' + package_name, ['launch/mapping.launch.yaml']),
        ('share/' + package_name, ['launch/navigation.launch.yaml']),
        ('share/' + package_name + '/resource/', ['resource/r2d2.urdf']),
        ('share/' + package_name + '/resource/rviz/', ['resource/rviz/mapping.rviz']),
        ('share/' + package_name + '/resource/rviz/', ['resource/rviz/navigation.rviz']),
        ('share/' + package_name + '/resource/models/box/', ['resource/models/box/model.config']),
        ('share/' + package_name + '/resource/models/box/', ['resource/models/box/model.sdf']),
        ('share/' + package_name + '/resource/models/r2d2/', ['resource/models/r2d2/model.config']),
        ('share/' + package_name + '/resource/models/r2d2/', ['resource/models/r2d2/model.sdf']),
        ('share/' + package_name + '/resource/models/simple_room/', ['resource/models/simple_room/model.config']),
        ('share/' + package_name + '/resource/models/simple_room/', ['resource/models/simple_room/model.sdf']),
        ('share/' + package_name + '/resource/models/wood_square/', ['resource/models/wood_square/model.config']),
        ('share/' + package_name + '/resource/models/wood_square/', ['resource/models/wood_square/model.sdf']),
        ('share/' + package_name + '/resource/worlds/', ['resource/worlds/r2d2.world']),
        ('share/' + package_name + '/resource/worlds/', ['resource/worlds/r2d2_follow_wall.world']),
        ('share/' + package_name + '/resource/worlds/', ['resource/worlds/r2d2_obstacle_avoidance.world']),
        ('share/' + package_name + '/resource/worlds/', ['resource/worlds/r2d2_obstacles_fixed.world'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fagner',
    maintainer_email='fagnerpimentel@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aula_1 = mobile_robot_navigation.aula_1:main',
            'aula_2 = mobile_robot_navigation.aula_2:main',
            'aula_3 = mobile_robot_navigation.aula_3:main',
            'aula_4 = mobile_robot_navigation.aula_4:main'
        ],
    },
)
