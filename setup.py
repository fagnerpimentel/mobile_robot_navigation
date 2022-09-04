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
        ('share/' + package_name + '/resource/models/r2d2/', ['resource/models/r2d2/model.config']),
        ('share/' + package_name + '/resource/models/r2d2/', ['resource/models/r2d2/model.sdf']),
        ('share/' + package_name + '/resource/models/simple_room/', ['resource/models/simple_room/model.config']),
        ('share/' + package_name + '/resource/models/simple_room/', ['resource/models/simple_room/model.sdf']),
        ('share/' + package_name + '/resource/models/wood_square/', ['resource/models/wood_square/model.config']),
        ('share/' + package_name + '/resource/models/wood_square/', ['resource/models/wood_square/model.sdf']),
        ('share/' + package_name + '/resource/worlds/', ['resource/worlds/r2d2.world']),
        ('share/' + package_name + '/resource/worlds/', ['resource/worlds/r2d2_follow_wall.world'])
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
        ],
    },
)
