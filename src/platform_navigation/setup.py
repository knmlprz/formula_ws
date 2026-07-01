from setuptools import find_packages, setup

package_name = 'platform_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lidar_gapfollower.launch.xml','launch/lidar_ittc.launch.xml' ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='damianek',
    maintainer_email='damste09@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gap_follower = platform_navigation.gap_follower:main',
            'ittc = platform_navigation.ittc:main',
            'cone_detector = platform_navigation.cone_detector:main',
            'cone_slam = platform_navigation.cone_slam:main',
            'centerline_planner = platform_navigation.centerline_planner:main',
            'pure_pursuit = platform_navigation.pure_pursuit:main',
        ],
    },
)
