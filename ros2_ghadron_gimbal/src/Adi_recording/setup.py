from setuptools import setup

package_name = 'adi_recording'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dtc-mrsd',
    maintainer_email='situjet@gmail.com',
    description='RTSP stream publisher with MCAP recording for ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtsp_node = adi_recording.rtsp_ros2:main'
        ],
    },
) 