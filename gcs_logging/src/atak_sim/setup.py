from setuptools import setup

package_name = 'atak_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'atak_main',
        'atak_rosless_sim'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'std_msgs',
        'cv_bridge',
        'numpy',
        'Pillow',
        'geopy',
        'staticmap',
        'opencv-python'
    ],
    zip_safe=True,
    maintainer='dtc-mrsd',
    maintainer_email='506346013@qq.com',
    description='ATAK simulation node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'atak_sim = atak_main:main',
            'atak_rosless = atak_rosless_sim:main',
        ],
    },
)

