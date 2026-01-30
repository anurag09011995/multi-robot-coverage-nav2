from setuptools import find_packages, setup
import glob

package_name = 'mobile2_bot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Resource and package.xml
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        # URDF and Xacro files
        ('share/' + package_name + '/urdf', glob.glob('urdf/*')),
        # Meshes
        ('share/' + package_name + '/meshes', glob.glob('meshes/*')),
        # Config files
        ('share/' + package_name + '/config', glob.glob('config/*')),
        # Worlds
        ('share/' + package_name + '/worlds', glob.glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anuragpa',
    maintainer_email='anurag.532f@gmail.com',
    description='Mobile bot description package fixed for ROS2/Gazebo',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'bridge_node = mobile2_bot_description.bridge_node:main',
            # 'tcp_ros_bridge = mobile2_bot_description.tcp_ros_bridge:main',
            'scan_stamper = mobile2_bot_description.scan_stamper:main',
            # 'tf_filter = mobile2_bot_description.tf_filter:main',
        ],
    },
)

