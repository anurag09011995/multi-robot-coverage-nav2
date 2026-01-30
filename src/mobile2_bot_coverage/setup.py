from setuptools import find_packages, setup

package_name = 'mobile2_bot_coverage'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anuragpa',
    maintainer_email='anurag.532f@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_grid_server = mobile2_bot_coverage.coverage_grid_server:main',
            'coverage_dispatcher = mobile2_bot_coverage.coverage_dispatcher:main',
        ],
    },
)
