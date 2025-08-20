from setuptools import setup

package_name = 'gpu_slam_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='GPU SLAM bot package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gpu_slam_node = gpu_slam_bot.gpu_slam_node:main',
            'autonomous_drive = gpu_slam_bot.autonomous_drive:main',  # add this line
        ],
    },
)
