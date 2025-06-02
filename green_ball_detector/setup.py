from setuptools import setup, find_packages

package_name = 'green_ball_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yerbiff',
    maintainer_email='yerbiff@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_ball_detector = green_ball_detector.cam_ball_detector:main',
            'lidar_ball_detector = green_ball_detector.lidar_ball_detector:main',
            'gt_ball = green_ball_detector.gt_ball:main',
        ],
    },

)
