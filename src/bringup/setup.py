from setuptools import find_packages, setup

package_name = 'bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/launch.py']),
        ('share/' + package_name, ['fuzz.py']),
        ('share/' + package_name + '/params', ['params/params.py']),
        ('share/' + package_name + '/plugins', ['plugins/coverage.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dirge',
    maintainer_email='2291303762@qq.com',
    description='fuzzer, 启动!',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_pose_publisher = bringup.goal_pose_publisher:main',
            'init_pose_publisher = bringup.init_pose_publisher:main'
        ],
    },
)
