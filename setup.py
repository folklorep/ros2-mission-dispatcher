from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'mission_dispatcher'

setup(
    name=package_name,
    version='0.1.0',
    # test/ 不是运行时包，排除掉，避免把测试代码装进 site-packages
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament 索引标记：colcon / ros2 pkg 通过这个文件判断包已安装
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # 让 `ros2 pkg prefix` 下能找到 package.xml（ros2 run / rosdep 需要）
        ('share/' + package_name, ['package.xml']),
        # 示例任务计划随包安装，方便用 $(ros2 pkg prefix)/share/... 直接引用
        (os.path.join('share', package_name, 'missions'), glob('missions/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yichen',
    maintainer_email='yichen@example.com',
    description='ROS2 移动机器人任务级调度与失败恢复节点',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_dispatcher = mission_dispatcher.node:main',
        ],
    },
)
