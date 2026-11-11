"""mission_dispatcher：ROS2 上的任务级调度与失败恢复。

分层约定：
- models.py / store.py：纯 Python，不 import rclpy，可离线 pytest；
- node.py：rclpy 薄适配层，只负责 ROS2 通信和把结果翻译成领域模型。
"""

__version__ = "0.1.0"
