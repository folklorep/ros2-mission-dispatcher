"""领域模型：任务步骤、执行结果、重试策略。

这个模块刻意不 import rclpy，原因有两个：

1. 可以在没有 ROS2 / Gazebo 的机器上直接用 pytest 验证调度语义；
2. 「哪些失败能重试、退避多久、什么算结束」属于业务规则，不应该和 ROS2
   的 action 状态码绑死。node.py 只负责把 Nav2 的 goal 状态翻译成这里的
   StepOutcome，规则本身留在纯 Python 层，方便讲清楚、也方便以后复用。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum


class StepType(str, Enum):
    """一个 mission 步骤的两种基本类型。

    枚举值只保留 NAVIGATE / WAIT 两个；YAML 里写成小写时，
    node.load_mission_plan 会统一转成大写再构造枚举。
    """

    NAVIGATE = "NAVIGATE"
    WAIT = "WAIT"


@dataclass(frozen=True)
class Step:
    """mission 中的一个步骤（不可变，创建后只读）。

    字段含义：
    - index: 从 0 开始、连续递增的步骤序号；数据库和续跑逻辑都用它对齐；
    - type: NAVIGATE 时用 x/y/yaw，WAIT 时用 seconds，另一组字段忽略；
    - x/y/yaw: 目标位姿，坐标系由 mission YAML 的 frame_id 指定（默认 map）；
    - seconds: WAIT 的停留时长；
    - label: 给人看的说明，会写进日志，方便对照实验现象。

    校验放在这里而不是 node.py：无论从 YAML 还是测试构造 Step，
    非法数据都应该在进入调度器之前就失败。
    """

    index: int
    type: StepType
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    seconds: float = 0.0
    label: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.type, StepType):
            raise ValueError(f"Step.type 必须是 StepType，收到 {self.type!r}")
        if self.index < 0:
            raise ValueError(f"Step.index 必须 >= 0，收到 {self.index}")
        if not math.isfinite(self.seconds) or self.seconds < 0:
            raise ValueError(f"Step.seconds 必须是有限且 >= 0 的数，收到 {self.seconds}")
        if self.type is StepType.NAVIGATE:
            # NaN / inf 传给 Nav2 会得到一个很难排查的 goal 拒绝，提前拦掉
            for name in ("x", "y", "yaw"):
                value = getattr(self, name)
                if not math.isfinite(value):
                    raise ValueError(f"NAVIGATE 步骤的 {name} 必须是有限数，收到 {value}")


class StepOutcome(str, Enum):
    """单个步骤一次尝试的结果。

    这是整个项目最关键的抽象：ROS2 的 GoalStatus 只有 SUCCEEDED / ABORTED /
    CANCELED 等少数状态，但调度器真正需要区分的是「这次失败值不值得再试」。
    因此 node.py 会把 ROS2 状态映射到这里，并额外区分出 REJECTED / TIMED_OUT。
    """

    SUCCEEDED = "SUCCEEDED"
    #: goal 根本没被接受：目标非法、Nav2 未就绪、BT 拒绝……属于前置条件不满足。
    REJECTED = "REJECTED"
    #: goal 被接受后执行失败：规划失败、控制器卡住、动态障碍导致 BT 返回 FAILURE。
    ABORTED = "ABORTED"
    #: 等结果超过 deadline。注意：此时 goal 可能仍在执行，必须先 cancel。
    TIMED_OUT = "TIMED_OUT"
    #: 用户通过 /mission/cancel 主动取消，或者 Nav2 侧报告 goal 被取消。
    CANCELED = "CANCELED"


class MissionState(str, Enum):
    """mission 整体状态。持久化到 missions.state 的就是这些字符串值。"""

    PENDING = "PENDING"
    RUNNING = "RUNNING"
    DONE = "DONE"
    FAILED = "FAILED"
    CANCELED = "CANCELED"


@dataclass(frozen=True)
class RetryPolicy:
    """步骤失败后的重试策略。

    attempts 的语义是「已经执行过的次数」：第一次执行时 attempts=1。
    max_attempts 包含第一次执行，所以 max_attempts=3 表示最多跑 3 次
    （1 次原始 + 2 次重试），而不是 1 + 3 次。

    四种失败为什么这么分类（这是面试里最值得讲的部分）：

    - REJECTED：goal 没有被接受，说明前置条件不满足（目标点非法、Nav2 没就绪、
      行为树拒绝）。同样的请求再发一次，只会得到同样的拒绝，重试没有意义；
      正确动作是停下来检查地图/配置/启动顺序。
    - ABORTED：goal 被接受后执行失败。常见原因是瞬态的：动态障碍、定位抖动、
      代价地图瞬时膨胀、控制器短暂卡住。退避后重发同样的目标有实际成功概率，
      所以可重试。
    - TIMED_OUT：deadline 到了还没拿到 result，但「没等到结果」不等于
      「goal 已经停止」。如果直接重发，旧 goal 可能还在控制机器人，两个 goal
      会争抢同一个底盘。因此必须先 cancel_goal_async()，确认取消动作已发出，
      再按 ABORTED 同样的退避策略重试。顺序错了比不重试更危险。
    - CANCELED：取消是用户意图（或外部抢占），不是故障。自动重试等于违抗
      用户命令，所以绝不重试。

    SUCCEEDED 当然不重试。未知结果保守处理为不重试，避免把系统带进
    不可预测的状态。
    """

    max_attempts: int = 3
    base_backoff_sec: float = 1.0
    #: 退避上限，避免任务失败后指数退避到几十分钟还没结束
    max_backoff_sec: float = 30.0

    def __post_init__(self) -> None:
        if self.max_attempts < 1:
            raise ValueError(f"max_attempts 必须 >= 1，收到 {self.max_attempts}")
        if not math.isfinite(self.base_backoff_sec) or self.base_backoff_sec < 0:
            raise ValueError(f"base_backoff_sec 必须有限且 >= 0，收到 {self.base_backoff_sec}")
        if not math.isfinite(self.max_backoff_sec) or self.max_backoff_sec < self.base_backoff_sec:
            raise ValueError(
                "max_backoff_sec 必须有限且不小于 base_backoff_sec，"
                f"收到 base={self.base_backoff_sec}, max={self.max_backoff_sec}"
            )

    def should_retry(self, outcome: StepOutcome, attempts: int) -> bool:
        """判断一次失败后是否还值得重试。

        Args:
            outcome: 本次尝试的结果。
            attempts: 已经执行过的次数（含本次），从 1 开始。

        Returns:
            True 表示应该退避后重试；False 表示应终止该步骤。
        """
        if outcome is StepOutcome.ABORTED or outcome is StepOutcome.TIMED_OUT:
            return attempts < self.max_attempts
        # SUCCEEDED / REJECTED / CANCELED 以及未知结果都不重试
        return False

    def backoff_for(self, attempts: int) -> float:
        """第 attempts 次失败后应该等多久（秒）。

        采用指数退避 base * 2^(attempts-1)，并截断到 max_backoff_sec。
        attempts <= 1 时按第一次失败处理，返回 base_backoff_sec。
        """
        attempt = max(1, int(attempts))
        delay = self.base_backoff_sec * (2 ** (attempt - 1))
        return min(delay, self.max_backoff_sec)
