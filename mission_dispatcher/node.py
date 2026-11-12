#!/usr/bin/env python3
"""ROS2 适配层：把 Nav2 的 NavigateToPose action 包装成任务级调度器。

这一层只做三件事，尽量保持「薄」：

1. 把 YAML 计划编译成 models.Step（load_mission_plan）；
2. 把 ROS2 的 goal 状态翻译成 models.StepOutcome（_map_goal_status）；
3. 把进度写进 store.MissionStore，把当前状态发到 /mission/status。

「哪些失败该重试、退避多久、什么时候算完成」全部在 models.RetryPolicy 和
store.MissionStore 里，因此没有 ROS2 环境也能用 pytest 验证核心语义。
node.py 里宁可多写注释和显式分支，也不做花哨抽象：这个文件要能被静态读懂。
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mission_dispatcher.models import (
    MissionState,
    RetryPolicy,
    Step,
    StepOutcome,
    StepType,
)
from mission_dispatcher.store import (
    STEP_CANCELED,
    STEP_FAILED,
    STEP_RUNNING,
    STEP_SUCCEEDED,
    MissionStore,
)

STATUS_TOPIC = "/mission/status"
CANCEL_SERVICE = "/mission/cancel"
NAVIGATE_ACTION = "navigate_to_pose"

#: 轮询 future / 检查取消标志的间隔。太小会空转，太大会让取消响应变慢。
_POLL_INTERVAL_SEC = 0.05
#: 等待 Nav2 action server 出现的上限；超过就认为前置条件不满足。
_ACTION_SERVER_WAIT_SEC = 30.0
#: 发出 cancel 后等待 Nav2 响应的上限。cancel 本身是异步的，这里只是拿回执。
_CANCEL_RESPONSE_WAIT_SEC = 1.0


def load_mission_plan(path: str) -> tuple[str, str, list[Step]]:
    """读取并校验 mission YAML，返回 (mission 名称, 坐标系, 步骤列表)。

    YAML 结构见 missions/demo_3_steps.yaml。这里做严格校验而不是「尽量容错」：
    任务计划错一个字段，机器人可能跑到完全错误的位置，早失败比晚失败便宜。
    """
    mission_path = Path(path)
    with mission_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)

    if not isinstance(data, dict):
        raise ValueError("mission 文件顶层必须是 mapping（包含 name/frame_id/steps）")

    raw_steps = data.get("steps")
    if not isinstance(raw_steps, list) or not raw_steps:
        raise ValueError("mission 文件必须包含非空的 steps 列表")

    steps: list[Step] = []
    for index, raw in enumerate(raw_steps):
        if not isinstance(raw, dict):
            raise ValueError(f"第 {index} 个步骤必须是 mapping，收到 {type(raw).__name__}")

        raw_type = str(raw.get("type", "")).strip().upper()
        try:
            step_type = StepType(raw_type)
        except ValueError as exc:
            raise ValueError(
                f"第 {index} 个步骤的 type={raw.get('type')!r} 非法，"
                "只支持 NAVIGATE / WAIT"
            ) from exc

        try:
            step = Step(
                index=index,
                type=step_type,
                x=float(raw.get("x", 0.0)),
                y=float(raw.get("y", 0.0)),
                yaw=float(raw.get("yaw", 0.0)),
                seconds=float(raw.get("seconds", 0.0)),
                label=str(raw.get("label", "")),
            )
        except (TypeError, ValueError) as exc:
            raise ValueError(f"第 {index} 个步骤字段非法: {exc}") from exc
        steps.append(step)

    name = str(data.get("name") or mission_path.stem)
    frame_id = str(data.get("frame_id") or "map")
    return name, frame_id, steps


# Nav2 在 NavigateToPose 的 result 里回传 error_code，取值来自 ComputePathToPose 的枚举。
# 下面这些不是瞬时故障：目标/起点在地图外、在障碍里、规划器配置错——
# 重试同一个目标必然再失败，所以归入「不可重试」，别把重试预算浪费在必然失败的目标上。
# 反过来 TF_ERROR(202) / TIMEOUT(207) / NO_VALID_PATH(208) 可能是瞬时的，保留重试。
NON_RETRYABLE_NAV_ERRORS = {
    201: "INVALID_PLANNER",
    203: "START_OUTSIDE_MAP",
    204: "GOAL_OUTSIDE_MAP",
    205: "START_OCCUPIED",
    206: "GOAL_OCCUPIED",
}


def _map_goal_status(status_code: int) -> StepOutcome:
    """把 Nav2 goal 的终态映射成领域结果。

    注意：这里不处理「还没结束」的状态（ACCEPTED/EXECUTING），因为调用方只在
    result future 完成后才调用它。未知状态按 ABORTED 处理（可重试），
    比无声无息当成成功安全。
    """
    if status_code == GoalStatus.STATUS_SUCCEEDED:
        return StepOutcome.SUCCEEDED
    if status_code == GoalStatus.STATUS_ABORTED:
        return StepOutcome.ABORTED
    if status_code == GoalStatus.STATUS_CANCELED:
        return StepOutcome.CANCELED
    return StepOutcome.ABORTED


class MissionDispatcher(Node):
    """任务调度节点。

    线程模型（为什么需要两个线程）：
    - 主线程跑 MultiThreadedExecutor.spin()，负责处理 action client 的响应、
      /mission/cancel 服务请求以及 ROS graph 事件；
    - 另一个 worker 线程顺序执行 mission，用轮询 future.done() 的方式等 Nav2，
      不占用 executor。否则导航期间服务回调无法执行，/mission/cancel 就会失效。

    worker 线程只通过 threading.Event 接收取消/关闭信号，不做线程强杀。
    Python 没有安全终止线程的手段，强杀会留下未提交事务和没取消的 Nav2 goal。
    """

    def __init__(self) -> None:
        super().__init__("mission_dispatcher")

        # ---------- 参数 ----------
        self.declare_parameter("mission_file", "")
        self.declare_parameter("db_path", "missions.db")
        self.declare_parameter("max_attempts", 3)
        self.declare_parameter("retry_backoff_sec", 1.0)
        self.declare_parameter("goal_timeout_sec", 90.0)
        self.declare_parameter("auto_resume", True)

        mission_file = str(self.get_parameter("mission_file").value or "")
        # 把相对路径转成绝对路径并打日志：WSL 下经常搞不清当前 cwd，
        # 事后看日志能立刻知道读的是哪份 YAML、写的是哪个数据库。
        self._mission_file = (
            os.path.abspath(os.path.expanduser(mission_file)) if mission_file else ""
        )
        db_path = str(self.get_parameter("db_path").value or "missions.db")
        if db_path == ":memory:":
            self._db_path = db_path
        else:
            self._db_path = os.path.abspath(os.path.expanduser(db_path))
        self._max_attempts = max(1, int(self.get_parameter("max_attempts").value))
        self._retry_backoff_sec = max(
            0.0, float(self.get_parameter("retry_backoff_sec").value)
        )
        self._goal_timeout_sec = max(
            1.0, float(self.get_parameter("goal_timeout_sec").value)
        )
        # 参数系统通常会把 true/false 解析成 bool，但如果用户传了字符串
        # （或某些 ROS2 版本按字符串处理），bool("false") 会等于 True。
        # 显式解析，避免 auto_resume 被意外打开/关闭。
        raw_auto_resume = self.get_parameter("auto_resume").value
        if isinstance(raw_auto_resume, str):
            self._auto_resume = raw_auto_resume.strip().lower() in (
                "1",
                "true",
                "yes",
                "on",
            )
        else:
            self._auto_resume = bool(raw_auto_resume)

        self._retry_policy = RetryPolicy(
            max_attempts=self._max_attempts,
            base_backoff_sec=self._retry_backoff_sec,
            # 保证上限不小于基数，否则用户把 base 调大时 RetryPolicy 会拒绝构造
            max_backoff_sec=max(30.0, self._retry_backoff_sec),
        )

        # ---------- 运行期状态 ----------
        self._cancel_requested = threading.Event()
        self._shutdown_requested = threading.Event()
        self._mission_id: Optional[int] = None
        self._mission_state = MissionState.PENDING
        self._mission_label = ""
        self._frame_id = "map"
        self._total_steps = 0
        self._mission_started_monotonic: Optional[float] = None
        # 最近一次发布状态时携带的步骤信息，mission 结束时复用它发终态
        self._last_step_index = 0
        self._last_attempts = 0
        self._last_error_code: Optional[str] = None
        self._worker: Optional[threading.Thread] = None

        # ---------- /mission/status ----------
        # depth=1 + transient_local：只保留「最新一条」状态，但晚启动的订阅者
        # 也能立刻拿到它。普通 volatile 话题做不到这一点：等用户开第三个终端
        # 执行 ros2 topic echo 时，节点可能已经跑完，状态消息早就丢了。
        # reliable 保证这条状态在 DDS 层不因网络/缓冲问题被丢弃。
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(String, STATUS_TOPIC, status_qos)

        # 取消服务只置一个 Event，真正的 cancel_goal_async 由 worker 线程在
        # 下一个检查点执行。这样避免服务回调跨线程操作 goal_handle。
        self._cancel_service = self.create_service(
            Trigger, CANCEL_SERVICE, self._handle_cancel
        )

        # ---------- 纯 Python 核心 ----------
        self._store = MissionStore(self._db_path)
        self._store.init_schema()

        # ---------- Nav2 action client ----------
        self._action_client = ActionClient(self, NavigateToPose, NAVIGATE_ACTION)

        self.get_logger().info(
            f"mission_dispatcher 启动: mission_file={self._mission_file or '(未设置)'}, "
            f"db_path={self._db_path}, auto_resume={self._auto_resume}, "
            f"max_attempts={self._max_attempts}, goal_timeout_sec={self._goal_timeout_sec}"
        )

    # ------------------------------------------------------------------
    # 生命周期：main() 调用 start_mission_thread()，Ctrl-C 时调用 request_shutdown()
    # ------------------------------------------------------------------

    def start_mission_thread(self) -> None:
        """启动 worker 线程。由 main() 在 executor 准备好之后调用。"""
        self._worker = threading.Thread(
            target=self._mission_worker,
            name="mission-dispatcher-worker",
            # daemon=True 只是最后兜底：正常路径 main() 会 join 等待它退出，
            # 避免 Python 在解释器退出阶段卡住。
            daemon=True,
        )
        self._worker.start()

    def request_shutdown(self) -> None:
        """请求 worker 在下一个检查点停下，并保留 RUNNING 状态以便下次续跑。"""
        self._shutdown_requested.set()

    def _shutdown_or_context_down(self) -> bool:
        """关闭请求已发出，或 ROS context 已经不可用。

        Ctrl-C 时 rclpy 会先关闭 context，main() 的 finally 可能还没来得及设置
        _shutdown_requested。如果只认 Event，worker 会把这次中断误判成用户取消，
        把 mission 写成 CANCELED，下次启动就不会续跑。把 context 状态也算进来，
        可以避免这个竞态。
        """
        return self._shutdown_requested.is_set() or not rclpy.ok()

    def join_worker(self, timeout_sec: float = 5.0) -> bool:
        """等待 worker 退出；返回 True 表示已退出。"""
        if self._worker is None:
            return True
        self._worker.join(timeout_sec)
        return not self._worker.is_alive()

    def _mission_worker(self) -> None:
        """worker 线程入口：跑 mission，异常时把 mission 标成 FAILED。"""
        try:
            self._run_mission_loop()
        except Exception:  # noqa: BLE001 - 这里必须兜住，否则线程静默死掉
            self.get_logger().exception("调度线程异常退出")
            if self._mission_id is not None:
                try:
                    self._finish_mission(
                        MissionState.FAILED, "调度线程内部异常，详见日志"
                    )
                except Exception:  # noqa: BLE001
                    self.get_logger().exception("异常收尾时写数据库也失败了")
        finally:
            # store 由 worker 独占使用并在退出时关闭；main() 只 join，不再访问 store
            self._store.close()
            self.get_logger().info("调度线程已退出")

    # ------------------------------------------------------------------
    # mission 主流程
    # ------------------------------------------------------------------

    def _run_mission_loop(self) -> None:
        """读计划 → 选择新建或续跑 → 顺序执行。"""
        steps = self._load_steps_or_log()
        if steps is None:
            return

        resume = self._select_resume(steps)
        if resume is None:
            mission_id = self._store.create_mission(
                steps, label=self._mission_label
            )
            start_index = 0
            self.get_logger().info(
                f"新建 mission {mission_id}（{self._mission_label}），"
                f"共 {len(steps)} 步，从第 0 步开始"
            )
        else:
            mission_id, start_index = resume
            self.get_logger().info(
                f"续跑 mission {mission_id}，从第 {start_index} 步继续"
                f"（共 {len(steps)} 步）"
            )

        self._execute_mission(mission_id, steps, start_index)

    def _load_steps_or_log(self) -> Optional[list[Step]]:
        """读取 YAML；失败只记日志并返回 None（不伪造一个 mission）。"""
        if not self._mission_file:
            self.get_logger().error(
                "参数 mission_file 为空，无事可做。示例："
                "ros2 run mission_dispatcher mission_dispatcher --ros-args "
                "-p mission_file:=/path/to/demo_3_steps.yaml"
            )
            return None

        try:
            name, frame_id, steps = load_mission_plan(self._mission_file)
        except (OSError, ValueError, yaml.YAMLError):
            self.get_logger().exception(f"读取 mission 文件失败: {self._mission_file}")
            return None

        self._mission_label = name
        self._frame_id = frame_id
        self._total_steps = len(steps)
        self.get_logger().info(
            f"已加载 mission '{name}'，frame_id={self._frame_id}，{len(steps)} 步"
        )
        return steps

    def _select_resume(self, steps: list[Step]) -> Optional[tuple[int, int]]:
        """决定是否续跑。

        数据库只存进度不存计划，所以续跑必须拿同一份 YAML 按 step_index 对齐。
        如果 YAML 的步骤数变了，说明计划已经变化，继续套用旧进度可能让机器人
        执行错误的任务，因此把旧 mission 标记为 FAILED 并新建一个。
        """
        if not self._auto_resume:
            return None

        resumable = self._store.find_resumable()
        if resumable is None:
            return None

        mission_id, next_index = resumable
        stored_steps = self._store.load_steps(mission_id)
        if len(stored_steps) != len(steps):
            self.get_logger().error(
                f"mission {mission_id} 在数据库里有 {len(stored_steps)} 步，"
                f"但 YAML 有 {len(steps)} 步。计划已变化，放弃续跑；"
                "旧 mission 标记为 FAILED，将新建 mission。"
            )
            self._store.set_mission_state(mission_id, MissionState.FAILED)
            return None

        self.get_logger().warning(
            f"发现未完成 mission {mission_id}，已完成 {next_index} 步；"
            "将按「整步重做」策略从下一步的边界继续"
        )
        return mission_id, next_index

    def _execute_mission(
        self, mission_id: int, steps: list[Step], start_index: int
    ) -> None:
        """顺序执行 steps[start_index:]，每一步内部处理重试。"""
        self._mission_id = mission_id
        self._mission_started_monotonic = time.monotonic()
        self._mission_state = MissionState.RUNNING
        self._store.set_mission_state(mission_id, MissionState.RUNNING)
        # start_index 可能等于 len(steps)（上次所有步骤都成功了，但进程在写终态前
        # 被杀）。这种边界下状态里的 step_index 仍应是最后一个合法下标。
        status_index = min(start_index, max(0, len(steps) - 1))
        self._publish_status(
            MissionState.RUNNING, step_index=status_index, attempts=0, error_code=None
        )

        for step in steps[start_index:]:
            # 阶段边界检查取消/关闭：上一步刚结束、还没发下一个 goal 时退出最安全
            if self._cancel_requested.is_set():
                self._finish_mission(MissionState.CANCELED, "收到 /mission/cancel")
                return
            if self._shutdown_or_context_down():
                # 保留 RUNNING，让下次启动的 auto_resume 能接上
                self._leave_running_for_resume(
                    "收到关闭请求，mission 保持 RUNNING 以便下次续跑"
                )
                return

            self.get_logger().info(
                f"开始步骤 {step.index + 1}/{len(steps)}: "
                f"{step.type.value} {step.label!r}"
            )
            outcome, attempts, error_code = self._execute_step_with_retries(step)

            if outcome is StepOutcome.SUCCEEDED:
                continue
            if outcome is StepOutcome.CANCELED:
                if self._shutdown_or_context_down():
                    self._leave_running_for_resume(
                        "步骤因节点关闭而中断，mission 保持 RUNNING 以便下次续跑"
                    )
                else:
                    self._finish_mission(MissionState.CANCELED, "mission 被取消")
                return

            self._finish_mission(
                MissionState.FAILED,
                f"步骤 {step.index} 最终失败（{error_code}, attempts={attempts}）",
            )
            return

        self._finish_mission(MissionState.DONE, "所有步骤执行成功")

    def _execute_step_with_retries(
        self, step: Step
    ) -> tuple[StepOutcome, int, Optional[str]]:
        """执行一个步骤，失败时按 RetryPolicy 决定退避重试或放弃。

        Returns:
            (最终结果, 已尝试次数, 最后错误码)
        """
        attempts = 0
        while True:
            # 阶段边界：每次尝试开始前都检查，取消后不再发新 goal
            if self._cancel_requested.is_set() or self._shutdown_or_context_down():
                return StepOutcome.CANCELED, attempts, StepOutcome.CANCELED.value

            attempts += 1
            self._store.update_step(
                self._mission_id, step.index, STEP_RUNNING, attempts, None
            )
            self._publish_status(
                MissionState.RUNNING, step.index, attempts, error_code=None
            )
            self.get_logger().info(
                f"步骤 {step.index} 第 {attempts}/{self._max_attempts} 次尝试"
            )

            outcome = self._execute_step_once(step)

            if outcome is StepOutcome.SUCCEEDED:
                self._store.update_step(
                    self._mission_id, step.index, STEP_SUCCEEDED, attempts, None
                )
                self._publish_status(
                    MissionState.RUNNING, step.index, attempts, error_code=None
                )
                return outcome, attempts, None

            error_code = outcome.value
            self._store.update_step(
                self._mission_id,
                step.index,
                STEP_CANCELED if outcome is StepOutcome.CANCELED else STEP_FAILED,
                attempts,
                error_code,
            )
            self._publish_status(
                MissionState.RUNNING, step.index, attempts, error_code=error_code
            )

            if outcome is StepOutcome.CANCELED:
                return outcome, attempts, error_code

            if not self._retry_policy.should_retry(outcome, attempts):
                self.get_logger().error(
                    f"步骤 {step.index} 结果为 {error_code}，按策略不再重试"
                )
                return outcome, attempts, error_code

            delay = self._retry_policy.backoff_for(attempts)
            self.get_logger().warning(
                f"步骤 {step.index} 第 {attempts} 次失败（{error_code}），"
                f"{delay:.2f}s 后重试"
            )
            if self._sleep_with_cancel(delay):
                # 等待退避期间收到取消：把 step 标成 CANCELED，保留 attempts 供排查
                self._store.update_step(
                    self._mission_id,
                    step.index,
                    STEP_CANCELED,
                    attempts,
                    StepOutcome.CANCELED.value,
                )
                return StepOutcome.CANCELED, attempts, StepOutcome.CANCELED.value

    def _execute_step_once(self, step: Step) -> StepOutcome:
        """执行一次步骤（不重试）。WAIT 不发 ROS 请求，NAVIGATE 调 Nav2。"""
        if step.type is StepType.NAVIGATE:
            return self._execute_navigate(step)
        return self._execute_wait(step)

    def _execute_wait(self, step: Step) -> StepOutcome:
        """模拟「在目标点做一件事」的停留。

        真实项目里这里可能是一次机械臂动作、拍照或服务调用；现在用 sleep 占位，
        让三步任务里出现一个不依赖 Nav2 的阶段，方便做 kill/续跑实验。
        """
        if step.seconds <= 0:
            return StepOutcome.SUCCEEDED

        deadline = time.monotonic() + step.seconds
        while True:
            if self._cancel_requested.is_set() or self._shutdown_requested.is_set():
                return StepOutcome.CANCELED
            if not rclpy.ok():
                return StepOutcome.CANCELED
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return StepOutcome.SUCCEEDED
            time.sleep(min(_POLL_INTERVAL_SEC, remaining))

    def _execute_navigate(self, step: Step) -> StepOutcome:
        """向 Nav2 发一个 NavigateToPose goal，等待并翻译结果。

        整个步骤共用一个 deadline（goal_timeout_sec），包含等待 action server、
        等待 goal 被接受、等待执行结果三段时间。这是有意保守：一个步骤不应该
        无限拖住 mission。deadline 只限制「等待」，超时后的取消是下一步动作。
        """
        deadline = time.monotonic() + self._goal_timeout_sec

        # 等待 Nav2 出现。独立设 30s 上限，避免 goal_timeout 很长时一直干等。
        server_deadline = min(
            deadline, time.monotonic() + _ACTION_SERVER_WAIT_SEC
        )
        if not self._action_client.server_is_ready():
            self.get_logger().info("等待 Nav2 action server 'navigate_to_pose' ...")
        if not self._wait_for_action_server(server_deadline):
            if self._cancel_requested.is_set() or self._shutdown_or_context_down():
                return StepOutcome.CANCELED
            self.get_logger().error(
                "Nav2 action server 在 "
                f"{_ACTION_SERVER_WAIT_SEC:.0f}s 内未就绪。"
                "请先启动仿真并确认 Nav2 生命周期节点已 active。"
            )
            # 前置条件不满足：重试同样的请求没有意义，直接让 mission 失败
            return StepOutcome.REJECTED

        goal = NavigateToPose.Goal()
        goal.pose = self._build_pose(step)

        try:
            send_goal_future = self._action_client.send_goal_async(goal)
        except Exception:  # noqa: BLE001 - 发送失败时请求大概率没发出去，可安全重试
            self.get_logger().exception("调用 send_goal_async 失败")
            return StepOutcome.ABORTED

        if not self._wait_for_future(send_goal_future, deadline):
            if self._cancel_requested.is_set() or self._shutdown_or_context_down():
                return StepOutcome.CANCELED
            # 没拿到 goal_handle，就无法调用 cancel_goal_async。此时 goal 有可能
            # 仍在 Nav2 侧被接受，重发会造成两个 goal 竞争，所以按 REJECTED 处理，
            # 不重试，并让用户检查 Nav2/DDS。这是本骨架明确的已知边界。
            self.get_logger().error(
                "等待 Nav2 接受 goal 超时，且尚未拿到 goal_handle，无法取消。"
                "为避免重复 goal 竞争，本步骤不重试。"
            )
            return StepOutcome.REJECTED

        try:
            goal_handle = send_goal_future.result()
        except Exception:  # noqa: BLE001 - action 层异常统一降级为可重试失败
            self.get_logger().exception("send_goal_async 的结果异常")
            return StepOutcome.ABORTED

        if goal_handle is None:
            self.get_logger().error("send_goal_async 返回了 None（不应发生）")
            return StepOutcome.REJECTED
        if not goal_handle.accepted:
            # 典型原因：目标点在障碍物里/超出地图、Nav2 未激活、BT 拒绝
            self.get_logger().warning(
                f"Nav2 拒绝了步骤 {step.index} 的 goal（前置条件不满足），不重试"
            )
            return StepOutcome.REJECTED

        try:
            result_future = goal_handle.get_result_async()
        except Exception:  # noqa: BLE001 - action 层异常统一降级为可重试失败
            self.get_logger().exception("调用 get_result_async 失败")
            return StepOutcome.ABORTED

        while not result_future.done():
            if self._cancel_requested.is_set() or self._shutdown_or_context_down():
                self._cancel_goal(goal_handle, "用户取消或节点关闭")
                return StepOutcome.CANCELED
            if time.monotonic() >= deadline:
                # 关键顺序：先 cancel，再判 TIMED_OUT。超时只代表「没等到结果」，
                # goal 可能还在控制机器人；不 cancel 就重发会让两个 goal 抢底盘。
                self._cancel_goal(goal_handle, "导航超时")
                return StepOutcome.TIMED_OUT
            time.sleep(_POLL_INTERVAL_SEC)

        try:
            result = result_future.result()
        except Exception:  # noqa: BLE001
            self.get_logger().exception("get_result_async 的结果异常")
            return StepOutcome.ABORTED

        if result is None:
            self.get_logger().error("get_result_async 返回了 None（不应发生）")
            return StepOutcome.ABORTED

        outcome = _map_goal_status(result.status)

        # Nav2 的 result 里还带了 error_code（取值来自 ComputePathToPose 的枚举）。
        # 只看 GoalStatus 会漏信息：「目标点在地图外」和「执行中卡住」都返回 ABORTED，
        # 但前者重试同一个目标永远不会成功，重试三次纯属浪费预算。
        # 实测：目标点 (10,10) 在地图外时，GoalStatus=6、error_code=204（GOAL_OUTSIDE_MAP）。
        error_code = None
        error_msg = ""
        inner = getattr(result, "result", None)
        if inner is not None:
            error_code = getattr(inner, "error_code", None)
            error_msg = getattr(inner, "error_msg", "") or ""

        if outcome is StepOutcome.ABORTED and error_code in NON_RETRYABLE_NAV_ERRORS:
            self.get_logger().warning(
                f"步骤 {step.index} 的 Nav2 error_code={error_code}"
                f"（{NON_RETRYABLE_NAV_ERRORS[error_code]}）不是瞬时故障，"
                f"重试同一个目标必然再失败，直接判失败"
            )
            outcome = StepOutcome.REJECTED

        if outcome is StepOutcome.SUCCEEDED:
            self.get_logger().info(f"步骤 {step.index} 导航成功")
        else:
            self.get_logger().warning(
                f"步骤 {step.index} 导航结束，GoalStatus={result.status} -> {outcome.value}，"
                f"error_code={error_code}，error_msg={error_msg!r}"
            )
        return outcome

    def _build_pose(self, step: Step) -> PoseStamped:
        """把 Step 的 x/y/yaw 转成 Nav2 需要的 PoseStamped。

        yaw 要转成四元数：绕 z 轴旋转 yaw 的四元数是
        (x=0, y=0, z=sin(yaw/2), w=cos(yaw/2))。
        """
        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(step.x)
        pose.pose.position.y = float(step.y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(float(step.yaw) / 2.0)
        pose.pose.orientation.w = math.cos(float(step.yaw) / 2.0)
        return pose

    # ------------------------------------------------------------------
    # 取消 / 等待 / 收尾
    # ------------------------------------------------------------------

    def _handle_cancel(self, request: Trigger.Request, response: Trigger.Response):
        """处理 /mission/cancel：只置位，不在这里碰 goal_handle。

        服务回调由 executor 线程执行，worker 线程可能正在操作同一个 goal_handle。
        把「取消意图」交给 worker，在它自己的检查点里做 cancel_goal_async，
        避免跨线程竞争。代价是取消最多延迟一个轮询周期（约 50ms）。
        """
        del request  # Trigger 请求没有字段
        self._cancel_requested.set()

        if self._mission_state in (
            MissionState.DONE,
            MissionState.FAILED,
            MissionState.CANCELED,
        ):
            response.success = False
            response.message = f"当前没有进行中的 mission（state={self._mission_state.value}）"
        else:
            response.success = True
            response.message = (
                "取消请求已收到；调度器会在下一个检查点停止，"
                "若 Nav2 goal 正在执行会先 cancel_goal_async"
            )
        self.get_logger().warning(f"/mission/cancel: {response.message}")
        return response

    def _wait_for_action_server(self, deadline: float) -> bool:
        """轮询 action server 是否就绪，直到 deadline 或被取消/关闭。"""
        while not self._action_client.server_is_ready():
            if self._cancel_requested.is_set() or self._shutdown_requested.is_set():
                return False
            if not rclpy.ok() or time.monotonic() >= deadline:
                return False
            time.sleep(0.1)
        return True

    def _wait_for_future(self, future, deadline: float) -> bool:
        """轮询等待 rclpy future 完成。

        worker 线程不能用 spin_until_future_complete（executor 在主线程），
        所以用 done() + sleep 轮询；每轮都检查取消/关闭，保证 Ctrl-C 响应及时。
        """
        while not future.done():
            if self._cancel_requested.is_set() or self._shutdown_requested.is_set():
                return False
            if not rclpy.ok() or time.monotonic() >= deadline:
                return False
            time.sleep(_POLL_INTERVAL_SEC)
        return True

    def _cancel_goal(self, goal_handle, reason: str) -> None:
        """发出 cancel_goal_async 并短暂等待回执，不阻塞太久。

        cancel 是异步的：即使 Nav2 回了 ERROR_NONE，也只代表它接受了取消请求，
        不代表底盘已经停下。真正「停稳」要靠下一次 goal 被接受前 Nav2 自己完成
        preempt；本实现只保证「取消请求先于重试发出」。
        """
        self.get_logger().warning(f"取消导航 goal（{reason}）")
        try:
            cancel_future = goal_handle.cancel_goal_async()
        except Exception:  # noqa: BLE001
            self.get_logger().exception(
                "调用 cancel_goal_async 失败，goal 可能仍在执行"
            )
            return

        # 注意：这里不能用 _wait_for_future，因为取消标志此刻必然已置位，
        # 那个函数会立刻返回 False。等回执时只关心「future 是否完成」和超时。
        cancel_deadline = time.monotonic() + _CANCEL_RESPONSE_WAIT_SEC
        while not cancel_future.done():
            if not rclpy.ok() or time.monotonic() >= cancel_deadline:
                break
            time.sleep(_POLL_INTERVAL_SEC)
        if not cancel_future.done():
            self.get_logger().warning(
                "取消请求已发出，但未在 "
                f"{_CANCEL_RESPONSE_WAIT_SEC:.1f}s 内收到 Nav2 回执；继续按原策略处理"
            )
            return

        try:
            cancel_response = cancel_future.result()
        except Exception:  # noqa: BLE001
            self.get_logger().exception("cancel_goal_async 的结果异常")
            return

        return_code = getattr(cancel_response, "return_code", None)
        if return_code == 0:
            self.get_logger().info("Nav2 已接受取消请求（return_code=0/ERROR_NONE）")
        else:
            self.get_logger().warning(
                f"Nav2 取消返回码 {return_code}（0 表示 ERROR_NONE）"
            )

    def _sleep_with_cancel(self, seconds: float) -> bool:
        """退避等待；返回 True 表示期间收到取消/关闭请求。"""
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            if self._cancel_requested.is_set() or self._shutdown_requested.is_set():
                return True
            if not rclpy.ok():
                return True
            remaining = deadline - time.monotonic()
            time.sleep(min(_POLL_INTERVAL_SEC, max(0.001, remaining)))
        return self._cancel_requested.is_set() or self._shutdown_requested.is_set()

    def _publish_status(
        self,
        state: MissionState,
        step_index: Optional[int] = None,
        attempts: int = 0,
        error_code: Optional[str] = None,
    ) -> None:
        """发布 /mission/status。

        state 是 mission 整体状态；step_index/attempts/error_code 描述当前步骤。
        elapsed 从本次进程开始执行 mission 算起（续跑后从 0 重新计），
        它只用于观察，不参与决策。
        """
        # context 已关闭时再 publish 可能抛 InvalidHandle；收尾阶段的状态
        # 已经写进 SQLite，不需要为了发一条状态把 worker 弄崩。
        if not rclpy.ok():
            return

        if step_index is not None:
            self._last_step_index = int(step_index)
        self._last_attempts = int(attempts)
        self._last_error_code = error_code

        elapsed = 0.0
        if self._mission_started_monotonic is not None:
            elapsed = max(0.0, time.monotonic() - self._mission_started_monotonic)

        payload = {
            "mission_id": self._mission_id,
            "step_index": self._last_step_index,
            "total_steps": self._total_steps,
            "state": state.value,
            "attempts": self._last_attempts,
            "error_code": self._last_error_code,
            "elapsed": round(elapsed, 3),
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        try:
            self._status_publisher.publish(message)
        except Exception:  # noqa: BLE001
            # 状态话题只是观测手段；发布失败不能反过来改变 mission 结果。
            # 真正的进度已经落在 SQLite 里。
            self.get_logger().warning("发布 /mission/status 失败（不影响进度落库）")

    def _finish_mission(self, state: MissionState, reason: str) -> None:
        """写终态、发最后一次状态、打日志。"""
        self._mission_state = state
        self._store.set_mission_state(self._mission_id, state)
        self._publish_status(state)
        self.get_logger().info(
            f"mission {self._mission_id} 结束: {state.value}（{reason}）"
        )

    def _leave_running_for_resume(self, reason: str) -> None:
        """节点关闭/进程中断时保留 RUNNING，并刷新 updated_at。

        刷新 updated_at 不只是为了“好看”：find_resumable 按它取最近一个，
        确保重启后优先接上这次被中断的任务。publish 在 context 已关闭时会
        自动跳过，但 SQLite 这条写必须发生。
        """
        self._store.set_mission_state(self._mission_id, MissionState.RUNNING)
        self.get_logger().info(reason)
        self._publish_status(MissionState.RUNNING)


def main(args=None) -> None:
    """节点入口：init → executor → worker 线程 → spin → 收尾落库。"""
    rclpy.init(args=args)
    node: Optional[MissionDispatcher] = None
    executor: Optional[MultiThreadedExecutor] = None
    try:
        node = MissionDispatcher()
        # 多线程 executor：worker 线程阻塞等 Nav2 时，服务回调仍能被处理，
        # 否则 /mission/cancel 要等导航结束才生效。
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        node.start_mission_thread()
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C 时 rclpy 的信号处理会主动关闭 context，spin() 可能抛
        # ExternalShutdownException；这不是错误，正常走 finally 收尾。
        pass
    finally:
        if node is not None:
            node.request_shutdown()
            if not node.join_worker():
                node.get_logger().warning(
                    "调度线程 5s 内未退出；它可能卡在 DDS 调用里。"
                    "已提交的进度都在 SQLite 中，重启后仍可续跑。"
                )
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
