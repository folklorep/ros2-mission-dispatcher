"""SQLite 持久化：mission / step 的执行进度。

这个模块同样不 import rclpy，测试可以在没有 ROS2 的机器上直接跑。

设计要点（为什么这样写）：
- 每次写操作使用 `with self._conn:`，即一个事务。进程被 kill 时最多丢掉
  「最后一个尚未提交的写入」，不会留下半条 mission/step 记录。
- 打开 WAL 模式。WAL 下读和写不互相阻塞，适合「调度线程写进度、调试时另开
  进程读数据库」的单机场景；崩溃后 WAL 也更容易保住已提交的事务。
- 数据库只存「进度」，不存「计划」。步骤的 x/y/yaw/seconds 仍然以 YAML 为唯一
  来源，续跑时按 step_index 对齐。这样表结构简单，也避免两份计划数据不一致；
  代价是续跑必须提供同一份 mission_file（见 README「已知边界」）。
"""

from __future__ import annotations

import os
import sqlite3
import threading
from datetime import datetime, timezone
from typing import Iterable, Optional

from mission_dispatcher.models import MissionState, Step

# step 的状态。和 MissionState 分开：step 没有 DONE/FAILED 这种整体语义，
# 只有「还没跑 / 正在跑 / 成功 / 失败 / 被取消」。
STEP_PENDING = "PENDING"
STEP_RUNNING = "RUNNING"
STEP_SUCCEEDED = "SUCCEEDED"
STEP_FAILED = "FAILED"
STEP_CANCELED = "CANCELED"

_STEP_STATES = frozenset(
    {STEP_PENDING, STEP_RUNNING, STEP_SUCCEEDED, STEP_FAILED, STEP_CANCELED}
)

_SCHEMA_STATEMENTS = (
    """
    CREATE TABLE IF NOT EXISTS missions (
        id         INTEGER PRIMARY KEY AUTOINCREMENT,
        label      TEXT    NOT NULL DEFAULT '',
        state      TEXT    NOT NULL,
        created_at TEXT    NOT NULL,
        updated_at TEXT    NOT NULL
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS steps (
        mission_id  INTEGER NOT NULL,
        step_index  INTEGER NOT NULL,
        state       TEXT    NOT NULL,
        attempts    INTEGER NOT NULL DEFAULT 0,
        error_code  TEXT,
        started_at  TEXT,
        finished_at TEXT,
        PRIMARY KEY (mission_id, step_index),
        FOREIGN KEY (mission_id) REFERENCES missions(id) ON DELETE CASCADE
    )
    """,
    "CREATE INDEX IF NOT EXISTS idx_missions_state ON missions(state)",
)


def _utcnow_iso() -> str:
    """返回可按字典序排序的 UTC ISO 时间戳（精确到微秒）。

    统一带 +00:00 后缀，保证同一列里的字符串排序等价于时间排序。
    """
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


def _mission_state_value(state: MissionState | str) -> str:
    """把 MissionState 枚举或字符串统一成要落库的字符串。"""
    value = state.value if isinstance(state, MissionState) else str(state)
    if value not in {item.value for item in MissionState}:
        raise ValueError(f"非法 mission 状态: {value!r}")
    return value


class MissionStore:
    """mission / step 进度的 SQLite 存储。

    线程安全：node.py 里连接在构造时创建，之后主要由调度线程使用，关闭时
    也可能从主线程调用，所以 connect 时设置 check_same_thread=False，
    并用一把 RLock 串行化所有数据库操作。这不是为了高并发，而是为了避免
    sqlite3 对象跨线程使用时的隐蔽错误。
    """

    def __init__(self, db_path: str | os.PathLike[str] = ":memory:") -> None:
        self._lock = threading.RLock()
        self._path = str(db_path)
        # check_same_thread=False 的理由见类 docstring；连接仍然只由本对象持有
        self._conn = sqlite3.connect(
            self._path,
            timeout=5.0,
            check_same_thread=False,
        )
        self._conn.row_factory = sqlite3.Row
        # WAL 是数据库级设置，对 :memory: 会返回 "memory"，不会报错
        self._conn.execute("PRAGMA journal_mode=WAL")
        # 打开外键约束，删 mission 时 step 会级联删除（当前版本没提供删除接口，
        # 但保持约束完整，避免以后加清理功能时留下孤儿行）
        self._conn.execute("PRAGMA foreign_keys=ON")

    def init_schema(self) -> None:
        """创建表和索引，可重复调用。"""
        with self._lock, self._conn:
            for statement in _SCHEMA_STATEMENTS:
                self._conn.execute(statement)

    def create_mission(self, steps: Iterable[Step], label: str = "") -> int:
        """新建 mission 及其全部 step（初始 PENDING），返回 mission_id。

        steps 的 index 必须是从 0 开始连续递增的；find_resumable 依赖这个约定
        用「连续成功前缀」计算断点，否则会出现「第 2 步成功了但第 1 步没做」
        这种无法安全续跑的状态。
        """
        step_list = list(steps)
        if not step_list:
            raise ValueError("mission 至少需要一个步骤")
        if [step.index for step in step_list] != list(range(len(step_list))):
            raise ValueError(
                "step.index 必须从 0 开始连续递增，收到 "
                f"{[step.index for step in step_list]}"
            )

        now = _utcnow_iso()
        with self._lock, self._conn:
            cursor = self._conn.execute(
                """
                INSERT INTO missions (label, state, created_at, updated_at)
                VALUES (?, ?, ?, ?)
                """,
                (str(label), MissionState.PENDING.value, now, now),
            )
            mission_id = int(cursor.lastrowid)
            self._conn.executemany(
                """
                INSERT INTO steps (mission_id, step_index, state, attempts, error_code)
                VALUES (?, ?, ?, 0, NULL)
                """,
                [(mission_id, step.index, STEP_PENDING) for step in step_list],
            )
        return mission_id

    def update_step(
        self,
        mission_id: int,
        step_index: int,
        state: str,
        attempts: int,
        error_code: Optional[str] = None,
    ) -> None:
        """更新单个步骤的状态/尝试次数/错误码。

        时间戳由本方法维护，调用方不用关心：
        - 进入 RUNNING：started_at 只在第一次写入时记录，finished_at 清空；
        - 进入终态（SUCCEEDED/FAILED/CANCELED）：finished_at 记为当前时间。
        """
        if state not in _STEP_STATES:
            raise ValueError(f"非法 step 状态: {state!r}")
        if int(attempts) < 0:
            raise ValueError(f"attempts 必须 >= 0，收到 {attempts}")

        now = _utcnow_iso()
        # RUNNING 表示「新一次尝试开始」，所以清空 finished_at；
        # 其他状态都是本次尝试的终态，写入 finished_at。
        finished_at = None if state == STEP_RUNNING else now

        with self._lock, self._conn:
            cursor = self._conn.execute(
                """
                UPDATE steps
                   SET state = ?,
                       attempts = ?,
                       error_code = ?,
                       started_at = COALESCE(started_at, ?),
                       finished_at = ?
                 WHERE mission_id = ? AND step_index = ?
                """,
                (
                    state,
                    int(attempts),
                    error_code,
                    now,
                    finished_at,
                    int(mission_id),
                    int(step_index),
                ),
            )
            if cursor.rowcount == 0:
                raise ValueError(
                    f"step 不存在: mission_id={mission_id}, step_index={step_index}"
                )

    def set_mission_state(self, mission_id: int, state: MissionState | str) -> None:
        """更新 mission 整体状态，同时刷新 updated_at（find_resumable 用它排序）。"""
        value = _mission_state_value(state)
        now = _utcnow_iso()
        with self._lock, self._conn:
            cursor = self._conn.execute(
                "UPDATE missions SET state = ?, updated_at = ? WHERE id = ?",
                (value, now, int(mission_id)),
            )
            if cursor.rowcount == 0:
                raise ValueError(f"mission 不存在: id={mission_id}")

    def find_resumable(self) -> Optional[tuple[int, int]]:
        """找最近一个未完成的 mission，返回 (mission_id, 已完成步数)。

        已完成步数按「从第 0 步开始的连续 SUCCEEDED 前缀」计算，而不是简单地
        数 SUCCEEDED 的条数。这样即使数据库里出现异常数据（比如中间一步失败、
        后面一步却成功），也只会从第一个未成功的步骤重做，绝不会跳过失败步骤。

        没有未完成 mission 时返回 None。
        """
        with self._lock:
            mission_row = self._conn.execute(
                """
                SELECT id
                  FROM missions
                 WHERE state IN (?, ?)
                 ORDER BY updated_at DESC, id DESC
                 LIMIT 1
                """,
                (MissionState.PENDING.value, MissionState.RUNNING.value),
            ).fetchone()
            if mission_row is None:
                return None

            mission_id = int(mission_row["id"])
            step_rows = self._conn.execute(
                """
                SELECT step_index, state
                  FROM steps
                 WHERE mission_id = ?
                 ORDER BY step_index
                """,
                (mission_id,),
            ).fetchall()

        completed = 0
        for row in step_rows:
            if row["state"] != STEP_SUCCEEDED:
                break
            completed += 1
        return mission_id, completed

    def load_steps(self, mission_id: int) -> list[dict]:
        """读取某个 mission 的所有 step 进度，按 step_index 升序。

        返回 list[dict] 而不是 sqlite3.Row：Row 在连接关闭后会失效，
        转成普通 dict 更安全，测试断言也更好写。
        """
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT mission_id, step_index, state, attempts,
                       error_code, started_at, finished_at
                  FROM steps
                 WHERE mission_id = ?
                 ORDER BY step_index
                """,
                (int(mission_id),),
            ).fetchall()
        return [dict(row) for row in rows]

    def close(self) -> None:
        """关闭连接。调用后本对象不可再用。"""
        with self._lock:
            self._conn.close()
