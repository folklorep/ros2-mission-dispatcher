"""纯 Python 核心测试：models.py + store.py。

这里刻意不 import rclpy / nav2_msgs，也不启动任何 ROS 进程，
因此在 Windows、macOS 或没装 ROS2 的 Ubuntu 上都能直接跑：

    python -m pytest test -q

测试覆盖两类风险：
1. 重试语义写错（把不可重试的 REJECTED/CANCELED 重试了，或 TIMED_OUT 没重试）；
2. 持久化断点算错（把失败步骤跳过、把已完成步骤重做、重启后丢进度）。
"""

import sys
from pathlib import Path

# 让 `python -m pytest test` 在源码目录直接跑时也能 import mission_dispatcher，
# 不依赖 colcon install 或 PYTHONPATH。必须在导入包之前执行。
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

# 文件库测试的工作目录：随仓库提交（见 runtime/.gitkeep），保证存在且可写
RUNTIME_DIR = Path(__file__).resolve().parent / "runtime"

import pytest  # noqa: E402

from mission_dispatcher.models import (  # noqa: E402
    MissionState,
    RetryPolicy,
    Step,
    StepOutcome,
    StepType,
)
from mission_dispatcher.store import (  # noqa: E402
    STEP_FAILED,
    STEP_PENDING,
    STEP_RUNNING,
    STEP_SUCCEEDED,
    MissionStore,
)


# ---------------------------------------------------------------------------
# fixtures
# ---------------------------------------------------------------------------


@pytest.fixture()
def store():
    """每个测试一个独立内存库，测试之间不污染。"""
    instance = MissionStore(":memory:")
    instance.init_schema()
    try:
        yield instance
    finally:
        instance.close()


@pytest.fixture()
def sample_steps():
    return [
        Step(index=0, type=StepType.NAVIGATE, x=0.5, y=0.0, yaw=0.0, label="去 A"),
        Step(index=1, type=StepType.WAIT, seconds=2.0, label="在 A 点停 2 秒"),
        Step(index=2, type=StepType.NAVIGATE, x=-1.0, y=-0.5, yaw=3.14, label="回 B"),
    ]


# ---------------------------------------------------------------------------
# RetryPolicy：失败分类是否正确
# ---------------------------------------------------------------------------


def test_rejected_is_not_retryable():
    """REJECTED 是前置条件不满足，重试同样的请求只会得到同样的拒绝。"""
    policy = RetryPolicy(max_attempts=5)
    assert policy.should_retry(StepOutcome.REJECTED, 1) is False
    assert policy.should_retry(StepOutcome.REJECTED, 2) is False


def test_aborted_retries_until_limit():
    """ABORTED 是执行中失败，可能瞬态，允许重试到 max_attempts。"""
    policy = RetryPolicy(max_attempts=3)
    assert policy.should_retry(StepOutcome.ABORTED, 1) is True
    assert policy.should_retry(StepOutcome.ABORTED, 2) is True
    assert policy.should_retry(StepOutcome.ABORTED, 3) is False
    assert policy.should_retry(StepOutcome.ABORTED, 4) is False


def test_timed_out_retries():
    """TIMED_OUT 可重试（前提是 node 已先 cancel 旧 goal，见 node.py）。"""
    policy = RetryPolicy(max_attempts=2)
    assert policy.should_retry(StepOutcome.TIMED_OUT, 1) is True
    assert policy.should_retry(StepOutcome.TIMED_OUT, 2) is False


def test_canceled_is_not_retryable():
    """CANCELED 是用户意图，不是故障；自动重试等于违抗用户命令。"""
    policy = RetryPolicy(max_attempts=5)
    assert policy.should_retry(StepOutcome.CANCELED, 1) is False
    assert policy.should_retry(StepOutcome.CANCELED, 2) is False


def test_success_is_not_retried():
    assert RetryPolicy().should_retry(StepOutcome.SUCCEEDED, 1) is False


def test_backoff_grows_exponentially_and_caps():
    policy = RetryPolicy(
        max_attempts=5, base_backoff_sec=1.0, max_backoff_sec=5.0
    )
    assert policy.backoff_for(0) == 1.0
    assert policy.backoff_for(1) == 1.0
    assert policy.backoff_for(2) == 2.0
    assert policy.backoff_for(3) == 4.0
    # 2^(4-1) = 8，被 max_backoff_sec 截断到 5
    assert policy.backoff_for(4) == 5.0


def test_retry_policy_validates_parameters():
    with pytest.raises(ValueError):
        RetryPolicy(max_attempts=0)
    with pytest.raises(ValueError):
        RetryPolicy(base_backoff_sec=-1.0)
    with pytest.raises(ValueError):
        RetryPolicy(base_backoff_sec=2.0, max_backoff_sec=1.0)


# ---------------------------------------------------------------------------
# Step：模型层校验
# ---------------------------------------------------------------------------


def test_step_rejects_invalid_values():
    with pytest.raises(ValueError):
        Step(index=-1, type=StepType.WAIT, seconds=1.0)
    with pytest.raises(ValueError):
        Step(index=0, type=StepType.WAIT, seconds=-1.0)
    with pytest.raises(ValueError):
        Step(index=0, type=StepType.NAVIGATE, x=float("nan"))


# ---------------------------------------------------------------------------
# MissionStore：建 mission / 更新 step / 断点查询
# ---------------------------------------------------------------------------


def test_create_mission_returns_id_and_pending_steps(store, sample_steps):
    mission_id = store.create_mission(sample_steps, label="demo")

    assert isinstance(mission_id, int)
    assert mission_id > 0

    rows = store.load_steps(mission_id)
    assert len(rows) == 3
    assert [row["step_index"] for row in rows] == [0, 1, 2]
    assert all(row["state"] == STEP_PENDING for row in rows)
    assert all(row["attempts"] == 0 for row in rows)
    assert all(row["error_code"] is None for row in rows)

    # 一步都没完成时，断点就是第 0 步
    assert store.find_resumable() == (mission_id, 0)


def test_update_step_records_attempts_error_and_timestamps(store, sample_steps):
    mission_id = store.create_mission(sample_steps)

    store.update_step(mission_id, 0, STEP_RUNNING, 1)
    row = store.load_steps(mission_id)[0]
    assert row["state"] == STEP_RUNNING
    assert row["attempts"] == 1
    assert row["started_at"] is not None
    assert row["finished_at"] is None
    first_started_at = row["started_at"]

    store.update_step(mission_id, 0, STEP_FAILED, 1, StepOutcome.ABORTED.value)
    row = store.load_steps(mission_id)[0]
    assert row["state"] == STEP_FAILED
    assert row["error_code"] == "ABORTED"
    assert row["finished_at"] is not None
    # 重试开始时 started_at 保留第一次尝试的时间，方便排查“卡了多久”
    assert row["started_at"] == first_started_at

    store.update_step(mission_id, 0, STEP_RUNNING, 2)
    row = store.load_steps(mission_id)[0]
    assert row["attempts"] == 2
    assert row["finished_at"] is None
    assert row["started_at"] == first_started_at


def test_update_unknown_step_raises(store):
    with pytest.raises(ValueError):
        store.update_step(999, 0, STEP_RUNNING, 1)


def test_create_mission_rejects_empty_or_non_contiguous_steps(store):
    with pytest.raises(ValueError):
        store.create_mission([])
    with pytest.raises(ValueError):
        # index 从 1 开始，无法用连续前缀算断点
        store.create_mission([Step(index=1, type=StepType.WAIT, seconds=1.0)])


def test_find_resumable_returns_contiguous_success_prefix(store, sample_steps):
    mission_id = store.create_mission(sample_steps)

    store.update_step(mission_id, 0, STEP_SUCCEEDED, 1)
    assert store.find_resumable() == (mission_id, 1)

    store.update_step(mission_id, 1, STEP_SUCCEEDED, 1)
    assert store.find_resumable() == (mission_id, 2)

    store.update_step(mission_id, 2, STEP_SUCCEEDED, 1)
    assert store.find_resumable() == (mission_id, 3)


def test_find_resumable_does_not_skip_failed_middle_step(store, sample_steps):
    """异常数据保护：即使第 2 步被标成功，只要第 1 步没成功，断点仍是第 1 步。"""
    mission_id = store.create_mission(sample_steps)

    store.update_step(mission_id, 0, STEP_SUCCEEDED, 1)
    store.update_step(mission_id, 1, STEP_FAILED, 3, StepOutcome.ABORTED.value)
    store.update_step(mission_id, 2, STEP_SUCCEEDED, 1)

    assert store.find_resumable() == (mission_id, 1)


def test_find_resumable_ignores_terminal_missions(store, sample_steps):
    mission_id = store.create_mission(sample_steps)
    store.set_mission_state(mission_id, MissionState.DONE)
    assert store.find_resumable() is None

    second_id = store.create_mission(sample_steps)
    store.set_mission_state(second_id, MissionState.CANCELED)
    assert store.find_resumable() is None

    third_id = store.create_mission(sample_steps)
    store.set_mission_state(third_id, MissionState.FAILED)
    assert store.find_resumable() is None


def test_find_resumable_picks_most_recent_unfinished(store, sample_steps):
    first_id = store.create_mission(sample_steps, label="first")
    second_id = store.create_mission(sample_steps, label="second")

    assert second_id != first_id
    resumable = store.find_resumable()
    assert resumable is not None
    assert resumable[0] == second_id


def test_file_database_survives_reopen(sample_steps):
    """用真实文件库验证“进程重启后还能找到断点”。

    库文件写在 test/runtime/（这个目录随仓库提交，保证一定存在且可写），
    不用 pytest 的 tmp_path：受限环境（Windows 沙箱、部分 CI）里系统临时
    目录可能建不了库，那属于环境问题，会掩盖真正的逻辑失败。
    """
    db_path = RUNTIME_DIR / "missions.db"
    for leftover in RUNTIME_DIR.glob("missions.db*"):
        leftover.unlink(missing_ok=True)

    try:
        first = MissionStore(db_path)
        first.init_schema()
        mission_id = first.create_mission(sample_steps)
        first.update_step(mission_id, 0, STEP_SUCCEEDED, 1)
        first.close()

        second = MissionStore(db_path)
        second.init_schema()
        try:
            assert second.find_resumable() == (mission_id, 1)
            rows = second.load_steps(mission_id)
            assert len(rows) == 3
            assert rows[0]["state"] == STEP_SUCCEEDED
            assert rows[1]["state"] == STEP_PENDING
        finally:
            second.close()
    finally:
        for leftover in RUNTIME_DIR.glob("missions.db*"):
            leftover.unlink(missing_ok=True)
