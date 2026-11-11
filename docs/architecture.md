# 架构说明

本文比 README 再深一层，重点回答四个问题：

1. mission 和 step 的状态机到底怎么走；
2. 四种失败结果分别怎么处理，为什么；
3. 持久化、超时、取消、线程之间的边界在哪里；
4. 为什么先用状态机，不直接上 BehaviorTree.CPP。

---

## 1. 分层与依赖方向

```text
                 ┌────────────────────────────────────────────┐
                 │                node.py                     │
                 │  rclpy Node / ActionClient / Publisher     │
                 │  Trigger 服务 / 参数 / 线程与生命周期        │
                 └───────────────┬────────────────────────────┘
                                 │ 依赖（import）
                                 ▼
                 ┌────────────────────────────────────────────┐
                 │        models.py        store.py           │
                 │  领域对象/策略           SQLite 持久化       │
                 │  纯 Python，无 ROS 依赖                     │
                 └───────────────▲────────────────────────────┘
                                 │ 测试依赖
                 ┌───────────────┴────────────────────────────┐
                 │              test/test_core.py             │
                 │        pytest，无需 ROS / Gazebo            │
                 └────────────────────────────────────────────┘
```

依赖是单向的：`node.py → models/store`，核心层绝不反向 import ROS。这样 `models.py` 里的失败分类和重试策略可以在任何机器上单独验证，也方便以后把核心逻辑复用到别的执行引擎（BT、行为树、工作流引擎）。

---

## 2. mission 状态机

```text
                         create_mission()
          (无 mission) ─────────────────────► PENDING
                                                │
                                                │ 调度器开始执行
                                                ▼
                                             RUNNING
                                                │
              ┌─────────────────────────────────┼──────────────────────────────┐
              │ 全部 step SUCCEEDED              │ 不可重试失败 / 重试耗尽        │ /mission/cancel
              ▼                                 ▼                              ▼
            DONE                             FAILED                        CANCELED

  进程被杀 / Ctrl-C / 优雅关闭时：RUNNING 保持不变 ──重启后 auto_resume──► 从断点继续
```

- `PENDING`：mission 已建，但还没开始（通常只在启动瞬间出现）。
- `RUNNING`：至少开始执行；数据库里每一步的 `state` 单独记录。
- `DONE`：所有步骤成功。终态。
- `FAILED`：某一步最终失败（不可重试或重试耗尽）。终态，`auto_resume` 不会捡它。
- `CANCELED`：用户取消。终态，`auto_resume` 不会捡它。
- **进程崩溃/关闭不是终态**：mission 仍是 `RUNNING`，下次启动按断点续跑。这是有意的：崩溃不是业务上的完成或失败。

`find_resumable()` 只查 `state IN ('PENDING', 'RUNNING')`，按 `updated_at DESC, id DESC` 取最近一个。这样“正常失败的 mission”不会被自动重跑，避免无限失败循环。

---

## 3. step 执行与重试状态机

```text
                         PENDING
                            │
                            │ update_step(RUNNING, attempts+1)
                            ▼
                         RUNNING
                            │
        ┌───────────────────┼─────────────────────────────┐
        │ SUCCEEDED         │ REJECTED / CANCELED         │ ABORTED / TIMED_OUT
        ▼                   ▼                             ▼
    SUCCEEDED            FAILED / CANCELED           先 cancel（TIMED_OUT）
        │                   │                             │
        │                   │                             ▼
        ▼                   ▼                     should_retry?
   下一步 step        终止 mission                       │
                                                    ┌───┴────┐
                                                   否│        │是
                                                    ▼        ▼
                                                 FAILED   退避 sleep
                                                             │
                                                             ▼
                                                     RUNNING（attempt+1）
```

关键点：

- **`attempts` 是已经执行过的次数**，`max_attempts` 包含第一次，所以 `max_attempts=3` 最多执行 3 次。
- **每次尝试开始前检查取消标志**；取消后不再发新 goal。
- **`TIMED_OUT` 的 cancel 必须先于判定/重试**，见第 5 节。
- 重试耗尽是普通的业务失败（`FAILED`），不是异常；进程不会退出，只是这个 mission 结束。

---

## 4. 四种 outcome 的处理表

| outcome | 触发场景 | 可重试？ | 调度器动作 | 理由 |
|---|---|---|---|---|
| `SUCCEEDED` | Nav2 goal 终态 `STATUS_SUCCEEDED`；WAIT 计时结束 | — | 标记 step `SUCCEEDED`，进入下一步 | 目标已达成，不需要额外处理 |
| `REJECTED` | goal 未被接受：目标在墙里/超出地图、`navigate_to_pose` server 未就绪、BT 拒绝；或接受阶段等待超时且没有 goal_handle | **否** | step 标 `FAILED`，mission 立即 `FAILED` | 前置条件不满足。同样的请求再发一次只会得到同样的拒绝，重试是在浪费时间和日志；应该修地图/配置/启动顺序。接受阶段超时的特殊情况见第 9 节 |
| `ABORTED` | goal 被接受后执行失败：规划失败、控制器卡住、BT 返回 FAILURE、动态障碍导致无法前进 | **是** | 指数退避 `base * 2^(n-1)` 后重发，最多 `max_attempts` 次 | 执行中失败经常是瞬态的（动态障碍、定位抖动、代价地图瞬时膨胀），重试有真实成功概率 |
| `TIMED_OUT` | 等待 result 超过 `goal_timeout_sec` 的 deadline | **是，但必须先 cancel** | 先 `goal_handle.cancel_goal_async()`，再退避重发 | “没等到结果”不等于“goal 已停止”。旧 goal 可能还在控制底盘；不 cancel 就重发，两个 goal 会抢同一个机器人。顺序错误比不重试更危险 |
| `CANCELED` | 用户 `/mission/cancel`；或 Nav2 报告 `STATUS_CANCELED`（外部抢占） | **否** | step 标 `CANCELED`，mission `CANCELED` | 取消是用户意图/外部决定，不是故障。自动重试等于违抗用户命令 |

> 未知 `GoalStatus` 在 `node.py` 里保守映射成 `ABORTED`（可重试），并打 warning。这样不会把未知状态误判为成功。

---

## 5. 超时与取消：为什么必须先 cancel

假设导航 goal 已经发出，deadline 到了：

```text
错误顺序：判定 TIMED_OUT → 重发 goal
    旧 goal 可能仍在执行 ──┐
    新 goal 被 Nav2 接受 ──┴──► 两个 goal 争抢同一个底盘 / 行为树互相抢占

正确顺序：cancel_goal_async() → 判定 TIMED_OUT → 退避 → 重发
    旧 goal 收到取消请求 ──► Nav2 停止/抢占旧 goal ──► 新 goal 才有唯一控制权
```

本实现的具体顺序（`node.py::_execute_navigate`）：

1. 发 goal 前记一个 `deadline = now + goal_timeout_sec`；
2. 等 action server、等 goal 被接受、等 result，全部共享这一个 deadline；
3. result 还没完成时循环检查：取消标志 → `_cancel_goal()`，返回 `CANCELED`；deadline 到 → `_cancel_goal()`，返回 `TIMED_OUT`；
4. `_cancel_goal()` 调 `goal_handle.cancel_goal_async()`，最多等 1 秒回执，但不会因为等回执而阻塞取消语义；
5. 返回后由重试循环决定是否退避重发。

注意：`cancel_goal_async()` 是异步的，返回成功只表示“取消请求被接受”，不表示“底盘已停稳”。本实现只保证“取消请求先于重试发出”，这是单机 demo 能达到的合理边界；生产环境要等 Nav2 的 result 变成 `STATUS_CANCELED` 或位置稳定后再发下一个 goal。

取消服务本身（`/mission/cancel`）：

- 服务回调在 executor 线程执行，只做 `self._cancel_requested.set()`；
- worker 线程在 50ms 轮询里看到标志，若 goal 在跑就调用 `cancel_goal_async()`；
- 不调用 `thread.kill()` 之类的强制终止：Python 没有安全的线程终止；强杀会留下未提交的 SQLite 事务和未取消的 Nav2 goal。

---

## 6. 持久化与断点续跑

### 表结构

```sql
missions(
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  label TEXT, state TEXT,
  created_at TEXT, updated_at TEXT
)

steps(
  mission_id INTEGER,
  step_index INTEGER,
  state TEXT,            -- PENDING/RUNNING/SUCCEEDED/FAILED/CANCELED
  attempts INTEGER,
  error_code TEXT,       -- StepOutcome 的字符串值，便于排查
  started_at TEXT,       -- 第一次尝试开始时间（重试不覆盖）
  finished_at TEXT,      -- 最近一次尝试结束时间；RUNNING 时清空
  PRIMARY KEY (mission_id, step_index)
)
```

时间统一用 UTC ISO-8601（精确到微秒，带 `+00:00`），字符串排序等价于时间排序。

### 事务与 WAL

- 每个公开写方法内部都是 `with self._lock, self._conn:`，即一次事务提交；
- `PRAGMA journal_mode=WAL`：单机下读写不互相阻塞，崩溃后已提交事务更不容易丢；
- `PRAGMA foreign_keys=ON`：为以后加“删除 mission”保留级联完整性；
- 进程被 `kill -9` 时，最多丢掉“最后一个未提交事务”，不会出现半条 mission/step 记录。

### 断点算法

```text
SELECT 最近一个 state IN ('PENDING','RUNNING') 的 mission
SELECT 该 mission 的 steps ORDER BY step_index
completed = 0
for step in steps:
    if step.state != 'SUCCEEDED': break
    completed += 1
return (mission_id, completed)
```

为什么是“连续成功前缀”而不是 `count(SUCCEEDED)`：如果数据库里出现“第 1 步失败、第 2 步却成功”的异常数据，数总数会返回 2，导致续跑跳过失败的第 1 步。按前缀计算只会返回 1，宁可重做也不跳过。测试 `test_find_resumable_does_not_skip_failed_middle_step` 覆盖了这个保护。

### 为什么“只存进度、不存计划”

YAML 是计划的唯一来源，数据库只存“走到哪了”。好处是表结构简单、计划可读可 diff；代价是：

- 续跑必须提供同一份 `mission_file`；
- 步数变化会被检测到（旧 mission 标 `FAILED`，新建）；
- **步数不变但坐标/标签改了，当前检测不到**，会按新计划继续——这是已知的语义漂移风险，后续可以给 YAML 算哈希存进 mission 记录来解决。

### 整步重做（at-least-once）

续跑粒度是 step，不是 step 内部。例如：

- WAIT 20 秒，第 10 秒被杀 → 重启重新等 20 秒；
- NAVIGATE 走到一半被杀 → 重启重发同一个目标，由 Nav2 重新规划。

这是“至少执行一次”语义，不是“恰好一次”。对导航类动作通常可以接受（去同一个位姿是幂等的）；对“拍照/抓取/下单”这类有副作用的动作，必须引入幂等键或步骤级去重。当前 demo 的 WAIT 只是 sleep，没有副作用；这是刻意简化的。

---

## 7. 线程模型

```text
主线程                               worker 线程
──────────────────────────────       ─────────────────────────────────────
rclpy.init()
MissionDispatcher()                  （构造时创建 store / publisher / client）
MultiThreadedExecutor()
executor.add_node(node)
node.start_mission_thread() ───────► _mission_worker()
executor.spin()                       _run_mission_loop()
  ├─ 处理 action 响应                   ├─ load YAML
  ├─ 处理 /mission/cancel 服务          ├─ create/resume mission
  └─ 处理 graph 事件                    ├─ for step in steps:
                                            ├─ send_goal_async
                                            ├─ poll future.done()
                                            ├─ publish /mission/status
                                            └─ update SQLite
Ctrl-C / context shutdown
  └─ request_shutdown() ───────────► 检查 _shutdown_requested
  └─ join_worker(5s)                  ├─ 取消在跑的 goal
                                      ├─ 保持 mission RUNNING
                                      └─ store.close()
executor.shutdown()
node.destroy_node()
rclpy.shutdown()
```

为什么 worker 线程不用 `spin_until_future_complete`：executor 在主线程 spin，worker 再去 spin 会冲突；这里用 `future.done()` + 50ms sleep 轮询，每一轮都检查取消/关闭标志，所以 Ctrl-C 最多延迟一个轮询周期。

为什么用 `MultiThreadedExecutor`：worker 阻塞等 Nav2 时，服务回调仍要能被处理，否则 `/mission/cancel` 要等导航结束才生效。默认 callback group 是互斥的，但每个回调都很短（置标志、返回 response），不会阻塞。

线程安全的边界：

- `MissionStore` 用 `RLock` + `check_same_thread=False`；实际只由 worker 线程使用，`main()` 只 join 不碰 store；
- `_mission_state` 只做单次读写（GIL 下原子），用于 cancel 服务回答“当前有没有任务”；
- `goal_handle` 只在 worker 线程操作；服务回调不碰它；
- 发布状态从 worker 线程调用 `publish()`，rclpy 的 publisher 允许这样用；消息在发布时被序列化。

---

## 8. 为什么不用 BehaviorTree.CPP

这不是“BT 不好”，而是**当前阶段用状态机更合适**。理由分四层：

### 8.1 当前问题是“任务级调度 + 持久化”，BT 解决的是“单次任务内反应式决策”

BT 擅长：在机器人执行一个任务时，根据条件实时选择分支（例如“电池低 → 去充电”“路径被堵 → 重新规划”）。
本项目当前要解决：多个步骤的顺序执行、跨进程的进度持久化、崩溃后的断点恢复、失败分类与退避。
后四件事 BT 不负责——BT 的 Retry/Timeout 节点只在树的生命周期内有效，进程一死，树的状态就没了。

### 8.2 先手写，才能知道每个 BT 节点对应哪条语义

手写状态机逼你把这些问题回答清楚：

- 哪些状态？`PENDING/RUNNING/DONE/FAILED/CANCELED`；
- 哪些边？成功、拒绝、执行失败、超时、取消；
- 谁负责取消？服务置标志，worker 在检查点执行；
- 断点存在哪？step 表 + 连续成功前缀；
- TIMED_OUT 和 ABORTED 有什么区别？前者要先 cancel。

把这些问题吃透之后，换 BT 才是“把已有的边映射成节点”，而不是“用 XML 猜语义”。例如：

| 当前实现 | 换成 BT 后的对应物 |
|---|---|
| `Step` 列表顺序执行 | `Sequence` |
| `RetryPolicy` 的退避重试 | `Retry` / `Recovery` 子树 + 自定义退避节点 |
| `goal_timeout_sec` | `Timeout` 装饰器 + 自定义 cancel 节点 |
| `/mission/cancel` | `halt()` / 停止 tick + 取消 action goal |
| step 进度落库 | 每个 action 节点的 `on_start/on_success/on_failure` 回调 |
| 断点续跑 | **仍在调度层**：BT 不负责跨进程恢复，需要在 tick 前把“从第几步开始”注入黑板 |

### 8.3 可观测性和可讲性

- 状态机只有 5 个状态、几条边，一张白板就能画完；
- BT 的 XML 把控制流藏进树结构，讲清楚“为什么这次失败重试了”需要同时解释树、黑板、tick 语义；
- 当前项目的卖点是“失败分类 + 持久化 + 取消语义”，不是“我会写 BT XML”。先把前者讲透，后者是增量。

### 8.4 演进路径

1. 保持 `models/store` 不变；
2. 把 mission 执行器从“手写 for 循环”换成 BT.CPP 的 `Sequence`；
3. 每个 `Step` 映射成一个 subtree：NAVIGATE → `NavigateToPose` action 节点，WAIT → `Sleep` 节点；
4. `RetryPolicy` 映射成 `Retry` 节点 + 自定义 `Backoff`；
5. 用 `halt()` 实现取消，验证与当前 `cancel_goal_async` 的语义一致；
6. 断点续跑仍在 BT 外层：读 SQLite 算出起始 step，跳过已完成前缀。

此时再写一篇“从手写状态机迁移到 BT”的对比，才是完整的作品集叙事。

---

## 9. 实现假设与待验证的 ROS2 API（重要）

当前开发环境没有 ROS2，代码是**静态编写、未在 Jazzy 上运行过**的。以下是依赖的 API 假设，第一次在真机/WSL 里跑时请逐条确认：

1. **`NavigateToPose.Goal()` 的字段名是 `pose`**（类型 `geometry_msgs/PoseStamped`）。代码只设置 `goal.pose`，没有设置 `behavior_tree`，意图是使用 Nav2 默认行为树。若字段名不同，`goal.pose = ...` 会在运行时抛 `AttributeError`。
2. **`send_goal_async(goal)` 返回 future，future 的结果是 goal handle**；`goal_handle.accepted` 是布尔属性。若 Jazzy 的返回结构有变化，需要调整 `_execute_navigate` 里的两行。
3. **`goal_handle.get_result_async()` 返回 future，结果的 `.status` 是 `action_msgs/msg/GoalStatus` 常量**。代码用 `GoalStatus.STATUS_SUCCEEDED / STATUS_ABORTED / STATUS_CANCELED`。
4. **`goal_handle.cancel_goal_async()` 返回 future，结果是 `action_msgs/srv/CancelGoal.Response`**，其中 `return_code == 0` 表示 `ERROR_NONE`。代码用 `getattr(response, "return_code", None)` 做防御；如果 Jazzy 返回结构不同，取消回执日志会显示 `None`，但取消请求本身已经发出。
5. **`ActionClient.server_is_ready()` 可以轮询**，不需要 executor 在调用线程 spin。本实现故意不用 `wait_for_server()`，自己轮询以便及时响应取消/关闭。
6. **`MultiThreadedExecutor()` 默认线程数足够**处理 action 响应 + Trigger 服务；如果发现 cancel 服务在高负载下延迟，可以显式 `MultiThreadedExecutor(num_threads=4)`。
7. **`rclpy.executors.ExternalShutdownException` 存在**，用于捕获 Ctrl-C 时 context 被外部关闭的情况。
8. **QoSProfile 支持 `depth` + `reliability` + `durability` 关键字**；transient_local + reliable + depth 1 是合法组合。
9. **`nav2_bringup` 的 `tb3_simulation_launch.py` 在你的 Jazzy 安装里存在且支持 `headless` 参数**；用 `ros2 launch nav2_bringup tb3_simulation_launch.py --show-args` 确认。
10. **`std_srvs/srv/Trigger` 的 response 字段是 `success` 和 `message`**；服务回调签名 `(request, response)`。

另外两个非 ROS 的假设：

- `pyyaml` 的 `safe_load` 对 `missions/demo_3_steps.yaml` 返回 dict；YAML 中 `type: NAVIGATE` 被解析成字符串；
- `sqlite3` 在 WSL/Ubuntu 的 Python 3.12 上支持 WAL；对 `:memory:` 执行 `PRAGMA journal_mode=WAL` 返回 `memory` 而不是报错。

第一次运行建议按这个顺序排错：先 `ros2 action list | grep navigate_to_pose` 确认 action 存在 → 单独跑一个最小 action client 脚本发固定目标 → 再跑本节点。这样能把“ROS2 API 用法问题”和“调度逻辑问题”分开。

---

## 10. 后续改进清单

按投入产出排序：

1. **把计划哈希落库**：mission 记录 YAML 的 SHA-256，续跑时校验；步数相同但内容变化时拒绝续跑，消除语义漂移。
2. **记录 Nav2 goal_id**：崩溃后能查询/取消遗留 goal，而不是靠 Nav2 抢占；需要持久化 goal UUID 或接受“重启后新 goal 抢占旧 goal”的假设。
3. **等待取消确认**：TIMED_OUT 后等 result 变成 `STATUS_CANCELED` 再重发，而不是只等 cancel 回执；更安全，但会拉长失败恢复时间。
4. **步骤级幂等键**：给有副作用的步骤（拍照/抓取）加 key，重做前查“是否已执行”，把“至少一次”收敛到“最多一次 + 人工确认”。
5. **换 BehaviorTree.CPP**：见第 8 节的映射表，保持 models/store 不变。
6. **多机器人**：mission 增加 `robot_id`，每个机器人一个 dispatcher，或引入集中式任务队列 + 租约（lease）。
7. **真实机器人**：加急停、速度限制、定位健康检查；仿真里“重试 3 次”在真机上可能需要更保守的策略。
8. **可观测性**：把 attempts/outcome/耗时导出成 Prometheus 指标或 structured log，跑 100 次实验自动填 README 的 `X`。
