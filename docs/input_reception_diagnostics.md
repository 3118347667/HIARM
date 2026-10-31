# 飞行节点输入隔离与分段计时

`flygripper_flight_node` 的输入在 `input_buffer.hpp` / `fsm.cpp` 中处理。控制循环仍独占飞行状态机和解码后的状态；输入回调只保存不可变消息指针和计时，不修改飞行状态。

## 线程和缓冲

| 回调队列 | 输入 | 工作线程 |
| --- | --- | --- |
| Imu | IMU | 独立 1 个 |
| Odom | EKF odometry | 独立 1 个 |
| Safety | RC、FCU state、extended state | 独立 1 个 |
| Ordinary | 轨迹、电池、电调、执行器状态 | 独立 1 个 |
| Control | 控制定时器 | 独立 1 个 |

每个输入话题有自己的短锁，锁内仅更新指针、标量和固定容量的 RC 环形缓冲。解码、日志及旧消息销毁在锁外。控制线程与诊断发布都用 `try_lock` 读取：锁忙时沿用原快照，保留原时间戳，正常执行超时检查，不等待输入锁。

RC 保留最多 64 条已进入回调的事件，按顺序处理，防止短暂 CH5 释放被后续高电平覆盖。控制线程暂时取不到锁时不清空事件。环形缓冲溢出会锁存异常，并在下次成功读取时触发已有接管逻辑。ROS 订阅队列仍有限长；本机制不能恢复到达回调之前已被 ROS 丢弃的消息。

同一订阅保持 `allow_concurrent_callbacks=false`，保持 ROS 的有序回调。缓冲还使用单调递增的回调编号拒绝较早回调的迟到写入；拒绝不会刷新最新数据或时间戳。编号不按源 `header.stamp` 排序，因此时间同步回退、冻结的传感器时间戳和发布者重启仍可进入原有数据有效性检查。

## 诊断字段

仍使用 `/flygripper/runtime_diagnostics`，约 10 Hz 发布，默认 `bags/bag.sh` 已包含该话题。新增详细字段前缀为 `imu`、`odom`、`rc`、`fc`、`trajectory`。

| 字段（省略前缀） | 含义 |
| --- | --- |
| `receipt_ros_s` | `ros::MessageEvent::getReceiptTime()`，本进程 ROS 收到该消息的时间 |
| `callback_ros_s`、`callback_steady_s` | 用户回调入口，分别用 ROS 时钟和单调时钟记录，位于输入锁之前 |
| `lock_acquired_steady_s` | 获得该话题输入锁的时间 |
| `saved_steady_s` | 消息和事件保存完成、释放锁之前的时间 |
| `age_at_receipt_ms` | ROS 接收时刻减源 `header.stamp` |
| `queue_delay_ms` | ROS 接收到用户回调入口的时间差，包括排队、调度和回调前反序列化 |
| `lock_wait_ms` | 从尝试获取输入锁到获取成功的单调时钟时差 |
| `store_ms` | 获锁到消息保存完成的耗时 |
| `queue_delay_max_ms`、`lock_wait_max_ms`、`store_max_ms` | 自节点启动以来的最大值，不是每 100 ms 清零；可捕捉诊断发布间隔中的短暂卡顿 |
| `callback_max_gap_ms` | 自启动以来相邻已接受消息的回调入口最大间隔 |
| `live_callback_gap_ms` | 此次诊断采样时距最新已保存回调入口的间隔 |
| `callbacks`、`accepted_updates`、`rejected_updates` | 已完成锁内处理的回调次数、接受次数、拒绝旧回调次数 |
| `live_snapshot_busy` | 诊断读取时锁忙，此次该输入字段退回控制快照（1），否则来自实时缓冲（0） |
| `sequence_gaps` | 接受消息的 `header.seq` 向前跳号累计，不能单凭它断言丢包发生在哪一层 |

控制循环还发布：

- `input_lock_copy_ms`：取各输入快照及 RC 事件的总耗时。
- `input_decode_ms`：快照后的转换、轨迹解码与状态保存耗时。
- `input_snapshot_ms`：以上两项之和，保留旧字段的总耗时含义。
- `input_snapshot_misses`：自启动以来控制线程取某个输入锁失败的累计次数；失败一次只表示本周期继续用旧快照。

旧字段 `imu/odom_header_age_ms`、`imu/odom_callback_gap_ms` 仍对应控制循环实际使用的快照；`imu/odom_age_at_callback_ms` 改为该快照在**锁前回调入口**的源数据年龄。原实现是在锁后记录，跨版本比较需注意。带 `live` 的字段以及新增分段字段来自诊断时的缓冲，可能比控制快照更新，不能跨不同消息直接相减。

## 如何判断延迟位置

- `age_at_receipt_ms` 已经很大：消息进入本节点前就较旧，或者源时钟映射有误。它包含采样、上游处理、传输和时钟误差，不能直接称为网络耗时。
- `queue_delay_ms` 大而 `lock_wait_ms` 小：优先检查回调队列、反序列化或系统线程调度。
- `lock_wait_ms` 大：输入写入在等待该话题锁；控制侧的 `input_snapshot_misses` 可辅助判断锁竞争。
- 输入持续更新，但控制快照年龄、`input_decode_ms` 或 `control_period_max_ms` 大：检查控制线程自身的停顿。

ROS 接收与回调入口使用 ROS 时钟，系统校时或 `/clock` 跳变会影响 `age_at_receipt_ms` 和 `queue_delay_ms`，可出现负值。等锁和保存耗时使用单调时钟，不能与 ROS 秒数直接相减。诊断只保留最近样本和最大值，并非逐消息完整追踪；本改动也没有测量网卡/串口硬件到达时间。

## 验证

`test/input_buffer.test` 覆盖普通队列阻塞时 IMU/EKF/RC 独立更新、IMU 阻塞时其他关键队列继续运行、锁竞争分段计时、控制取锁不等待、旧回调拒绝、并发快照一致性、RC 短脉冲/溢出及锁外消息销毁。`test/auto_takeoff.test` 使用真实飞行节点和模拟 MAVROS，包含新诊断字段、短暂 CH5 接管，以及原有起飞、超时、模式切换、轨迹状态等回归检查。

这些测试验证软件行为，不代表板端最坏延迟或实飞摆动已消失。CPU 整体调度停顿、上游发布/串口问题和控制线程中的重操作仍需用部署后的日志判断。
