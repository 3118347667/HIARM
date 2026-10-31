# 小车抓取规划参数参考

本文列出工作区 [`launch/planning.launch`](launch/planning.launch) 当前配置的参数，供使用 [`grasping.launch`](launch/grasping.launch) 的真机小车抓取查阅。表中值是源码默认值，不代表板端当前加载的值；设备、滤波、速度和闭爪相关设置均需结合实际运行与记录验证。抓取流程、数据含义和调参顺序见 [移动小车抓取](MOVING_GRASP.md)。

## 从启动文件到运行参数

`./scripts/grasp_car.sh` → `grasping.launch` → 在 `/drone0` 下包含 `planning.launch` 并传入 `car_grasp=true`。实际参数名是 `/drone0/planning/<表中的 ROS 参数名>`；`num_worker_threads` 和 `time_forward` 例外，分别属于 `/drone0/manager` 和 `/drone0/traj_server`。

- 本表的默认值按 `car_grasp=true` 展开；直接启动 `planning.launch` 时 `car_grasp` 默认为 `false`，速度来源和两个闭爪默认门限不同，详见表格。
- 标为 `arg → 参数` 的条目，修改启动文件中对应 `arg` 的 `value` 或 `default`；其他条目修改对应 `<param value>`。例如 `grasp_height` 设置的是 `pos_dz`，ROS 参数服务器上查询的是 `pos_dz`。
- `grasping.launch` 没有透传这些调参入口，且 `planning.launch` 中多数 `arg` 使用固定的 `value`。不要假定向 `grasp_car.sh` 附加 `vmax:=...` 可以覆盖它们。修改工作区文件后，将改动同步到实际运行的板端工作区，再重启规划器。
- 规划器、预测器和优化器在初始化时读取这些参数；运行中 `rosparam set` 不会更新它们已缓存的值。`rosparam get` 显示的是参数服务器当前值，若运行期间被更改，不一定等于节点实际使用的缓存值。
- `bags/bag.sh` 可以记录 `/drone0/planning/rosparam_snapshot`：录包程序发现参数树后发布快照，树变化时重新发布。如果抓取规划器未启动或话题没有消息，bag 可能没有快照。快照不是二进制版本证明，也不能证明节点把录制后修改的参数重新读入；应核对快照时间、节点启动时间和实际部署版本。

### 调参前确认

```bash
rosparam get /drone0/planning
rosparam get /flygripper/servo_node/teleop_mode
rostopic hz /vrpn_client_node/HIARM_car/pose
rostopic hz /vrpn_client_node/HIARM_car/twist
rostopic hz /ekf/ekf_odom
```

以上是运行机器上的只读检查。话题频率无法单独证明消息新鲜或两端时钟同步；启动文件值、参数服务器值、程序实际读取值与实飞效果应分开看。调整前记录当前值和 bag，每次优先只改一项。

## 话题、循环和模式

| 启动配置 / ROS 参数 | 小车模式默认 | 作用和调参边界 |
|---|---:|---|
| `car_grasp` | `true`，由外层传入 | 启动文件分支选择器，不是运行中的独立 ROS 参数；选择 Twist 速度来源、飞行轨迹确认及小车抓取闭爪默认门限。直接启动 `planning.launch` 时为 `false`。 |
| `target_name_` → `~target` | `/vrpn_client_node/HIARM_car/pose` | 小车 `nav_msgs/Odometry` 位姿话题 remap；使用与 EKF 同一世界坐标系的数据。不是 `~target_name_` 参数。 |
| `target_twist_topic` → `~target_twist` | `/vrpn_client_node/HIARM_car/twist` | 小车原点线速度 `geometry_msgs/TwistStamped` remap；不是 `~target_twist_topic` 参数。 |
| `odom_topic` → `~odom` | `/ekf/ekf_odom` | 无人机状态输入 remap；坐标系和参考点需与抓取点标定匹配。 |
| `target_velocity_source` | `twist` | 小车模式必需独立 Twist；`pose_difference` 使用位姿原点位置差分，供兼容路径使用。Twist 丢失时不会自动切换；直接启动普通模式默认 `pose_difference`。 |
| `plan_hz` | `20 Hz` | 规划器定时频率，不是舵机循环、IMU 输入或飞行控制循环频率。改动会影响重规划频率和处理负载。 |
| `pregrasp_enabled` | `true` | 仅小车抓取入口启用预接近预测；普通 `planning.launch` 默认关闭。 |
| `require_flight_status` | `true` | 小车模式须先收到本次轨迹的飞行节点执行状态才开始接近；普通模式为 `false`，没有状态话题也可生成规划轨迹。闭爪判断仍要求本次轨迹状态有效。 |
| `pregrasp_distance` | `0.25 m` | 抓取点后方的水平偏移；进入 0.30 m 后锁定抓取点模式，不要求先抵达该点，直到新的触发才重置。修改后需重新启动规划器。 |
| `debug` | `false` | 选择独立 debug 计时路径；不是正常抓取性能开关。 |
| `/drone0/manager/num_worker_threads` | `2` | nodelet manager 工作线程数，不是轨迹速度或频率。 |
| `/drone0/traj_server/time_forward` | `1.0 s` | 启动文件设置了此参数，但当前 `traj_server.cpp` 未读取它；修改此值不应被当成调整前视或真机飞行轨迹的方法。 |

## 目标预测与滤波

| 配置 / ROS 参数 | 默认 | 作用和调参边界 |
|---|---:|---|
| `target_velocity_cutoff_hz` | `3.0 Hz` | 小车 XYZ 线速度一阶低通，进入位置预测、跟随、闭爪和异常速度判断。调低减小噪声但延迟真实变化；`0` 旁路滤波，负值/非有限数回退 3 Hz。首帧与超时间断后的首帧直接使用测量值。 |
| `tracking_dur` | `1.5 s` | 远距离刚体预测的时间跨度及轨迹优化时长基准；调大增加预测范围，也增加模型误差与计算工作。不是闭爪等待时间。 |
| `tracking_dt` | `0.2 s` | 远距离预测采样间隔，也用于优化跟踪代价采样；减小增加采样密度和计算量。不是 20 Hz 规划周期。 |
| `prediction/vmax` | `4.0 m/s` | 远距离刚体预测中抓取点预测速度的有效性上限；达到或超过该值预测失败。不是无人机速度上限。近距离 `predictAt` 当前另以代码固定 `4.0 m/s` 检查，不能认为只改此参数覆盖所有路径。 |
| `prediction/rho_a` | `1.0` | 旧的搜索式 `Predict::predict` 的加速度代价权重；当前正常小车接近使用 `predictRigidBody`，不使用该搜索权重，调它不应被当作小车跟随加速度调参。 |
| `target_yaw_filter_tau` | `0.1 s` | 从连续小车位姿估计车身 yaw 角速度时的一阶平滑参数；用于刚体位置预测，**不是**航向速度方向滤波的 Hz 参数。 |
| `target_yaw_rate_limit` | `3.0 rad/s` | 小车姿态差分得到的测量 yaw 角速度有效范围；超出时当前实现将估计值重置为 0，不是简单截断到 ±3。 |
| `replan_head_time` | `0.003 s` | 新轨迹起始时间相对当前 ROS 时间的前移量；涉及消息延迟、轨迹衔接与新鲜度，先分析日志再改。不是预测时域。 |

速度方向的独立滤波在下文“航向”中；它与 `target_velocity_cutoff_hz` 串联。阈值和跟踪用的是小车滤波线速度，不是直接用未经滤波的 Twist；飞机实际里程计速度没有经过这个小车滤波器。

## 抓取点、闭爪和抬升

| 配置 / ROS 参数 | 默认 | 作用和调参边界 |
|---|---:|---|
| `pos_dx`、`pos_dy` | `0.0 m`、`0.0 m` | 抓取参考点相对小车动捕刚体原点的局部 X/Y 偏移，会随小车姿态旋转。偏移变大或改变符号前，先标定坐标轴、动捕刚体原点、无人机 EKF 点和夹爪安装偏差。 |
| `grasp_height` → `pos_dz` | `0.09 m` | 相同局部坐标系下的 Z 偏移；`rosparam get` 查询 `pos_dz`。不是物体实际高度，也不是世界系固定 Z。 |
| `grasp_position` | `-12.7 rad` | 直接发送的位置命令；约 `-727.7°`，并非 `-12.7°`。实际运动、行程和负载必须另行验证。 |
| `grasp_open_position` | `0.0 rad` | 重新触发时张爪位置命令，是否张开由实际机构决定。 |
| `grasp_start_dis` | `0.02 m` | 预测抓取参考点与无人机 EKF 位置的三维距离门限，不是实体表面间距；调大可能在偏差更大时闭爪。普通模式默认为 `0.03 m`。 |
| `grasp_max_relative_speed` | `0.10 m/s` | 闭爪时抓取点预测速度与无人机速度的三维相对速度上限，不是无人机绝对速度；普通模式为 `0.05 m/s`。 |
| `grasp_max_yaw_error` | `0.174533 rad`，约 `10°` | 实际航向与规划目标航向的闭爪误差上限。 |
| `grasp_gate_duration` | `0.15 s` | 距离、相对速度、yaw、航向就绪和飞行状态需同时连续达标；条件中断会重置。增加时更难被短暂达标触发，也会延迟闭爪。 |
| `grasp_close_duration` | `0.85 s` | 首次发闭爪命令后的跟随计时，计时到期进入抬升；不是收到舵机反馈后计时。 |
| `grasp_open_duration` | `0.85 s` | 非首次触发时发送张爪命令后的等待时间，也可能继续等待制动结束；没有物理开口反馈确认。 |
| `post_grasp_lift` | `0.20 m` | 抬升时目标抓取点世界 Z 方向增加的平滑高度偏移。物体是否被抬起要另行确认。 |
| `grasp_lift_duration` | `2.0 s` | 抬升偏移的平滑变化时间；改变时会改变参考速度和加速度，需要同时考虑 `vmax`、`amax`。 |
| `grasp_follow_horizon` | `0.5 s` | 闭爪、抬升及近目标跟随连接轨迹初始预测时长，生成时会逐步尝试更长时域至至少 1.5 s。不是闭爪时长。 |
| `grasp_vx`、`grasp_vy`、`grasp_vz` | 均 `0.0 m/s` | 加到远距离优化末端目标速度的世界系分量；近距离 `followTrajectory` 未加此偏移，不能视为所有阶段统一的跟随速度增量。 |

闭爪状态只表示规划器发出了命令和计时进度；`/flygripper/actuator_command`、舵机写入及 ACK、有效实时反馈和物理抓取是不同证据层。

## 航向

| 配置 / ROS 参数 | 默认 | 作用和调参边界 |
|---|---:|---|
| `velocity_heading` | `true` | 根据小车世界系水平速度方向构建航向。`false` 保持新任务初始化的飞机航向并跳过运动/静止方向就绪条件，yaw 误差和其他闭爪门限仍有效，Twist 仍必需。 |
| `hold_yaw` → `hold_yaw_on_trigger` | `true` | 旧的通用航向保持选项；当前真机小车 `publishMotion` 使用显式航向多项式，`velocity_heading` 选择速度方向或保持航向，因此不能仅靠这个参数关闭速度航向。 |
| `grasp_heading_offset` | `0.0 rad` | 当运动方向可靠时，加在速度方向上的飞机目标 yaw 偏移；不是抓取点位置偏移。 |
| `grasp_yaw_velocity_filter_hz` | `1.0 Hz` | 前级 3 Hz 小车速度之后的方向向量滤波，以及速度方向角速度前馈的平滑。降低更稳但更滞后；须为正有限值，缺失或无效时代码回退到 `0.6366197723675814 Hz`。旧 `grasp_yaw_velocity_filter_tau` 不再读取，需同步启动文件和二进制。 |
| `grasp_yaw_speed_low` | `0.05 m/s` | 方向滤波中的低速/静止门限；与输入噪声、截止频率和确认时间共同决定何时可闭爪。 |
| `grasp_yaw_speed_high` | `0.08 m/s` | 高速方向确认及 yaw 角速度前馈门限；读取后至少为 `grasp_yaw_speed_low`。 |
| `grasp_yaw_direction_dwell` | `0.25 s` | 静止和较高速度方向确认时长；较低速度方向确认用两倍时长。不是最终的 `grasp_gate_duration`。 |
| `grasp_yaw_rate_max` | `0.60 rad/s`，约 `34.4°/s` | 航向多项式角速度与方向角速度前馈上限；不是小车车身 `target_yaw_rate_limit`。 |
| `grasp_yaw_acc_max` | `1.20 rad/s²` | 航向多项式角加速度上限；与角速度上限共同影响转向响应。 |
| `grasp_yaw` | `0.0 rad` | 远距离接近方向几何中相对小车刚体朝向的偏移，与 `rhoGrasp` 的方向偏好有关；不是飞机速度航向的 `grasp_heading_offset`。`rhoGrasp=0` 时不施加该方向代价。 |

## 速度、加速度和优化器

| 配置 / ROS 参数 | 默认 | 作用和调参边界 |
|---|---:|---|
| `vmax` | `0.70 m/s` | 规划三维速度参数。远距离优化对超限加惩罚，不能保证绝对不超过；近距离跟随、闭爪和抬升连接轨迹在发布前检查整条轨迹。初始化需要飞机速度不超过该值。实际飞机速度仍可能超出参考。 |
| `amax` | `0.70 m/s²` | 规划三维加速度参数，远距离为软代价，近距离跟随轨迹有发布前检查。制动为保持已有参考的连续性可临时允许当前参考已超过的速度或加速度。 |
| `K` | `8` | 每段轨迹积分惩罚的采样分段数；增大计算量，不是轨迹速度或控制增益。 |
| `rhoT` | `20.0` | 对优化轨迹超出时长基准的时间代价权重；越大通常越不愿延长轨迹，但最终结果还受其他代价影响。 |
| `rhoP` | `10000.0` | 空间走廊位置违反惩罚权重，影响优化参考，不提供实机障碍物避让保证。 |
| `rhoV` | `10000.0` | 远距离优化速度超 `vmax` 时的惩罚权重；调大可增加超限代价，仍非硬约束。 |
| `rhoA` | `5000.0` | 远距离优化加速度超 `amax` 时的惩罚权重；仍非硬约束。 |
| `rhoTracking_` → `rhoTracking` | `1000.0` | 优化中位置跟踪代价权重；权重变化与时间、速度、加速度等代价共同作用，不能单独视为控制器 `k_pos`。 |
| `rhosVisibility_` → `rhosVisibility` | `0.0` | 远距离优化的可见方向代价权重；0 表示此代价关闭，不表示环境或全部规划逻辑关闭。 |
| `rho_grasp` → `rhoGrasp` | `0.0` | 远距离优化中偏好从目标刚体指定方向接近的代价权重；0 时不施加此方向代价。与飞机 yaw 控制分开。 |
| `theta_clearance` | `0.8 rad` | 可见区域构建及方向约束中使用的角度余量；即使可见方向权重为 0，也可能影响可见区域几何。 |
| `clearance_d` | `0.2 m` | 优化空间走廊面的距离余量；当前自由空间模型不能仅凭此值推断真实障碍物安全距离。 |
| `tolerance_d` | `0.02 m` | 优化跟踪的 Z 方向误差容差，以及着陆分支相关容差；不是闭爪三维距离门限 `grasp_start_dis`。 |
| `tracking_dist` | `1.0 m` | 可见区域几何构建使用的参考半径，旧 fake/debug 路径也读取；不是当前真机抓取时始终与物体保持 1 m。 |
| `grasp_tole_dis` | `0.25` | 当前代码读取但正常抓取分支没有使用它作为闭爪门限；不要用它代替 `grasp_start_dis` 调闭爪距离。 |

参数之间会相互制约。先确认跟踪误差、实际反馈和执行器状态，不能仅凭增加优化权重推断实机会更稳定或更安全。飞控控制器的 `velocity_max`、`tilt_angle_max_deg`、增益和实际姿态属于另一层；参考限速不等于实际速度或倾角的物理硬上限。

## 数据新鲜度与异常退出

| 配置 / ROS 参数 | 默认 | 作用和调参边界 |
|---|---:|---|
| `target_timeout` | `0.20 s` | 小车位姿 header 年龄及接收间隔上限；先排查动捕、时钟和丢包，调大可能允许更旧目标数据。 |
| `target_twist_timeout` | `0.20 s` | Twist 接收新鲜度及滤波历史失效的上限；新样本还需时间戳非零且递增。默认不等同于检查 header 绝对年龄。 |
| `target_twist_check_header_age` | `false` | 是否额外检查 Twist header 与本机 ROS 时间的年龄；打开前先验证当前 VRPN 时钟映射和延迟。 |
| `target_pose_twist_max_skew` | `0.15 s` | 位姿和速度在规划器接收时刻的最大差值；是接收时间近似，不证明测量同步或绝对延迟。 |
| `odom_timeout` | `0.20 s` | 飞机 EKF 消息 header 年龄及接收间隔上限，影响能否继续规划。 |
| `flight_status_timeout` | `0.20 s` | 飞行轨迹状态的接收和 header 年龄上限；状态还须匹配本次任务轨迹。 |
| `grasp_abort_distance` | `0.20 m` | 闭爪时预测抓取点相对飞机的三维位置误差上限；不是闭爪触发门限。代码确保其不小于 `grasp_start_dis`。 |
| `grasp_lift_abort_distance` | `0.10 m` | 抬升期间，含抬升偏移的期望点与飞机之间的三维误差上限，独立于闭爪距离上限。 |
| `grasp_abort_relative_speed` | `0.50 m/s` | 闭爪阶段检查三维相对速度，抬升阶段只检查水平相对速度；速度滤波可能延迟异常响应。代码确保其不小于闭爪相对速度门限。 |
| `grasp_abort_yaw_error` | `0.785398 rad`，约 `45°` | 闭爪和抬升阶段 yaw 误差异常上限；代码确保其不小于闭爪 yaw 门限。 |

这些限值超过后会请求制动并锁定任务异常；飞控已退出轨迹控制时，不保证制动轨迹实际执行。先排查消息丢失、时间戳、参考跟踪和坐标系，再考虑调整阈值。
