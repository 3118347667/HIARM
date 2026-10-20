# ArduPilot 自动起飞

真机 `fsm.yaml` 默认启用自动起飞，使用 `command_mode: 0`、
`GUIDED_NOGPS` 和 `/mavros/setpoint_raw/attitude`。
高度闭环由本项目的位置控制器完成，飞控接收姿态和 0～1 推力。
`params/hiarm.params` 中的 `GUID_OPTIONS=9` 包含 bit 3；节点进入控制前
还会通过 `/mavros/param/get` 核对飞控实际值的该位。

## 操作

1. 保持 CH5、CH6 关闭，运行 `./scripts/run.sh`。录包会同步启动，
   并记录 `/mavros/extended_state`、控制器数据和 `/flygripper/reference`。
2. 确认动捕/EKF 正常，通过遥控器完成解锁。
3. 将 CH5 从低位拨到高位。节点检查传感器、RC、飞控连接及解锁状态，
   根据新鲜的 `/mavros/extended_state` 判断地面/空中状态。
4. 在地面时，请求 `GUIDED_NOGPS`，等 `/mavros/state` 确认后，记录此刻
   的位置和偏航，沿竖直方向升高 `takeoff_height`。默认相对升高 1 米，
   爬升参考限制为 0.25 m/s、0.3 m/s² 和 1 m/s³，参考过程约 8.75 秒。
5. 到达目标且位置误差和速度持续满足条件 1 秒后，保持该目标悬停。
   起飞完成后将四个操纵轴回中，随后才能使用原来的速度遥控。
   起飞期间的 CH6 触发会被消耗，完成后需重新拨动 CH6 才进入轨迹模式。

如果机体已在空中，CH5 进入控制后保持当前位置，不再额外爬升。
节点启动时 CH5 已在高位不会自动触发，需先拨低再拨高。
未解锁、地面状态未知或配置不合法时会拒绝本次触发，修复后重新拨动 CH5。

## 退出与故障

CH5 拨低，或起飞中出现未离地超时、整体超时、跟踪误差过大、
必要状态数据中断时，请求 `handover_mode`，真机默认 `ALT_HOLD`。
APM 接管后油门杆决定升降，请将油门杆放在适合 AltHold 接管的位置。
模式确认前，在输入数据有效时保持当前位置的控制参考；传感器失效时
停止发送计算出的姿态指令，并继续尝试请求飞控接管。
请求失败会明确报警，不能把它当作已接管。

飞控或飞手主动切到其他模式、飞控上锁后，节点停止自主控制；
不会把飞控抢回 GUIDED。必须重新拨动 CH5 才能再次进入。
姿态输出模式下关闭 ROS 节点不发送零推力，丢失指令由飞控处理。
`GUID_TIMEOUT=3` 是本机参数记录中的后备行为，不提供动捕位置保持；
正常结束应先完成飞控模式接管，再停止节点。

## 参数与验证

参数位于 `src/flygripper_flight/params/fsm.yaml`。
`takeoff_height` 表示相对此次起飞位置的升高量，起飞开始时冻结；
修改高度不改变已经执行的起飞。速度、加速度、jerk、稳定判据和超时
在节点启动时加载。增大高度时，`takeoff_timeout` 必须大于生成的轨迹
时长与稳定等待时间之和，否则拒绝起飞。

`auto_takeoff: false` 可恢复只进入悬停的行为。仿真参数默认保持关闭。

无硬件测试使用以下命令：

```bash
clang++ -std=c++17 -Wall -Wextra -Werror \
  -I src/flygripper_flight/include \
  src/flygripper_flight/test/test_takeoff.cpp -o /tmp/hiarm-test-takeoff
/tmp/hiarm-test-takeoff

# 在 ROS One/ROS Noetic 工作区环境中运行；rostest 会创建独立 ROS master。
source devel/setup.bash
rostest flygripper_flight auto_takeoff.test
```

单元检查覆盖爬升限制、连续稳定判断、相对高度、重复起飞和错误条件。
ROS 测试运行真实飞行节点，模拟 MAVROS 服务与传感器，覆盖模式确认、
地面状态、解锁、推力参数、CH6、低油门、重复起飞、数据中断和接管。
测试中的里程计跟随参考值，不模拟飞机动力学、APM 电机状态机或落地检测。
本次实现已通过 ARM64 ROS One 编译、爬升参考单元检查和 12 个 ROS 场景。
尚未在 ArduPilot SITL 或真机飞行中验证。首次部署仍需在匹配固件的 SITL、
拆桨检查及受控真机试验中验证首次离地与接管过程。
