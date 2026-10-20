# Dependencies
## UART servo permissions on the board

`servo_node` requests FIFO priority 95. For the `rock` account, install
`/etc/security/limits.d/90-rock-servo.conf` with:

```text
rock - rtprio 95
```

Reconnect SSH after installing the file. Confirm `ulimit -r` prints `95` and
the account has read/write access to `/dev/ttyS4` through the `dialout` group.
`launch_with_root.sh` then launches the UART servo as the current user without
background password prompts. Other actuator configurations retain the sudo path.

If `pub_morph.sh` has no effect, check
`rostopic info /flygripper/actuator_command`:
`/flygripper/servo_node` must appear as a subscriber.

Install some tools
```shell
sudo apt install -y python3-catkin-tools 
```

# Introduction to Can
## Can-Usb converter
- Can-Usb converter with socket-can (candleLight firmware)
- Can-Usb converter with uart (DM Tech)

## Can utilities
系统安装  can-utils 之后，可以对can0 设备操作：
```shell
#linux系统安装  can-utils，如果是嵌入式系统，需要移植can-utils工具
sudo apt-get install can-utils          

# 查看can口
ifconfig -a
# 设置can波特率为 1M
sudo ip link set can0 type can bitrate 1000000 
# 启动can
sudo ifconfig can0 up
#配置txqueuelen的传输数据的缓冲区的存储长度
sudo ifconfig can0 txqueuelen 1000        

# 关闭can
sudo ifconfig can0 down
# can0发送ID为0x999的数据0xdeadbeef
cansend can0 999#DEADBEEF
# can0接收数据
candump can0                            
```


