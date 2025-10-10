# Dependencies
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


