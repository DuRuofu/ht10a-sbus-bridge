CH1  右摇杆X   转向
CH2  右摇杆Y   前进/后退
CH3  左摇杆Y   预留
CH4  左摇杆X   预留

CH5  SWA（二段）  急停 / 使能 （自动回中,拨一下切换状态,默认状态为急停）
CH6  SWB（二段）  手动 / 自动控制权  （手动控制模式下，才能发送控制指令）

CH7  SWC（二段）  预留
CH8  SWD（三段）  速度档位（低（50%） / 中（75%）  / 高（100%）））

CH9  VRA   旋钮  最大线速度调节
CH10 VRA   旋钮  最大角速度调节

## 使用说明

运行节点:

```
colcon build
source install/setup.bash
ros2 run rc_control_node rc_control_node --ros-args -p port:=/dev/ttyUSB0
```

默认参数：

- 串口端口：/dev/ttyUSB0
- 波特率：115200

功能说明
- 急停/使能：通过 CH5 (SWA) 控制，默认为急停状态，拨动开关可切换状态
- 手动/自动模式：通过 CH6 (SWB) 切换，仅在手动模式下发送控制指令
- 速度控制：CH1/CH2 控制转向和前进后退
- 速度档位：CH8 (SWD) 提供三档速度调节（50%/75%/100%）
- 速度限制：CH9/CH10 旋钮调节最大线速度和角速度
- 数据发布：节点发布 cmd_vel 消息到 /cmd_vel 话题
