# HeroRobot_fyt2023

2022年英雄机器人改进：
---
  由一个STM32改成两个STM32作主控，完整代码基本调试完成（仍有Bug），改进了四轮底盘的功率控制

2023年英雄机器人改进：
---
  双STM32的控制代码调试稳定（修复了两个单片机通信中断的bug）；
  尝试加入与计算机视觉的协同控制，击打匀速旋转物体的命中率较低（仍待修复）

系统架构：
---
  采用 CubeMX 生成的 Keil MDK 工程文件基础上添加其他对应相应功能文件，以实时操作系统 freeRTOS 为核心编写控制代码，操作系统共包含云台，底盘，射击，信息解析等 8 项任务。机器人控制算法主要采用串级 PID 算法，任务内容主要包括计算输出电机控制，电机反馈信息接受与处理，基于陀螺仪的角度解算，基于裁判系统实时信息的功率，热量控制功能及其他机器人附加功能。


运行流程：
---
  底盘和云台分别设置STM32作为主控板，双板之间由CAN2作为通信线进行通信。
  云台主控主要负责整车逻辑控制，接收处理控制信息，将处理后的控制信息转发给底盘主控；底盘主控主要作为中转站，处理裁判系统和超级电容的数据，并向云台主控反馈部分控制信息（包括发弹标志信息，偏航电机角度等）。
  以下为数据流图：

  主要使用串级PID算法处理数据，以底盘移动控制为例：首先在云台主控接收遥控器的目标速度，通过CAN2发送预处理后的信息到底盘主控，底盘主控进行麦克纳姆轮的运动学解算，并以速度环为内环，电流环为外环，将其与CAN1通信接收电机实际速度与实际电流输入 PID 计算函数，得出计算值输出再通过CAN1通信输出到电机上，期间输入输出均经过滤波处理。

重点功能：
---
  机器人发弹逻辑：英雄机器人的发弹原理主要是转动橡胶的摩擦轮，靠摩擦力和挤压将42mm的弹丸射出
    控制摩擦轮转动，初始时根据裁判系统射速上限自动设定旋转速度；
    当机器人接收到遥控器发送的发弹信号时，控制底盘拨弹电机以固定转速旋转，推动弹丸至摩擦轮处，经摩擦轮加速后弹丸射出；
    弹丸射出后，会检测到带动摩擦轮旋转的电机掉速，此时认为打出一颗弹丸，立刻停止底盘拨弹电机的旋转。
    另外，根据实际非理想的情况，增设：检测弹丸卡死的防堵转机制、检测弹丸打空的自动停转机制。

修复的Bug:
---
  1.解决了云台主控和底盘主控的CAN2通讯中断和通讯数据错误问题。CAN通讯为半双工通信，两主控板以相同频率互相发送CAN数据报时会产生冲突导致数据发送失败，失败累积会导致CAN线进入休眠，通信停止。现两主控板采用不同的频率互发CAN数据，并使能CAN线的自动唤醒等功能，使机器人基本不会出现因CAN通讯问题导致的疯车和瘫痪问题。
  2.解决了偶然出现的云台疯转问题。经调试发现是将陀螺仪数据以16位传输导致溢出，选择将16位数据扩充到32位。

待修复的Bug:
---
  1.更换新的云台结构后云台后坐力明显变大，调整新的pid参数变得非常困难
  2.云台不能很好的对视觉识别数据做出响应，响应通常速度较慢或振荡。

PS:
---
感谢上海交通大学云汉交龙战队、大连交通大学的开源分享
