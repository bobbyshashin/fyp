# fyp
**Final Year Project at HKUST: Collaboration between UAV &amp; UGV (with LI Zimo &amp; ZHANG Yuanzhao)**

##TODO:

- Basic framework based on ROS (almost done)
- UGV control
- UGV onboard sensor fusion (imu+gps+encoder)
- AprilTag / Aruco Code detection & positioning (UAV/UGV互相观测校准)
- Auto-landing?
- To be continued...

![](http://s5.sinaimg.cn/large/001PLcxJgy6LwX57B0oe4&690)

（极不完全版的）DJI 经纬 M100入门开发教程

从读代码开始：

我们拿DJI Onboard SDK -- ROS Example例程为例，其他的例程（Qt, STM32, Command Line）大同小异。

https://github.com/dji-sdk/Onboard-SDK-ROS

这里面有7个repository / ROS Package，暂时先忽略掉其中5个，一开始我们从dji_sdk和dji_sdk_lib这两个入手

我们先来看dji_sdk_lib这个repository
顾名思义，这是DJI Onboard SDK的库（libraries），阅读这里的代码可以知道目前的SDK都提供了哪些库函数，具体可以实现什么样的功能

首先，我们点进include文件夹来看看头文件们

打开DJI_API.h, 可以看到这里定义了一个名叫CoreAPI的class，这是一个核心的类

接下来找到这个class里面的一个函数：getBroadcastData()；返回值类型为“BroadcastData”，这是一个包含了几乎所有飞控回发的飞行数据的结构体。那么，这个BroadcastData里面究竟包含了哪些数据呢？

我们打开DJI_Type.h来看看这个结构体的定义:
```
typedef struct BroadcastData {

    unsigned short dataFlag;
    TimeStampData timeStamp;
    QuaternionData q;
    CommanData a;
    VelocityData v;
    CommonData w;
    PositionData w;
    MagnetData mag;
    GPSData gps; // For A3 Flight Controller only
    RTKData rtk; // For A3 Flight Controller only
    RadioData rc;
    GimbalData gimbal;
    FlightStatus status;
    BatteryData battery;
    CtrlInfoData ctrlInfo;

    uint8_t activation;

} BroadcastData;
```
可以清楚地看到，里面有时间戳、四元数、位置数据、磁力计、遥控器通道、云台数据、电池电量等等信息，每类数据的具体格式的定义也在这个头文件里面。

比如遥控器通道数据RadioData：
```
typedef struct RadioData
{
  int16_t roll; 
  int16_t pitch;
  int16_t yaw;
  int16_t throttle;
  int16_t mode;
  int16_t gear;

} RadioData;
```
roll, pitch, yaw就是横滚、俯仰、偏航杆量，throttle是油门杆量，mode是遥控器左上方的飞行模式拨杆：P / A / F , gear是右下角自动返航按键上的拨杆

接下来举个栗子，如果想要获得目前遥控器通道的油门杆量，只要在自己的程序里调用这个函数：

`int16_t my_throttle = coreAPI->getBroadcastData().RadioData.throttle;`

（其中coreAPI是一个事先创建的CoreAPI指针类的对象）



![](http://s5.sinaimg.cn/large/001PLcxJgy6LwX57B0oe4&690)
##避坑指南

**常见Bug汇总：**

收不到M100飞控回发数据：
--- PC调参软件DJI Assistant 2里 “启用API控制”未勾选

无法使用SDK控制飞行器：
--- 多种可能性：
    1. PC调参软件DJI Assistant 2里 “启用API控制”未勾选
    2. 遥控器档位未拨到F档
    3. 未夺取控制权（Obtain Control）

DJI Go App不提示激活（新设备）：
--- 换一个移动设备（iOS换成Android, vice versa...）

Guidance不工作：
--- Guidance固件并非最新：http://www.dji.com/cn/product/guidance/info#downloads

（个人感觉90%来自新用户报告的Guidance无法使用的bug都是因为固件没升级……首先确保PC端Guidance调参软件的版本为最新，再进入调参软件里检查Guidance固件是否为最新，因为Guidance调参软件是不能自己更新的…必须从官网下载最新的安装）
