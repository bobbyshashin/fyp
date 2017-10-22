# fyp
**Collaborative Air-Ground Target Searching in Complex Environments**

- Our [conference paper](./uav_ugv_collaboration.pdf) is accepted by [SSRR 2017](http://www.ssrr-conference.org/2017/)

## DJI Manifold Setup Manual 

### 1. Install ROS Indigo (Ubuntu armhf)

- First follow the instructions [here](http://wiki.ros.org/indigo/Installation/UbuntuARM)

- Then install several 3rd-party ROS Packages:
```
sudo apt-get install ros-indigo-cv-bridge
sudo apt-get install ros-indigo-aruco
sudo apt-get install ros-indigo-camera-info-manager
sudo apt-get install ros-indigo-v4l-utils
```

### 2. Install CUDA 6.5: 

- Download the source file from [here](http://developer.download.nvidia.com/embedded/L4T/r21_Release_v3.0/cuda-repo-l4t-r21.3-6-5-prod_6.5-42_armhf.deb)

- Installation: 
```
sudo dpkg -i cuda-repo-l4t-r21.3-6-5-prod_6.5-42_armhf.deb
sudo apt-get update
sudo apt-get install cuda-toolkit-6-5
```

- Set GPU to be accessible by current user:

```
sudo usermod -a -G video $USER
```


- Set the environment variable:

```
gedit ~/.bashrc
```

- Then add the following lines to ~/.bashrc:

```
export PATH=/usr/local/cuda-6.5/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-6.5/lib:$LD_LIBRARY_PATH
```


- Then source the .bashrc file again:

```
source ~/.bashrc
```

### 3. Install OpenCV4Tegra:

- Download the source file from [here](http://developer.download.nvidia.com/embedded/OpenCV/L4T_21.2/libopencv4tegra-repo_l4t-r21_2.4.10.1_armhf.deb)

- Install the dependencies:
```
sudo dpkg -i libopencv4tegra-repo_l4t-r21_2.4.10.1_armhf.deb
sudo apt-get update
sudo apt-get install libopencv4tegra libopencv4tegra-dev libopencv4tegra-python
sudo apt-get install libgtk2.0-dev pkg-config
```

### 4. Install OpenCV 2.4.10

- Download the source file from [here](https://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.4.10/opencv-2.4.10.zip/download)

- Unzip:

```
unzip opencv-2.4.10.zip
```

- Compile OpenCV:

Under the parent directory of "opencv-2.4.10", make another directory named "build":
```
mkdir build
cd build
cmake -DWITH_CUDA=ON -DCUDA_ARCH_BIN="3.2" -DCUDA_ARCH_PTX="" -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF ../opencv-2.4.10/
```
Take a look at the message after cmake command is completed. If `Use CUDA` is `Yes`, then CUDA can be used.

PS: If "cmake" cannot be find after typing the above commands, please install the essential applications first:
```
sudo apt-get install build-essential make cmake g++
```

- Install OpenCV:
```
sudo make -j4 install
```

- Modify the environment variables:
```
echo "# Use OpenCV and other custom-build libraries" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/" >> ~/.bashrc
source ~/.bashrc
```

## TODO:

- Basic framework based on ROS (almost done)
- UGV control
- UGV onboard sensor fusion (imu+gps+encoder) 
- AprilTag / Aruco Code detection & positioning (UAV / UGV 互相观测校准)
- Auto-landing?
- To be continued...

![](http://s5.sinaimg.cn/large/001PLcxJgy6LwX57B0oe4&690)

## （极不完全版的）DJI 经纬 M100入门开发教程

**从读代码开始：**

我们拿[DJI Onboard SDK -- ROS Example](https://github.com/dji-sdk/Onboard-SDK-ROS)为例，其他的例程（Qt, STM32, Command Line）大同小异。

这里面有7个repository / ROS Package，暂时先忽略掉其中5个，一开始我们从_dji_sdk_和_dji_sdk_lib_这两个入手

我们先来看_dji_sdk_lib_这个repository
顾名思义，这是DJI Onboard SDK的库（libraries），阅读这里的代码可以知道目前的SDK都提供了哪些库函数，具体可以实现什么样的功能

首先，我们点进_include_文件夹来看看头文件们

打开_DJI_API.h_, 可以看到这里定义了一个名叫_CoreAPI_的class，这是一个核心的类

接下来找到这个class里面的一个函数：_getBroadcastData()_；返回值类型为_BroadcastData_，这是一个包含了几乎所有飞控回发的飞行数据的结构体。那么，这个_BroadcastData_里面究竟包含了哪些数据呢？

我们打开_DJI_Type.h_来看看这个结构体的定义:
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
可以清楚地看到，里面有时间戳、四元数、位置数据、磁力计、遥控器通道、云台数据、电池电量等等信息，每类数据的具体定义也在此头文件内。

比如遥控器通道数据_RadioData_：
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
_roll_, _pitch_, _yaw_：横滚、俯仰、偏航杆量  
_throttle_：油门杆量  
_mode_：遥控器左上方的飞行模式拨杆：P / A / F  
_gear_：遥控器右下角自动返航按键上的拨杆（原本用于控制起落架）  

接下来举个栗子，如果想要获得目前遥控器通道的油门杆量，只要在自己的程序里调用这个函数：

`int16_t my_throttle = coreAPI->getBroadcastData().RadioData.throttle;`

（其中_coreAPI_是一个事先创建的pointer,类型为_CoreAPI*_）



![](http://s5.sinaimg.cn/large/001PLcxJgy6LwX57B0oe4&690)

## 避坑指南

**常见Bug汇总：**

- 收不到M100飞控回发数据：  
--- PC调参软件DJI Assistant 2里 “启用API控制”未勾选

- 无法使用SDK控制飞行器：  
--- 多种可能性：
    1. PC调参软件DJI Assistant 2里 “启用API控制”未勾选
    2. 遥控器档位未拨到F档
    3. 未夺取控制权（Obtain Control）

- DJI Go App不提示激活（新设备）：  
--- 换一个移动设备（iOS换成Android, vice versa...）

- Guidance不工作：  
--- Guidance固件并非最新：http://www.dji.com/cn/product/guidance/info#downloads

（个人感觉90%来自新用户报告的Guidance无法使用的bug都是因为固件没升级……首先确保PC端Guidance调参软件的版本为最新，再进入调参软件里检查Guidance固件是否为最新，因为Guidance调参软件是不能自己更新的…必须从官网下载最新的安装）
