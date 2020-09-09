[toc]
# TI mmWave Radar ROS32底盘设计文档

## 一、概述

### 1.1 硬件规格

>TI mmWave Radar ROS32基于DJI RoboMaste机器人平台基于模块化设计，支持快拆结构，各模块可单独编程与调试使用。

#### 1.1.1 结构组成

 1. 底盘模块
   
   + 使用麦克纳姆轮，支持全向运动
   + 动力系统由RoboMaster M3508 P19 无刷直流减速电机 和RoboMaster C620 电子调速器组成
   + 使用RoboMaster 开发板C型 (STM32F427) 作为主控板
   + 接口：19: firmware config pin.

2. 云台模块

+ 采用云台支持1自由度的旋转运动
+ 云台动力系统由RoboMaster GM6020 无刷电机 (包含电调) 组成
+ 使用RoboMaster Development Board Type C (STM32F427) 作为主控板
+ 接口：17: pwm pin 18: trigger pin


#### 1.1.2 硬件参数
|  结构   | 参数  |
|  ----  | :----:  |
| 整机尺寸  | 未定 |
| 重量（含电池）  | 未定 |

|  性能   | 参数  |
|  ----  | :----:  |
| 最大前进速度  | 3m/s |
| 最大平移速度  | 2m/s |
| 云台 Yaw轴 转动角度  | -90°~90° |

|  电池   | 参数  |
|  ----  | :----:  |
| 型号  | DJI TB47D / DJI TB48D |
| 类型  | LiPo 6s |
| 电压  | 22.8v |
| 电池容量  | 4500 mAH / 5700 mAH |

|  通信接口   | 参数  |
|  ----  | :----:  |
| 接口类型  | Micro USB |
| 通信方式  | STM32虚拟串口 |
| 波特率  | 921600 |

#### 1.1.3 硬件体系

1. 主控 MCU：STM32F407IGHx，配置运行频率180MHz
2. 模块通信方式：CAN；CAN设备：电机电调、陀螺仪模块
3. 上下层通信方式：USB虚拟串口
4. 麦轮安装方式：O型

### 1.2 软件框架

>整个软件系统解耦了驱动模块，功能模块和算法模块，以增强平台的可移植性，提高不同领域开发者合作开发的效率。

整个软件系统架构如下图所示

![软件架构](软件架构.png)

开发者基于该平台可以自底向上进行编程开发，来控制机器人完成不同的任务，整个框架分为两个部分

作为底层计算平台，STM32微控制器以有限的计算能力运行RTOS（实时操作系统）完成实时各种任务：
+ 以编码器或IMU为反馈的闭环控制任务
+ 传感器信息的采集、预处理与转发，用于姿态估计
+ 环境感知，包括实时定位、建图、目标检测、识别和追踪
+ 运动规划，包括全局规划（路径规划），局部规划（轨迹规划）
+ 基于观察-动作的决策模型接口，适配各种不同的决策框架，例如FSM（有限状态机），Behavior Tree（行为树）以及其他基于学习的决策框架。

两个运算平台之间通过串口以规定的开源协议进行通信，平台同时提供了基于ROS的API接口，可以在ROS中利用C++或Python编程，进行二次开发。基于这个接口，开发者可以控制机器人底盘和云台的运动，控制发射机构以输入的频率、射速与数量发射弹丸，同时实时接收传感器信息与比赛数据等功能。更高级的应用中，开发者可以根据roborts_base相关文档定义自己的协议，以扩展机器人的功能。

#### 1.2.1 嵌入式微控器

1. 使用免费及开源的 freertos 操作系统，兼容其他开源协议 license；
2. 使用标准 CMSIS-RTOS 接口，方便不同操作系统或平台间程序移植；
3. 提供一套抽象步兵机器人bsp，简化上层逻辑；
   
    ```
    RoboRTS-Firmware
    ├── application #上层应用任务，包括系统任务
    ├── bsp #C型开发板适配包
    ├── components   #通用机器人模块，包括命令行，驱动模块和系统组件
    ├── doc         #说明文档
    ├── MDK-ARM  #armcc工具链
    └── tools #cmake gnu toolchain. You should install make, cmake, arm-none-eabi and set env value.
    ```
4. 软件环境
   + Toolchain/IDE : MDK-ARM V5 / arm-none-eabi
   + package version: STM32Cube FW_F4 V1.24.0
   + FreeRTOS version: 10.0.0
   + CMSIS-RTOS version: 1.02

#### 1.2.2 x86机载平台

1. 整体系统以 机器人传感器->感知->决策->规划->控制->执行器 的环路进行架构，不同模块具体以ROS Package的形式维护，模块和其数据流如下图所示 
   ![数据流图](数据流图.png)
2. 中心模块集成传感器模块（雷达、相机、IMU等）、嵌入式控制平台（执行实时任务，如闭环控制和数据采集与预处理）与执行器（电机等），负责sensing和control两大任务，具体ROS Package为包含嵌入式SDK的roborts_base，雷达的mmwave_drive。
3. ROS Package
   |  Package   | 功能  | 内部依赖  |
   |  :----:  | :----:  |:----:  |
   | roborts  | Meta-package | - |
   | roborts_base  | 嵌入式通信接口 |roborts_msgs |
   | roborts_bringup	  | 启动包 | roborts_base、roborts_common、roborts_localization、roborts_costmap、roborts_msgs、roborts_planning|

## 二、快速入门

### 2.1 外设端口配置

根据硬件接口（串口，USB或者ACM）来配置/etc/udev/rules.d中的udev文件，分别实现STM32设备虚拟串口和激光雷达的设备绑定：

首先连接STM32设备的虚拟串口，lsusb可以查看Vendor和Product的ID，然后创建并配置/etc/udev/rules.d/roborts.rules文件
```bash
  KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777", SYMLINK+="serial_sdk"
```
同理配置雷达。然后重新加载并启动udev服务，可能需要重新插拔设备后生效。

### 2.2 软件依赖配置

默认安装ROS Kinetic。若使用其它平台，可参考ROS Wiki的安装教程安装ROS。安装ROS所需第三方依赖包，以及SuiteSparse，Glog，Protobuf等其他依赖。
```bash
sudo apt-get install -y ros-kinetic-opencv3             \
                        ros-kinetic-cv-bridge           \
                        ros-kinetic-image-transport     \
                        ros-kinetic-stage-ros           \
                        ros-kinetic-map-server          \
                        ros-kinetic-laser-geometry      \
                        ros-kinetic-interactive-markers \
                        ros-kinetic-tf                  \
                        ros-kinetic-pcl-*               \
                        ros-kinetic-libg2o              \
                        ros-kinetic-rplidar-ros         \
                        ros-kinetic-rviz                \
                        protobuf-compiler               \
                        libprotobuf-dev                 \
                        libsuitesparse-dev              \
                        libgoogle-glog-dev              \
```
安装RoboRTS基础功能包
```bash
# 创建工作空间文件夹
mkdir -p roborts_ws/src
# 切换至src目录
cd roborts_ws/src
# 下载RoboRTS源码
git clone https://github.com/RoboMaster/RoboRTS
# 编译源码
cd ..
catkin_make 
# 加载环境变量
source devel/setup.bash

```

## 三、SDK

### 3.1 机器人驱动模块
机器人驱动模块是连接底层嵌入式控制板与上层机载电脑的桥梁，通过虚拟串口，基于一套通用的协议来定义双向传输的数据种类，进而完成数据互传的任务。

在机载端，roborts_base提供了ROS接口，接收底层嵌入式控制板发送的数据并控制其完成对机器人的模式切换和运动控制。

roborts_base依赖roborts_msgs中定义的相关消息类型。模块文件目录如下所示

```bash
roborts_base
├── CMakeLists.txt
├── cmake_module
│   └── FindGlog.cmake
├── chassis                          #底盘模块ROS接口封装类
│   ├── chassis.cpp
│   └── chassis.h
├── gimbal                           #云台模块ROS接口封装类
│   ├── gimbal.cpp
│   └── gimbal.h
├── config
│   └── roborts_base_parameter.yaml  #参数配置文件
├── roborts_base_config.h            #参数读取类
├── roborts_base_node.cpp            #核心节点Main函数
├── ros_dep.h                        #包括所有协议对应ROS消息的头文件
├── roborts_sdk                     
│   ├── ...
└── package.xml
```
其中roborts_sdk文件内原生协议SDK，无任何ROS的依赖，详见

```bash
    ├── roborts_sdk
    │   ├── hardware                 #硬件层完成对协议数据的收发
    │   │   ├── hardware_interface.h #硬件层基类
    │   │   ├── serial_device.cpp    #串口设备实现类
    │   │   └── serial_device.h
    │   ├── protocol                 #协议层完成对协议的解包与打包
    │   │   ├── protocol.cpp         #协议层类
    │   │   ├── protocol_define.h    #协议具体定义消息类型的头文件
    │   │   └── protocol.h
    │   ├── dispatch                 #分发层完成对消息的分发
    │   │   ├── dispatch.h           #分发层类
    │   │   ├── execution.cpp        #分发层执行类
    │   │   ├── execution.h
    │   │   ├── handle.cpp           #roborts_sdk三层的对外接口类
    │   │   └── handle.h
    │   ├── sdk.h                    #roborts_sdk对外头文件
    │   ├── test
    │   │   └── sdk_test.cpp         #协议测试文件
    │   └── utilities
    │       ├── circular_buffer.h    #环形缓冲池
    │       ├── crc.h                #crc校验文件
    │       ├── log.h                #日志记录文件
    │       └── memory_pool.h        #内存池
```

在该模块的核心运行节点roborts_base_node中，创建所需模块的对象（如底盘、云台）并初始化后，即可正常执行通信任务。

其ROS节点图示如下：
![底盘模块](底盘模块.png)

输入

   + /cmd_vel (geomtry_msgs/Twist)：底盘速度的控制量，即在下一个控制周期内，底盘做匀速运动。
   + /cmd_vel_acc (roborts_msgs/TwistAccel)：底盘速度与加速度的控制量，即在下一个控制周期内，底盘以给定的速度做匀加速度运动。
   + /cmd_gimbal_angle (roborts_msgs/GimbalAngle)：云台角度的控制量，根据相对角度标志位判断是绝对角度控制还是相对角度控制。
   + /set_gimbal_mode (roborts_msgs/GimbalMode)：设定云台的模式。

输出

   + /odom (nav_msgs/Odometry)：底盘里程计信息。
   + /uwb (geometry_msgs/PoseStamped)：底盘在UWB坐标系中的位姿信息。
   + /tf (tf/tfMessage)：从base_link->odom的变换和从base_link->gimbal的变换。

### 3.2 雷达驱动模块

#### 3.2.1 文件结构
TI提供的最主要驱动代码文件夹是“ti\_mmwave\_rospkg”当然要顺利运行还需要另一个文件夹“serial”但是因为我们不需要改动后者，所以解析的内容旨在分析“ti\_mmwave\_rospkg”源码文件夹。

```bash
ti_mmwave_rospkg
├──  cfg #存放默认雷达配置文件，每个启动项的配置文件与应用同名。
├── include  #存放头文件，定义了数据格式等
├── launch   #存放启动项文件
├── docs         #说明文档
├── src  #驱动源码
└── srv #自定义ROS服务
```

#### 3.2.2 服务

其中的srv自定义服务用于将配置文件载入雷达flash当中，需要启动的节点除了mmWaveQuickConfiguickConfig外还需要mmWaveCommSrv用于接收配置文件。启动launch文件的时候会自动启动mmWaveCommSrv。

服务传递的信息格式也很简单，发收分别通过string型的字符串comm和resp完成。对应的头文件和源文件实现也比较常规。唯一值得注意的地方是在编程时头文件内需要包含服务格式“mmWaveCLI”对应的头文件，这个文件是在编译过程当中自动生成的，因此需要在CMakeLists中写好。

#### 3.3.3 节点

先介绍一下PC处理数据的流程，该过程分为三步：

* 利用“serial”库从串口获取数据。
* 利用PC内的两块缓存区，缓存1获取数据A时（主要是找到魔数，并把它去掉），缓存2对之前获取到的数据B进行重新排序。这个过程是同时发生的。
* 当A完成解包，B完成排序后，缓存1把A给缓存2处理，自己再读入新的数据。（乒乓缓存）

#### 3.3.4数据解包

读取数据的过程很简单，检测一个长度为8字节的固定开头即可（magic word）。我们比较关心的是每一帧数据是如何被ROS理解的。这里每种型号的EVM发送的数据格式都是类似的，只有一些小细节不太一样。比如，1443的包头字长是40字，而其他型号有44个字，这是因为14系列缺少一个关于子帧数的信息。但是这些细节不影响我们分析，因此我们在这里将其忽略。详细的解包要求见下表：

|  index  | context |
|:-------:|:-------:|
|0x00~0x04| version |
|0x05~0x08| total packet length |
|0x09~0x12| EVM platform |
|0x13~0x16| frame number |
|0x17~0x20| cpu cycles time |
|0x21~0x24| objects number |
|0x21~0x24| TLVs number |
|0x25~0x28| sub frame number |

该表是数据包包头的内容解析，较为的简单，如果一个数据包的包头解析正确，且total packet length正确，就可以对该包的剩余部分，即检测数据部分进行解析。这里还需要额外提到的还有一点，关于TLV（Tag Length Value）。TLV是一种节点间的通信协议，通信前需要确定协议的形式和长度，这些在TLVs number中已经包含了。

检测数据会被存入RScan当中，并作为一个消息在之后发布。（随后要用到多个雷达协同工作时话题名称需要重新更改、编译）RScan可以看作一个一维向量，长度是探测到的物体个数。

```c++
    DataUARTHandler_pub = nodeHandle->advertise< sensor_msgs::PointCloud2 >("RScan", 100);
```

对RScan进行内存分配后，计算所有目标点可能出现的空间范围，其中用到的最大俯仰角和方向角是一般被设为90度。

```c++
    maxAllowedElevationAngleDeg = 90; // Use max angle if none specified
    maxAllowedAzimuthAngleDeg = 90; // Use max angle if none specified
```

RScan是pointcloud2型的消息，因此需要将对应的雷达数据转化为点云。转化的方式根据EVM当中烧录的SDK文件不同而有所变化。在3.x之后的版本当中获取检测数据和转换为点云的方式更为简单，因此我们以3.x之后的版本为例来进行解析。

首先先声明存储雷达数据的类实体：

```c++
    mmwData.numObjOut = mmwData.header.numDetectedObj;
```

之后对雷达数据当中的每个目标点做处理，分别拷贝每个检测目标的坐标（xyz单位m）以及速度信息（单位m/s），像这样：

```c++
    memcpy( &mmwData.newObjOut.x, &currentBufp->at(currentDatap), sizeof(mmwData.newObjOut.x));//获取x坐标信息
    currentDatap += ( sizeof(mmwData.newObjOut.x) );//移动数据指针到y坐标信息首位
```

接下来需要把这些坐标信息转化为ROS能够接受的RScan格式，这其中涉及到一个坐标轴的转化问题。ROS默认的x轴平行于EVM数据默认的y轴；其y轴平行于EMV的x轴、且指向相反；两者z轴重叠。

```c++
     RScan->points[i].x = mmwData.newObjOut.y;   // ROS standard coordinate system X-axis is forward which is the mmWave sensor Y-axis
     RScan->points[i].y = -mmwData.newObjOut.x;  // ROS standard coordinate system Y-axis is left which is the mmWave sensor -(X-axis)
     RScan->points[i].z = mmwData.newObjOut.z;   // ROS standard coordinate system Z-axis is up which is the same as mmWave sensor Z-axis
```

3.x以后的版本当中将幅度值存在了整个检测目标数据之后读取，方式类似坐标，获取后直接传回RScan即可。但是诸如检测目标的速度信息，以及噪声等信息，虽然驱动当中写了相关的代码可以在指针指向该数据时做一些处理，但最终都没有赋值给RScan，也就是没有传入到ROS系统当中，因此不再赘述。