# 1.项目介绍
我们的整个项目是基于Dynamixel公司的DynamixelSDK构建的，包括了调试自稳模式PID的balance.py文件和实现卡尔曼滤波的Kalman.py文件和实现所有功能（包括移动和自稳）的final.py文件。
Kalman.py文件参考了，其余代码为自己编写
https://github.com/rocheparadox/Kalman-Filter-Python-for-mpu6050
# 2.环境配置

由于我们的项目是基于ROBOTIS公司的SDK设计的我们首先要配置SDK的环境。具体流程：
首先克隆我们的库
```
git clone https://github.com/Splinting/Mobile-Hexapod-Robot-LANGZHU/MainControl
```
接着依据我们的setup.py预处理文件
```
cd ~/MainControl/python 
sudo python setup.py install
```
除此之外我们仍需要一些额外库配置numpy进行矩阵计算，smbus完成树莓派的IIC通讯(如果您使用的是python3)可以使用以下命令配置
```
sudo apt-get update
pip3 install numpy
pip3 install smbus
```
接着
就可以执行我们的控制代码
```
cd ~/MainControl/python/tests/protocol1.0
python final.py
python balance.py
```
这里只是简单,介绍具体流程可以参考ROBOTIS公司的官方介绍：[DYNAMIXEL Quick Start Guide in Python - YouTube](https://www.youtube.com/watch?v=LAizFTTdL8o)) 
# 3.具体使用
# 3.1final.py完整形态
首先代码会自检串口连接和舵机通讯，在完成所有操作后会提示模式选择
```Python
"Press key to Start, and chooose model 'm' for low move and 'b' for balence and 'l' for dance and 'h' for high step"
```
如果选择了移动模式每个循环都会读取键盘指令，执行前进后退旋转等指令。同时每个模式都可以通过“ESC”退出程序，或者通过“b”，“d”来实现模式之间的转化，每次成功的指令都会有相应的提示
```Python
"Change pattern to high step!!!"
"Finish!!!"
```
在选择进入balance模式的时候，为了IMU的稳定初始化和机体姿态的预设，会暂停3秒钟在进入自稳状态
# 3.2balance.py自稳模式
在这个文件里我们实现了自稳的调控，对外的接口有五个
```Python
Kp = 0.1
Ki = 0.0
Kd = 0.02
SPE_INTEGRAL_START_ERR = 2.0
SPE_INTEGRAL_MAX_VAL = 20.0
```
可以通过对这五个值得修改实现对pid调控，也可以使用
```Python
print("roll"+str(roll)+"pitch"+str(pitch))
```
打印出实时的更新值，通过绘图软件绘图检测pid的调试情况

 
