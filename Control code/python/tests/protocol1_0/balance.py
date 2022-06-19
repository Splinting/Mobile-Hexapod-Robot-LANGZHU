from hashlib import new
import os
from random import sample
import unittest
#mpu6050 needed library
from Kalman import KalmanAngle
import smbus			#import SMBus module of I2C
import time
import math
#joint calculation needed library
from numpy import *
import numpy as np
#prepare the system
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

#prepare the AX-12A environment
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_MX_TORQUE_ENABLE       = 24                  # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION       = 30
ADDR_MX_PRESENT_POSITION    = 36
ADDR_MX_MOVING_SPPED        = 32

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 1000000
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1023            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

#define the motion parameter
test_leg = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18]
Steering_Num = 18
Frame_Num = 20

model = 0

Goal_angular = []

Origin_banlance_angular = [512, 210, 814, 512, 814, 210, 511, 210, 814, 511, 814, 210, 512, 210, 814, 513, 814, 210]

Present_position = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,]
# function defined to help to calculate the joint angular
rad2deg = 57.29578
j3bias=10.91
#机械位形信息
#三条腿长
l1=45
l2=93.5
l3=124.14
#六个关节base到中心（世界坐标系原点）的距离/角度信息
xx1=115.0
yy1=65.0
yy2=95.0
theta1=80.0
theta2=89.93
theta3=100.0
#六个脚在地面的接触点到中心的初始距离信息（初始偏置）
xxx1=142.15
yyy1=224.71
yyy2=257
zzz1=-118
#定义扩展旋转平移矩阵
def TROTZ(theta):
    r = theta/rad2deg
    TR = np.array([[math.cos(r),-math.sin(r),0.0,0.0],[math.sin(r),math.cos(r),0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]])
    return TR

def TRANSL(x,y,z):
    TL = np.array([[1.0,0.0,0.0,x],[0.0,1.0,0.0,y],[0.0,0.0,1.0,z],[0.0,0.0,0.0,1.0]])
    return TL   

def cd2joint(pxx):
    x = pxx[0]
    y = pxx[1]
    z = pxx[2]
    A=math.sqrt(x*x+y*y)-l1
    B=-z
    if(A>0):
        assert(A*A+(z-l2)*(z-l2))>l3*l3,"不在工作空间内!"
        assert(A*A+z*z)<(l2+l3)*(l2+l3),"不在工作空间内!"
    else:
        assert(A*A+z*z)>(l3-l2)*(l3-l2),"不在工作空间内!"
        assert(A*A+(z+l2)*(z+l2))<l3*l3,"不在工作空间内!"

    #取关节角度
    #r1
    r1=math.atan2(y,x)*rad2deg
    assert r1>-60,"r1<-60, dangerous!"
    assert r1<60,"r1>60, dangerous!"

    #高机位
    if((A*A+z*z)>(l2*l2+l3*l3)):
        #r3
        ccos=math.acos((l2*l2+l3*l3-A*A-B*B)/(2*l2*l3))*rad2deg
        if(ccos>90):
            ccos=180-ccos
        r3=180-ccos-j3bias
        assert r3>90-j3bias,"r3<79.09, dangerous! (可能是低机位的解)"
        assert r3<(180-j3bias),"r3>169.09, dangerous!"
        #r2
        delta=math.atan((l3*math.sin(ccos/rad2deg))/(l2+(l3*math.cos(ccos/rad2deg))))
        ttan=math.atan(B/A)
        r2=-90-(ttan-delta)*rad2deg
        assert r2<0,"r2>0, dangerous!(可能是低机位的解)"
        assert r2>-180,"r2<-180, dangerous!"    
    #低机位   
    else:  
        #r2 
        ccos=math.acos((A*A+B*B+l2*l2-l3*l3)/(2*math.sqrt(A*A+B*B)*l2))*rad2deg
        r2=ccos-(math.atan2(B,A)*rad2deg)-90.0
        assert r2>=-180,"r2<=-180, dangerous!"
        assert r2<0,"r2>0, dangerous!"
        #r3
        ccos2=math.acos((-A*A-B*B+l2*l2+l3*l3)/(2*l2*l3))*rad2deg
        if ccos2>=90.0:
            ccos2 = 180.0 - ccos2
        r3=ccos2-10.91
        assert r3>0,"r3<0, dangerous!"
        assert r3<(90-j3bias),"r3>79.09, dangerous!（可能是高机位的解）"

    q=np.array([[r1],[r2],[r3]])
    return q  

def rotXYAng2jointAng(i,rx,ry):
    #机身脚位置初始偏置
    xo = np.array([xxx1,0.0,-xxx1,xxx1,0.0,-xxx1])   
    yo = np.array([-yyy1,yyy2,-yyy1,yyy1,-yyy2,yyy1]) 
    zo = zzz1

    #(rx，ry)-> p0.0（世界坐标系的目标位置）
    x0=xo[i-1]*math.cos(ry/rad2deg)
    y0=yo[i-1]*math.cos(rx/rad2deg)
    z0=zo-xo[i-1]*math.sin(ry/rad2deg)+yo[i-1]*math.sin(rx/rad2deg)
    
    p00 = np.array([[x0],[y0],[z0],[1]])
    #p0.0->px  (世界坐标到关节坐标)
    pxx = worldCo2jointCo(i,p00)
    #px->ang （关节坐标到关节角）
    try:
        q = cd2joint(pxx)
    except AssertionError as e:
        q = findNext(i,p00)
    return q

def worldCo2jointCo(i,pn):
    pn = np.array([float(pn[0]),float(pn[1]),float(pn[2]),1.0])
    if i == 1:
        Tx_0=TROTZ(theta1).dot(TRANSL(-xx1,yy1,0.0))
    elif i == 2:
        Tx_0=TROTZ(-theta2).dot(TRANSL(0.0,-yy2,0.0))
    elif i == 3:
        Tx_0=TROTZ(theta3).dot(TRANSL(xx1,yy1,0.0))    
    elif i == 4:
        Tx_0=TROTZ(-theta1).dot(TRANSL(-xx1,-yy1,0.0))
    elif i == 5:
        Tx_0=TROTZ(theta2).dot(TRANSL(0.0,yy2,0.0)) 
    elif i == 6:
        Tx_0=TROTZ(-theta3).dot(TRANSL(xx1,-yy1,0.0))
    else:
        Tx_0=np.array([[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]])
        print("sb")
        
    pxx=Tx_0.dot(pn)
    return pxx

def findNext(ii,p00):
    cnt = 0
    pn = np.zeros_like(p00)
    pn[:] = p00
    flag = 0
    while(cnt < 4):
        pn[2] = pn[2] - zzz1
        if(p00[2]>-117.9):
            pn = pn*1.2
        else:
            pn = pn*0.8   
        pn[2]=pn[2]+zzz1   
        cnt=cnt+1 
        pxn=worldCo2jointCo(ii,pn)  

        try:
            qn=cd2joint(pxn)
            flag = 1
            break
        except AssertionError as e:
            continue
        
    assert flag!=0,"无解"
    return qn
# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel Torque
Online = 1
for i in range(len(test_leg)):
    DXL_ID = test_leg[i]
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
       print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
       Online = 0
    elif dxl_error != 0:
       print("%s" % packetHandler.getRxPacketError(dxl_error))
       Online = 0
    else:
        print("Dynamixel %d  has been successfully connected \n", DXL_ID)

if (Online==1):
    print("All the servo is online\n")

# Change Dynamixel Speed
Speed = 1
for i in range(len(test_leg)):
    DXL_ID = test_leg[i]
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPPED, 112)
    if dxl_comm_result != COMM_SUCCESS:
       print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
       Online = 0
    elif dxl_error != 0:
       print("%s" % packetHandler.getRxPacketError(dxl_error))
       Online = 0
    else:
        print("Dynamixel %d  has been changed speed \n", DXL_ID)

if (Speed==1):
    print("All the servo changed speed\n")
file = open("Angular_balance", 'w')    
while 1:      
    for i in range(len(test_leg)):
        DXL_ID = test_leg[i]
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION,Origin_banlance_angular[i])
    for i in range(len(test_leg)):
        DXL_ID = test_leg[i]
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPPED, 100)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            Online = 0
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            Online = 0
        else:
            print("Dynamixel %d  has been changed speed \n", DXL_ID)
    time.sleep(5)        
    #initialization of mpu        
    kalmanX = KalmanAngle()
    kalmanY = KalmanAngle()

    RestrictPitch = True	#Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
    radToDeg = 57.2957786
    kalAngleX = 0
    kalAngleY = 0
    #some MPU6050 Registers and their Address
    PWR_MGMT_1   = 0x6B
    SMPLRT_DIV   = 0x19
    CONFIG       = 0x1A
    GYRO_CONFIG  = 0x1B
    INT_ENABLE   = 0x38
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    GYRO_XOUT_H  = 0x43
    GYRO_YOUT_H  = 0x45
    GYRO_ZOUT_H  = 0x47


    #Read the gyro and acceleromater values from MPU6050
    def MPU_Init():
        #write to sample rate register
        bus.write_byte_data(DeviceAddress, SMPLRT_DIV, 7)

        #Write to power management register
        bus.write_byte_data(DeviceAddress, PWR_MGMT_1, 1)

        #Write to Configuration register
        #Setting DLPF (last three bit of 0X1A to 6 i.e '110' It removes the noise due to vibration.) https://ulrichbuschbaum.wordpress.com/2015/01/18/using-the-mpu6050s-dlpf/
        bus.write_byte_data(DeviceAddress, CONFIG, int('0000110',2))

        #Write to Gyro configuration register
        bus.write_byte_data(DeviceAddress, GYRO_CONFIG, 24)

        #Write to interrupt enable register
        bus.write_byte_data(DeviceAddress, INT_ENABLE, 1)


    def read_raw_data(addr):
        #Accelero and Gyro value are 16-bit
            high = bus.read_byte_data(DeviceAddress, addr)
            low = bus.read_byte_data(DeviceAddress, addr+1)

            #concatenate higher and lower value
            value = ((high << 8) | low)

            #to get signed value from mpu6050
            if(value > 32768):
                    value = value - 65536
            return value


    bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
    DeviceAddress = 0x68   # MPU6050 device address

    MPU_Init()

    time.sleep(1)
    #Read Accelerometer raw value
    accX = read_raw_data(ACCEL_XOUT_H)
    accY = read_raw_data(ACCEL_YOUT_H)
    accZ = read_raw_data(ACCEL_ZOUT_H)

    #print(accX,accY,accZ)
    #print(math.sqrt((accY**2)+(accZ**2)))
    if (RestrictPitch):
        roll = math.atan2(accY,accZ) * radToDeg
        pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
    else:
        roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
        pitch = math.atan2(-accX,accZ) * radToDeg
    print(roll)
    kalmanX.setAngle(roll)
    kalmanY.setAngle(pitch)
    gyroXAngle = roll
    gyroYAngle = pitch
    compAngleX = roll
    compAngleY = pitch

    timer = time.time()
    flag = 0
    #important parameter for balance 
    err_pitch = 0.0
    err_pitch_last = 0.0
    err_pitch_integral = 0.0
    err_roll = 0.0
    err_roll_last = 0.0
    err_roll_integral = 0.0
    bias_roll = 0.0
    bias_pitch = 0.0
    Kp = 0.1
    Ki = 0.0
    Kd = 0.02
    SPE_INTEGRAL_START_ERR = 2.0
    SPE_INTEGRAL_MAX_VAL = 20.0

    while 1:
        if(flag >100): #Problem with the connection
            print("There is a problem with the connection")
            flag=0
            continue
        try:
            #Read Accelerometer raw value
            accX = read_raw_data(ACCEL_XOUT_H)
            accY = read_raw_data(ACCEL_YOUT_H)
            accZ = read_raw_data(ACCEL_ZOUT_H)

            #Read Gyroscope raw value
            gyroX = read_raw_data(GYRO_XOUT_H)
            gyroY = read_raw_data(GYRO_YOUT_H)
            gyroZ = read_raw_data(GYRO_ZOUT_H)

            dt = time.time() - timer
            timer = time.time()

            if (RestrictPitch):
                roll = math.atan2(accY,accZ) * radToDeg
                pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
            else:
                roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
                pitch = math.atan2(-accX,accZ) * radToDeg

            gyroXRate = gyroX/131
            gyroYRate = gyroY/131

            if (RestrictPitch):

                if((roll < -90 and kalAngleX >90) or (roll > 90 and kalAngleX < -90)):
                    kalmanX.setAngle(roll)
                    complAngleX = roll
                    kalAngleX   = roll
                    gyroXAngle  = roll
                else:
                    kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

                if(abs(kalAngleX)>90):
                    gyroYRate  = -gyroYRate
                    kalAngleY  = kalmanY.getAngle(pitch,gyroYRate,dt)
            else:

                if((pitch < -90 and kalAngleY >90) or (pitch > 90 and kalAngleY < -90)):
                    kalmanY.setAngle(pitch)
                    complAngleY = pitch
                    kalAngleY   = pitch
                    gyroYAngle  = pitch
                else:
                    kalAngleY = kalmanY.getAngle(pitch,gyroYRate,dt)

                if(abs(kalAngleY)>90):
                    gyroXRate  = -gyroXRate
                    kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

            #angle = (rate of change of angle) * change in time
            gyroXAngle = gyroXRate * dt
            gyroYAngle = gyroYAngle * dt

            #compAngle = constant * (old_compAngle + angle_obtained_from_gyro) + constant * angle_obtained from accelerometer
            compAngleX = 0.93 * (compAngleX + gyroXRate * dt) + 0.07 * roll
            compAngleY = 0.93 * (compAngleY + gyroYRate * dt) + 0.07 * pitch

            if ((gyroXAngle < -180) or (gyroXAngle > 180)):
                gyroXAngle = kalAngleX
            if ((gyroYAngle < -180) or (gyroYAngle > 180)):
                gyroYAngle = kalAngleY

        except Exception as exc:
            flag += 1
        #pidcontroll
        err_pitch = 0 - pitch
        err_roll = 0 - roll
        #累计项
        if(err_pitch > -SPE_INTEGRAL_START_ERR and err_pitch < SPE_INTEGRAL_START_ERR):
            err_pitch_integral += err_pitch
            if(err_pitch_integral > SPE_INTEGRAL_MAX_VAL):
                err_pitch_integral = SPE_INTEGRAL_MAX_VAL
            if(err_pitch_integral < -SPE_INTEGRAL_MAX_VAL):
                err_pitch_integral = -SPE_INTEGRAL_MAX_VAL
             

        if(err_roll > -SPE_INTEGRAL_START_ERR and err_roll < SPE_INTEGRAL_START_ERR):         
            err_roll_integral += err_roll  
            if(err_roll_integral > SPE_INTEGRAL_MAX_VAL):
                err_roll_integral = SPE_INTEGRAL_MAX_VAL   
            if(err_roll_integral < -SPE_INTEGRAL_MAX_VAL):
                err_roll_integral = -SPE_INTEGRAL_MAX_VAL          

  
        #结果计算
        output_pitch = Kp * err_pitch + Ki*err_pitch_integral + Kd*(err_pitch-err_pitch_last)
        output_roll = Kp * err_roll + Ki*err_roll_integral + Kd*(err_roll-err_roll_last)
        #误差传递
        err_roll_last = err_roll
        err_pitch_last = err_pitch

        #calculate update angular
        Goal_angular = [0.0]*18
        sign = [-1,1,1, -1,-1,-1, -1,1,1, -1,-1,-1, -1,1,1, -1,-1,-1]
        for i in range(0,6):
            try:
                q = rotXYAng2jointAng(i+1,bias_roll+output_roll,bias_pitch+output_pitch)
            except AssertionError as e:    
                print("No solution for joint angular")
            Goal_angular[i*3] = 512+round(sign[i*3]*float(q[0])/0.29)
            Goal_angular[i*3+1] = 512+round(sign[i*3+1]*float(q[1])/0.29)
            Goal_angular[i*3+2] = 512+round(sign[i*3+2]*float(q[2])/0.29)
        # Write goal posssssition
        for i in range(len(test_leg)):
            DXL_ID = test_leg[i]
            final_angular = Goal_angular[i]
            #print("Number "+str(i+1)+" angular "+str(final_angular))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, final_angular)
            Present_position[DXL_ID-1], dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
            while abs(Present_position[DXL_ID-1] - final_angular) > 300:
                Present_position[DXL_ID-1], dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
        #print information update the bias
        bias_pitch += output_pitch
        bias_roll += output_roll

# Disable Dynamixel Torque
for i in range(len(test_leg)):
    DXL_ID = test_leg[i]
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
      print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
      print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
