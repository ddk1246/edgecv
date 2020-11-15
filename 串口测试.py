import serial  # 导入模块
import serial.tools.list_ports
import time

port_list = list(serial.tools.list_ports.comports())

print(port_list)
if len(port_list) == 0:
    print('无可用串口')
else:
    for i in range(0, len(port_list)):
        print(port_list[i])

try:
    # 端口，GNU / Linux上的/ dev / ttyUSB0 等 或 Windows上的 COM3 等
    portx = "COM3"
    # 波特率，标准值之一：50,75,110,134,150,200,300,600,1200,1800,2400,4800,9600,19200,38400,57600,115200
    bps = 9600
    # 超时设置,None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）
    timex = 5
    # 打开串口，并得到串口对象

    #(port=None, baudrate=9600, bytesize=EIGHTBITS, parity=PARITY_NONE, stopbits=STOPBITS_ONE, timeout=None, xonxoff=False, rtscts=False, writeTimeout=None, dsrdtr=False, interCharTimeout=None)
    ser = serial.Serial(portx, bps,timeout=timex)
    # 写数据
    myinput = [0X55, 0X55, 0X08, 0X03, 0X01, 0XE8, 0X03, 0X01,0XD4,0X03]  # 需要发送的十六进制数据
    ser.write(myinput)
    # time.sleep(0.8)
    # ser.write([0x55,0x55,2,7])
    ser.write([0X55, 0X55, 0X08, 0X03, 0X01, 0XE8, 0X03, 0X02, 0XDC, 0X03])

    ls=ser.read()
    print(ls)
    ser.close()  # 关闭串口

except Exception as e:
    print("---异常---：", e)


import math
import numpy as np
targetX = 0.2
targetY = 0.0
lastX = 0.2
lastY = 0.0
def model(x,y,alpha):
    l2=0.17
    l1=0.088
    l0=0.1035

    m = l2 * math.cos(alpha) - x
    n = l2 * math.sin(alpha) - y
    k = (l1 * l1 - l0 * l0 - m * m - n * n) / 2 / l0 # 中间变量
    a = m * m + n * n # 解一元二次方程
    b = -2 * n * k
    c = k * k - m * m

    if (b * b - 4 * a * c <= 0): # b ^ 2 - 4ac 小于0即无实根，直接返回
        return 1 # 返回1，作错误


    theta1 = (-b + math.sqrt(b * b - 4 * a * c)) / 2 / a # 求解二元一次方程，只取其中一个，另外一个解是(-b + sqrt(b * b - 4 * a * c)) / 2 / a
    theta1 = math.asin(theta1) * 180 / math.pi # 将弧度换算为角度

    if (theta1 > 90):
        theta1 = 90 # 限制最大角度为正负90度
    elif (theta1 < -90):
        theta1 = -90

    k = (l0 * l0 - l1 * l1 - m * m - n * n) / 2 / l1
    a = m * m + n * n # 解一元二次方程
    b = -2 * n * k
    c = k * k - m * m

    if (b * b - 4 * a * c <= 0): # 方程无实根就不做求解
        return 2 #返回2， 作错误标记


    s1ps2 = (-b - math.sqrt(b * b - 4 * a * c)) / 2 / a
    s1ps2 = math.asin(s1ps2) * 180 / math.pi # 将弧度换算为角度

    if (s1ps2 > 90):
        theta2 = 90
    elif (s1ps2 < -90):
        theta2 = -90

    theta2 = s1ps2 - theta1
    if (theta2 > 90):
        theta2 = 90 # 限制最大角度为正负90度
    elif (theta2 < -90):
        theta2 = -90  #限制最大角度为正负90度

    theta3 = alpha * 180 / math.pi - theta1 - theta2 # 求5号舵机角度
    if (theta3 > 180):
        theta3 = 180
    if (theta3 < 0):
        theta3 = 0 # 控制舵机的最大角度180

    # // 将求得的三个角度，转换为对应的脉宽，然后控制舵机转动。
    # // 需要注意的是机械臂的舵机开机后为1500位置。我们将此位置定为0度，即0度为180度舵机的90度位置。
    # // 因舵机安装方向的不同，舵机角度的正负方向要根据舵机安装方向调整
    #               // 将舵机角度转为公式为
    #               // (2000 * 角度 / 180 + 500)
    #               // 上式的角度的范围是0 - 180度

    print("%f\r\n".format(theta3))

    return 0 # 一切正常返回0