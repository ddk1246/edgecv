import serial
import serial.tools.list_ports
import time
import math


class LeArm:
    def __init__(self, ser):
        self.ser = ser
        self.head = [0x55, 0x55]
        self.x3D = 0
        self.y3D = 0
        self.z3D = 0

    def reset(self):
        t = 1000
        cmd = [{'ID': i, 'value': 1500} for i in range(1, 7)]
        self.controlMOVE(cmd, t)
        time.sleep(t/1000)

    def stop(self):
        Myinput = [0x55, 0x55, 2, 7]
        self.ser.write(Myinput)

    def controlMOVE(self, cmd, ctime=400):
        #example : cmd =[{ID:1,value:1500},]
        # self.stop()
        lenCmd = len(cmd)
        Myinput = [0x55, 0x55, lenCmd * 3 + 5, 3, lenCmd,
                   ctime & 0xff, ctime >> 8]

        for c in range(lenCmd):
            Myinput.append(cmd[c]['ID'])
            Myinput.append(cmd[c]['value'] & 0xff)
            Myinput.append(cmd[c]['value'] >> 8)

        self.ser.write(Myinput)
        self.ser.write([1, 1, 1])  # 垃圾数据以清零

    def InverseKinematics(self, x, y, alpha, ctime=1000):
        limtheta = 82
        l2 = 0.17
        l1 = 0.088
        l0 = 0.1035

        m = l2 * math.cos(alpha) - x
        n = l2 * math.sin(alpha) - y
        k = (l1 * l1 - l0 * l0 - m * m - n * n) / 2 / l0  # 中间变量
        a = m * m + n * n  # 解一元二次方程
        b = -2 * n * k
        c = k * k - m * m

        if (b * b - 4 * a * c <= 0):  # b ^ 2 - 4ac 小于0即无实根，直接返回
            return 1  # 返回1，作错误

        theta1 = (-b - math.sqrt(
            b * b - 4 * a * c)) / 2 / a  # 求解二元一次方程，只取其中一个，另外一个解是(-b + sqrt(b * b - 4 * a * c)) / 2 / a
        theta1 = math.asin(theta1) * 180 / math.pi  # 将弧度换算为角度
        # print('th1=', theta1)
        if (theta1 > limtheta):
            theta1 = limtheta  # 限制最大角度为正负90度
            return 1
        elif (theta1 < -limtheta):
            theta1 = -limtheta
            return 1

        k = (l0 * l0 - l1 * l1 - m * m - n * n) / 2 / l1
        a = m * m + n * n  # 解一元二次方程
        b = -2 * n * k
        c = k * k - m * m

        if (b * b - 4 * a * c <= 0):  # 方程无实根就不做求解
            return 2  # 返回2， 作错误标记

        s1ps2 = (-b + math.sqrt(b * b - 4 * a * c)) / 2 / a
        s1ps2 = math.asin(s1ps2) * 180 / math.pi  # 将弧度换算为角度

        theta2 = s1ps2 - theta1
        # print('th2=', theta2)
        if (theta2 > limtheta):
            theta2 = limtheta
            return 2  # 限制最大角度为正负90度
        elif (theta2 < -limtheta):
            theta2 = -limtheta  # 限制最大角度为正负90度
            return 2
        theta3 = alpha * 180 / math.pi - theta1 - theta2  # 求5号舵机角度
        # print('th3=',theta3)
        if (theta3 > limtheta):
            theta3 = limtheta
            return 3
        elif (theta3 < -limtheta):
            theta3 = -limtheta  # 控制舵机的最大角度180
            return 3
        # // 将求得的三个角度，转换为对应的脉宽，然后控制舵机转动。
        # // 需要注意的是机械臂的舵机开机后为1500位置。我们将此位置定为0度，即0度为180度舵机的90度位置。
        # // 因舵机安装方向的不同，舵机角度的正负方向要根据舵机安装方向调整
        #               // 将舵机角度转为公式为
        #               // (2000 * 角度 / 180 + 500)
        #               // 上式的角度的范围是0 - 180度
        cmd = [[{'ID': 5, 'value': int(2000 * (90.0 + theta1) / 180.0 + 500.0)}],
               [{'ID': 4, 'value': int(2000 * (90.0 - theta2) / 180.0 + 500.0)}],
               [{'ID': 3, 'value': int(2000 * (90.0 - theta3) / 180.0 + 500.0)}]]
        for i in cmd:
            # print(i)
            self.controlMOVE(i, ctime)
        return 0  # 一切正常返回0

    def point3D(self, y, x, z, ctime=900):
        #内置延时
        self.z3D = z
        self.x3D = y
        self.y3D = x
        r = math.sqrt(x ** 2 + y ** 2)
        theta0 = math.atan(y / x) / math.pi * 180

        i = math.pi
        while (self.InverseKinematics(z, r, i, ctime) != 0):
            i -= 0.01
            if (i <= 0):
                break
        # print('i=', i)
        cmd = [{'ID': 6, 'value': int(2000 * (90.0 - theta0) / 180.0 + 500.0)}]
        self.controlMOVE(cmd, ctime)
        time.sleep(ctime / 1000)

    def grab(self):
        cmd = [{'ID': 1, 'value': 600}]
        self.controlMOVE(cmd)
        time.sleep(0.4)
        self.point3D(self.x3D, self.y3D, self.z3D - 0.077) #会更新z3D
        time.sleep(0.1)

        cmd = [{'ID': 1, 'value': 1500}]
        self.controlMOVE(cmd)
        time.sleep(0.4)
        self.point3D(self.x3D, self.y3D, 0.1)

    def relax(self):
        cmd = [{'ID': 1, 'value': 600}]
        self.controlMOVE(cmd)
        time.sleep(0.4)

    def hold(self):
        cmd = [{'ID': 1, 'value': 1500}]
        self.controlMOVE(cmd)

if __name__ == '__main__':

    port_list = list(serial.tools.list_ports.comports())
    print(port_list)
    if len(port_list) == 0:
        print('无可用串口')
    else:
        for i in range(0, len(port_list)):
            print(port_list[i])

    ser = serial.Serial('COM8', 9600, timeout=5)

    learm = LeArm(ser)
    learm.point3D(-0.26,0.01, 0.1)


#     print(learm.lastX, learm.lastY)
#     learm.grab()
#     time.sleep(2)
#     learm.reset()
    # cmd=[{'ID': 3, 'value': 896}, {'ID': 4, 'value': 1163}, {'ID': 5, 'value': 900}]

    # cmd = [{'ID': 3, 'value': 896}]
    # learm.controlMOVE3(cmd)

    # cmd = [{'ID': 4, 'value': 1163}]
    # learm.controlMOVE3(cmd)
    #
    # cmd = [{'ID': 5, 'value': 900}]
    # learm.controlMOVE3(cmd)
