import serial
import serial.tools.list_ports
import time
import math


class LeArm:
    def __init__(self, ser):
        self.ser = ser
        self.head = [0x55, 0x55]
        self.ID = 0x01  # 1-6
        self.length = 0x05
        self.CMD_FULL = 0x03
        self.time = 1000
        self.value = 1500
        self.zero = [0X55, 0X55, 0X08, 0X03, 0X01, 0XE8, 0X03, 0X01, 0XDC, 0X05]  # 需要发送的十六进制数据
        self.lastX = 0
        self.lastY = 0
        self.targetX = 0
        self.targetY = 0
        self.x3D = 0
        self.y3D = 0
        self.z3D = 0

    def reset(self):
        t = 400
        cmd = [{'ID': i, 'value': 1500} for i in range(1, 7)]
        self.controlMOVE3(cmd, t)
        time.sleep(t)

    def stop(self):
        Myinput = [0x55, 0x55, 2, 7]
        self.ser.write(Myinput)

    def controlMOVE3(self, cmd, ctime=400):
        # cmd =[{ID:1,value:1500},]
        # self.stop()
        lenCmd = len(cmd)
        Myinput = [0x55, 0x55, lenCmd * 3 + 5, 3, lenCmd,
                   ctime & 0xff, ctime >> 8]

        for c in range(lenCmd):
            Myinput.append(cmd[c]['ID'])
            Myinput.append(cmd[c]['value'] & 0xff)
            Myinput.append(cmd[c]['value'] >> 8)

        # l = [hex(Myinput[i]) for i in range(lenCmd * 3 + 5)]
        # print(l)

        self.ser.write(Myinput)
        self.ser.write([1, 1, 1])  # 垃圾数据以清零

    def model(self, x, y, alpha):
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
            self.targetX = self.lastX
            self.targetY = self.lastY
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
            self.targetX = self.lastX
            self.targetY = self.lastY
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
        # print(cmd)
        for i in cmd:
            print(i)
            self.controlMOVE3(i, 1000)
        # ServoSetPluseAndTime(5, (2000 * (90.0 - theta1) / 180.0 + 500.0), 50) #控制舵机转动， 给根据公式转换脉宽， 以舵机的中位为基准。
        # ServoSetPluseAndTime(4, (2000 * (90.0 + theta2) / 180.0 + 500.0), 50)
        # ServoSetPluseAndTime(3, (2000 * (theta3) / 180.0 + 500.0), 50)

        self.lastX = self.targetX  # 将当前位置，替换为目标位置
        self.lastY = self.targetY  #
        return 0  # 一切正常返回0

    def point2D(self, y, x, z):
        self.z3D = z
        self.x3D = y
        self.y3D = x
        r = math.sqrt(x ** 2 + y ** 2)
        theta0 = math.atan(y / x) / math.pi * 180

        i = math.pi
        while (learm.model((z), r, i) != 0):
            i -= 0.01
            if (i <= 0):
                break
        print('i=', i)
        cmd = [{'ID': 6, 'value': int(2000 * (90.0 - theta0) / 180.0 + 500.0)}]

        self.controlMOVE3(cmd, 1000)

    def grab(self):
        cmd = [{'ID': 1, 'value': 600}]
        self.controlMOVE3(cmd)
        time.sleep(1)
        learm.point2D(self.x3D, self.y3D, self.z3D - 0.08)
        time.sleep(1)
        cmd = [{'ID': 1, 'value': 1500}]
        self.controlMOVE3(cmd)
        time.sleep(1)
        learm.point2D(self.x3D, self.y3D, self.z3D + 0.08)

    def relax(self):
        cmd = [{'ID': 1, 'value': 600}]
        self.controlMOVE3(cmd)
        time.sleep(1)
    def hold(self):
        cmd = [{'ID': 1, 'value': 1500}]
        self.controlMOVE3(cmd)

if __name__ == '__main__':

    port_list = list(serial.tools.list_ports.comports())
    print(port_list)
    if len(port_list) == 0:
        print('无可用串口')
    else:
        for i in range(0, len(port_list)):
            print(port_list[i])

    ser = serial.Serial('COM3', 9600, timeout=5)
    learm = LeArm(ser)
    learm.point2D(-0.25, 0.2, -0.01)
    print(learm.lastX, learm.lastY)
    learm.grab()
    # cmd=[{'ID': 3, 'value': 896}, {'ID': 4, 'value': 1163}, {'ID': 5, 'value': 900}]

    # cmd = [{'ID': 3, 'value': 896}]
    # learm.controlMOVE3(cmd)

    # cmd = [{'ID': 4, 'value': 1163}]
    # learm.controlMOVE3(cmd)
    #
    # cmd = [{'ID': 5, 'value': 900}]
    # learm.controlMOVE3(cmd)
