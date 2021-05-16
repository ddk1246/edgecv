import cv2 as cv
import numpy as np
import time
from LeArm import LeArm
import serial
import threading


class ShapeAnalysis:
    def __init__(self, cap):
        self.cap = cap
        self.Frame = []
        self.resultFrame = []
        self.elements = []
        self.contours = []
        self.shapes = {'triangle': 0, 'rectangle': 0, 'polygons': 0, 'circles': 0, 'square': 0}
        self.thr = threading.Thread(target=self.imageShow)
        self.thr.daemon = 1

        self.TARGET_COLORS = {"Red": (120, 50, 53), "Blue": (53, 99, 147),
                              "Orange": (134, 80, 46), "Purple": (146, 98, 128),
                              "Green": (76, 113, 55)}  # RGB值

    def imageShow(self):
        while 1:
            cv.imshow("input image", self.resultFrame)
            cv.waitKey(400)

    def setFrame(self, frame):
        self.frame = frame

    def analysis(self, frame):
        self.elements = []
        flag = 1
        h, w, ch = frame.shape
        result = np.zeros((h, w, ch), dtype=np.uint8)
        # 二值化图像
        print("start to detect lines...\n")
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # gray = cv.GaussianBlur(gray, (5, 5), 0)
        ret, binary = cv.threshold(gray, 115, 255, cv.THRESH_BINARY_INV)

        m = np.zeros_like(binary)
        m[120:400, 80:560] = 255
        dst = cv.bitwise_and(binary, m)
        result = cv.bitwise_and(frame, frame, mask=dst)
        self.contours, hierarchy = cv.findContours(dst, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        for cnt in range(len(self.contours)):
            # 提取与绘制轮廓
            if cv.contourArea(self.contours[cnt]) < 800:
                continue
            cv.drawContours(result, self.contours, cnt, (0, 255, 0), 2)

            # 轮廓逼近
            epsilon = 0.01 * cv.arcLength(self.contours[cnt], True)
            approx = cv.approxPolyDP(self.contours[cnt], epsilon, True)

            # 分析几何形状
            corners = len(approx)
            shape_type = ""
            (x, y, w, h) = cv.boundingRect(approx)
            ar = w / float(h)
            if ar < 0.8 or ar > 1.3:
                shape_type = "rectangle"
            elif corners <= 5:
                count = self.shapes['triangle']
                count = count + 1
                self.shapes['triangle'] = count
                shape_type = "triangle"
            elif 5 < corners <= 8:
                (x, y, w, h) = cv.boundingRect(approx)
                ar = w / float(h)
                shape_type = "square"  # if ar >= 0.95 and ar <= 1.05 else "rectangle"
                count = self.shapes[shape_type]
                count = count + 1
                self.shapes[shape_type] = count

            elif corners > 8:
                count = self.shapes['circles']
                count = count + 1
                self.shapes['circles'] = count
                shape_type = "circle"
                if cv.contourArea(self.contours[cnt]) > 3000:
                    shape_type = "square"

            try:
                # 求解中心位置
                mm = cv.moments(self.contours[cnt])
                cx = int(mm['m10'] / mm['m00'])
                cy = int(mm['m01'] / mm['m00'])
                cv.circle(result, (cx, cy), 3, (0, 0, 255), -1)

                # 颜色分析
                color = frame[cy][cx]
                color_str = "(" + str(color[0]) + ", " + str(color[1]) + ", " + str(color[2]) + ")"

                cargoColor = color[::-1]  # BGR转RGB
                differences = [[color_difference(cargoColor, target_value), target_name] for target_name, target_value
                               in self.TARGET_COLORS.items()]
                differences.sort()
                my_color_name = differences[0][1]
                # 计算面积与周长
                p = cv.arcLength(self.contours[cnt], True)
                area = cv.contourArea(self.contours[cnt])
                cv.putText(result, str(shape_type + '  ' + str(my_color_name)), (cx - 30, cy - 20),
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                if shape_type=="circle":
                    cy=cy-17
                print("周长: %.3f, 面积: %.3f 颜色: %s 形状: %s " % (p, area, color_str, shape_type))
                element = {'shape': shape_type, 'cx': cx, 'cy': cy, 'color': cargoColor, 'colorName': my_color_name}
                self.elements.append(element)

            except Exception as e:
                pass
            self.resultFrame = result
        return result


def color_difference(color1, color2):
    return sum([abs(component1 - component2) for component1, component2 in zip(color1, color2)])


def main():
    # 端口及机械臂初始化
    ser = serial.Serial('COM16', 9600, timeout=5)
    learm = LeArm(ser)
    # 摄像头及分析模块初始化
    cap = cv.VideoCapture(1)
    ld = ShapeAnalysis(cap)

    flag = 1
    # 手眼标定仿射矩阵
    m = np.array([[4.43100133e-06, -6.13883183e-04, 2.43638333e-01],
                  [6.10930452e-04, -1.66112753e-05, 1.72203651e-01]])
    # 机械臂卸载位置
    trianglePlace = [0.28, 0.01]
    circlePlace = [0.28, 0.08]
    squarePlace = [0.15, 0.01]
    rectanglePlace = [0.15, 0.08]

    learm.reset()  # 机械臂复原
    ret, frame = cap.read()
    ld.analysis(frame)
    ld.thr.start()  # 显示线程

    while flag:
        # element = {'shape': shape_type, 'cx': cx, 'cy': cy, 'color': color}
        # {'triangle': 0, 'rectangle': 0, 'polygons': 0, 'circles': 0, 'square': 0}
        print(ld.elements)
        for ele in ld.elements:
            point = np.zeros([2, 1])
            point[0] = ele['cy']  # 由于历史原因与标定.py匹配,即输入第0项为cy，第1项为cx
            point[1] = ele['cx']
            cir = np.matmul(m[:, 0:2], point).T[0] + m[:, 2]  # 摄像机坐标系转换为机械臂坐标系

            useShape = 1
            shape = ele['shape']
            colorName = ele['colorName']
            if useShape:
                if shape == "circle":
                    ax = circlePlace
                elif shape == 'rectangle':
                    ax = rectanglePlace
                elif shape == 'square':
                    ax = squarePlace
                else:
                    ax = trianglePlace
            else:
                if colorName == "Red":
                    ax = trianglePlace
                elif colorName == "Blue":
                    ax = circlePlace
                elif colorName == "Orange":
                    ax = squarePlace
                elif colorName == "Green":
                    ax = rectanglePlace
                else:
                    ax = rectanglePlace

            print('shape=', ele['shape'], ' cx=', ele['cx'], ' cy=', ele['cy'], ' color=', ele['color'], ' colorName=',
                  colorName,
                  ' \t cmd = ', cir)
            # 机械臂抓取
            learm.relax()
            learm.point3D(cir[0], cir[1], 0.012)
            learm.grab()
            # 机械臂放置
            learm.point3D(ax[0], ax[1], 0.1)
            time.sleep(0.2)
            learm.relax()
        # 复原
        learm.reset()
        time.sleep(3)  # 采样延时
        ret, frame = cap.read()
        ld.analysis(frame)

    # cv.destroyAllWindows()


if __name__ == "__main__":
    main()
