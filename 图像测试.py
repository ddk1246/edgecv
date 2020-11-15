import cv2 as cv
import numpy as np
import time
from LeArm import LeArm
import serial
import threading


class ShapeAnalysis:
    def __init__(self, cap):
        self.cap = cap
        self.elements = []
        self.contours = []
        self.shapes = {'triangle': 0, 'rectangle': 0, 'polygons': 0, 'circles': 0, 'square': 0}
        self.TARGET_COLORS = {"Red": (161, 60, 52), "Blue": (53, 99, 147),
                              "Orange": (148, 124, 35), "Purple": (146, 98, 128)}  # RGB值
        # self.thr = threading.Thread(target=self.imageShow)
        # self.thr.daemon = 1
        # self.thr.start()

    def imageShow(self):
        ret, frame = self.cap.read()
        h, w, ch = frame.shape
        result = np.zeros((h, w, ch), dtype=np.uint8)
        for cnt in range(len(self.contours)):
            # 提取与绘制轮廓
            if cv.contourArea(self.contours[cnt]) < 800:
                continue
            cv.drawContours(result, self.contours, cnt, (0, 255, 0), 2)
        cv.imshow("input image", result)

    def analysis(self, frame):
        self.elements = []
        flag = 1
        h, w, ch = frame.shape
        result = np.zeros((h, w, ch), dtype=np.uint8)
        # 二值化图像
        print("start to detect lines...\n")
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # gray = cv.GaussianBlur(gray, (5, 5), 0)
        ret, binary = cv.threshold(gray, 130, 255, cv.THRESH_BINARY_INV)
        cv.imshow("binary", binary)
        m=np.zeros_like(binary)
        m[80:430, 80:560] = 255
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
                print("周长: %.3f, 面积: %.3f 颜色: %s 形状: %s " % (p, area, color_str, shape_type))
                element = {'shape': shape_type, 'cx': cx, 'cy': cy, 'color': color}
                print(element)
                self.elements.append(element)
            except Exception as e:
                pass
        cv.imshow('result', result)
        return result


def color_difference (color1, color2):
    return sum([abs(component1-component2) for component1, component2 in zip(color1, color2)])





def main():
    cap = cv.VideoCapture(1)
    ld = ShapeAnalysis(cap)

    flag = 1
    num = 0
    m = np.array([[5.19903115e-04, -8.88044277e-06, -1.77732829e-01],
                  [-4.12314164e-05, -5.52085144e-04, 4.38210960e-01]])
    TARGET_COLORS = {"Red": (126, 42, 32), "Blue":(33, 69, 127),
                     "Orange":(148,99,25),"Purple":(128,0,128)} #RGB值

    while flag:

        ret, frame = cap.read()
        ld.analysis(frame)
        hsv=cv.cvtColor(frame,cv.COLOR_BGR2HSV)
        for ele in ld.elements:
            cargoColor = ele['color'][::-1] #BGR转RGB
            # hsv = cv.cvtColor(cargoColor, cv.COLOR_BGR2HSV)

            differences = [[color_difference(cargoColor, target_value), target_name] for target_name, target_value in
                           TARGET_COLORS.items()]
            differences.sort()  # sorted by the first element of inner lists
            my_color_name = differences[0][1]

            print(my_color_name)

        time.sleep(1)
        # flag=0
        # frame = cv.flip(frame, 1)
        # print(frame.shape)
        cv.imshow("sxt", frame)
        k = cv.waitKey(400)
        if k == 27:
            cap.release()
            break
        elif k == ord("s"):
            cv.imwrite(str(num) + ".png", frame)
            num += 1

        # cv.waitKey(0)
    # cv.destroyAllWindows()


if __name__ == "__main__":
    main()
