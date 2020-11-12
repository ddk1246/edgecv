import cv2 as cv
import numpy as np
import time
from LeArm import LeArm
import serial
class ShapeAnalysis:
    def __init__(self):
        self.elements=[]
        self.contours=[]
        self.shapes = {'triangle': 0, 'rectangle': 0, 'polygons': 0, 'circles': 0,'square':0}

    def imageShow(self,frame):
        # h, w, ch = frame.shape
        # result = np.zeros((h, w, ch), dtype=np.uint8)
        # # 二值化图像
        # print("start to detect lines...\n")
        # gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # gray = cv.GaussianBlur(gray, (5, 5), 0)
        # ret, binary = cv.threshold(gray, 114, 255, cv.THRESH_BINARY_INV | cv.THRESH_OTSU)
        cv.imshow("input image", frame)
    def analysis(self, frame):
        self.elements=[]
        flag=1
        h, w, ch = frame.shape
        result = np.zeros((h, w, ch), dtype=np.uint8)
        # 二值化图像
        print("start to detect lines...\n")
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        gray = cv.GaussianBlur(gray, (5, 5), 0)
        ret, binary = cv.threshold(gray, 126, 255, cv.THRESH_BINARY_INV )
        # cv.imshow("input image", frame)
        m = np.array(binary)
        m[:, :] = 0
        m[80:400, 80:560] = 255
        dst = cv.bitwise_and(binary, m)
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
            if corners == 3:
                count = self.shapes['triangle']
                count = count+1
                self.shapes['triangle'] = count
                shape_type = "triangle"
            if corners == 4:
                (x, y, w, h) = cv.boundingRect(approx)
                ar = w / float(h)
                shape_type = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
                count = self.shapes[shape_type]
                count = count + 1
                self.shapes[shape_type] = count


            if corners >= 10:
                count = self.shapes['circles']
                count = count + 1
                self.shapes['circles'] = count
                shape_type = "circle"
            if 4 < corners < 10:
                count = self.shapes['polygons']
                count = count + 1
                self.shapes['polygons'] = count
                shape_type = "polygons"

            try:
                # 求解中心位置
                mm = cv.moments(self.contours[cnt])
                cx = int(mm['m10'] / mm['m00'])
                cy = int(mm['m01'] / mm['m00'])
                cv.circle(result, (cx, cy), 3, (0, 0, 255), -1)

                # 颜色分析
                color = frame[cy][cx]
                color_str = "(" + str(color[0]) + ", " + str(color[1]) + ", " + str(color[2]) + ")"

                # 计算面积与周长
                p = cv.arcLength(self.contours[cnt], True)
                area = cv.contourArea(self.contours[cnt])
                cv.putText(result, shape_type, (cx - 20, cy - 20),
                            cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                print("周长: %.3f, 面积: %.3f 颜色: %s 形状: %s "% (p, area, color_str, shape_type))
                element = {'shape':shape_type,'cx':cx,'cy':cy,'color':color}
                self.elements.append(element)
            except Exception as e:
                pass
        return result




def main():
    ser = serial.Serial('COM7', 9600, timeout=5)
    learm = LeArm(ser)
    ld = ShapeAnalysis()
    cap = cv.VideoCapture(1)
    flag=1
    num=0
    m = np.array([[-4.88974406e-06 , 5.98781798e-04 ,-1.34553948e-01],
 [-6.43413726e-04 ,-3.10712068e-05  ,5.00919164e-01]])

    learm.reset()
    ret, frame = cap.read()
    ld.analysis(frame)
    print(ld.elements)
    trianglePlace = [-0.3, 0.01]
    circlePlace = [-0.3, 0.01]
    squarePlace = [-0.3, 0.01]
    rectanglePlace = [-0.3, 0.01]
    while flag:

        ret, frame = cap.read()
        # cv.imshow('mainwin',frame)
        # element = {'shape': shape_type, 'cx': cx, 'cy': cy, 'color': color}
        # {'triangle': 0, 'rectangle': 0, 'polygons': 0, 'circles': 0, 'square': 0}
        for ele in ld.elements:
            # if ele['shape']=='polygons':
            #     continue
            point=np.zeros([2,1])
            point[0]=ele['cy']
            point[1]=ele['cx']
            if ele['color'][0]<100:
                ax=circlePlace[0]
            else:
                ax=-1*circlePlace[0]
            print('shape=',ele['shape'],' cx=',ele['cx'],' cy=',ele['cy'],' color=',ele['color'],end='')
            cir=np.matmul(m[:, 0:2], point).T[0] + m[:, 2]
            print(' \t cmd = ',cir)
            learm.point2D(cir[0], cir[1], 0.01,3000)
            # cv.waitKey(0)
            # return

            learm.grab()
            time.sleep(1.5)
            if ele['shape']=='triangle':
                learm.point2D(trianglePlace[0], trianglePlace[1],  0.08)
                time.sleep(2)
                learm.relax()

            elif ele['shape']=='rectangle':
                learm.point2D(rectanglePlace[0], rectanglePlace[1],  0.08)
                time.sleep(2)
                learm.relax()
            elif ele['shape']=='square':
                learm.point2D(squarePlace[0], squarePlace[1],  0.08)
                time.sleep(2)
                learm.relax()
            elif ele['shape']=='circle':
                learm.point2D(ax, circlePlace[1],  0.08)
                time.sleep(2)
                learm.relax()
            elif ele['shape']=='polygons':
                learm.point2D(ax, circlePlace[1],  0.08)
                time.sleep(2)
                learm.relax()
        ld.elements=[]
        learm.reset()
        # ld.analysis(frame)





    # cv.destroyAllWindows()

if __name__ == "__main__":
    main()
