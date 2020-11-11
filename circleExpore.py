import cv2 as cv
import numpy as np

class ShapeAnalysis:
    def __init__(self):
        self.elements=[]
        self.shapes = {'triangle': 0, 'rectangle': 0, 'polygons': 0, 'circles': 0,'square':0}

    def analysis(self, frame):
        h, w, ch = frame.shape
        result = np.zeros((h, w, ch), dtype=np.uint8)
        # 二值化图像
        print("start to detect lines...\n")
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        gray = cv.GaussianBlur(gray, (5, 5), 0)
        ret, binary = cv.threshold(gray, 200, 255, cv.THRESH_BINARY_INV | cv.THRESH_OTSU)
        cv.imshow("input image", frame)

        contours, hierarchy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        for cnt in range(len(contours)):
            # 提取与绘制轮廓
            if cv.contourArea(contours[cnt]) < 10:
                continue
            cv.drawContours(result, contours, cnt, (0, 255, 0), 2)

            # 轮廓逼近
            epsilon = 0.01 * cv.arcLength(contours[cnt], True)
            approx = cv.approxPolyDP(contours[cnt], epsilon, True)

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
                shape_type = "pentagon"

            try:
                # 求解中心位置
                mm = cv.moments(contours[cnt])
                cx = int(mm['m10'] / mm['m00'])
                cy = int(mm['m01'] / mm['m00'])
                cv.circle(result, (cx, cy), 3, (0, 0, 255), -1)

                # 颜色分析
                color = frame[cy][cx]
                color_str = "(" + str(color[0]) + ", " + str(color[1]) + ", " + str(color[2]) + ")"

                # 计算面积与周长
                p = cv.arcLength(contours[cnt], True)
                area = cv.contourArea(contours[cnt])
                cv.putText(result, shape_type, (cx - 20, cy - 20),
                            cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                print("周长: %.3f, 面积: %.3f 颜色: %s 形状: %s "% (p, area, color_str, shape_type))
                element = {'shape':shape_type,'cx':cx,'cy':cy,'color':color}
                self.elements.append(element)
            except Exception as e:
                pass
        cv.imshow("Analysis Result", result)
        cv.imwrite("./test-result.png", result)
        return self.shapes

    def draw_text_info(self, image):
        c1 = self.shapes['triangle']
        c2 = self.shapes['rectangle']
        c3 = self.shapes['polygons']
        c4 = self.shapes['circles']
        cv.putText(image, "triangle: "+str(c1), (10, 20), cv.FONT_HERSHEY_PLAIN, 1.2, (255, 0, 0), 1)
        cv.putText(image, "rectangle: " + str(c2), (10, 40), cv.FONT_HERSHEY_PLAIN, 1.2, (255, 0, 0), 1)
        cv.putText(image, "polygons: " + str(c3), (10, 60), cv.FONT_HERSHEY_PLAIN, 1.2, (255, 0, 0), 1)
        cv.putText(image, "circles: " + str(c4), (10, 80), cv.FONT_HERSHEY_PLAIN, 1.2, (255, 0, 0), 1)
        return image

if __name__ == "__main__":
    ld = ShapeAnalysis()
    cap = cv.VideoCapture(1)
    flag=1
    while flag:
        ret, frame = cap.read()
        ld.analysis(frame)
        # flag=0
        # frame = cv.flip(frame, 1)
        # print(frame.shape)
        # cv.imshow("sxt", frame)
        k = cv.waitKey(400)
        if k == 27:
            cap.release()
            break
        elif k == ord("s"):
            cv.imwrite("004.jpg", frame)
    src = cv.imread("1.png")

    ld.analysis(src)
    print(ld.elements)
    # cv.waitKey(0)
    # cv.destroyAllWindows()


