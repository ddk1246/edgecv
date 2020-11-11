import cv2
import numpy as np
# 读取图片并转至灰度模式
img = cv2.imread('learm.jpg', 1)
img = cv2.resize(img, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_AREA)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
lower_blue = np.array([0,0,46])
upper_blue = np.array([128,43,220])
# get mask 利用inRange()函数和HSV模型中蓝色范围的上下界获取mask，mask中原视频中的蓝色部分会被弄成白色，其他部分黑色。
mask = cv2.inRange(hsv, lower_blue, upper_blue)
cv2.imshow('Mask', mask)

# detect blue 将mask于原视频帧进行按位与操作，则会把mask中的白色用真实的图像替换：
res = cv2.bitwise_and(img, img, mask=mask)
# cv2.imshow('gray',gray)
# 二值化，取阈值为235
ret, thresh = cv2.threshold(gray, 0, 100, cv2.THRESH_BINARY)
cv2.estimateAffine2D()
# 寻找图像中的轮廓
contours, hierarchy = cv2.findContours(thresh, 2, 1)

# 寻找物体的凸包并绘制凸包的轮廓
for cnt in contours:
    hull = cv2.convexHull(cnt)
    length = len(hull)
    # 如果凸包点集中的点个数大于5
    if length > 5:
        # 绘制图像凸包的轮廓
        for i in range(length):
            cv2.line(img, tuple(hull[i][0]), tuple(hull[(i+1)%length][0]), (0,0,255), 2)

cv2.imshow('finger', img)
cv2.waitKey()