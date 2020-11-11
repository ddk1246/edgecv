import cv2  as cv
import numpy as np



d=np.random.randn(3,2)
e=np.random.randn(3,2)
img=cv.imread('ok.jpg',0)
m=cv.estimateAffine2D(d,e)
print(m[0][:,0:2])
print('ord=',np.matmul(m[0][:,0:2],d[0,:].T)+m[0][:,2],'\ndst= ',e)
dst=cv.warpAffine(img,m[0],img.shape, flags=cv.INTER_LINEAR,borderMode=cv.BORDER_REFLECT,borderValue=(255,255,255),)
cv.imshow('wi',img)
cv.imshow('wti',dst)

import cv2
import numpy as np

# 这里说一下旋转的opencv中为旋转提供的三个要素
# 旋转的中心点（center）
# 旋转角度()
# 旋转后进行放缩
# 我们可以通过cv2.getRotationMatrix2D函数得到转换矩阵

img = cv2.imread('ok.jpg')
rows,cols,_ = img.shape

matrix = cv2.getRotationMatrix2D((cols/2,rows/2),90,1)

print(matrix)
# # 得到变换的矩阵，通过这个矩阵再利用warpAffine来进行变换
# # 第一个参数就是旋转中心，元组的形式，这里设置成相片中心
# # 第二个参数90，是旋转的角度
# # 第三个参数1，表示放缩的系数，1表示保持原图大小
#
# img1 = cv2.warpAffine(img,matrix,(cols,rows))
#
# cv2.imshow('img',img)
# cv2.imshow('img1',img1)
cv.waitKey(0)




