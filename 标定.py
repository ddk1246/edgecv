import cv2  as cv
import numpy as np


#图像坐标（Y,X）
a=np.array([[336,210],
           [261,178],
           [243,246],
           [325,259],
           [ 239,458],
           [385,435],
           [253,346],
            ])
#机械臂坐标（x,y）
b=np.array([[-0.05,0.25],
           [-0.07,0.3],
           [-0.025,0.3],
           [0.,0.25],
           [0.1,0.3],
           [0.08,0.21],
           [0.03,0.29],
            ])
img=cv.imread('ok.jpg',0)
m=cv.estimateAffine2D(a,b)
print(m[0])
print('ord=',np.matmul(m[0][:,0:2],a[0,:].T)+m[0][:,2],'\ndst= ',e)




# point=np.zeros([2,1])
# point[0],point[1]=240,378.5
# m=np.array([[ 5.77339270e-04,-4.90445181e-05,-1.21037120e-01],
#  [-1.09997657e-05,-5.95652116e-04 ,4.68204196e-01]])
# # print(m,end='\n\n')
# print(np.matmul(m[:,0:2], point).T[0] +m[:,2])

# print(b[0])
# dst=cv.warpAffine(img,m[0],img.shape, flags=cv.INTER_LINEAR,borderMode=cv.BORDER_REFLECT,borderValue=(255,255,255),)
# cv.imshow('wi',img)
# cv.imshow('wti',dst)

# import cv2
import numpy as np

# 这里说一下旋转的opencv中为旋转提供的三个要素
# 旋转的中心点（center）
# 旋转角度()
# 旋转后进行放缩
# 我们可以通过cv2.getRotationMatrix2D函数得到转换矩阵

# img = cv2.imread('ok.jpg')
# rows,cols,_ = img.shape

# matrix = cv2.getRotationMatrix2D((cols/2,rows/2),90,1)

# print(matrix)
# # 得到变换的矩阵，通过这个矩阵再利用warpAffine来进行变换
# # 第一个参数就是旋转中心，元组的形式，这里设置成相片中心
# # 第二个参数90，是旋转的角度
# # 第三个参数1，表示放缩的系数，1表示保持原图大小
#
# img1 = cv2.warpAffine(img,matrix,(cols,rows))
#
# cv2.imshow('img',img)
# cv2.imshow('img1',img1)
# cv.waitKey(0)


