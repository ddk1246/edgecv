import cv2
import numpy as np


def nothing(x):
    pass


def mythreshold(image):
    # open image
    # image_org = cv2.imread(image)
    # transe image to gray
    image_gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    cv2.namedWindow("image")
    cv2.createTrackbar("threshold", "image", 0, 255, nothing)

    while True:
        mythreshold = cv2.getTrackbarPos("threshold", "image")
        ret, image_bin = cv2.threshold(image_gray, mythreshold, 255,
                                        cv2.THRESH_BINARY_INV )
        cv2.imshow("image", image_bin)
        if cv2.waitKey(10) & 0xFF == ord("q"):
            break
    cv2.destroyAllWindows()


def main():
    cap = cv2.VideoCapture(1)
    flag = 1
    num = 0
    m = [[5.49253839e-04, -1.51629133e-05, -1.57567568e-01],
         [-1.59446295e-05, -5.84923484e-04, 4.11060449e-01]]
    while flag:
        ret, frame = cap.read()
        # ld.analysis(frame)
        path = "004.jpg"
        mythreshold(frame)




    # img=cv2.imread("000.png")
    # print(img.shape)
    # m=np.array(img)
    # # m=0
    # m[:,:]=0
    # m[80:400,80:560]=255
    # print(m.shape)
    # dst=cv2.bitwise_and(img,m)
    # cv2.imshow('a',dst)
    # cv2.imshow('b',m)
    # cv2.waitKey(0)


if __name__ == '__main__':
    main()