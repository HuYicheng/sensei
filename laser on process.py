import cv2
import numpy as np

img=cv2.imread('E:\Download\Ximea-2023-Phantom-laser-rotate\collectedData/7\laser-2023-03-01-15-34-17\camera0/round_0\laser_on.jpg')
img1=cv2.imread('E:\Download\Ximea-2023-Phantom-laser-rotate\collectedData/7\laser-2023-03-01-15-34-17\camera0/round_0\laser_off.jpg')

img=img[:,:,2]
img1=img1[:,:,2]
img2=np.int32(img)-np.int32(img1)
img3=np.uint8(np.where(img2>15,255,0))

cv2.imwrite('img1.jpg',img3)
cv2.waitKey(0)



