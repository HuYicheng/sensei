from ximea import xiapi
import cv2
import time

from ximea import xiapi
from datetime import datetime
import sys
import cv2
import os
import time
import serial
import winsound
import numpy as np


com = serial.Serial(port='COM6',
                        baudrate=115200,
                        bytesize=8,
                        parity=serial.PARITY_NONE,
                        stopbits=1, xonxoff=0,
                        rtscts=0,
                        timeout=1)
print(com.is_open)
com.flushInput()
com.flushOutput()

command = bytearray([0x05, 0x00, 0x00, 0x00, 0x01, 0x01])  # identify
com.write(command)
com.flushInput()
com.flushOutput()
#time.sleep(1)

com.write(bytearray([0x10, 0x02, 0x01, 0x01, 0x21, 0x01]))  # enable
com.flushInput()
com.flushOutput()




cam1 = xiapi.Camera(dev_id=0) #create instance for cameras
cam2 = xiapi.Camera(dev_id=1)

# create instance for first connected camera
# cam = xiapi.Camera()

# start communication
print('Opening first camera...')
cam1.open_device()
cam2.open_device()

# settings

cam1.set_imgdataformat('XI_RGB24')
cam1.set_exposure(50000)
cam1.disable_auto_wb()
cam1.set_wb_kr(1.110)
cam1.set_wb_kg(0.985)
cam1.set_wb_kb(2.5)
cam1.set_gain(0)
cam1.set_downsampling('XI_DWN_2x2')
cam1.set_gain(5)

cam2.set_imgdataformat('XI_RGB24')
cam2.set_exposure(50000)
cam2.disable_auto_wb()
cam2.set_wb_kr(1.110)
cam2.set_wb_kg(1.001)
cam2.set_wb_kb(2.435)
cam2.set_gain(0)
cam2.set_downsampling('XI_DWN_2x2')
cam1.set_gain(5)

# create instance of Image to store image data and metadata
img1 = xiapi.Image()
img2 = xiapi.Image()

# start data acquisition
print('Starting data acquisition...')
cam1.start_acquisition()
cam2.start_acquisition()


video_root=r'E:\Download\Ximea-2023-Phantom-laser-rotate\video'
if not os.path.exists(video_root):
        os.makedirs(video_root)
video_path=os.path.join(video_root,'output.mp4')

size = 640*2, 480
duration = 2
fps = 20
fourcc = cv2.VideoWriter_fourcc(*'MP4V')
out = cv2.VideoWriter(video_path, fourcc, fps, (size[0], size[1]), True)

try:
    print('Starting video. Press CTRL+C to exit.')
    t0 = time.time()
    while True:
        # get data and pass them from camera to img
        cam1.get_image(img1)
        cam2.get_image(img2)

        # create numpy array with data from camera. Dimensions of the array are
        # determined by imgdataformat
        data1 = img1.get_image_data_numpy()
        data2 = img2.get_image_data_numpy()

        # print(data1.shape)

        # show acquired image with time since the beginning of acquisition
        font = cv2.FONT_HERSHEY_SIMPLEX
        text = '{:5.2f}'.format(time.time() - t0)
        cv2.putText(data1, text, (900, 150), font, 4, (255, 255, 255), 2)
        cv2.putText(data2, text, (900, 150), font, 4, (255, 255, 255), 2)
        # cv2.imshow('cam1', data1)
        # cv2.imshow('cam2', data2)


        data=np.concatenate((data1,data2),axis=1)
        data_resized=cv2.resize(data, dsize=(size[0], size[1]))
        cv2.imshow('cam1 & cam2',data_resized)
        out.write(data_resized)


        cv2.waitKey(1)

except KeyboardInterrupt:
    cv2.destroyAllWindows()
    out.release()

# stop data acquisition
print('Stopping acquisition...')
cam1.stop_acquisition()
cam2.stop_acquisition()

# stop communication
cam1.close_device()
cam2.close_device()

print('Done.')
