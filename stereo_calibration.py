from ximea import xiapi
from datetime import datetime
import sys
import cv2
import os
import time
import serial
import winsound
import matplotlib as mpl
import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
from screeninfo import get_monitors
import glob



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


# Load Ximea cameras
CAMERAS_ON_SAME_CONTROLLER = 2
interface_data_rate = 2400
SAFE_MARGIN_PERCENTS = 10

cam1 = xiapi.Camera(dev_id=0) #create instance for cameras
cam2 = xiapi.Camera(dev_id=1)

print('Opening cameras...') #start communication
cam1.open_device()
cam2.open_device()

interface_data_rate = cam1.get_limit_bandwidth()  #set interface data rate
print("interface data rate is {}".format(interface_data_rate))
camera_data_rate = int(interface_data_rate / CAMERAS_ON_SAME_CONTROLLER)
print('camera_data_rate is {}'.format(camera_data_rate))

cam1.set_limit_bandwidth(camera_data_rate)  #set data rate
cam2.set_limit_bandwidth(camera_data_rate)

print('Camera 1 serial number: ' + str(cam1.get_device_sn())) #print device serial numbers
print('Camera 2 serial number: ' + str(cam2.get_device_sn()))

#Ximea settings

cam1.set_imgdataformat('XI_RGB24')
cam1.set_exposure(200000)
cam1.disable_auto_wb()
cam1.set_wb_kr(1.110)
cam1.set_wb_kg(0.980)
cam1.set_wb_kb(2.5)
cam1.set_gain(0)
cam1.set_downsampling('XI_DWN_2x2')

cam2.set_imgdataformat('XI_RGB24')
cam2.set_exposure(200000)
cam2.disable_auto_wb()
cam2.set_wb_kr(1.110)
cam2.set_wb_kg(1.001)
cam2.set_wb_kb(2.435)
cam2.set_gain(0)
cam2.set_downsampling('XI_DWN_2x2')



print('Cam1: Exposure was set to %i us' %cam1.get_exposure())
print('Cam2: Exposure was set to %i us' %cam2.get_exposure())
print('Cam1: Downsampling was set to %s us' %cam1.get_downsampling())
print('Cam2: Downsampling was set to %s us' %cam2.get_downsampling())
print('Cam1: Downsampling was set to %s us' %cam1.get_downsampling_type())
print('Cam2: Downsampling was set to %s us' %cam2.get_downsampling_type())

cam1.enable_horizontal_flip()
cam2.enable_horizontal_flip()
# cam1.enable_auto_wb()
# cam2.enable_auto_wb()

img1 = xiapi.Image() #create instance of Image to store image data and metadata
img2 = xiapi.Image()

# Start Ximea data acquisition
# print('Starting data acquisition...\n')
cam1.start_acquisition()
cam2.start_acquisition()

# Get folder of pattern images
rootdir = os.getcwd()


patternDir = os.path.join(rootdir, 'GrayCode_pattern')
patterns = sorted(glob.glob(patternDir + '/*.png'))

projector=get_monitors()[1]
print(projector)

def showPatternImg(patternCnt):
    plt.close()
    mpl.rcParams['toolbar'] = 'None'
    fig = plt.figure(frameon=False)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.axis('off')
    plt.imshow(Image.open(patterns[patternCnt]))
    figManager = plt.get_current_fig_manager()
    figManager.window.setGeometry(projector.x, projector.y, projector.width, projector.height)
    figManager.full_screen_toggle()
    plt.show(block=False)
    # print('Pattern %d is projected' %patternCnt)
    plt.pause(1)
    plt.clf()


# Get current dir and creat folders for the day
pthRoot = os.path.join(rootdir, 'calibrationData','13')
pthRoot_C0 = os.path.join(pthRoot, 'camera0')
pthRoot_C1 = os.path.join(pthRoot, 'camera1')

if not os.path.isdir(pthRoot):
    os.makedirs(pthRoot)
if not os.path.isdir(pthRoot_C0):
    os.makedirs(pthRoot_C0)
if not os.path.isdir(pthRoot_C1):
    os.makedirs(pthRoot_C1)

showPatternImg(23)

while True:
    k = cv2.waitKey(1) & 0Xff
    if k == 27:
        break

    print('------Please Check View and Enter Esc for Exit Display------')
    while True:

        cam1.get_image(img1)
        cam2.get_image(img2)
        data_raw1 = img1.get_image_data_numpy()
        data_raw2 = img2.get_image_data_numpy()
        data_raw_1=data_raw1[1000:1250,500:750,:]
        data_raw_2 = data_raw2[1000:1250,500:750,:]
        cv2.imshow('CAM 1_full', cv2.resize(data_raw1, dsize=(640,480), interpolation=cv2.INTER_CUBIC))
        cv2.imshow('CAM 2_full', cv2.resize(data_raw2, dsize=(640,480), interpolation=cv2.INTER_CUBIC))


        cv2.imshow('CAM 1_focus',data_raw_1)
        cv2.imshow('CAM 2_focus',data_raw_2)

        # cv2.imshow('CAM 1',data_raw1)
        # cv2.imshow('CAM 2', data_raw2)
        k = cv2.waitKey(1) & 0Xff
        if k == 27:
            cv2.destroyAllWindows()
            print('Laparoscope Image Check Done!')
            break

    # Check if continue
    while True:
        value = input('Do You Want to Continue the Data Collection? y:yes; n:no \n')
        if value == 'y':
            break
        elif value == 'n':
            sys.exit()
        else:
            print('Please Use y or n')

    # Capture RGB image for camera0 and camera1

    time.sleep(1)

    cam1.get_image(img1)
    cam2.get_image(img2)
    data_raw_on_1 = img1.get_image_data_numpy()
    data_raw_on_2 = img2.get_image_data_numpy()
    case=len(os.listdir(pthRoot_C0))
    path_RGB_C0 = os.path.join(pthRoot_C0, str(case)+'.jpg')
    path_RGB_C1 = os.path.join(pthRoot_C1, str(case)+'.jpg')
    cv2.imwrite(path_RGB_C0, data_raw_on_1)
    cv2.imwrite(path_RGB_C1, data_raw_on_2)
    time.sleep(2)

    cv2.destroyAllWindows()
    print('case %s has been collected'%case)

com.close()
winsound.Beep(600,1000)
