from ximea import xiapi
import cv2
import time
from datetime import datetime
import os
from tracemalloc import start
import matplotlib.pyplot as plt
import matplotlib as mpl
from PIL import Image
from importlib.resources import path
from math import degrees
from tkinter import Tk,messagebox
from numpy import angle
import sys
import cv2
import os
# import thorlabs_apt as apt
import time
import matplotlib.pyplot as plt
import glob
import nidaqmx
import serial
import winsound


com = serial.Serial(port='COM6',
                    baudrate=115200,
                    bytesize=8,
                    parity=serial.PARITY_NONE,
                    stopbits=1,
                    xonxoff=0,
                    rtscts=0,
                    timeout=1)
print('the status of com is %s'%com.is_open)
com.flushInput()
com.flushOutput()

command = bytearray([0x05, 0x00, 0x00, 0x00, 0x01, 0x01])  # identify
com.write(command)
com.flushInput()
com.flushOutput()
#time.sleep(1)

ser = serial.Serial(port="COM3",
                    baudrate=115200,
                    bytesize=8,
                    timeout=2,
                    stopbits=serial.STOPBITS_ONE)
print('the status of ser is %s'%ser.is_open)

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
cam1.set_exposure(1000000)
cam1.disable_auto_wb()
cam1.set_wb_kr(1.110)
cam1.set_wb_kg(0.985)
cam1.set_wb_kb(2.5)

cam2.set_imgdataformat('XI_RGB24')
cam2.set_exposure(1000000)
cam2.disable_auto_wb()
cam2.set_wb_kr(1.110)
cam2.set_wb_kg(1.001)
cam2.set_wb_kb(2.435)

# cam1.set_gain(1)
# cam2.set_gain(1)

print('Cam1: Exposure was set to %i us' %cam1.get_exposure())
print('Cam2: Exposure was set to %i us' %cam2.get_exposure())

print('cam1.autowb=%s' % cam1.is_auto_wb())
print('cam2.autowb=%s' % cam2.is_auto_wb())

# print('cam1.kr=%.3f' % cam1.get_wb_kr())
# print('cam1.kg=%.3f' % cam1.get_wb_kg())
# print('cam1.kb=%.3f' % cam1.get_wb_kb())
# print('cam2.kr=%.3f' % cam2.get_wb_kr())
# print('cam2.kg=%.3f' % cam2.get_wb_kg())
# print('cam2.kb=%.3f' % cam2.get_wb_kb())
#
# print('cam1.gain=%.3f' % cam1.get_gain())
# print('cam2.gain=%.3f' % cam2.get_gain())

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

# Initiate flags / counters
round = 0
step = 36
max_round = 360/step
rotRatio = 0.9166/5

# task = nidaqmx.Task()
# task.ao_channels.add_ao_voltage_chan("Dev1/ao1")

# Display camera view before collection
# Laparoscope
print('------Please Check View With No Laser and Pattern and Enter Esc for Exit Display------')
while True:
    com.write(bytearray([0x10, 0x02, 0x01, 0x01, 0x21, 0x01]))  # enable
    com.flushInput()
    com.flushOutput()
    # task.write(-9, auto_start=True)

    cam1.set_exposure(400000)
    cam2.set_exposure(400000)
    cam1.disable_auto_wb()
    cam2.disable_auto_wb()
    # print(cam1.is_auto_wb())
    # print(cam2.is_auto_wb())
    cam1.get_image(img1)
    cam2.get_image(img2)
    data_raw1 = img1.get_image_data_numpy()
    data_raw2 = img2.get_image_data_numpy()
    cv2.imshow('CAM 1', cv2.resize(data_raw1, dsize=(640, 480), interpolation=cv2.INTER_CUBIC))
    cv2.imshow('CAM 2', cv2.resize(data_raw2, dsize=(640, 480), interpolation=cv2.INTER_CUBIC))
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


# Get current dir and creat folders for the day
#pthRoot = os.path.join(rootdir, 'collectedData', 'laser-' + datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))

pthRoot = os.path.join(rootdir, 'collectedData', 'sensei_visual')
pthRoot_C0 = os.path.join(pthRoot, 'camera0')
pthRoot_C1 = os.path.join(pthRoot, 'camera1')
pthRoot_sensei = os.path.join(pthRoot,'sensei')

if not os.path.isdir(pthRoot):
    os.makedirs(pthRoot)
if not os.path.isdir(pthRoot_C0):
    os.makedirs(pthRoot_C0)
if not os.path.isdir(pthRoot_C1):
    os.makedirs(pthRoot_C1)
if not os.path.isdir(pthRoot_sensei):
    os.makedirs(pthRoot_sensei)

# Move rotation stage to "home" position (aka Position where angle = 0)
import thorlabs_apt as apt
apt_list = apt.list_available_devices()
motor = apt.Motor(apt_list[0][1])
print('Homing rotational stage...')
motor.move_home(True)
print('Finished rotational stage homing...Start to collect data...')
motor.set_stage_axis_info(0.0, 360.0, 2, 1.0)
time.sleep(1)

while True:
    k = cv2.waitKey(1) & 0Xff
    if k == 27:
        break

    # Round start
    print('------Round', round, 'Start------')

    # Create round folder
    # pthRoot_round_C0 = os.path.join(pthRoot_C0, 'round_' + str(round))
    # pthRoot_round_C1 = os.path.join(pthRoot_C1, 'round_' + str(round))
    # if not os.path.isdir(pthRoot_round_C0):
    #     os.makedirs(pthRoot_round_C0)
    # if not os.path.isdir(pthRoot_round_C1):
    #     os.makedirs(pthRoot_round_C1)

    pthRoot_rgb_C0 = os.path.join(pthRoot_C0, 'rgb')
    pthRoot_rgb_C1 = os.path.join(pthRoot_C1, 'rgb')

    if not os.path.isdir(pthRoot_rgb_C0):
        os.makedirs(pthRoot_rgb_C0)
    if not os.path.isdir(pthRoot_rgb_C1):
        os.makedirs(pthRoot_rgb_C1)


    case=len(os.listdir(pthRoot_rgb_C0))


    cam1.set_exposure(700000)
    cam2.set_exposure(700000)

    # cam1.disable_auto_wb()
    # cam2.disable_auto_wb()
    # print('cam1.autowb=%s'%cam1.is_auto_wb())
    # print('cam2.autowb=%s'%cam2.is_auto_wb())
    # print('cam1.kr=%.3f'%cam1.get_wb_kr())
    # print('cam1.kg=%.3f'%cam1.get_wb_kg())
    # print('cam1.kb=%.3f'%cam1.get_wb_kb())
    # print('cam2.kr=%.3f'%cam2.get_wb_kr())
    # print('cam2.kg=%.3f'%cam2.get_wb_kg())
    # print('cam2.kb=%.3f'%cam2.get_wb_kb())
    # print('cam1.gain=%.3f'%cam1.get_gain())
    # print('cam2.gain=%.3f'%cam2.get_gain())



    com.write(bytearray([0x10, 0x02, 0x01, 0x01, 0x21, 0x01]))  # enable
    com.flushInput()
    com.flushOutput()
    #time.sleep(1)

    # task.write(3, auto_start=True)
    time.sleep(7)

    cam1.get_image(img1)
    cam2.get_image(img2)
    data_raw_on_1 = img1.get_image_data_numpy()
    data_raw_on_2 = img2.get_image_data_numpy()
    path_RGB_rgb_C0 = os.path.join(pthRoot_rgb_C0, str(case)+'.jpg')
    path_RGB_rgb_C1 = os.path.join(pthRoot_rgb_C1, str(case)+'.jpg')
    cv2.imwrite(path_RGB_rgb_C0, data_raw_on_1)
    cv2.imwrite(path_RGB_rgb_C1, data_raw_on_2)
    time.sleep(7)

    pthSensei = os.path.join(pthRoot_sensei, str(case)+".txt")
    data_raw = ser.readline()
    a = data_raw.decode()  # a is str
    with open(pthSensei, "a") as f:
        f.write(a)
        # f.write('\n')

    # k = cv2.waitKey(1) & 0Xff
    # if k == 27:
    #     break

    print("Images written..., rotate for next round...")
    #time.sleep(1)
    # Rotational stage move
    if round < max_round-1:
        motor.move_by(step * rotRatio, True)
        time.sleep(1)
        round += 1
    else:
        round = 0
        motor.move_home(True)
        cv2.destroyAllWindows()
        break

print('%d pictures has been collected'%(case+1))

com.close()
winsound.Beep(600,1000)

