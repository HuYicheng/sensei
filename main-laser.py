from ximea import xiapi
import matplotlib as mpl
from PIL import Image
import sys
import cv2
import os
import time
from datetime import datetime
import matplotlib.pyplot as plt
import glob
import nidaqmx
import serial
import winsound
from screeninfo import get_monitors

time_start=datetime.now().strftime("%H:%M:%S")
print('Start at %s'%time_start)

print('\nInitializing')

#set the port of shutter
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
cam1.set_gain(0)
cam1.set_downsampling('XI_DWN_2x2')

cam2.set_imgdataformat('XI_RGB24')
cam2.set_exposure(1000000)
cam2.disable_auto_wb()
cam2.set_wb_kr(1.110)
cam2.set_wb_kg(1.001)
cam2.set_wb_kb(2.435)
cam2.set_gain(0)
cam2.set_downsampling('XI_DWN_2x2')

print('cam1.autowb=%s' % cam1.is_auto_wb())
print('cam2.autowb=%s' % cam2.is_auto_wb())


cam1.enable_horizontal_flip()
cam2.enable_horizontal_flip()

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

task = nidaqmx.Task()
task.ao_channels.add_ao_voltage_chan("Dev1/ao1")


print('\nChecking')
# Display camera view before collection
# Laparoscope
print('------Please Check View With No Laser and Pattern and Enter Esc for Exit Display------')
showPatternImg(23)
while True:

    com.write(bytearray([0x10, 0x02, 0x01, 0x01, 0x21, 0x01]))  # enable
    com.flushInput()
    com.flushOutput()
    task.write(-9, auto_start=True)

    cam1.set_exposure(100000)
    cam2.set_exposure(100000)
    cam1.disable_auto_wb()
    cam2.disable_auto_wb()
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

com.write(bytearray([0x10, 0x02, 0x01, 0x02, 0x21, 0x01]))   # 0x02 for disable
com.flushInput()
com.flushOutput()
#time.sleep(1)



# Check laser
print('------Please Check Laser Position and Press Esc for Laser Off------')
# Laser on
while True:
    cam1.set_exposure(250000)
    cam2.set_exposure(250000)
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
        # Laser off
        task.write(9, auto_start=True)
        task.start()
        task.stop()
        task.close()
        cv2.destroyAllWindows()
        print('Laser Check Done!')
        break

print('------Please Check projector------')
showPatternImg(21)
# Laser on
while True:
    cam1.set_exposure(100000)
    cam2.set_exposure(100000)
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
        # Laser off
        cv2.destroyAllWindows()
        print('Projector Check Done!')
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

pthRoot = os.path.join(rootdir, 'collectedData', 'test')

if not os.path.isdir(pthRoot):
    os.makedirs(pthRoot)

pthRoot_C0 = os.path.join(pthRoot, 'camera0')
pthRoot_C1 = os.path.join(pthRoot, 'camera1')

if not os.path.isdir(pthRoot_C0):
    os.makedirs(pthRoot_C0)
if not os.path.isdir(pthRoot_C1):
    os.makedirs(pthRoot_C1)

pthRoot_rgb_C0 = os.path.join(pthRoot_C0, 'rgb')
pthRoot_rgb_C1 = os.path.join(pthRoot_C1, 'rgb')

if not os.path.isdir(pthRoot_rgb_C0):
    os.makedirs(pthRoot_rgb_C0)
if not os.path.isdir(pthRoot_rgb_C1):
    os.makedirs(pthRoot_rgb_C1)

pthRoot_rgb_line_C0 = os.path.join(pthRoot_C0, 'rgb_line')
pthRoot_rgb_line_C1 = os.path.join(pthRoot_C1, 'rgb_line')

if not os.path.isdir(pthRoot_rgb_line_C0):
    os.makedirs(pthRoot_rgb_line_C0)
if not os.path.isdir(pthRoot_rgb_line_C1):
    os.makedirs(pthRoot_rgb_line_C1)

pthRoot_laser_on_C0 = os.path.join(pthRoot_C0, 'laser_on')
pthRoot_laser_on_C1 = os.path.join(pthRoot_C1, 'laser_on')

if not os.path.isdir(pthRoot_laser_on_C0):
    os.makedirs(pthRoot_laser_on_C0)
if not os.path.isdir(pthRoot_laser_on_C1):
    os.makedirs(pthRoot_laser_on_C1)

pthRoot_laser_off_C0 = os.path.join(pthRoot_C0, 'laser_off')
pthRoot_laser_off_C1 = os.path.join(pthRoot_C1, 'laser_off')

if not os.path.isdir(pthRoot_laser_off_C0):
    os.makedirs(pthRoot_laser_off_C0)
if not os.path.isdir(pthRoot_laser_off_C1):
    os.makedirs(pthRoot_laser_off_C1)

pthRoot_stl_C0 = os.path.join(pthRoot_C0, 'stl')
pthRoot_stl_C1 = os.path.join(pthRoot_C1, 'stl')

if not os.path.isdir(pthRoot_stl_C0):
    os.makedirs(pthRoot_stl_C0)
if not os.path.isdir(pthRoot_stl_C1):
    os.makedirs(pthRoot_stl_C1)

print('\nCollecting')

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
    print('\n------Round', round, 'Start------')

    case=len(os.listdir(pthRoot_rgb_C0))

    # Capture RGB image for camera0 and camera1 with laser on
    # Laser on
    task = nidaqmx.Task()
    task.ao_channels.add_ao_voltage_chan("Dev1/ao1")

    cam1.set_exposure(200000)
    cam2.set_exposure(200000)

    showPatternImg(23)

    com.write(bytearray([0x10, 0x02, 0x01, 0x01, 0x21, 0x01]))  # enable
    com.flushInput()
    com.flushOutput()
    #time.sleep(1)

    task.write(3, auto_start=True)
    time.sleep(2)

    cam1.get_image(img1)
    cam2.get_image(img2)
    data_raw_on_1 = img1.get_image_data_numpy()
    data_raw_on_2 = img2.get_image_data_numpy()
    path_RGB_rgb_C0 = os.path.join(pthRoot_rgb_C0, str(case)+'.jpg')
    path_RGB_rgb_C1 = os.path.join(pthRoot_rgb_C1, str(case)+'.jpg')
    cv2.imwrite(path_RGB_rgb_C0, data_raw_on_1)
    cv2.imwrite(path_RGB_rgb_C1, data_raw_on_2)
    path_RGB_rgb_line_C0 = os.path.join(pthRoot_rgb_line_C0, str(case)+'.jpg')
    path_RGB_rgb_line_C1 = os.path.join(pthRoot_rgb_line_C1, str(case)+'.jpg')
    cv2.imwrite(path_RGB_rgb_line_C0, data_raw_on_1)
    cv2.imwrite(path_RGB_rgb_line_C1, data_raw_on_2)
    time.sleep(2)
    print('RGB collected')

    cam1.set_exposure(250000)
    cam2.set_exposure(250000)

    com.write(bytearray([0x10, 0x02, 0x01, 0x02, 0x21, 0x01]))   # 0x02 for disable
    com.flushInput()
    com.flushOutput()
    time.sleep(2)

    cam1.get_image(img1)
    cam2.get_image(img2)
    data_raw_on_1 = img1.get_image_data_numpy()
    data_raw_on_2 = img2.get_image_data_numpy()
    path_RGB_laser_off_C0 = os.path.join(pthRoot_laser_off_C0, str(case)+'.jpg')
    path_RGB_laser_off_C1 = os.path.join(pthRoot_laser_off_C1, str(case)+'.jpg')
    cv2.imwrite(path_RGB_laser_off_C0, data_raw_on_1)
    cv2.imwrite(path_RGB_laser_off_C1, data_raw_on_2)
    time.sleep(1)
    print('Laser Off collected')

    # Laser on
    task.write(-9, auto_start=True)
    time.sleep(2)

    cam1.get_image(img1)
    cam2.get_image(img2)
    data_raw_off_1 = img1.get_image_data_numpy()
    data_raw_off_2 = img2.get_image_data_numpy()
    path_RGB_laser_on_C0 = os.path.join(pthRoot_laser_on_C0, str(case)+'.jpg')
    path_RGB_laser_on_C1 = os.path.join(pthRoot_laser_on_C1, str(case)+'.jpg')
    cv2.imwrite(path_RGB_laser_on_C0, data_raw_off_1)
    cv2.imwrite(path_RGB_laser_on_C1, data_raw_off_2)
    time.sleep(1)
    print('Laser On collected')

    pthRoot_stl_case_C0 = os.path.join(pthRoot_stl_C0, str(case))
    pthRoot_stl_case_C1 = os.path.join(pthRoot_stl_C1, str(case))

    if not os.path.isdir(pthRoot_stl_case_C0):
        os.makedirs(pthRoot_stl_case_C0)
    if not os.path.isdir(pthRoot_stl_case_C1):
        os.makedirs(pthRoot_stl_case_C1)

    task.write(3, auto_start=True)#laser off
    time.sleep(1)

    for patternCnt in range(22):
        showPatternImg(patternCnt)

        time.sleep(1)

        cam1.get_image(img1)
        cam2.get_image(img2)
        data_raw_off_1 = img1.get_image_data_numpy()
        data_raw_off_2 = img2.get_image_data_numpy()
        path_stl_case_C0 = os.path.join(pthRoot_stl_case_C0, '{:02d}.jpg'.format(patternCnt))
        path_stl_case_C1 = os.path.join(pthRoot_stl_case_C1, '{:02d}.jpg'.format(patternCnt))
        cv2.imwrite(path_stl_case_C0, data_raw_off_1)
        cv2.imwrite(path_stl_case_C1, data_raw_off_2)
        time.sleep(1)
    print('Stl Collected')

    task.write(3, auto_start=True)
    task.start()
    task.stop()
    task.close()

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

print('\n%d cases has been collected'%(case+1))

com.close()
winsound.Beep(600,1000)

time_end=datetime.now().strftime("%H:%M:%S")
print('\nEnd at %s'%time_end)