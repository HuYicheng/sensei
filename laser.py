

#import cv2
#import time
import nidaqmx
import datetime

# with nidaqmx.Task() as task:
task = nidaqmx.Task()
task.ao_channels.add_ao_voltage_chan("Dev1/ao1")
# task.write([1.1, 2.2, 3.3], auto_start=True)
task.write(-10, auto_start=True)
while True:
    # time.sleep(3)
    value = input('Do you like the laser location? \n')
    if value == 'y':
        task.write(0, auto_start=True)
        break

task.start()
task.stop()
task.close()

'''
task = nidaqmx.Task()
task.ao_channels.add_ao_voltage_chan("Dev1/ao1")
# task.write([1.1, 2.2, 3.3], auto_start=True)
task.write(-3, auto_start=True)
time.sleep(3)
task.write(3, auto_start=True)

task.start()
task.stop()
task.close()
'''

