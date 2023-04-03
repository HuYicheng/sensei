import os,shutil
import cv2

root='E:\Download\Ximea-2023-Phantom-laser-rotate\collectedData'
b=0
for pack in range(12):
    path = os.path.join(root,str(pack))
    exp = os.listdir(path)
    for time in exp:
        # print(exp.index(time))
        path_exp=os.path.join(path,time)
        path_cam0=os.path.join(path_exp,'camera0')
        path_cam1=os.path.join(path_exp,'camera1')
        for round in range(10):
            path_round_cam0=os.path.join(path_cam0,'round_'+str(round))
            path_round_cam1=os.path.join(path_cam1,'round_'+str(round))
            # print(os.path.join(path_round_cam0,'rbg.jpg'))
            # print(os.path.join(path_round_cam1,'rbg.jpg'))
            # print(os.path.join(root,'all_data','cam0','rgb',str(b)+'.jpg'))
            # print(os.path.join(root,'all_data','cam1','rgb',str(b)+'.jpg'))
            # print(os.path.join(root,'all_data','cam0','laser_on',str(b)+'.jpg'))
            # print(os.path.join(root,'all_data','cam1','laser_on',str(b)+'.jpg'))
            # print(os.path.join(root,'all_data','cam0','laser_off',str(b)+'.jpg'))
            # print(os.path.join(root,'all_data','cam1','laser_off',str(b)+'.jpg'))

            shutil.copyfile(os.path.join(path_round_cam0,'rgb.jpg'),os.path.join(root,'all_data','cam0','rgb',str(b)+'.jpg'))
            shutil.copyfile(os.path.join(path_round_cam1,'rgb.jpg'),os.path.join(root,'all_data','cam1','rgb',str(b)+'.jpg'))
            shutil.copyfile(os.path.join(path_round_cam0,'laser_on.jpg'),os.path.join(root,'all_data','cam0','laser_on',str(b)+'.jpg'))
            shutil.copyfile(os.path.join(path_round_cam1,'laser_on.jpg'),os.path.join(root,'all_data','cam1','laser_on',str(b)+'.jpg'))
            shutil.copyfile(os.path.join(path_round_cam0,'laser_off.jpg'), os.path.join(root,'all_data','cam0','laser_off',str(b)+'.jpg'))
            shutil.copyfile(os.path.join(path_round_cam1,'laser_off.jpg'), os.path.join(root,'all_data','cam1','laser_off',str(b)+'.jpg'))
            b += 1

