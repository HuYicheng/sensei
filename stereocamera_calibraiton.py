import cv2

import matplotlib.pyplot as plt  # plt 用于显示图片
import matplotlib.image as mpimg  # mpimg 用于读取图片

import sys
import numpy as np
import glob
import os

rootdir = os.getcwd()
pthRoot = os.path.join(rootdir, 'calibrationData','13')
pthRoot_C0 = os.path.join(pthRoot, 'camera0')
pthRoot_C1 = os.path.join(pthRoot, 'camera1')

class StereoCalibration(object):
    def __init__(self):
        self.imagesL = self.read_images(pthRoot_C0)
        self.imagesR = self.read_images(pthRoot_C1)
        self.m1 = None
        self.m2 = None
        self.d1 = None
        self.d2 = None
        self.R = None
        self.T = None
        self.baseline=None

    def read_images(self, cal_path):
        filepath = glob.glob(cal_path + '/*.jpg')
        filepath.sort()
        return filepath

    # 标定图像
    def calibration_photo(self):
        # 设置要标定的角点个数
        x_nums = 8
        y_nums = 6

        world_point = np.zeros((x_nums * y_nums, 3), np.float32)  # 生成x_nums*y_nums个坐标，每个坐标包含x,y,z三个元素
        world_point[:, :2] = np.mgrid[:x_nums, :y_nums].T.reshape(-1, 2)  # mgrid[]生成包含两个二维矩阵的矩阵，每个矩阵都有x_nums列,y_nums行
        # .T矩阵的转置
        # reshape()重新规划矩阵，但不改变矩阵元素
        # 保存角点坐标
        world_position = []
        image_positionl = []
        image_positionr = []
        # 设置角点查找限制
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # 获取所有标定图
        for ii in range(101):

            image_path_l = self.imagesL[ii]
            image_path_r = self.imagesR[ii]

            image_l = cv2.imread(image_path_l)
            image_r = cv2.imread(image_path_r)
            gray_l = cv2.cvtColor(image_l, cv2.COLOR_RGB2GRAY)
            gray_r = cv2.cvtColor(image_r, cv2.COLOR_RGB2GRAY)

            # 查找角点
            #         ok,corners = cv2.findChessboardCorners(gray,(x_nums,y_nums),None)
            ok1,cornersl = cv2.findChessboardCorners(gray_l,(x_nums,y_nums),None)
            ok2,cornersr = cv2.findChessboardCorners(gray_r,(x_nums,y_nums),None)
            # ok1, cornersl = cv2.findCirclesGrid(gray_l, (x_nums, y_nums), None)
            # ok2, cornersr = cv2.findCirclesGrid(gray_r, (x_nums, y_nums), None)

            self.world = world_point
            print(ok1 & ok2)
            if ok1 & ok2:
                # 把每一幅图像的世界坐标放到world_position中


                center_spacing = 15  ## 圆心的位置距离，这一个其实不重要
                world_position.append(world_point * center_spacing)
                # 获取更精确的角点位置
                exact_cornersl = cv2.cornerSubPix(gray_l, cornersl, (11, 11), (-1, -1), criteria)
                exact_cornersr = cv2.cornerSubPix(gray_r, cornersr, (11, 11), (-1, -1), criteria)
                # 把获取的角点坐标放到image_position中
                image_positionl.append(exact_cornersl)
                image_positionr.append(exact_cornersr)
                # 可视化角点
                image_l = cv2.drawChessboardCorners(image_l,(x_nums,y_nums),exact_cornersl,ok1)
                image_r = cv2.drawChessboardCorners(image_r,(x_nums,y_nums),exact_cornersr,ok2)
                image=np.hstack((image_l,image_r))
                cv2.namedWindow('image_corner', 0)
                cv2.resizeWindow('image_corner', 2448//2, 1840//4)  # 初始窗口大小
                cv2.imshow('image_corner',image)
                cv2.waitKey(3)
        # 计算内参数
        image_shape = gray_l.shape[::-1]

        retl, mtxl, distl, rvecsl, tvecsl = cv2.calibrateCamera(world_position, image_positionl, image_shape, None,None)
        retr, mtxr, distr, rvecsr, tvecsr = cv2.calibrateCamera(world_position, image_positionr, image_shape, None,None)
        print('ml = ', mtxl)
        print('mr = ', mtxr)
        print('dl = ', distl)
        print('dr = ', distr)
        self.m1 = mtxl
        self.m2 = mtxr
        self.d1 = distl
        self.d2 = distr

        # 计算误差
        self.cal_error(world_position, image_positionl, mtxl, distl, rvecsl, tvecsl)
        self.cal_error(world_position, image_positionr, mtxr, distr, rvecsr, tvecsr)

        ##双目标定
        self.stereo_calibrate(world_position, image_positionl, image_positionr, mtxl, distl, mtxr, distr, image_shape)

    def cal_error(self, world_position, image_position, mtx, dist, rvecs, tvecs):
        # 计算偏差
        mean_error = 0
        for i in range(len(world_position)):
            image_position2, _ = cv2.projectPoints(world_position[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(image_position[i], image_position2, cv2.NORM_L2) / len(image_position2)
            mean_error += error
        print("total error: ", mean_error / len(image_position))

    def stereo_calibrate(self, objpoints, imgpoints_l, imgpoints_r, M1, d1, M2, d2, dims):
        flags = 0
        flags |= cv2.CALIB_FIX_INTRINSIC
        flags |= cv2.CALIB_USE_INTRINSIC_GUESS
        flags |= cv2.CALIB_FIX_FOCAL_LENGTH
        flags |= cv2.CALIB_ZERO_TANGENT_DIST
        stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)
        ret, M1, d1, M2, d2, R, T, E, F = cv2.stereoCalibrate(
            objpoints, imgpoints_l,
            imgpoints_r, M1, d1, M2,
            d2, dims,
            criteria=stereocalib_criteria, flags=flags)
        baseline=np.linalg.norm(T)
        print(R)
        print(T)
        print(baseline)
        self.R = R
        self.T = T
        self.baseline=baseline



class stereoCamera(object):
    def __init__(self):
        # 左相机内参数
        self.cam_matrix_left =  np.array([[2.84525888e+03,0.00000000e+00,1.02109209e+03],[0.00000000e+00,2.86052215e+03,1.39848196e+03],[0.00000000e+00,0.00000000e+00,1.00000000e+00]])
        self.cam_matrix_right = np.array([[2.80262721e+03,0.00000000e+00,1.40090626e+03],[0.00000000e+00,2.81485044e+03,1.38589861e+03],[0.00000000e+00,0.00000000e+00,1.00000000e+00]])

        # 左右相机畸变系数:[k1, k2, p1, p2, k3]
        self.distortion_l = np.array([[-0.16131281,0.67745708,0.00330204,-0.00949782,-0.22828643]])
        self.distortion_r = np.array([[-0.27166724,0.47052358,0.00415972,-0.00575537,0.33973046]])
        # 旋转矩阵

        self.R = np.array([[ 0.99988162,0.00389399,-0.01488561],[-0.00306736,0.99847301,0.05515653],[ 0.01507766,-0.05510434,0.99836675]])
        # 平移矩阵
        self.T = np.array([[ -5.42058745],[-10.60804364],[ -4.82106943]])
        self.baseline = np.linalg.norm(self.T)
        self.h=1840
        self.w=2448


def preprocess(img1, img2):
    # 彩色图->灰度图
    im1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    im2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    # 直方图均衡
    im1 = cv2.equalizeHist(im1)
    im2 = cv2.equalizeHist(im2)

    return im1, im2


# 消除畸变
def undistortion(image, camera_matrix, dist_coeff):
    undistortion_image = cv2.undistort(image, camera_matrix, dist_coeff)

    return undistortion_image


# 获取畸变校正和立体校正的映射变换矩阵、重投影矩阵
# @param：config是一个类，存储着双目标定的参数:config = stereoconfig.stereoCamera()

def getRectifyTransform(height, width, config):
    # 读取内参和外参
    left_K = config.cam_matrix_left
    right_K = config.cam_matrix_right
    left_distortion = config.distortion_l
    right_distortion = config.distortion_r
    R = config.R
    T = config.T

    # 计算校正变换
    height = int(height)
    width = int(width)
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(left_K, left_distortion, right_K, right_distortion,
                                                      (width, height), R, T, alpha=-1)

    map1x, map1y = cv2.initUndistortRectifyMap(left_K, left_distortion, R1, P1, (width, height), cv2.CV_16SC2)
    map2x, map2y = cv2.initUndistortRectifyMap(right_K, right_distortion, R2, P2, (width, height), cv2.CV_16SC2)
    print(width, height)

    return map1x, map1y, map2x, map2y, Q

# def getRectifyTransform(m1, d1, m2, d2, width, height, r, t):
#     R1, R2, P1, P2, Q, _roi1, _roi2 = \
#     cv2.stereoRectify(cameraMatrix1=m1,
#                       distCoeffs1=d1,
#                       cameraMatrix2=m2,
#                       distCoeffs2=d2,
#                       imageSize=(width, height),
#                       R=r,
#                       T=t,
#                       # flags=0,
#                       flags=cv2.CALIB_ZERO_DISPARITY + cv2.CALIB_USE_INTRINSIC_GUESS,
#                       # flags = cv2.CALIB_ZERO_DISPARITY,
#                       alpha=0.0
#                       )
#
#     map1_x, map1_y = cv2.initUndistortRectifyMap(
#         cameraMatrix=m1,
#         distCoeffs=d1,
#         R=R1,
#         newCameraMatrix=P1,
#         size=(width, height),
#         m1type=cv2.CV_32FC1)
#
#     map2_x, map2_y = cv2.initUndistortRectifyMap(
#         cameraMatrix=m2,
#         distCoeffs=d2,
#         R=R2,
#         newCameraMatrix=P2,
#         size=(width, height),
#         m1type=cv2.CV_32FC1)
#
#     f = Q[2, 3]
#     baseline = 1./Q[3, 2]
#
#     return map1_x, map1_y, map2_x, map2_y, f, baseline, Q


# 畸变校正和立体校正
def rectifyImage(image1, image2, map1x, map1y, map2x, map2y):
    rectifyed_img1 = cv2.remap(image1, map1x, map1y, cv2.INTER_LINEAR)
    rectifyed_img2 = cv2.remap(image2, map2x, map2y, cv2.INTER_LINEAR)

    return rectifyed_img1, rectifyed_img2

# 立体校正检验----画线
def draw_line1(image1, image2):
    # 建立输出图像
    height = max(image1.shape[0], image2.shape[0])
    width = image1.shape[1] + image2.shape[1]

    output = np.zeros((height, width, 3), dtype=np.uint8)
    output[0:image1.shape[0], 0:image1.shape[1]] = image1
    output[0:image2.shape[0], image1.shape[1]:] = image2

    for k in range(15):
        cv2.line(output, (0, height//15 * (k + 1)), (2 * width, height//15 * (k + 1)), (0, 255, 0), thickness=2,
                 lineType=cv2.LINE_AA)  # 直线间隔：100

    return output


if __name__ == '__main__':
    # calibration = StereoCalibration()
    # calibration.calibration_photo()

    config = stereoCamera()  # 读取相机内参和外参
    # map1_x, map1_y, map2_x, map2_y, f, baseline, Q = getRectifyTransform(config.cam_matrix_left,config.distortion_l,config.cam_matrix_right,config.distortion_r,config.w,config.h,config.R, config.T)
    map1_x, map1_y, map2_x, map2_y, Q=getRectifyTransform(1840,2448,config)

    #
    for case in range(0,101):
        imgL = cv2.imread(os.path.join(pthRoot,'camera1',str(case) + '.jpg'))
        imgR = cv2.imread(os.path.join(pthRoot,'camera0',str(case) + '.jpg'))
    # #     imgL , imgR = preprocess(imgL ,imgR )
    #
    #     height, width = imgL.shape[0:2]
    #
    #
    # # 去畸变
    #     imgL = undistortion(imgL, config.cam_matrix_left, config.distortion_l)
    #     imgR = undistortion(imgR, config.cam_matrix_right, config.distortion_r)
    #
    # # 去畸变和几何极线对齐
        iml_rectified, imr_rectified = rectifyImage(imgL, imgR, map1_x, map1_y, map2_x, map2_y)
        linepic = draw_line1(iml_rectified, imr_rectified)
        plt.imshow(linepic)
        plt.show()


