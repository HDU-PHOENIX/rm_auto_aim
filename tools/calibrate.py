#!python3

import cv2
import glob
import json
import mindvision
import numpy as np
import optparse
import os
import pprint
import time
import argparse

def parse():
    parse = optparse.OptionParser()
    parse.add_option('-s', '--save', dest='save', action='store_true', default=True, help='save images used for calibration')
    parse.add_option('-c', '--cache', dest='cache', action='store_true', default=False, help='use cached images for calibration')
    parse.add_option('-d', '--device', dest='device', type='int', default=0, help='device id')
    parse.add_option('--sn', dest='sn', type='string', default=None, help='serial number of the camera')
    parse.add_option('--grid-size', dest='grid_size', type='float', default=20, help='size of the grid in mm')
    parse.add_option('--auto-save', dest='auto_save', action='store_true', default=False, help='auto save images')
    parse.add_option('--auto-save-speed', dest='auto_save_speed', type='float', default=2.0, help='wait time between two saved images (unit: second)')
    return parse.parse_args()

# def 

if __name__ == "__main__":
    options, args = parse()
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # 定义棋盘格的尺寸（棋盘格内角点）
    rows, cols = 8, 12
    size = (rows, cols)

    # 迭代终止条件（最大误差容忍度0.001 + 最大迭代次数30）
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)

    # 定义 3D 点的世界坐标
    obj_p = np.zeros((rows*cols, 3), np.float32)
    obj_p[:, :2] = np.mgrid[0:rows, 0:cols].T.reshape(-1, 2)
    obj_p *= options.grid_size
    obj_points = []  # 存储棋盘图像 3D 点向量
    img_points = []  # 存储棋盘图像 2D 点向量

    flash=np.zeros((1024,1280,3),np.uint8)
    flash.fill(255)
    flash_weight=0.0

    now=time.time()

    cv2.namedWindow('frame', cv2.WINDOW_NORMAL)

    if options.save and not os.path.exists('outputs'):
        os.mkdir('outputs')
    if not options.sn and not options.cache:
        camera = mindvision.MindVision(1280, 1024)
        options.sn = camera.device_info.GetSn()
        # camera = cv2.VideoCapture(0)
        
    if not options.cache:
        source = map(lambda pack: pack[1], camera.get_frames())
    else:
        source = map(cv2.imread, glob.glob('outputs/*.jpg'))
    # frame=np.zeros((1024,1280,3),np.uint8)
    for frame in source:
    # while camera.read(frame):
        if not options.cache:
            copy = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 找棋盘角，若找到所需数量的角，则 ret = true
        ret, corners = cv2.findChessboardCornersSB(gray, size, None, cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
        # 绘制并显示角
        cv2.drawChessboardCorners(frame, size, corners, ret)
        frame=cv2.addWeighted(frame,1-flash_weight,flash,flash_weight,0)
        if flash_weight > 0:
            flash_weight -= 0.2
        cv2.imshow('frame', frame)
        key_pressed = cv2.waitKey(0 if options.cache else 1)
        if ret and (key_pressed == ord('s') or options.auto_save and time.time()-now>options.auto_save_speed):
            now=time.time()
            print("Save one image")
            if not options.cache:
                flash_weight = 1.0
            if options.save and not options.cache:
                cv2.imwrite(time.strftime('outputs/%Y-%m-%d-%H-%M-%S.jpg', time.localtime()), copy)
            obj_points.append(obj_p)
            # 细化给定二维点的像素坐标
            corners = cv2.cornerSubPix(gray, corners, (9, 9),(-1,-1), criteria)
            img_points.append(corners)
        elif key_pressed == ord('q'):
            break
    cv2.destroyAllWindows()
    if len(obj_points) > 0:
        diff, mtx, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, frame.shape[:2][::-1], None, None, criteria=criteria)
        data = {'intrinsics': [mtx[0, 0], 0, mtx[0, 2], 0, mtx[1, 1], mtx[1, 2], 0, 0, 1],
                'extrinsics': [1.65, 0.1, 0.01],
                'distortion': [dist[0, 0], dist[0, 1], dist[0, 2], dist[0, 3], dist[0, 4]]}
        pprint.PrettyPrinter(indent=4).pprint(data)
        json.dump(data, open(f'./{options.sn}.json', 'w'), indent=4)
        print("重投影误差： %lf" % diff)
    else:
        print("no image provide for calibration")
