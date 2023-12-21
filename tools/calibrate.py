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
    parse.add_option('-s', '--save', dest='save', action='store_true', default=False, help='save images used for calibration')
    parse.add_option('-c', '--cache', dest='cache', action='store_true', default=False, help='use cached images for calibration')
    parse.add_option('-d', '--device', dest='device', type='int', default=0, help='device id')
    parse.add_option('--sn', dest='sn', type='string', default=None, help='serial number of the camera')
    parse.add_option('--grid-size', dest='grid_size', type='float', default=20, help='size of the grid in mm')
    return parse.parse_args()

# def 

if __name__ == "__main__":
    options, args = parse()
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    rows, cols = 6, 9          #需要比标定板的尺寸都小一
    size = (rows, cols)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    obj_p = np.zeros((rows*cols, 3), np.float32)
    obj_p[:, :2] = np.mgrid[0:rows, 0:cols].T.reshape(-1, 2)
    obj_p *= options.grid_size
    obj_points = []
    img_points = []

    if options.save and not os.path.exists('outputs'):
        os.mkdir('outputs')
    if not options.sn and not options.cache:
        camera = mindvision.MindVision(1280, 1024)
        options.sn = camera.device_info.GetSn()
    if not options.cache:
        source = map(lambda pack: pack[1], camera.get_frames())
    else:
        source = map(cv2.imread, glob.glob('images/*.jpg'))
    for frame in source:
        if not options.cache:
            copy = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCornersSB(gray, size, None, cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
        cv2.drawChessboardCorners(frame, size, corners, ret)
        cv2.imshow('frame', frame)
        key_pressed = cv2.waitKey(0 if options.cache else 1)
        if ret and key_pressed == ord('s'):
            print("Save one image")
            if options.save:
                cv2.imwrite(time.strftime('outputs/%Y-%m-%d-%H-%M-%S.jpg', time.localtime()), copy)
            obj_points.append(obj_p)
            img_points.append(corners)
        elif key_pressed == ord('q'):
            break
    cv2.destroyAllWindows()
    if len(obj_points) > 0:
        _, mtx, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, frame.shape[:2][::-1], None, None)
        data = {'intrinsics': [mtx[0, 0], mtx[0, 2], mtx[1, 1], mtx[1, 2]],
                'extrinsics': [1.65, 0.1, 0.01],
                'distortion': [dist[0, 0], dist[0, 1], dist[0, 2], dist[0, 3], dist[0, 4]]}
        pprint.PrettyPrinter(indent=4).pprint(data)
        json.dump(data, open(f'./{options.sn}.json', 'w'), indent=4)
    else:
        print("no image provide for calibration")
