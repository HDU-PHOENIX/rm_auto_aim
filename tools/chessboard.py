#!python3
import cv2
import numpy as np
import optparse
import os


def parse():
    parser = optparse.OptionParser()
    parser.add_option('-W', '--width', type='int', default=None, help='width of the board')
    parser.add_option('-H', '--height', type='int', default=None, help='height of the board')
    parser.add_option('-S', '--size', type='int', default=None, help='size of the chessboard, both width and height')
    options, args = parser.parse_args()
    options.width = options.width or options.size or 10
    options.height = options.height or options.size or 7
    return options, args


if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    options, args = parse()
    img = np.zeros((options.height, options.width), dtype=np.uint8)
    img[::2, ::2] = 255
    img[1::2, 1::2] = 255
    img = cv2.resize(img, (0, 0), None, 100, 100, cv2.INTER_NEAREST)
    cv2.imwrite("cb.jpg", img)
    cv2.imshow("img", img)
    cv2.waitKey(0)
