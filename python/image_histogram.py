#!/usr/bin/env python
import cv2
import numpy as np
from skimage.filters import gabor
from matplotlib import pyplot as plt
import argparse

def calc_3_channel_hist(img):
    h = np.zeros((300,256,3))

    bins = np.arange(256).reshape(256,1)
    color = [ (255,0,0),(0,255,0),(0,0,255) ]

    for ch, col in enumerate(color):
        hist_item = cv2.calcHist([img],[ch],None,[256],[0,255])
        cv2.normalize(hist_item,hist_item,0,255,cv2.NORM_MINMAX)
        hist=np.int32(np.around(hist_item))
        pts = np.column_stack((bins,hist))
        cv2.polylines(h,[pts],False,col)

    h=np.flipud(h)

    return h

def calc_1_channel_hist(gray):
    h = np.zeros((300,256,1))
    bins = np.arange(256).reshape(256,1)

    hist_item = cv2.calcHist([gray], [0], None, [256], [0, 256])
    cv2.normalize(hist_item,hist_item,0,255,cv2.NORM_MINMAX)
    hist=np.int32(np.around(hist_item))
    pts = np.column_stack((bins,hist))
    cv2.polylines(h,[pts],False,1)

    h=np.flipud(h)

    return h

def show_3_channel_hist(h):
    cv2.imshow('colorhist',h)


def plot_hist(img_gray):
    plt.hist(img_gray.ravel(),256,[0,256]); plt.savefig('foo.png', bbox_inches='tight')

if __name__ == "__main__":
     # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--path", type=str,
                    help="Path to frames") 
    args = vars(ap.parse_args())
    #img_path = '/Users/markpp/Desktop/code/data/DanpoData/CENTER_A/MAPPED_COLOR/MAPPED_COLOR_0_C_A.png'
    img_bgr = cv2.imread(args["path"], 1)#[300:(1500), 500:1200]
    #img_path = '/Users/markpp/Desktop/code/data/DanpoData/CENTER_A/MAPPED_COLOR/MAPPED_COLOR_0_C_A.png'

    #img_bgr = cv2.resize(img_bgr, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
    #img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    img_lab  = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2Lab)
    #img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)

    # Create mask to disable unwanted regions before feature extraction
    img_gray = img_lab[:, :, 0]


    cv2.imshow("3", calc_3_channel_hist(img_bgr))
    cv2.imshow("1", calc_1_channel_hist(img_gray))

    #cv2.imshow("filt_imag", watermarked)
    cv2.waitKey()


    cv2.destroyAllWindows()
