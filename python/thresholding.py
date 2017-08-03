#!/usr/bin/env python
import cv2
import numpy as np
from skimage.filters import gabor
from matplotlib import pyplot as plt
import argparse

first = True

def setting_kernel(x):
    print("Kernel size set to: {}".format(cv2.getTrackbarPos('kernel', 'image')))


def setting_thresh(x):
    print("Threshold set to: {}".format(cv2.getTrackbarPos('threshold', 'image')))


if __name__ == "__main__":
     # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--path", type=str,
                    help="Path to frames")
    args = vars(ap.parse_args())
    img_bgr = cv2.imread(args["path"], 1)

    img_lab  = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2Lab)
    #img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)

    # Create mask to disable unwanted regions before feature extraction
    img_gray = img_lab[:, :, 0]
    # Create a window
    cv2.namedWindow('image')
    cv2.moveWindow('image', 200, 200)

    # create trackbars for color change
    cv2.createTrackbar('kernel', 'image', 7, 25, setting_kernel)
    cv2.createTrackbar('threshold', 'image', 200, 255, setting_thresh)

    # get current positions of four trackbars
    current_kernel = cv2.getTrackbarPos('kernel', 'image')
    old_kernel = current_kernel
    my_thresh = cv2.getTrackbarPos('threshold', 'image')
    old_thresh = my_thresh

    #T, thresh_img = cv2.threshold(img_gray, my_thresh, 255, cv2.THRESH_BINARY_INV)
    while(1):


        # get current positions of four trackbars
        current_kernel = cv2.getTrackbarPos('kernel', 'image')
        my_thresh = cv2.getTrackbarPos('threshold', 'image')
        if first or my_thresh != old_thresh or current_kernel != old_kernel:

            kernel = np.ones((current_kernel, current_kernel),np.uint8)
            dilation = cv2.dilate(img_gray,kernel,iterations = 1)
            T, thresh_img = cv2.threshold(dilation, my_thresh, 255, cv2.THRESH_BINARY_INV)
            old_thresh = my_thresh

        #dst = cv2.addWeighted(img_bgr,0.5,cv2.cvtColor(filt_real, cv2.COLOR_GRAY2BGR),0.5,0)
        #dual_vis = np.concatenate((img_bgr, cv2.cvtColor(thresh_img, cv2.COLOR_GRAY2BGR)), axis=1)
        #res = cv2.bitwise_and(img_bgr,cv2.cvtColor(thresh_img, cv2.COLOR_GRAY2BGR))

        dual_vis = np.concatenate((img_gray, thresh_img), axis=1)

        cv2.imshow("image", dual_vis)
        first = False


        #cv2.imshow("filt_imag", watermarked)
        k = cv2.waitKey(100) & 0xFF
        #print k
        if k == 27:
            break

    cv2.destroyAllWindows()
