#!/usr/bin/env python
import cv2
import numpy as np
from skimage.filters import gabor
import argparse


def setting_frequency(x):
    print("Frequency set to: {}".format(cv2.getTrackbarPos('frequency', 'image')/20.0))
    #pass


def setting_theta(x):
    print("Theta set to: {}".format(cv2.getTrackbarPos('theta', 'image')/10.0))

# http://scikit-image.org/docs/dev/api/skimage.filters.html#skimage.filters.gabor



if __name__ == "__main__":
     # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--path", type=str,
                    help="Path to frames")
    args = vars(ap.parse_args())
    img_bgr = cv2.imread(args["path"], 1)
    img_bgr = cv2.resize(img_bgr, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
    img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

    # Create a window
    cv2.namedWindow('image')
    cv2.moveWindow('image', 200, 200)

    # create trackbars for color change
    cv2.createTrackbar('frequency', 'image', 6, 8, setting_frequency)
    cv2.createTrackbar('theta', 'image', 3, 40, setting_theta)

    # get current positions of four trackbars
    freq = cv2.getTrackbarPos('frequency', 'image')
    theta = cv2.getTrackbarPos('theta', 'image')
    old_freq = freq
    old_theta = theta

    filt_real, filt_imag = gabor(img_gray, frequency=freq/20.0, theta=theta/10.0)
    while (1):
        # get current positions of four trackbars
        freq = cv2.getTrackbarPos('frequency', 'image')
        if freq < 1: freq = 1
        theta = cv2.getTrackbarPos('theta', 'image')

        if theta != old_theta or freq != old_freq:
            filt_real, filt_imag = gabor(img_gray, frequency=freq/20.0, theta=theta/10.0)
            old_theta = theta
            old_freq = freq

        #
        dst = cv2.addWeighted(img_bgr,0.5,cv2.cvtColor(filt_real, cv2.COLOR_GRAY2BGR),0.5,0)
        dual_vis = np.concatenate((img_bgr, dst), axis=1)
        cv2.imshow("image", dual_vis)
        #cv2.imshow("filt_imag", watermarked)
        k = cv2.waitKey(100) & 0xFF
        #print k
        if k == 27:
            break

    cv2.destroyAllWindows()
