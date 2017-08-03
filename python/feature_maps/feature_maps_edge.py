#!/usr/bin/env python
import cv2
import numpy as np
import argparse


def setting_frequency(x):
    print("Frequency set to: {}".format(cv2.getTrackbarPos('frequency', 'image')/20.0))
    #pass


def setting_sigma(x):
    print("Sigma set to: {}".format(cv2.getTrackbarPos('sigma', 'image')))

# http://scikit-image.org/docs/dev/api/skimage.filters.html#skimage.filters.gabor



if __name__ == "__main__":
     # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--path", type=str,
                    help="Path to frames")
    args = vars(ap.parse_args())
    img_bgr = cv2.imread(args["path"], 1)
    #img_bgr = cv2.resize(img_bgr, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
    img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

    sigma_values = [3,5,7,9]
    # Create a window
    cv2.namedWindow('image')
    cv2.moveWindow('image', 200, 200)

    # create trackbars for color change
    cv2.createTrackbar('frequency', 'image', 6, 8, setting_frequency)
    cv2.createTrackbar('sigma', 'image', 0, 3, setting_sigma)

    # get current positions of four trackbars
    freq = cv2.getTrackbarPos('frequency', 'image')
    sigma = cv2.getTrackbarPos('sigma', 'image')
    old_freq = freq
    old_sigma = sigma

    edge_img = cv2.Canny(img_gray,100,200,apertureSize=sigma_values[sigma])
    while (1):
        # get current positions of four trackbars
        freq = cv2.getTrackbarPos('frequency', 'image')
        if freq < 1: freq = 1
        sigma = cv2.getTrackbarPos('sigma', 'image')

        if sigma != old_sigma or freq != old_freq:
            edge_img = cv2.Canny(img_gray,100,200,apertureSize=sigma_values[sigma])
            old_sigma = sigma
            old_freq = freq

        #
        dual_vis = np.concatenate((img_gray, edge_img), axis=1)
        cv2.imshow("image", dual_vis)
        #cv2.imshow("filt_imag", watermarked)
        k = cv2.waitKey(100) & 0xFF
        #print k
        if k == 27:
            break

    cv2.destroyAllWindows()
