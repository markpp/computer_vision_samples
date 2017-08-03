import matplotlib.pyplot as plt
import numpy as np
import cv2
from skimage.feature import greycomatrix, greycoprops
import argparse

if __name__ == "__main__":
     # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--path", type=str,
                    help="Path to frames")
    args = vars(ap.parse_args())
    img_bgr = cv2.imread(args["path"], 1)
    img_grey = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    sarraster = np.array(img_grey)
    #sarraster is satellite image, testraster will receive texture
    testraster = np.copy(sarraster)
    testraster[:] = 0

    for i in range(testraster.shape[0] ):

        for j in range(testraster.shape[1] ):

            #windows needs to fit completely in image
            if i <3 or j <3:
                continue
            if i > (testraster.shape[0] - 4) or j > (testraster.shape[0] - 4):
                continue

            #Calculate GLCM on a 7x7 window
            glcm_window = sarraster[i-3: i+4, j-3 : j+4]
            glcm = greycomatrix(glcm_window, [1], [0],  symmetric = True, normed = True )

            #Calculate contrast and replace center pixel
            contrast = greycoprops(glcm, 'contrast')
            testraster[i,j]= contrast

    sarplot = plt.imshow(testraster, cmap = 'gray')
    plt.show()
