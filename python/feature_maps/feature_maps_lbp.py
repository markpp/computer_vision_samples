#!/usr/bin/env python
import cv2
import numpy as np
from skimage.feature import local_binary_pattern
from skimage.color import label2rgb
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
import argparse

methods = ['default', 'ror', 'uniform', 'var']


def setting_radius(x):
    print("Radius set to: {}".format(cv2.getTrackbarPos('radius', 'image')))


def setting_n_points(x):
    print("n_points set to: {}".format(cv2.getTrackbarPos('n_points', 'image')+8))


def setting_method_id(x):
    print("method set to: {}".format(methods[cv2.getTrackbarPos('method_id', 'image')]))

# http://scikit-image.org/docs/dev/auto_examples/features_detection/plot_local_binary_pattern.html

if __name__ == "__main__":
     # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--path", type=str,
                    help="Path to frames")
    args = vars(ap.parse_args())
    img_bgr = cv2.imread(args["path"], 1)
    img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    img_gray = cv2.GaussianBlur(img_gray, (3, 3), 1)

    # Create a window
    cv2.namedWindow('image')
    cv2.moveWindow('image', 200, 200)

    # create trackbars for color change
    cv2.createTrackbar('radius', 'image', 3, 10, setting_radius)
    cv2.createTrackbar('n_points', 'image', 0, 24, setting_n_points)
    cv2.createTrackbar('method_id', 'image', 0, 3, setting_method_id)

    first = True
    # get current positions of four trackbars
    radius = cv2.getTrackbarPos('radius', 'image')
    n_points = cv2.getTrackbarPos('n_points', 'image')
    method_id = cv2.getTrackbarPos('method_id', 'image')
    old_radius = radius
    old_n_points = n_points
    old_method_id = method_id

    img_lbp = img_gray.copy()
    #lbp = local_binary_pattern(img_lbp.copy(), n_points, radius, 'default')

    while (1):
        # get current positions of four trackbars
        radius = cv2.getTrackbarPos('radius', 'image')
        if radius < 1: radius = 1
        n_points = cv2.getTrackbarPos('n_points', 'image')
        #if n_points < 1: n_points = 1
        method_id = cv2.getTrackbarPos('method_id', 'image')


        if first or radius!=old_radius or n_points!=old_n_points or method_id!=old_method_id:
            lbp = local_binary_pattern(img_lbp.copy(), n_points+8, radius, methods[method_id])
            first = False
            old_n_points = n_points
            old_radius = radius
            old_method_id = method_id
            X = lbp.ravel().astype('uint8').reshape(-1, 1)

            kmeans = KMeans(n_clusters=4, random_state=0).fit(X)
            img_clusters = kmeans.labels_.reshape(lbp.shape)*50
            #dual_vis = np.concatenate((lbp.astype('uint8'), img_gray), axis=1)
            #cv2.imshow("image", dual_vis)
            tri_vis = np.concatenate((lbp.astype('uint8'), img_clusters.astype('uint8'), img_gray), axis=1)
            cv2.imshow("image", tri_vis)


            plt.scatter(X, X, c=kmeans.labels_, alpha=0.5)
            plt.draw()
            plt.pause(0.0001)
        k = cv2.waitKey(100) & 0xFF
        if k == 27:
            break
        if k == 32:
            #lbp_plot(img_bgr, lbp)
            pass

    cv2.destroyAllWindows()
