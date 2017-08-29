#!/usr/bin/env python
import cv2
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.widgets import Slider

import argparse

kernel_size = 5

def show_kernels():
  # simple averaging filter without scaling parameter
  mean_filter = np.ones((3,3))

  # creating a guassian filter
  x = cv2.getGaussianKernel(5,10)
  gaussian = x*x.T

  # different edge detecting filters
  # scharr in x-direction
  scharr = np.array([[-3, 0, 3],
                     [-10,0,10],
                     [-3, 0, 3]])
  # sobel in x direction
  sobel_x= np.array([[-1, 0, 1],
                     [-2, 0, 2],
                     [-1, 0, 1]])
  # sobel in y direction
  sobel_y= np.array([[-1,-2,-1],
                     [0, 0, 0],
                     [1, 2, 1]])
  # laplacian
  laplacian=np.array([[0, 1, 0],
                      [1,-4, 1],
                      [0, 1, 0]])

  filters = [mean_filter, gaussian, laplacian, sobel_x, sobel_y, scharr]
  filter_name = ['mean_filter', 'gaussian','laplacian', 'sobel_x', \
                  'sobel_y', 'scharr_x']
  fft_filters = [np.fft.fft2(x) for x in filters]
  fft_shift = [np.fft.fftshift(y) for y in fft_filters]
  mag_spectrum = [np.log(np.abs(z)+1) for z in fft_shift]

  for i in xrange(6):
      plt.subplot(2,3,i+1),plt.imshow(mag_spectrum[i],cmap = 'gray')
      plt.title(filter_name[i]), plt.xticks([]), plt.yticks([])

  plt.show()


if __name__ == "__main__":
   # construct the argument parser and parse the arguments
  ap = argparse.ArgumentParser()
  ap.add_argument("-p", "--path", type=str,
                  help="Path to frames")
  args = vars(ap.parse_args())

  # loading image
  #img0 = cv2.imread('SanFrancisco.jpg',)
  img0 = cv2.imread(args["path"])

  # converting to gray scale
  gray = cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY)

  # remove noise
  img = cv2.GaussianBlur(gray,(3,3),0)

  # convolute with proper kernels
  laplacian = cv2.Laplacian(img,cv2.CV_64F)
  sobelx = cv2.Sobel(img,cv2.CV_64F,1,0,ksize=kernel_size)  # x
  sobely = cv2.Sobel(img,cv2.CV_64F,0,1,ksize=kernel_size)  # y

  plt.subplot(2,2,1),plt.imshow(img,cmap = 'gray')
  plt.title('Original'), plt.xticks([]), plt.yticks([])
  plt.subplot(2,2,2),plt.imshow(laplacian,cmap = 'gray')
  plt.title('Laplacian'), plt.xticks([]), plt.yticks([])
  plt.subplot(2,2,3),plt.imshow(sobelx,cmap = 'gray')
  plt.title('Sobel X'), plt.xticks([]), plt.yticks([])
  plt.subplot(2,2,4),plt.imshow(sobely,cmap = 'gray')
  plt.title('Sobel Y'), plt.xticks([]), plt.yticks([])

  plt.show()
