import numpy as np
import cv2
from matplotlib import pyplot as plt

img = cv2.imread('/Users/markpp/Desktop/code/data/DanpoData/CENTER_A/MAPPED_COLOR/MAPPED_COLOR_0_C_A.png')              # img.shape : (413, 620, 3)

#img = cv2.imread('/Users/markpp/Desktop/messi.jpeg')
img_grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#cv2.imshow("Out", img)
#cv2.waitKey()

f = np.fft.fft2(img_grey)
fshift = np.fft.fftshift(f)
magnitude_spectrum = 20*np.log(np.abs(fshift))

plt.subplot(121),plt.imshow(img_grey, cmap = 'gray')
plt.title('Input Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(magnitude_spectrum, cmap = 'gray')
plt.title('Magnitude Spectrum'), plt.xticks([]), plt.yticks([])

plt.show()

rows, cols = img_grey.shape
crow,ccol = rows/2 , cols/2
fshift[crow-30:crow+30, ccol-30:ccol+30] = 0
f_ishift = np.fft.ifftshift(fshift)
img_back = np.fft.ifft2(f_ishift)
img_back = np.abs(img_back)

plt.subplot(131),plt.imshow(img_grey, cmap = 'gray')
plt.title('Input Image'), plt.xticks([]), plt.yticks([])
plt.subplot(132),plt.imshow(img_back, cmap = 'gray')
plt.title('Image after HPF'), plt.xticks([]), plt.yticks([])
plt.subplot(133),plt.imshow(img_back)
plt.title('Result in JET'), plt.xticks([]), plt.yticks([])

plt.show()