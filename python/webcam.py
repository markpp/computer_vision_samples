# USAGE
# python fps_demo.py
# python fps_demo.py --display 1

# import the necessary packages
from __future__ import print_function
from imutils.video import WebcamVideoStream
import argparse
import imutils
import cv2
import numpy as np

def overlay_image_alpha(img, img_overlay, pos, alpha=0.5):
    """Overlay img_overlay on top of img at the position specified by
    pos and blend using alpha_mask.

    Alpha mask must contain values within the range [0, 1] and be the
    same size as img_overlay.
    """

    x, y = pos

    # Image ranges
    y1, y2 = max(0, y), min(img.shape[0], y + img_overlay.shape[0])
    x1, x2 = max(0, x), min(img.shape[1], x + img_overlay.shape[1])

    # Overlay ranges
    y1o, y2o = max(0, -y), min(img_overlay.shape[0], img.shape[0] - y)
    x1o, x2o = max(0, -x), min(img_overlay.shape[1], img.shape[1] - x)

    # Exit if nothing to do
    if y1 >= y2 or x1 >= x2 or y1o >= y2o or x1o >= x2o:
        return

    channels = img.shape[2]

    #alpha = alpha_mask[y1o:y2o, x1o:x2o]
    alpha_inv = 1.0 - alpha

    for c in range(channels):
        img[y1:y2, x1:x2, c] = (alpha * img_overlay[y1o:y2o, x1o:x2o, c] +
                                alpha_inv * img[y1:y2, x1:x2, c])

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-d", "--display", type=int, default=-1,
    help="Whether or not frames should be displayed")
args = vars(ap.parse_args())

# grab a pointer to the video stream and initialize the FPS counter
print("[INFO] sampling frames from webcam...")
stream = cv2.VideoCapture(0)

# loop over some frames
while 1:
    # grab the frame from the stream and resize it to have a maximum
    # width of 400 pixels
    (grabbed, frame) = stream.read()
    frame = imutils.resize(frame, width=600)

    h = np.zeros((128,256,3))
    bins = np.arange(256).reshape(256,1)
    color = [ (255,0,0),(0,255,0),(0,0,255) ]

    for ch, col in enumerate(color):
        hist_item = cv2.calcHist([frame],[ch],None,[256],[0,255])
        cv2.normalize(hist_item,hist_item,0,h.shape[0],cv2.NORM_MINMAX)
        hist=np.int32(np.around(hist_item))
        pts = np.column_stack((bins,hist))
        cv2.polylines(h,[pts],False,col)

    h=np.flipud(h)

    output = frame.copy()

    h_img = np.uint8(h)

    h_ratio = 0.5
    h_img = cv2.resize(h_img,None,fx=h_ratio, fy=h_ratio, interpolation = cv2.INTER_LINEAR)

    overlay_image_alpha(output, h_img[:, :, 0:3], (frame.shape[1]-h_img.shape[1]-10, 10), 0.7)

    cv2.imshow('frame', output)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

# do a bit of cleanup
stream.release()
cv2.destroyAllWindows()
