#!/usr/bin/env python
import cv2
import numpy as np
import argparse

auto_continue = False

if __name__ == "__main__":
     # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--path", type=str,
                    help="Path to frames")
    args = vars(ap.parse_args())

    cap = cv2.VideoCapture(args["path"])

    frame_nr = 0
    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret:
            #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            img_show = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
            # clockwise rotation
            img_show = cv2.transpose(img_show)
            img_show = cv2.flip(img_show,flipCode=1)

            cv2.putText(img_show,"frame_{}".format(str(frame_nr)), (20,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
            cv2.imshow('frame', img_show)

            if auto_continue:
                key = cv2.waitKey(10) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('c'):
                    auto_continue = not auto_continue
            else:
                key = cv2.waitKey() & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('c'):
                    auto_continue = not auto_continue
                elif key == ord('x'):
                    cv2.imwrite("out/frame_{}.png".format(str(frame_nr)), frame)

            frame_nr = frame_nr + 1
        else:
            print("no frame")
            break
    cap.release()
    cv2.destroyAllWindows()
