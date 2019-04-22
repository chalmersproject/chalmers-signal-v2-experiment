"""
Just trying to capture images from my computer's camera
"""

"""
Script Source: https://github.com/jagracar/OpenCV-python-tests/blob/master/OpenCV-tutorials/videoAnalysis/opticalFlow.py

Simply display the contents of the webcam with optional mirroring using OpenCV
via the new Pythonic cv2 interface.  Press <esc> to quit.
"""

import cv2
import imutils
import numpy as np
from imutils.object_detection import non_max_suppression
from imutils import paths

# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())


def show_webcam(mirror=False):
    cam = cv2.VideoCapture(0)
    while True:
        # grab a frame from the camera
        ret_val, img = cam.read()
        # convert to greyscale and change size to reduce detection time
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = imutils.resize(img, width=min(400, img.shape[1]))
        # detect people in the image
        (rects, weights) = hog.detectMultiScale(img, winStride=(4,4), \
                                                    padding=(8,8), \
                                                    scale=1.05 )
        # draw the original bounding boxes
        for (x,y,w,h) in rects:
            cv2.rectangle(img, (x,y), (x + w, y + h), (0,255,255), 2)
        # apply non-maxima non_max_suppression
        rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
        # draw the final bounding boxes
        for (xA, yA, xB, yB) in pick:
            cv2.rectangle(img, (xA, yA), (xB, yB), (0, 255, 0), 2)

        if mirror:
            img = cv2.flip(img, 1)

        cv2.imshow('my webcam', img)
        if cv2.waitKey(1) == 27:
            break  # esc to quit
    cv2.destroyAllWindows()


def main():
    show_webcam(mirror=True)


if __name__ == '__main__':
    main()
