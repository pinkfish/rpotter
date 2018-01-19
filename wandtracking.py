#!/bin/python
import io
import sys
sys.path.insert(1, '/usr/lib/python2.7/dist-packages/picamera')
import picamera
import numpy as np
import cv2
import threading
import math
import time
from picamera.array import PiRGBArray
from picamera import PiCamera


class WandTracking:
    def __init__(self):
        self.resetFrame = True
        self.wandMask = None
        self.baseCircles = None
        self.baseFrame = None
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10,
                      0.03))
        self.dilation_params = (5, 5)
        self.movment_threshold = 80

    def ResetFrame(self):
        self.resetFrame = True

    def StartResetFrameTimer(self):
        threading.Timer(3, self.ResetFrame)


    def JustCenters(self, circles):
        circles.shape = (circles.shape[1], 1, circles.shape[2])
        return circles[:,:,0:2]


    # Finds the wand and puts the resulting base frame into the
    def FindWand(self, frame, circlesInImage):
        self.wandMask = np.zeros_like(frame)
        self.baseCircles = self.JustCenters(circlesInImage)
        self.baseFame = frame

    def TrackWand(self, frame):
        print 'here <<<%s>>>' % (self.baseCircles)
        movingCircles, st, err = cv2.calcOpticalFlowPyrLK(
            self.baseFrame, frame, self.baseCircles, None, **self.lk_params)
        good_new = self.baseCircles[st == 1]
        good_old = movingCircles[st == 1]

        print 'new %s' % (good_new)
        print 'old %s' % (good_old)

    def Run(self):
        # initialize the camera and grab a reference to the raw camera capture
        camera = PiCamera()
        camera.resolution = (640, 480)
        camera.framerate = 32
        rawCapture = PiRGBArray(camera, size=(640, 480))

        # allow the camera to warmup
        time.sleep(0.1)

        # capture frames from the camera
        str = camera.capture_continuous(
            rawCapture, format="bgr", use_video_port=True)
        while True:
            frame = str.next()
            # grab the raw NumPy array representing the image, then initialize the timestamp
            # and occupied/unoccupied text
            image = frame.array

            # Process the frame, flip it
            cv2.flip(image, 1, image)
            # Convert to greyscale and normalize it
            grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            cv2.equalizeHist(grey)
            grey = cv2.GaussianBlur(grey, (9, 9), 1.5)
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            grey = clahe.apply(grey)

            # Find a circle in the frame.
            # Only process if we find a circle...
            if self.resetFrame:
                self.resetFrame = False
                circlesInImage = cv2.HoughCircles(
                    grey,
                    cv2.HOUGH_GRADIENT,
                    3,
                    100,
                    param1=100,
                    param2=30,
                    minRadius=4,
                    maxRadius=15)
                # Only setup stuff if we find a circle.
                print 'Reset frame %s' % (circlesInImage)
                if circlesInImage is not None:
                    self.FindWand(grey, circlesInImage)
                    self.StartResetFrameTimer()
                else:
                    print "No circles"
            else:
                self.TrackWand(grey)

            # show the frame
            cv2.imshow("Frame", image)
            key = cv2.waitKey(1) & 0xFF

            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)

            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break

cv2.namedWindow("Frame")
wand = WandTracking()
wand.Run()
