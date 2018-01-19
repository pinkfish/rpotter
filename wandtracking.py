#!/bin/python
mport io
import sys
sys.path.insert(1, '/usr/lib/python2.7/dist-packages/picamera')
import picamera
import numpy as np
import cv2
import threading
import math
import time

class WandTracking:
    def __init__(self):
        self.resetFrame = True
        self.wandMask = None
        self.baseCircles = None
        self.lk_params = dict( winSize  = (15,15),
                        maxLevel = 2,
                        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self.dilation_params = (5, 5)
        self.movment_threshold = 80

   def ResetFrame(self):
       self.resetFrame = True

   def StartResetFrameTimer(self):
       threading.Timer(3, ResetFrame)


   # Finds the wand and puts the resulting base frame into the
   def FindWand(self, frame, circlesInImage):
       self.wandMask = np.zeros_like(frame)
       print circlesInImage
       self.baseCircles = circlesInImage

   def TrackWand(self, frame, circlesInImage):
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.baseFrame, frame, self.baseCircles, None, **self.lk_params)


   def Run(self):
        # initialize the camera and grab a reference to the raw camera capture
        camera = PiCamera()
        camera.resolution = (640, 480)
        camera.framerate = 32
        rawCapture = PiRGBArray(camera, size=(640, 480))

        # allow the camera to warmup
        time.sleep(0.1)

        # capture frames from the camera
        str = camera.capture_continuous(rawCapture, format="bgr", use_video_port=True)
        while True:
                frame = str.next()
                # grab the raw NumPy array representing the image, then initialize the timestamp
                # and occupied/unoccupied text
                image = frame.array

                # Process the frame, flip it
                cv2.flip(image, 1, image)
                # Convert to greyscale and normalize it
                grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                equalizeHist(grey)
                grey = cv2.GaussianBlur(grey, (9, 9), 1.5)
                clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
                grey = clahe.apply(grey)

                # Find a circle in the frame.
                    # Only process if we find a circle...
                if self.resetFrame:
                    self.resetFrame = False
                    circlesInImage = cv2.HoughCircles(grey, cv2.HOUGH_GRADIENT, 3, 100, param1=100, param2=30, minRadius=4, maxRadius=15)
                    # Only setup stuff if we find a circle.
                    if circlesInImage != None:
                        FindWand(grey, circlesInImage)
                        StartResetFrameTimer()
                    else:
                        print "No circles"
                else:
                    TrackWand(grey)

                # show the frame
                cv2.imshow("Frame", image)
                key = cv2.waitKey(1) & 0xFF

                # clear the stream in preparation for the next frame
                rawCapture.truncate(0)

                # if the `q` key was pressed, break from the loop
                if key == ord("q"):
                        break
