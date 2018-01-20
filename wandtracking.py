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
from sets import Set

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
        self.maxObjects = 15
        self.spells = {}
        self.trackingColor = (0,0,255)
        self.movementThreshold = 80

    def ResetFrame(self):
        print 'ResetyFrame'
        self.resetFrame = True

    def StartResetFrameTimer(self):
        print 'StartResetTimer'
        threading.Timer(3, self.ResetFrame).start()


    def JustCenters(self, circles):
        circles.shape = (circles.shape[1], 1, circles.shape[2])
        return circles[:,:,0:2]

    def UpdateGuesture(self, startX, startY, endX, endY, objectId):
        if (startX < endX - 5) and (abs(startY - endY) < 1):
            self.movements[objectId].add("left")
        if endX < startX - 5 and abs(startY - endY) < 1:
            self.movements[objectId].add("right")
        if startY < endY - 5 and abs(startX - endX) < 1:
            self.movements[objectId].add("up")
        if endY < startY - 5 and abs(startX - endX) < 1:
            self.movements[objectId].add("down")

    def CheckSpell(self, objectId):
        guesture = ''.join(map(str, self.movements[objectId]))
        if guesture in self.spells:
            self.spells[guesture](guesture)

    # Finds the wand and puts the resulting base frame into the
    def FindWand(self, frame, circlesInImage):
        print 'Refressh base'
        self.wandMask = np.zeros_like(frame)
        self.baseCircles = self.JustCenters(circlesInImage)
        self.baseFrame = frame
        self.movements = [Set() for x in range(self.maxObjects)]

    def TrackWand(self, frame):
        movingCircles, st, err = cv2.calcOpticalFlowPyrLK(
            self.baseFrame, frame, self.baseCircles, None, **self.lk_params)
        good_new = self.baseCircles[st == 1]
        good_old = movingCircles[st == 1]

        for objectId, (new, old) in enumerate(zip(good_new, good_old)):
            startX, startY = new.ravel()
            endX, endY = old.ravel()
            # Only track up to 15 circles.
            if objectId < self.maxObjects:
                self.CheckSpell(objectId)
                dist = math.hypot(startX - endX, startY - endY)
                if dist > self.movementThreshold:
                    self.UpdateGuesture(startX, startY, endX, endY, objectId)
                    print '%s: (%s, %s) -> (%s, %s)' % (objectId, startX, startY, endX, endY)
                    cv2.line(self.wandMask, (startX, startY), (endX, endY), self.trackingColor, 2)
                    cv2.circle(frame, (startX, startY), 5, self.trackingColor, -1)
                    cv2.putText(frame, str(objectId), (startX, startY), cv2.FONT_HERSHEY_SIMPLEX, 1.0, self.trackingColor)
                    # Only look for a new base frame from when movement starts
                    self.StartResetFrameTimer()
                else:
                    cv2.circle(frame, (startX, startY), 5, self.trackingColor, -1)
                    cv2.putText(frame, str(objectId), (startX, startY), cv2.FONT_HERSHEY_SIMPLEX, 1.0, self.trackingColor)


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
                if circlesInImage is not None:
                    self.resetFrame = False
                    self.FindWand(grey, circlesInImage)
                else:
                    print "No circles"
            else:
                self.TrackWand(grey)

            # show the frame
            cv2.imshow("Frame", grey)
            key = cv2.waitKey(1) & 0xFF

            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)

            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break

cv2.namedWindow("Frame")
wand = WandTracking()
wand.Run()
