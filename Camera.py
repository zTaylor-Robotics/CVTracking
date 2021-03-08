from __future__ import print_function
import datetime
import os.path
from os import path
from threading import Thread
import numpy as np
import cv2 as cv
import imutils
import os

#This is the ferrari class
#When using this entandum with other programs, use obj.read() to get the next frame, then use obj.stop() to end the stream.
class camObjThreaded:
    W = 5000
    H = 5000
    calibPathS = "calibCache/cam"
    calibPathE = ["_mtx.txt", "_dist.txt", "_rvecs.txt", "_tvecs.txt"]
    
    #Mk2 def __init__()
    def __init__(self, src, width = 0, height = 0):
        self.camNum = src
        self.stream = cv.VideoCapture(src, cv.CAP_DSHOW)
        self.aRat = self.getStreamAspectRatio()
        self.initStreamDimensions(width, height)
        self.stream.set(cv.CAP_PROP_AUTOFOCUS, 0)
        self.stopped = False
        self.calbool = False

        #Prompt User to identify camera and initializes self.frame for early self.read acquisitions
        (_, frame) = self.stream.read()
        self.frame = imutils.resize(frame, width = 640, height = int(width / self.aRat))
        cv.imshow("ID", self.frame)
        cv.waitKey(4000) #waits 4 seconds before closing, or any key press will exit the window. Purpose: display camera image to identify camera.
        cv.destroyAllWindows()
        self.ID = input("Please input the camera identity:")


        self.start() #Start threaded stream
        #Tests to see if cameras are calibrated (i.e. a folder in calibCache for the specified camera ID with the proper mtx and dist .txt files)
        testPath = self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height)
        if path.exists(testPath+self.calibPathE[0]):
            if path.exists(testPath+self.calibPathE[1]):
                self.mtx = np.loadtxt(
                    self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[0])
                self.dist = np.loadtxt(
                    self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[1])
                self.calbool = True
            else:
                print("Please Calibrate Cameras to Continue")
                self.calibrate()
        else:
            print("Please Calibrate Cameras to Continue")
            self.calibrate()
        self.calFrame = self.frame #initializes the variable for early calls of self.read

    def getStreamAspectRatio(self):
        self.stream.set(cv.CAP_PROP_FRAME_WIDTH, self.W)
        self.stream.set(cv.CAP_PROP_FRAME_HEIGHT, self.H)
        w = self.stream.get(cv.CAP_PROP_FRAME_WIDTH)
        h = self.stream.get(cv.CAP_PROP_FRAME_HEIGHT)
        return w/h

    #Can be later user defined if ever needed
    def initStreamDimensions(self, width, height):
        if width == 0 or height == 0:
            if width == 0:
                self.height = height
                self.width = int(height*self.aRat)
                self.stream.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
                self.stream.set(cv.CAP_PROP_FRAME_WIDTH, self.width)

            if height == 0:
                self.width = width
                self.height = int(width / self.aRat)
                self.stream.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
                self.stream.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)

        else:
            self.height = height
            self.width = width
            self.stream.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
            self.stream.set(cv.CAP_PROP_FRAME_WIDTH, self.width)

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        fps = FPS().start() #records the fps of the camera stream
        while True:
            if self.stopped:
                fps.stop() #stops the fps recording of the camera stream
                print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
                print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
                return

            (self.grabbed, self.frame) = self.stream.read()
            if self.calbool == True:
                tempFrame = self.frame
                #tempFrame = imutils.resize(tempFrame, width = 1280)
                h, w = tempFrame.shape[:2]
                newcameramtx, roi = cv.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
                tempFrame = cv.undistort(tempFrame, self.mtx, self.dist, None, newcameramtx)
                x, y, w, h = roi
                tempFrame = tempFrame[y:y + h, x:x + w]
                #self.calFrame = cv.rotate(tempFrame, cv.ROTATE_90_CLOCKWISE)
                self.calFrame = tempFrame
            else:
                self.frame = imutils.resize(self.frame, width = 1280)
                #self.calFrame = cv.rotate(self.frame, cv.ROTATE_90_CLOCKWISE)
                self.calFrame = self.frame
            fps.update()

    def read(self):
        return self.calFrame

    def stop(self):
        self.stopped = True
        self.stream.release()

    def calibrate(self):
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        input('Press [ENTER] to start calibration')

        print('Calibration will take 20 images: ')
        print('Take checkerboard, position it, and then press [ENTER] to add image to list:')

        objp = np.zeros((6 * 7, 3), np.float32)
        # objp = np.zeros((3 * 5, 3), np.float32)
        objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)
        # objp[:, :2] = np.mgrid[0:5, 0:3].T.reshape(-1, 2)

        objpoints = []
        imgpoints = []
        count = 0
        i = 0
        while i < 20:
            while True:
                frame = self.read()
                frameS = imutils.resize(frame, width=640)
                cv.imshow("Calibration", frameS)

                if cv.waitKey(1) == 13:
                    break

            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

            ret, corners = cv.findChessboardCorners(gray, (7, 6), None)
            # ret, corners = cv.findChessboardCorners(gray, (5, 3), None)

            if ret == True:
                objpoints.append(objp)
                corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners)
                cv.drawChessboardCorners(frame, (7, 6), corners, ret)
                # cv.drawChessboardCorners(frame, (5, 3), corners, ret)
                frame = imutils.resize(frame, width=640)
                cv.imshow("Good Image", frame)
                cv.imwrite("calibCache/cam" + self.ID + "/goodImg/img" + str(count) + ".png", frame)
                cv.waitKey(500)
            else:
                print("Image failed, please try again")
                frame = imutils.resize(frame, width=640)
                cv.imwrite("calibCache/cam" + self.ID + "/badImg/img" + str(count) + ".png", frame)
                i -= 1
            i += 1
            count += 1
        cv.destroyAllWindows()

        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        np.savetxt(self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[0], mtx)
        np.savetxt(self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[1], dist)
        # np.savetxt(self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[2], rvecs)
        # np.savetxt(self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[3], tvecs)
        self.mtx = mtx
        self.dist = dist
        self.rvecs = rvecs
        self.tvecs = tvecs
        print('Camera is Calibrated')

    def showCalibFeed(self):
        fps = FPS().start()
        print("Press [ENTER] to stop feed.")
        while True:
            cFrame = self.read()
            cv.imshow(self.ID + "Calib Feed", imutils.resize(cFrame, width=640))
            if cv.waitKey(1) == 13:
                break
            fps.update()
        fps.stop()
        print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
        print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
        cv.destroyAllWindows()
        self.calbool = True

#This class is used to track fps in realtime
#Only implemented in the calibrated feeds copy and paste in other applications if needed elsewhere
class FPS:
    def __init__(self):
        self._start = None
        self._end = None
        self._numFrames = 0

    def start(self):
        self._start = datetime.datetime.now()
        return self

    def stop(self):
        self._end = datetime.datetime.now()

    def update(self):
        self._numFrames += 1

    def elapsed(self):
        return (self._end - self._start).total_seconds()

    def fps(self):
        return self._numFrames / self.elapsed()
