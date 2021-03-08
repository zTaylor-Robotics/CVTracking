from __future__ import print_function
import datetime
from threading import Thread
import argparse
import numpy as np
import cv2 as cv
import imutils
import os


#This is the ferrari class
#When using this entandum with other programs, use obj.read() to get the next frame, then use obj.stop() to end the stream.
class camObjThreaded:
    W = 3000
    H = 3000
    calibPathS = "calibCache/cam"
    calibPathE = ["_mtx.txt", "_dist.txt", "_rvecs.txt", "_tvecs.txt"]

    def __init__(self, src, width):
        self.stream = cv.VideoCapture(src, cv.CAP_DSHOW)
        self.camNum = src
        self.width = width

        #Resizes the stream to a predetermined width based on inherent resolution. Also turns autofocus off, just cuz
        self.stream.set(cv.CAP_PROP_FRAME_WIDTH, self.W)
        self.stream.set(cv.CAP_PROP_FRAME_HEIGHT, self.H)
        self.rat = (self.stream.get(cv.CAP_PROP_FRAME_WIDTH) / self.stream.get(cv.CAP_PROP_FRAME_HEIGHT))
        self.stream.set(cv.CAP_PROP_FRAME_WIDTH, int(self.width))
        self.height = int(self.width / self.rat)
        self.stream.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
        self.stream.set(cv.CAP_PROP_AUTOFOCUS, 1)

        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False
        self.calbool = False
        self.start()

        cv.imshow("ID", self.frame)
        cv.waitKey()
        self.ID = input("Please input the camera identity:")
        cv.destroyAllWindows()

        test = input("Has the " + self.ID + " camera been calibrated for width and height = "
                     + str(self.width) + "x" + str(int(self.width / self.rat)) + "? [Y]/[N]: ")

        if test == 'Y' or test == 'y':
            # pass check to the next stage if the camera is already calibrated
            passToss = input("Would you like to recalibrate the camera? [Y]/[N]: ")
            if passToss == 'Y' or passToss == 'y':
                os.remove(
                    self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[0])
                os.remove(
                    self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[1])
                self.calibrate()
            else:
                self.mtx = np.loadtxt(
                    self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[0])
                self.dist = np.loadtxt(
                    self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[1])
                self.calbool = True
        else:
            self.calibrate()

        #insert live stream joke here
        #self.calFrame = cv.rotate(self.frame, cv.ROTATE_90_CLOCKWISE)
        self.calFrame = self.frame

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

        print('Calibration will 20 images: ')
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
        np.savetxt(self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[1],
                   dist)
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
        self.stop()

#Old version, do not use unless u r DESPERATELY BAD AT CODE THINGS
class camObj:
    W = 3000
    H = 3000
    calibPathS = "calibCache/cam"
    calibPathE = ["_mtx.txt", "_dist.txt", "_rvecs.txt", "_tvecs.txt"]

    #sets the object to a /dev/video# with a predefined width which obeys cameras root aspect ratio
    def __init__(self, camNum, width):
        self.camNum = camNum
        self.width = width
        self.initCam()

    def __str__(self):
        return "This is the object which operates camera: " + str(self.camNum)

    def initCam(self):
        self.cap = cv.VideoCapture(int(self.camNum), cv.CAP_DSHOW)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, self.W)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.H)
        self.rat = (self.cap.get(cv.CAP_PROP_FRAME_WIDTH) / self.cap.get(cv.CAP_PROP_FRAME_HEIGHT))
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, int(self.width))
        self.height = int(self.width/self.rat)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv.CAP_PROP_AUTOFOCUS, 0)
        ret, initFrame = self.cap.read()
        cv.imshow("ID", initFrame)
        cv.waitKey()
        self.ID = input("Please input the camera identity:")
        cv.destroyAllWindows()

        test = input("Has the " + self.ID + " camera been calibrated for width and height = "
                     + str(self.width) + "x" + str(int(self.width / self.rat)) + "? [Y]/[N]: ")

        if test == 'Y' or test == 'y':
            #pass check to the next stage if the camera is already calibrated
            passToss = input("Would you like to recalibrate the camera? [Y]/[N]: ")
            if passToss == 'Y' or passToss == 'y':
                os.remove(self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[0])
                os.remove(self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[1])
                self.calibrate()
            else:
                self.mtx = np.loadtxt(self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[0])
                self.dist = np.loadtxt(self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[1])
        else: self.calibrate()

    def calibrate(self):
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        input('Press [ENTER] to start calibration')

        print('Calibration will 20 images: ')
        print('Take checkerboard, position it, and then press [ENTER] to add image to list:')

        objp = np.zeros((6*7, 3), np.float32)
        #objp = np.zeros((3 * 5, 3), np.float32)
        objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1,2)
        #objp[:, :2] = np.mgrid[0:5, 0:3].T.reshape(-1, 2)

        objpoints = []
        imgpoints = []
        count = 0
        i = 0
        while i < 20:
            while True:
                ret, frame = self.cap.read()
                frameS = imutils.resize(frame, width = 640)
                cv.imshow("Calibration", frameS)

                if cv.waitKey(1) == 13:
                    break

            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

            ret, corners = cv.findChessboardCorners(gray, (7,6), None)
            #ret, corners = cv.findChessboardCorners(gray, (5, 3), None)

            if ret == True:
                objpoints.append(objp)
                corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1,-1), criteria)
                imgpoints.append(corners)
                cv.drawChessboardCorners(frame, (7, 6), corners, ret)
                #cv.drawChessboardCorners(frame, (5, 3), corners, ret)
                frame = imutils.resize(frame,width = 640)
                cv.imshow("Good Image", frame)
                cv.imwrite("calibCache/cam"+self.ID+"/goodImg/img"+ str(count) +".png", frame)
                cv.waitKey(500)
            else:
                print("Image failed, please try again")
                frame = imutils.resize(frame, width=640)
                cv.imwrite("calibCache/cam"+self.ID+"/badImg/img" + str(count) + ".png", frame)
                i -= 1
            i += 1
            count += 1
        cv.destroyAllWindows()

        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        np.savetxt(self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[0], mtx)
        np.savetxt(self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[1], dist)
        #np.savetxt(self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[2], rvecs)
        #np.savetxt(self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[3], tvecs)
        self.mtx = mtx
        self.dist = dist
        self.rvecs = rvecs
        self.tvecs = tvecs
        print('Camera is Calibrated')

    def showFeed(self):
        print("Press [ENTER] to stop feed.")
        while True:
            frame = self.getFrame()
            cv.imshow(self.ID + " Feed", imutils.resize(frame, width = 640))
            if cv.waitKey(1) == 13:
                break
        cv.destroyAllWindows()

    def getFrame(self):
        ret, frame = self.cap.read()
        return frame

    def showCalibFeed(self):
        fps = FPS().start()
        print("Press [ENTER] to stop feed.")
        while True:
            frame = self.getCalibFrame()
            cv.imshow(self.ID +"Calib Feed", imutils.resize(frame, width = 640))
            if cv.waitKey(1) == 13:
                break
            fps.update()
        fps.stop()
        print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
        print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
        cv.destroyAllWindows()

    def getCalibFrame(self):
        frame = self.getFrame()
        h, w = frame.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
        dst = cv.undistort(frame, self.mtx, self.dist, None, newcameramtx)
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        return dst

    def release(self):
        self.cap.release()

#This class is used to implement multithreading in I/O
class WebcamVideoStream:
    def __init__(self, src = 0):
        self.stream = cv.VideoCapture(src, cv.CAP_DSHOW)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        Thread(target = self.update, args = ()).start()
        return self

    def update(self):
        while True:
            if self.stopped:
                return
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True

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
