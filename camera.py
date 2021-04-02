from __future__ import print_function
from threading import Thread
from os import path
import numpy as np
import cv2 as cv
import datetime

#sudo apt-get install v4l-utils as a possible error fix
class camObjThreaded:

    #saves path strings for easy reading
    calibPathS = "calibCache/cam"
    calibPathE = ["_mtx.txt", "_dist.txt"]
    arucoType = "DICT_5x5_100"
    arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_5X5_100)
    arucoParams = cv.aruco.DetectorParameters_create()

    def __init__(self, src, arucoSize, camOrder, camIDList, width = 0, height = 0):
        #Saves the input variables and default inputs for program use
        self.camNum = int(src)
        self.width = int(width)
        self.height = int(height)
        self.arucoSize = arucoSize
        self.camOrder = camOrder #Letters associated with the proper order of the cameras  [L,  CL, C,  CR, R]
        self.camIDList = camIDList #Marker IDs associated with the cameras mentioned above [50, 51, 52, 53, 54]
        # sets testing flags which will be used to halt the stream and to determine if calibration has occurred.
        self.calibrationFlag = False
        self.stopped = False

        #Creates a camera stream with the opencv VideoCapture
        self.stream = cv.VideoCapture(self.camNum, cv.CAP_DSHOW)
        self.aspectRatio = 0
        (_,self.frame) = self.stream.read()
        self.uncalibFrame = self.frame

        # initializes system parameters with empty/false values
        self.ID = ""
        self.cameraWRTGlobalTMat = np.array([])
        self.cameraWRTGlobalTMatFlag = False
        self.mtx = np.array([])
        self.dist = np.array([])
        self.calibratedFrame = self.frame

        #Runs the camera through a start up process
        self.startUp()

    def start(self):
        Thread(target=self.update, args=()).start()
        self.getID()
        self.setCalib()
        return self

    def update(self):
        fps = FPS().start()
        while True:
            if self.stopped:
                fps.stop()
                print("[INFO] Camera " + self.ID + " approx. FPS: {:.2f}".format(fps.fps()))
                return
            (_, frame) = self.stream.read()
            self.uncalibFrame = frame
            if self.calibrationFlag:
                h, w = frame.shape[:2]
                newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))
                frame = cv.undistort(frame, self.mtx, self.dist, None, newCameraMatrix)
                x, y, w, h = roi
                frame = frame[y:y+h, x:x+w]
                self.calibratedFrame = frame
            else:
                self.frame = frame
            fps.update()

    def read(self):
        if self.calibrationFlag: return self.calibratedFrame
        else: return self.frame
        
    def uncalRead(self):
        return self.uncalibFrame

    def stop(self):
        self.stopped = True
        self.stream.release()

    def arucoDetect(self, arucoSize):
        frame = self.read()
        (corners, ids, rejected) = cv.aruco.detectMarkers(frame, self.arucoDict, parameters=self.arucoParams)
        if len(corners) > 0:
            ids = ids.flatten()
            return True, ids
        else:
            return False, 0

    def calibrate(self):
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        input('Press [ENTER] to start calibration')

        print('Calibration will take 20 images: ')
        print('Take checkerboard, position it, and then press [ENTER] to add image to list:')

        objp = np.zeros((6 * 7, 3), np.float32)
        objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

        objpoints = []
        imgpoints = []
        count = 0
        i = 0
        while i < 20:
            while True:
                frame = self.uncalRead()
                frameS = cv.resize(frame, (640, int(640/self.aspectRatio)))
                cv.imshow("Calibration", frameS)
                if cv.waitKey(1) == 13:
                    break

            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            ret, corners = cv.findChessboardCorners(gray, (7, 6), None)

            if ret:
                objpoints.append(objp)
                corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners)
                cv.drawChessboardCorners(frame, (7, 6), corners, ret)

                frame = cv.resize(frame, (640, int(640/self.aspectRatio)))
                cv.imshow("Good Image", frame)
                cv.imwrite("calibCache/cam" + self.ID + "/goodImg/img" + str(count) + ".png", frame)
                cv.waitKey(500)
            else:
                print("Image failed, please try again")
                cv.imwrite("calibCache/cam" + self.ID + "/badImg/img" + str(count) + ".png", frameS)
                i -= 1
            i += 1
            count += 1
        cv.destroyAllWindows()

        ret, mtx, dist, _, _ = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        np.savetxt(self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[0], mtx)
        np.savetxt(self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[1], dist)
        self.mtx = mtx
        self.dist = dist

        print('Camera is Calibrated')
        self.calibrationFlag = True
        return

    def getID(self):
        print("Camera_"+str(self.camNum)+" is locating marker...")
        found = False
        while True:
            flag, ids = self.arucoDetect(self.arucoSize)
            if flag:
                for id in ids:
                    try:
                        index = self.camIDList.index(id)
                        found = True
                        break
                    except ValueError:
                        found = False
                        pass
            if found:
                self.ID = self.camOrder[index]
                print("Camera_"+str(self.camNum)+" is cam"+self.ID)
                return

    def setCalib(self):
        if path.exists(self.calibPathS + self.ID + "/" + self.ID + "wrtG.txt"):
            self.cameraWRTGlobalTMat = np.loadtxt(self.calibPathS + self.ID + "/" + self.ID + "wrtG.txt")
            self.cameraWRTGlobalTMatFlag = True
            print("#-----Camera Relation to Global Frame is Defined---------#")
        else:
            self.printError("Camera relation to global frame is undefined", "CalibError")

        testPath = self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height)
        if path.exists(testPath + self.calibPathE[0]):
            if path.exists(testPath + self.calibPathE[1]):
                self.mtx = np.loadtxt(
                    self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[0])
                self.dist = np.loadtxt(
                    self.calibPathS + self.ID + "/" + str(self.width) + "x" + str(self.height) + self.calibPathE[1])
                self.calibrationFlag = True
            else:
                self.printError("Calibration files are not found, please calibrate camera", "CalibError")
        else:
            self.printError("Calibration files are not found, please calibrate camera", "CalibError")
        return

    def printError(self, errorString, errorFlag):
        print("[ERROR]-----------------------------------------------------------------------------------------#")
        print("       "+errorFlag+":"+errorString)
        print("#-----------------------------------------------------------------------------------------------#")
        return

    #An in-depth start up process which defines camera properties
    def startUp(self):
        self.stream.set(cv.CAP_PROP_AUTOFOCUS,0)
        self.stream.set(cv.CAP_PROP_FRAME_WIDTH, 5000)
        self.stream.set(cv.CAP_PROP_FRAME_HEIGHT, 5000)
        w = self.stream.get(cv.CAP_PROP_FRAME_WIDTH)
        h = self.stream.get(cv.CAP_PROP_FRAME_HEIGHT)
        self.aspectRatio = w/h

        if self.width == 0 or self.height == 0:
            if self.width == 0:
                self.width = int(self.height * self.aspectRatio)
                self.stream.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
                self.stream.set(cv.CAP_PROP_FRAME_WIDTH, self.width)

            if self.height == 0:
                self.height = int(self.width / self.aspectRatio)
                self.stream.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
                self.stream.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)

        else:
            self.stream.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
            self.stream.set(cv.CAP_PROP_FRAME_WIDTH, self.width)

        (_, frame) = self.stream.read()

        self.start()
        return

    def showFeed(self):
        print("Press [ENTER] to stop feed.")
        while True:
            frame = self.read()
            cv.imshow("Camera "+ self.ID, frame)
            if cv.waitKey(1) == 13:
                break
        cv.destroyAllWindows()

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
