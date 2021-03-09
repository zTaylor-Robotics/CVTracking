from __future__ import print_function
from Camera import camObjThreaded as cmt
from Camera import FPS
import numpy as np
import cv2 as cv
import pandas as pd
import time
import imutils
import os
import math

class track:
    arucoType = "DICT_5x5_1000"
    arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_5X5_100)
    arucoParams = cv.aruco.DetectorParameters_create()
    def __init__(self, camObjList, arucoSize, markerID = 0, targetID = 100):
        self.camObjList = camObjList
        self.tarBool = False
        self.markerID = markerID
        self.targetID = targetID
        self.arucoSize = arucoSize
        self.targetLocation = 0

    def __str__(self):
        return "This is a tracker which takes in frames and detects aruco markers, nothin but a tool"

    def projectTracker(self, trialName):
        path = "trialCache/trial_" + trialName + ".csv"
        csv = open(path, "w+")
        csv.close()
        totalDist = 0
        startA = True
        startB = True
        clock = 0
        prevTime = 0

        fps = FPS().start()
        while True:
            if self.tarBool and startA:
                print("Target Location:")
                print(self.targetLocation)
                startA = False
            currentTime = time.time()
            for i in self.camObjList:
                bool, frame, markID, m2camTMat = self.arucoDetect(i)
                frame = imutils.resize(frame, width = 480)
                cv.imshow("cam"+i.ID, frame)
                if bool and markID == self.markerID:
                    self.storeResults(m2camTMat, markID, path, clock)
                    clock += (currentTime - prevTime)
                    break
            if cv.waitKey(1) == 13:
                break
            prevTime = currentTime
            fps.update()
        fps.stop()
        print("[INFO] approx. Loops per second: {:.2f}".format(fps.fps()))
        cv.destroyAllWindows()
        self.tarBool = False
        self.targetLocation = 0

    def storeResults(self, m2camTMat, markID, path, clock):
        m2globTMat = self.getMarkerRelationToGlobal(m2camTMat,markID)
        rMat, tvec = self.tMat2rT(m2globTMat)
        Rx, Ry, Rz = self.rotationMatrixToEulerAngles(rMat)
        df = pd.DataFrame({'ID': [id], 'Time': clock, 'X': tvec[0], 'Y': tvec[1], 'Z': tvec[2], 'Rx':Rx, 'Ry':Ry, 'Rz':Rz})
        with open(path, 'a') as f:
            if clock == 0:
                df.to_csv(f, header=True, index=False, line_terminator='\n')
            else:
                df.to_csv(f, header=False, index=False, line_terminator='\n')

    def getTargetLocation(self):
        return self.targetLocation

    def getMarkerRelationToGlobal(self, m2camTMat, camID):
        for i in self.camObjList:
            if i.ID == camID:
                cam2globTMat = i.c2gTmat
                return cam2globTMat*m2camTMat
        return m2camTMat

    def arucoDetect(self, camObj):
        while True:
            frame = camObj.read()
            (corners, ids, rejected) = cv.aruco.detectMarkers(frame, self.arucoDict, parameters = self.arucoParams)
            if len(corners) > 0:
                rvec, tvec, markerPoints = cv.aruco.estimatePoseSingleMarkers(corners, self.arucoSize, camObj.mtx, camObj.dist)
                (rvec - tvec).any()
                ids=ids.flatten()
                cv.aruco.drawAxis(frame, camObj.mtx, camObj.dist, rvec, tvec, self.arucoSize*0.4)
                #cv.aruco.drawDetectedMarkers(frame, corners, ids)
                for (markID, markTvec, markRvec) in zip(ids, tvec, rvec):
                    if markID == self.markerID:
                        m2camTMat = self.rT2tMat(markRvec[0],markTvec[0]) #Gets the transformation of the aruco markers location and orientation w.r.t. the camera frame
                        #print(mark_wrt_cam_tMat)
                        return True, frame, markID, m2camTMat

                    #defines target location as an object parameter if the specified target is found
                    if markID == self.targetID and self.tarBool == False:
                        t2g = self.getMarkerRelationToGlobal(self.rT2tMat(markRvec[0], markTvec[0]))
                        _, ttvec = self.tMat2rT(t2g)
                        self.targetLocation = ttvec
                        self.tarBool = True

                    else: return False, frame, "", 0
            else: return False, frame, "", 0

    def rotationMatrixToEulerAngles(self, R):
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        return np.array([x, y, z])

    def getCameraRelations1Ar(self, camObj1, camObj2, targetID):
        print('Please position the aruco marker in view of both cameras')
        print('Then press enter to get camera relations')
        while True:
            while True:
                frame1 = camObj1.read()
                frame2 = camObj2.read()
                frame1t = imutils.resize(frame1, width=640)
                frame2t = imutils.resize(frame2, width=640)
                cv.imshow("Cam"+camObj1.ID, frame1t)
                cv.imshow("Cam"+camObj2.ID, frame2t)
                if cv.waitKey(1) == 13:
                    break
            bool1, frame1, cX1, cY1, marker1, rvec1, tvec1 = self.arucoDetect(camObj1, arucoSize)
            bool2, frame2, cX2, cY2, marker2, rvec2, tvec2 = self.arucoDetect(camObj2, arucoSize)
            print(bool1)
            print(bool2)
            if bool1 and bool2:
                aruWRTcam1 = np.zeros(shape=(3, 3))
                cv.Rodrigues(rvec1, aruWRTcam1)
                aruWRTcam2 = np.zeros(shape=(3, 3))
                cv.Rodrigues(rvec2, aruWRTcam2)
                print(aruWRTcam1)
                print(aruWRTcam2)
                cam2WRTcam1 = aruWRTcam1 * np.linalg.inv(aruWRTcam2)
                cam1WRTcam2 = aruWRTcam2 * np.linalg.inv(aruWRTcam1)

                trans12aruco = tvec1[0]
                trans22aruco = tvec2[0]
                trans12tran2 = trans12aruco - trans22aruco
                trans22trans1 = trans22aruco - trans12aruco

                np.savetxt(
                    "calibCache/cam" + camObj1.ID + "/rotationMatrices/" + "cam" + camObj1.ID + "WRT" + "cam" + camObj2.ID,
                    cam1WRTcam2)
                np.savetxt(
                    "calibCache/cam" + camObj2.ID + "/rotationMatrices/" + "cam" + camObj2.ID + "WRT" + "cam" + camObj1.ID,
                    cam2WRTcam1)
                np.savetxt("calibCache/cam" + camObj1.ID + "/translationMatrices/" + camObj1.ID + "2" + camObj2.ID,
                           trans12tran2)
                np.savetxt("calibCache/cam" + camObj2.ID + "/translationMatrices/" + camObj2.ID + "2" + camObj1.ID,
                           trans22trans1)
                break
            else: print('Error: Please reposition Aruco to be seen by both cameras')

        cv.destroyAllWindows()
        camObj1.stop()
        camObj2.stop()
        print("Relations Found")

    def getCameraRelations2Ar(self, camObj1, camObj2, arucoSize, AR1ID, AR2ID, L):
        pass

    def getCameraRelationsL2G(self, camObj, arucoSize, targetID):
        pass

    #Turns the rotation matrix and translation matrix into a transformation matrix.
    def rT2tMat(self, rvec, tvec):
        rmat, _ = cv.Rodrigues(rvec)
        transMtx = np.zeros((4,4))
        for i in range(4):
            for j in range(4):
                if i < 3 and j < 3: transMtx[i,j] = rmat[i,j]
                elif j == 3 and i < 3:
                    transMtx[i,3] = tvec[i]
                elif i == 3 and j < 3: transMtx[3,j] = 0
                else: transMtx[3,3] = 1
        return transMtx

    def tMat2rT(self, transMtx):
        rmat = np.zeros((3,3))
        tvec = []
        for i in range(3):
            for j in range(4):
                if j < 3: rmat[i,j] = transMtx[i,j]
                elif j == 3: tvec.append(transMtx[i,3])
        return rmat, tvec


