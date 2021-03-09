from __future__ import print_function
from Camera import camObjThreaded as cmt
from Camera import FPS
import numpy as np
import cv2 as cv
import pandas as pd
import time
import imutils

class track:
    arucoType = "DICT_5x5_1000"
    arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_5X5_100)
    arucoParams = cv.aruco.DetectorParameters_create()
    def __init__(self):
        pass
    def __str__(self):
        return "This is a tracker which takes in frames and detects aruco markers, nothin but a tool"
    def arucoDetect(self, camObj, arucoSize, targetID):
        while True:
            frame = camObj.read()
            (corners, ids, rejected) = cv.aruco.detectMarkers(frame, self.arucoDict, parameters = self.arucoParams)
            if len(corners) > 0:
                rvec, tvec, markerPoints = cv.aruco.estimatePoseSingleMarkers(corners, arucoSize, camObj.mtx, camObj.dist)
                (rvec - tvec).any()
                ids=ids.flatten()
                cv.aruco.drawDetectedMarkers(frame, corners)
                cv.aruco.drawAxis(frame, camObj.mtx, camObj.dist, rvec, tvec, 3)
                for (markerCorner, markerID, markerTvec, markerRvec) in zip(corners, ids, tvec, rvec):
                    corners = markerCorner.reshape((4,2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners

                    #topRight = (int(topRight[0]), int(topRight[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))
                    #bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    #bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))

                    #cv.line(frame, topLeft, topRight, (0, 255, 0), 2)
                    #cv.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                    #cv.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                    #cv.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

                    #cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    #cY = int((topLeft[1] + bottomRight[1]) / 2.0)

                    #cv.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
                    cv.putText(frame, str(markerID), (topLeft[0], topLeft[1] - 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    if markerID == targetID:
                        print(markerTvec)
                    #print(np.linalg.norm(np.array((topLeft[0],topLeft[1],0))-np.array((bottomLeft[0],bottomLeft[1],0))))
                        return True, frame, markerID, markerRvec[0], markerTvec[0]
                    else: return False, frame, "", 0, 0
            else: return False, frame, "", 0, 0


    def arucoTestFeed(self,camObj, arucoSize, trialName, targetID):
        path = "trialCache/trial_"+trialName+".csv"
        csv = open(path, "w+")
        csv.close()
        totalDist = 0
        start = True
        clock = 0
        prevTime = 0
        fps = FPS().start()
        while True:
            currentTime = time.time()
            bool, frame, markerID, rvec, tvec = self.arucoDetect(camObj, arucoSize, targetID)
            if bool == True:
                #For project convention, if ID = targetID == 0, then its the human trial
                #For project convention, if ID = targetID != 0, then its the robot trial
                if markerID == 0:
                    id = "Human"
                else:
                    id = "Robot"

                #If this is the first maker grab, then it initializes prevPoint
                if start:
                    prevPoint = tvec
                    start = False
                else:
                    currentPoint = tvec
                    totalDist += np.linalg.norm(currentPoint-prevPoint)
                    prevPoint = currentPoint

                h, w = frame.shape[:2]
                df = pd.DataFrame({'ID':[id], 'Time': clock, 'X': tvec[0], 'Y': tvec[1], 'Z': tvec[2]})
                with open(path, 'a') as f:
                    if clock == 0:
                        df.to_csv(f, header = True, index = False, line_terminator='\n')
                    else:
                        df.to_csv(f, header = False, index = False, line_terminator='\n')
                clock += (currentTime - prevTime)
            cv.imshow("Feed", frame)
            if cv.waitKey(1) == 13:
                break
            prevTime = currentTime
            fps.update()
        fps.stop()
        print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
        print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
        cv.destroyAllWindows()

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
    def rT2TransformationMtx(self, rvec, tvec):
        rmat, _ = cv.Rodrigues(rvec)
        transMtx = np.zeros(4,4)
        for i in range(4):
            for j in range(4):
                if i != 3 and j != 3: transMtx[i][j] = rmat[i][j]
                elif j == 3 and i != 3: transMtx[i][3] = tvec[i]
                elif i == 3 and j != 3: transMtx[3][j] = 0
                else: transMtx[3][3] = 1
        print(transMtx)
        return transMtx

    def Tmat2RotTrans(self, transMtx):
        rmat = np.zeros(3,3)
        tvec = []
        for i in range(3):
            for j in range(4):
                if j != 3: rmat[i][j] = transMtx[i][j]
                elif j == 3: tvec[i] = transMtx[i][3]
        print(rmat)
        print(tvec)
        return rmat, tvec


