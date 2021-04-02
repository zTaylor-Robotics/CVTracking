from __future__ import print_function
from camera import camObjThreaded as cmt
from camera import FPS
import cv2 as cv
import pandas as pd
import numpy as np
import os
import time
import math

#NOTE: Global ID will always be 7
class Track:
    #Sets the aruco library to a predefined one
    arucoType = "DICT_5x5_100"
    arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_5X5_100)
    arucoParams = cv.aruco.DetectorParameters_create()

    #sets the order for the project and defines all the relational matrices
    projOrder = ["L", "CL", "C", "CR", "R"]
    path4RelMats = ["L/LwrtG.txt", "CL/CLwrtG.txt", "C/CwrtG.txt", "CR/CRwrtG.txt", "R/RwrtG.txt"]

    #projFlag is the users way of specifying that the
    #-robotArucoID = the ID of the aruco marker which is on top of the robot
    #-targetID = the ID of the marker which represents the target
    def __init__(self, camObjList, streamEnable, write_enable, robotArucoIDs, targetID = 99, sortEnable = True):
        #Preprocessing step for camera list to arrange the camera objects in proper reading order
        self.sortEnable = sortEnable
        self.camObjList = self.sortCamObjList(camObjList)
        self.streamEnable = streamEnable
        self.writeEnable = write_enable
        #Uncomment the following two lines to allow for debugging
        #self.camObjList = camObjList
        #self.camObjList[0].cameraWRTGlobalTMatFlag = True

        #Initializes information about the robot: Defaults to 0
        self.robotArucoIDs = robotArucoIDs

        #Initializes information about the target: Defaults to 99
        self.targetID = targetID
        self.targetFlag = False
        self.targetLocation = np.zeros((1,3))


    def getCamError(self, A1ID, A2ID, L, arucoSize):
        camera = self.camObjList[0]

        pathCSV = "calibCache/cam"+camera.ID+"/camError.csv"
        csv = open(pathCSV, "w+")
        csv.close()
        df = pd.DataFrame({'ID': [0], 'Ex': 0, 'Ey': 0, 'Ez': 0, 'Erx': 0,'Ery': 0, 'Erz': 0})
        with open(pathCSV, 'a') as f:
            df.to_csv(f, header=True, index=False, line_terminator='\n')
        count = 0
        while True:
            for camera in self.camObjList:
                aDFlag, frame, ids, tvec, rvec = self.arucoDetect(camera, arucoSize)
                frame = cv.resize(frame, (640, int(640 / camera.aspectRatio)))
                cv.imshow("Cam_" + camera.ID, frame)
                if aDFlag:
                    TBool, index1, index2 = self.getIndex12(A1ID, A2ID, ids)
                    if TBool:
                        rvec1 = rvec[index1]
                        tvec1 = tvec[index1]
                        TMat1 = self.rtvec2TMat(rvec1[0], tvec1[0])
                        rvec2 = rvec[index2]
                        tvec2 = tvec[index2]
                        TMat2 = self.rtvec2TMat(rvec2[0], tvec2[0])
                        A2wrtA1 = np.dot(np.linalg.inv(TMat1), TMat2)
                        A2A1rMat, A2A1tvec = self.TMat2rtvec(A2wrtA1)
                        print(A2A1rMat)
                        A2A1tvec = A2A1tvec[0]
                        Rx, Ry, Rz = self.rotationMatrixToEulerAngles(A2A1rMat)
                        df = pd.DataFrame({'ID':[count], 'Ex': A2A1tvec[0], 'Ey': A2A1tvec[1]-L, 'Ez':A2A1tvec[2], 'Erx':Rx, 'Ery':Ry, 'Erz':Rz})
                        with open(pathCSV, 'a') as f:
                            df.to_csv(f, header=False, index=False, line_terminator='\n')
                        count += 1

                if cv.waitKey(1) == 13:
                    cv.destroyAllWindows()
                    return

    #directories need to be deleted of the same name or else these will go poopy-dook
    def projectTracker(self, trialName, arucoSize):
        #intial test to see if all the cameras have a defined relationship with the global frame
        if self.testGlobalRelations():
            # sets target file target and initializes the file location
            os.mkdir("trialCache/trial_"+trialName)
            for camera in self.camObjList:
                os.mkdir("trialCache/trial_"+trialName+"/camera_"+camera.ID)

            pathCSV = "trialCache/trial_" + trialName + "/data.csv"
            csv = open(pathCSV, "w+")
            csv.close()

            startA = True
            clock = 0
            start = time.time()
            prevTime = 0
            count = 0
            fps = FPS().start()

            DFList = []
            for i in self.robotArucoIDs:
                DFList.append(pd.DataFrame({'ID':[i],'Time':0, 'X': 0, 'Y': 0, 'Z': 0, 'Rx':0, 'Ry':0, 'Rz':0}))
            alpha = pd.concat(DFList, axis=1)
            #DFList = []
            with open(pathCSV, 'a') as f:
                alpha.to_csv(f, header=True, index=False, line_terminator='\n')

            try:
                while True:
                    DFList.clear()
                    DFList = []
                    for i in self.robotArucoIDs:
                        DFList.append(pd.DataFrame( {'ID': [i], 'Time': 0, 'X': 0, 'Y': 0, 'Z': 0, 'Rx': 0, 'Ry': 0,'Rz': 0}))
                    currentTime = time.time() - start
                    for camera in self.camObjList:
                        flag, frame, ids, tvec, rvec = self.arucoDetect(camera, arucoSize)

                        frame = cv.resize(frame, (640, int(640/camera.aspectRatio)))

                        #shows every image, commenting out will make the trials run faster
                        if self.streamEnable:
                            cv.imshow("cam"+camera.ID, frame)

                        if self.writeEnable:
                            #stores every image, could be slow
                            path = "trialCache/trial_"+trialName+"/camera_"+camera.ID+"/image"+str(count).zfill(6)+".png"
                            cv.imwrite(path, frame)

                        if flag:
                            for (markID, markTvec, markRvec) in zip(ids, tvec, rvec):
                                #if detected marker is the robot's aruco ID, then execute data storage
                                #print(markTvec)
                                if markID in self.robotArucoIDs:
                                    robotWrtCameraTMat = self.rtvec2TMat(markRvec[0], markTvec[0])
                                    #takes the markers location and rotation wrt to the camera and transforms it to a relationship with the global frame
                                    temp = np.dot(camera.cameraWRTGlobalTMat,robotWrtCameraTMat)

                                    DFList = self.toDF(DFList, temp, markID, clock)
                                    #self.storeResults(temp, markID, pathCSV, clock)
                                    clock += (currentTime - prevTime)

                                #if the targets location is found, relate that location to the global frame and store once
                                elif markID == self.targetID and not self.targetFlag:
                                    targetWrtCameraTMat = self.rtvec2TMat(markRvec[0], markTvec[0])
                                    targetWrtGlobalTMat = np.dot(camera.cameraWRTGlobalTMat,targetWrtCameraTMat)
                                    _, targetTvec = self.TMat2rtvec(targetWrtGlobalTMat)
                                    self.targetFlag = True #target location is found
                                    self.targetLocation = targetTvec
                    self.storeResults(DFList, pathCSV)
                    if cv.waitKey(1) == 13:
                        break
                    prevTime = currentTime
                    count += 1
                    fps.update()
            except KeyboardInterrupt:
                print(["Press Ctrl-C to terminate the trial"])
                pass
            fps.stop()
            print("[INFO]: approx. loops per second: {:.2f}".format(fps.fps()))
            if self.targetFlag:
                print("[INFO]: Target Location was found at: ")
                print(self.targetLocation)
            else: print("[INFO]: Target was not found during the trial")
            cv.destroyAllWindows()
            self.targetFlag = False
            self.targetLocation = 0

        else: print("Please get the specified camera relationships to the global frame")

    def arucoDetect(self, camObj, arucoSize):
        frame = camObj.read()
        (corners, ids, rejected) = cv.aruco.detectMarkers(frame, self.arucoDict, parameters = self.arucoParams)
        if len(corners) > 0:
            rvec, tvec, markerPoints = cv.aruco.estimatePoseSingleMarkers(corners, arucoSize, camObj.mtx, camObj.dist)
            (rvec-tvec).any()
            ids = ids.flatten()
            for (r, t) in zip(rvec, tvec):
                cv.aruco.drawAxis(frame, camObj.mtx, camObj.dist, r, t, arucoSize)
            return True, frame, ids, tvec, rvec
        else: return False, frame, 0, 0, 0

    #This function gets the 5 cameras relationship to the global frame, this is designed to get all relationships in one go
    #This may be altered in the future to get single camera relationships if others have been found successfully
    #The process is done by finding the L cameras relationship to the global frame
    #Then the cameras relationship between the CL and L camera is found, then this is pre-multiplied by the left
    #   cameras relationship to the global frame. and so on and so on
    #The relationships will then be stored in the cameras directory with only its relation to the global frame
    def getCameraRelations2Global(self, globalArucoID, A1ID, A2ID, arucoSize, L, calibMethod):
        flag = True
        path = "calibCache/cam"
        temp = np.array([])
        #sets up the  test board where the translation from one marker to the other marker is a simple translation from
        #   marker 1 to marker 2.

        #-----------Method #1 -------------#
        if calibMethod == 1:
            A2wrtA1 = np.identity(4)
            A2wrtA1[1,3] = L

        #-----------Method #2 -------------#
        #Highly accurate providing a perfect world where the cameras are perfectly calibrated with no inaccuracies.
        if calibMethod == 2:
            A2wrtA1 = self.getCalBoardTMats(A1ID, A2ID, arucoSize)

        print(A2wrtA1)
        if self.camObjList[0].ID != "L":
            self.printError("Please initialize the left camera to begin extrinsic calibration.", "CalibError:")
            flag = False
        if flag:
            for index, camera in enumerate(self.camObjList):
                if camera.ID == "L":
                    print("Position marker for the global frame and press [ENTER]:")
                    while True:
                        aDFlag, frame, ids, tvec, rvec = self.arucoDetect(camera, arucoSize)
                        if cv.waitKey(1) == 13 and aDFlag:
                            bool = False
                            for (markID, markTvec, markRvec) in zip(ids, tvec, rvec):
                                if markID == globalArucoID:
                                    bool = True
                                    globalWRTCameraTMat = self.rtvec2TMat(markRvec[0], markTvec[0])
                                    cameraWRTGlobalTmat = np.linalg.inv(globalWRTCameraTMat)
                                    temp = cameraWRTGlobalTmat
                                    np.savetxt(path+self.path4RelMats[index],temp)
                                    break
                            if bool:
                                cv.destroyAllWindows()
                                break
                            else: print("Please press [ENTER] to try again!")
                        frame = cv.resize(frame, (640, int(640 / camera.aspectRatio)))
                        cv.imshow("Camera_" + camera.ID, frame)
                elif camera.ID != "L":
                    print("[INFO]: Position markers to be seen by both cameras where marker 1 is in frame 1")
                    print("    and marker 2 is in frame 2")
                    print("    Both markers need axis projected on them to be considered for this step")
                    print("[INFO]:Then press [ENTER] to test the current images")
                    A1wrtPreviousCameraTMat = np.array([])
                    A2wrtCurrentCameraTMat = np.array([])
                    currentCameraWRTGlobalTMat = np.array([])

                    #cameraB = current camera
                    #self.camObjList[index-1] = previous cameraA
                    #Example: cameraCL = current camera, previous camera = cameraL
                    while True:
                        boolA, frameA, idsA, tvecA, rvecA = self.arucoDetect(self.camObjList[index -1], arucoSize)
                        boolB, frameB, idsB, tvecB, rvecB = self.arucoDetect(camera, arucoSize)

                        if cv.waitKey(1) == 13 and boolA and boolB:
                            test = [False, False]
                            for (markID, markTvec, markRvec) in zip(idsA, tvecA, rvecA):
                                if markID == A1ID:
                                    A1wrtPreviousCameraTMat = self.rtvec2TMat(markRvec[0], markTvec[0])
                                    test[0] = True
                                    break
                            for (markID, markTvec, markRvec) in zip(idsB, tvecB, rvecB):
                                if markID == A2ID:
                                    A2wrtCurrentCameraTMat = self.rtvec2TMat(markRvec[0], markTvec[0])
                                    test[1] = True
                                    break
                            if test[0] and test[1]:
                                #uses the following maths to get the new TMat
                                currentCameraWRTGlobalTMat = np.dot(temp,np.dot(A1wrtPreviousCameraTMat,np.dot(A2wrtA1, np.linalg.inv(A2wrtCurrentCameraTMat))))
                                temp = currentCameraWRTGlobalTMat
                                np.savetxt(path+self.path4RelMats[index],temp)
                                cv.destroyAllWindows()
                                break
                        frameA = cv.resize(frameA, (640, int(640 / self.camObjList[index - 1].aspectRatio)))
                        frameB = cv.resize(frameB, (640, int(640 / camera.aspectRatio)))

                        cv.imshow("Camera_" + self.camObjList[index - 1].ID, frameA)
                        cv.imshow("Camera_" + camera.ID, frameB)
            input("Calibration Complete. Press [ENTER] to proceed..")

    def getIndex12(self, A1ID, A2ID, ids):
        index1 = -1
        index2 = -1
        for index, j in enumerate(ids):
            if j == A1ID:
                index1 = index
            if j == A2ID:
                index2 = index
        if index1 >= 0 and index2 >= 0:
            print(True)
            return True, index1, index2
        else:
            return False, -1, -1

    def getCalBoardTMats(self, A1ID, A2ID, arucoSize):
        print("Please take the test board and place it in one of the camera views then press [ENTER]")
        while True:
            for camera in self.camObjList:
                aDFlag, frame, ids, tvec, rvec = self.arucoDetect(camera, arucoSize)
                frame = cv.resize(frame, (640, int(640 / camera.aspectRatio)))
                cv.imshow("Cam_" + camera.ID, frame)
                if cv.waitKey(1) == 13 and aDFlag:
                    TBool, index1, index2 = self.getIndex12(A1ID, A2ID, ids)
                    if TBool:
                        rvec1 = rvec[index1]
                        tvec1 = tvec[index1]
                        TMat1 = self.rtvec2TMat(rvec1[0], tvec1[0])
                        rvec2 = rvec[index2]
                        tvec2 = tvec[index2]
                        TMat2 = self.rtvec2TMat(rvec2[0], tvec2[0])
                        A2wrtA1 = np.dot(np.linalg.inv(TMat1), TMat2)

                        cv.destroyAllWindows()
                        return A2wrtA1
                    else: print("Please try again by pressing [ENTER]")

    def printError(self, errorString, errorFlag):
        print("[ERROR]-----------------------------------------------------------------------------------------#")
        print("       "+errorFlag+":"+errorString)
        print("#-----------------------------------------------------------------------------------------------#")
        return

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

    def rtvec2TMat(self, rvec, tvec):
        rMat = cv.Rodrigues(rvec)[0]
        TMat = np.zeros((4, 4))
        for i in range(4):
            for j in range(4):
                if i < 3 and j < 3: TMat[i, j] = rMat[i, j]
                elif j == 3 and i < 3: TMat[i, 3] = tvec[i]
                elif i == 3 and j < 3: TMat[3, j] = 0
                else:TMat[3, 3] = 1
        return TMat

    #Sorts the camera objects to be in the proper order for the project.
    def sortCamObjList(self, camObjList):
        if self.sortEnable:
            temp = []
            tempCL = []
            for camera in camObjList:
                temp.append(camera.ID)
            for i in range(len(temp)):
                x = temp.index(self.projOrder[i])
                tempCL.append(camObjList[x])
            return tempCL
        else: return camObjList

    def storeResults(self, DFList, path):
        alpha = pd.concat(DFList, axis = 1)
        with open(path, 'a') as f:
            alpha.to_csv(f, header=False, index=False, line_terminator='\n')

    def testGlobalRelations(self):
        for camera in self.camObjList:
            if not camera.cameraWRTGlobalTMatFlag:
                self.printError("Camera " + camera.ID + ": relationship to global frame is missing", "GlobalFrameError:")
                return False
        return True

    def TMat2rtvec(self, TMat):
        rMat = np.zeros((3, 3))
        tvec = np.zeros((1,3))
        for i in range(3):
            for j in range(4):
                if j < 3:
                    rMat[i, j] = TMat[i, j]
                elif j == 3:
                    tvec[0,i] = TMat[i, 3]
        return rMat, tvec

    def toDF(self,DFList, robotWRTGlobalTMat, markID, clock):
        rMat, tvec = self.TMat2rtvec(robotWRTGlobalTMat)
        tvec = tvec[0]
        Rx, Ry, Rz = self.rotationMatrixToEulerAngles(rMat)
        for index, df in enumerate(DFList):
            if df.at[0,'ID'] == markID:
                DFList[index] = pd.DataFrame({'ID': [markID], 'Time': clock, 'X': tvec[0], 'Y': tvec[1], 'Z': tvec[2], 'Rx': Rx, 'Ry': Ry,'Rz': Rz})
                break
        return DFList


