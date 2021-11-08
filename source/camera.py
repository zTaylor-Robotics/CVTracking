from __future__ import print_function
from threading import Thread
from os import path
import os
import numpy as np
import cv2 as cv
import datetime

#sudo apt-get install v4l-utils as a possible error fix
class Cam:
    if os.path.isfile("./calib_cache/log.txt"):
        file = open("./calib_cache/log.txt",'r')
        a = file.readlines()
        file.close()
        calib_path_d = a[1][6:29] + '/cam_'
    calib_path_f = ["/mtx.txt", "/dist.txt"]
    aruco_type = "DICT_5x5_100"
    aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_5X5_100)
    aruco_params = cv.aruco.DetectorParameters_create()

    def __init__(self, source, cam_id_list, width = 1280, height = 720):
        self.source = int(source)
        self.width = int(width)
        self.height = int(height)
        self.aspect_ratio = self.width / self.height
        self.cam_id_list = cam_id_list

        #sets testing flags
        self.calibration_flag = False
        self.stopped = False
        self.global_relationship_flag = False
        self.save_flag = False
        self.temp_flag = False

        #sets initial stream values
        self.stream = cv.VideoCapture(self.source, cv.CAP_V4L2)

        (_,self.frame) = self.stream.read()

        #attribute initialization without values to be set later
        self.global_relationship = None
        self.ID = None
        self.mtx = None
        self.dist = None
        self.save_path = None
        self.img_count = None

        self.stream.set(cv.CAP_PROP_AUTOFOCUS, 1)
        self.stream.set(4, self.height)
        self.stream.set(3, self.width)
        self.start()

#threaded stream functions
    def start(self):
        Thread(target = self.update, args = ()).start()
        cv.waitKey(1)
        self.get_id()
        self.set_calib()
        return self

    def update(self):
        fps = FPS().start()
        while True:
            if self.stopped:
                fps.stop()
                print("[INFO] Camera " + self.ID + " approx. FPS: {:.2f}".format(fps.fps()))
                return
            if self.save_flag:
                cv.imwrite(self.save_path + "/image_" + str(self.img_count).zfill(5) + ".png", self.frame)
            (_, self.frame) = self.stream.read()
            fps.update()

    def read(self):
        if self.save_flag:
            self.img_count += 1
        return self.frame

    def stop(self):
        self.stopped = True
        self.stream.release()

#exterior function to the stream
    def set_path(self, path):
        self.save_path = path
        return

    def start_save(self):
        self.save_flag = True
        self.img_count = 0
        return

    def stop_save(self):
        self.save_flag = False
        return

    def get_id(self):
        if self.cam_id_list is not None:
            #print("Source_" + str(self.source) + " is locating marker ...")
            #cv.waitKey(100)
            found = False
            while True:
                flag, ids = self.aruco_detect()
                if flag:
                    for id in ids:
                        try:
                            index = self.cam_id_list.index(id)
                            found = True
                            break
                        except ValueError:
                            found = False
                            pass
                if found:
                    self.ID = str(self.cam_id_list[index])
                    #print("Source_" + str(self.source) + " is cam_" + self.ID)
                    #cv.waitKey(100)
                    if not path.exists(self.calib_path_d + self.ID):
                        path_a = self.calib_path_d + self.ID
                        path_b = self.calib_path_d + self.ID + "/" + str(self.width) + "x" + str(self.height)
                        os.mkdir(path_a)
                        os.mkdir(path_b)
                        file = open(path_b + "/mtx.txt", "w")
                        file.close()
                        file = open(path_b + "/dist.txt", "w")
                        file.close()
                    return
        else: self.ID = str(self.source)

    def set_calib(self):
        if path.exists(self.calib_path_d + self.ID + "/global_relation.txt"):
            self.global_relationship = np.loadtxt(self.calib_path_d + self.ID + "/global_relation.txt")
            self.global_relationship_flag = True
            #print("...camera relation to global frame is defined")
            #cv.waitKey(100)
        #else: 
            #self.print_error("Camera relation to global is undefined", "CalibError")
            #cv.waitKey(100)

        test_path = self.calib_path_d + self.ID + "/" + str(self.width) + "x" + str(self.height)
        if path.exists(test_path + self.calib_path_f[0]):
            if path.exists(test_path + self.calib_path_f[1]):
                self.mtx = np.loadtxt(test_path + self.calib_path_f[0])
                self.dist = np.loadtxt(test_path+self.calib_path_f[1])
            #else: 
                #self.print_error("Calibration files are not found, please calibrate camera", "CalibError")
                #cv.waitKey(100)
        #else: 
            #self.print_error("Calibration files are not found, please calibrate camera", "CalibError")
            #cv.waitKey(100)
        return

    def aruco_detect(self):
        frame = self.read()
        (corners, ids, _) = cv.aruco.detectMarkers(frame, self.aruco_dict, parameters = self.aruco_params)
        if len(corners) > 0:
            ids = ids.flatten()
            return True, ids
        else:
            return False, 0

    def print_error(self, error_string, error_flag):
        print("[ERROR]------------------------------------------------------------------#")
        print("       " + error_flag + ":" + error_string)
        print("#------------------------------------------------------------------------#")
        return

    def show_feed(self):
        #print("Press [ENTER] to stop feed.")
        while True:
            frame = self.read()
            frame = cv.resize(frame, (640, int(640 / self.aspect_ratio)))
            cv.imshow("cam_ "+ self.ID, frame)
            if cv.waitKey(1) == 13:
                break
        cv.destroyAllWindows()

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
