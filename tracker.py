from __future__ import print_function
from camera import Cam
import cv2 as cv
import pandas as pd
import numpy as np
import os
import time
import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

class Track:
    #Sets the aruco library the markers are associated with
    aruco_type = "DICT_5x5_100"
    aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_5X5_100)
    aruco_params = cv.aruco.DetectorParameters_create()

    def __init__(self, cam_list, stream_enable, write_enable, robot_ids):
        self.cam_list = cam_list
        self.stream_enable = stream_enable
        self.write_enable = write_enable
        self.robot_ids = robot_ids

#main functions ----------------------------------------------------------------------------#
    def project_tracker(self, participant_id, aruco_size):
        #test to see if all cameras have a relationship to the global frame
        if self.global_test():
            #creates a file for each participant to store all of their trials
            os.mkdir("trial_cache/" + participant_id)
            count = 0
            #Begins trial looping
            while True:
                count += 1 #counter for trial number
                input("Press [ENTER] to begin Trial #" + str(count))

                #path creation for different trials
                trial_name = "trial_" + str(count)
                path_csv = "trial_cache/" + participant_id + trial_name + "/data.csv"

                #folder creation for different trials
                os.mkdir("trial_cache/" + participant_id + trial_name)
                for camera in self.cam_list:
                    os.mkdir("trial_cache/" + participant_id + trial_name + "/camera_" + camera.ID)
                csv = open(path_csv, "w+")
                csv.close()

                #setting the base time function to reset with each trial
                clock = 0
                start_time = time.time()
                current_time = start
                img_count = 0

                #creates data frames for different ids so that each object can be tracked uniquely
                empty_df_list =[]

                for i in self.robot_ids:
                    df = pd.DataFrame({'ID':[i],'Time':0, 'X': 0, 'Y': 0, 'Z': 0, 'Rx':0, 'Ry':0, 'Rz':0})
                    empty_df_list.append(df)

                empty_df_list = pd.concat(empty_df_list, axis = 1)
                with open(path_csv, 'a') as f:
                    empty_df_list.to_csv(f, header = True, index = False, line_terminator = '\n')

                try:
                    while True:
                        df_list = []
                        for i in self.robot_ids:
                            df = pd.DataFrame({'ID': [i], 'Time': 0, 'X': 0, 'Y': 0, 'Z': 0, 'Rx': 0, 'Ry': 0, 'Rz': 0})
                            df_list.append(df)
                        df = pd.DataFrame({'ID': ["Target"], 'Time': 0, 'X': 0, 'Y': 0, 'Z': 0, 'Rx': 0, 'Ry': 0, 'Rz': 0})
                        df_list.append(df)
                        #sets time state
                        previous_time = current_time
                        current_time = time.time() - start_time

                        #checks each camera from ids
                        for camera in self.cam_list:
                            flag, frame, ids, t_vects, r_vects = self.detect_arucos(camera, aruco_size)
                            frame = cv.resize(frame, (640, int(640 / camera.aspect_ratio)))

                            #shows/saves the frames based on setting chosen in config
                            if self.stream_enable:
                                cv.imshow("cam"+camera.ID, frame)

                            if self.write_enable:
                                path = "trial_cache/" + participant_id + trial_name + "/camera_" + camera.ID\
                                       + "/image" + str(count).zfill(6) + ".png"
                                cv.imwrite(path, frame)

                            #if aruco markers were found
                            if flag:
                                for(m_id, m_t_vec, m_r_vec) in zip(ids, t_vects, r_vects):
                                    if m_id in self.robot_ids:
                                        t_mat = self.vect_to_tmat(m_t_vec[0], m_r_vec[0])
                                        temp = np.dot(camera.global_relationship, t_mat)
                                        df_list = self.to_df(df_list, temp, m_id, clock)
                                        clock += current_time - previous_time

                        self.store_results(df_list, path_csv)
                        if cv.waitKey(1) == 13:
                            break
                        img_count += 1
                        fps.update()
                except KeyboardInterrupt:
                    pass
                fps.stop()
                print("Trial #" + str(count) + "has completed...")
                print("[INFO]: approx. loops per second: {:.2f}".format(fps.fps()))
                cv.destroyAllWindows()

        else: print("Please get the specified camera relationships to the global frame")

    #gets the camera relationships to the global frame.
    #stage represents the stage of extrinsic calibration:
    #----Stage 0 = get all camera relationships to the global frame (fresh start)
    #----Stage 1 = get new camera relationship to the global frame (additive)
    #----Stage 2 = re-do a specific camera extrinsic calibration
    def get_global_relations(self, global_id, id_a, id_b, aruco_size, length, stage = 0, camera_to_change = "99"):
        #sets up the necessary calibration components
        path = "calib_cache/cam_"
        b_wrt_a = np.identity(4)
        b_wrt_a[1, 3] = length

        if stage == 0:
            #get location of the global frame with respect to one of the cameras
            print("Place the global marker in one of the camera frames")
            print("Then press [ENTER] to set the global frame")
            while True:
                bool = False
                for camera in self.cam_list:
                    frame = camera.read()
                    frame = cv.resize(frame, (640, int(640 / camera.aspect_ratio)))
                    cv.imshow("Camera_" + camera.ID, frame)
                if cv.waitKey(500) == 13:
                    for camera in self.cam_list:
                        flag, frame, ids, t_vects, r_vects = self.detect_arucos(camera, aruco_size)
                        if flag:
                            for (m_id, m_tvect, m_rvect) in zip(ids, t_vects, r_vects):
                                if m_id == global_id:
                                    bool = True
                                    camera.global_relationship_flag = True
                                    t_mat = self.vect_to_tmat(m_tvect[0], m_rvect[0])
                                    t_mat = np.linalg.inv(t_mat)
                                    camera.global_relationship = t_mat
                                    np.savetxt(path + camera.ID + "/global_relation.txt",t_mat)
                                    break
                    print("Press [ENTER] to try again!")
                if bool:
                    cv.destroyAllWindows()
                    break
            #get relationships between all cameras by dragging the test board between cameras
            self.global_relations_work(b_wrt_a)

        elif stage == 1:
            self.global_relations_work(b_wrt_a)

        elif stage == 2:
            for index, camera in enumerate(self.cam_list):
                if camera.ID is camera_to_change:
                    camera.global_relationship_flag = False
                    break
            self.global_relations_work(b_wrt_a)

    def global_relations_work(self, b_wrt_a):
        while True:
            tmat_a_flag = False
            tmat_b_flag = False
            cam_a_flag = False
            cam_b_flag = False
            bool = False

            for location, camera in enumerate(self.cam_list):
                flag, frame, ids, t_vects, r_vects = self.detect_arucos(camera, aruco_size)
                frame = cv.resize(frame, (640, int(640 / camera.aspect_ratio)))
                cv.imshow("Cam_" + camera.ID, frame)

                if (id_a in ids) and not (id_b in ids):
                    index = ids.index(id_a)
                    tmat_a = self.vect_to_tmat(t_vects[index][0], r_vects[index][0])
                    tmat_a_flag = True
                    cam_a_flag = camera.global_relationship_flag
                    cam_a_index = location

                elif (id_b in ids) and not (id_a in ids):
                    index = ids.index(id_b)
                    tmat_b = self.vect_to_tmat(t_vects[index][0], r_vects[index][0])
                    tmat_b_flag = True
                    cam_b_flag = camera.global_relationship_flag
                    cam_b_index = location

            if tmat_a_flag and tmat_b_flag and cam_a_flag and not cam_b_flag:
                global_relation = np.dot(self.cam_list[cam_a_index].global_relationship,
                                         np.dot(tmat_a, np.dot(b_wrt_a, np.linalg.inv(tmat_b))))
                np.savetxt(path + self.cam_list[cam_b_index].ID + "/global_relationship.txt", global_relation)
                self.cam_list[cam_b_index].global_relationship = global_relation
                self.cam_list[cam_b_index].global_relationship = True

            elif tmat_a_flag and tmat_b_flag and cam_b_flag and not cam_a_flag:
                global_relation = np.dot(self.cam_list[cam_b_index],
                                         np.dot(tmat_b, np.dot(np.linalg.inv(b_wrt_a), np.linalg.inv(tmat_a))))
                np.savetxt(path + self.cam_list[index].ID + "/global_relationship.txt", global_relation)
                self.cam_list[cam_a_index].global_relationship = global_relation
                self.cam_list[cam_a_index].global_relationship = True

            count = 0
            for camera in self.cam_list:
                if camera.global_relationship_flag:
                    count += 1

            if count is len(self.cam_list):
                cv.destroyAllWindows()
                print("Calibration Complete...")
                self.plot_results()
                break

    def plot_results(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection = '3d')
        x = np.empty((0,3), float)
        y = np.empty((0,3), float)
        z = np.empty((0,3), float)
        for camera in self.cam_list:
            t_vect, _ = self.tmat_to_comp(camera.global_relationship)
            t = t_vect[0]
            x = np.append(x, t[0], axis = 0)
            y = np.append(y, t[1], axis=0)
            z = np.append(z, t[2], axis=0)
        ax.scatter(x, y, z, c='r', marker='o')
        ax.set_xlabel('X(m)')
        ax.set_ylabel('Y(m))')
        ax.set_zlabel('Z(m)')
        plt.show()

#assisting functions -----------------------------------------------------------------------#
    def detect_arucos(self, camera, aruco_size):
        frame = camera.read()
        (corners, ids, _)  = cv.aruco.detectMarkers(frame, self.aruco_dict, parameters = self.aruco_params)
        if len(corners) > 0:
            r_vects, t_vects, mpoints = cv.aruco.estimatePoseSingleMarkers(corners, aruco_size, camera.mtx, camera.dist)
            (rvec-tvec).any()
            ids = ids.flatten()
            for (r, t) in zip(r_vects, t_vects):
                cv.aruco.drawAxis(frame, camera.mtx, camera.dist, r, t, aruco_size)
            return True, frame, ids, t_vects, r_vects
        else: return False, frame, None, None, None

    def global_test(self):
        for camera in self.cam_list:
            if not camera.global_relationship_flag:
                self.printError("Camera " + camera.ID + ": relationship to global frame is missing", "GlobalFrameError:")
                return False
        return True

    def vect_to_tmat(self, t_vect, r_vect):
        r_mat = cv.Rodrigues(r_vect)[0]
        t_mat = np.zeros((4,4))
        for i in range(4):
            for j in range(4):
                if i < 3 and j < 3: t_mat[i, j] = r_mat[i, j]
                elif j == 3 and i < 3: t_mat[i, 3] = t_vect[i]
                elif i == 3 and j < 3: t_mat[3, j] = 0
                else: t_mat[3, 3] = 1
        return t_mat

    def tmat_to_comp(self, t_mat):
        r_mat = np.zeros((3, 3))
        t_vect = np.zeors((1,3))
        for i in range(3):
            for j in range(4):
                if j < 3: r_mat[i, j] = t_mat[i, j]
                elif j == 3: t_vect[0, i] = t_mat[i, 3]
        return r_mat, t_vect

    def to_df(self, df_list, t_mat, id, clock):
        r_mat, t_vect = self. tmat_to_comp(t_mat)
        t_vect = t_vect[0]
        r_x, r_y, r_z = self.rmat_to_angles(r_mat)
        for index, df in enumerate(df_list):
            if df.at[0,'ID'] == id:
                df_list[index] = pd.DataFrame({'ID': [markID], 'Time': clock, 'X': t_vect[0], 'Y': t_vect[1], 'Z': t_vect[2], 'Rx': Rx, 'Ry': Ry, 'Rz': Rz})
                break
        return df_list

    def rmat_to_angles(self, r_mat):
        sy = math.sqrt(r_mat[0, 0] * r_mat[0, 0] + r_mat[1, 0] * r_mat[1, 0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(r_mat[2, 1], r_mat[2, 2])
            y = math.atan2(-r_mat[2, 0], sy)
            z = math.atan2(r_mat[1, 0], r_mat[0, 0])
        else:
            x = math.atan2(-r_mat[1, 2], r_mat[1, 1])
            y = math.atan2(-r_mat[2, 0], sy)
            z = 0
        return np.array([x, y, z])

    def storeResults(self, df_list, path):
        df_list = pd.concat(df_list, axis = 1)
        with open(path, 'a') as f:
            df_list.to_csv(f, header=False, index=False, line_terminator='\n')
