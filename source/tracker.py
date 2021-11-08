from __future__ import print_function

import sys
sys.path.append('./source')

from camera import Cam
from camera import FPS
import threading
import cv2 as cv
import pandas as pd
import numpy as np
import os
import os.path as osp
import time
import math
import shutil
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
import matplotlib.pyplot as plt
from shutil import copyfile
from distutils.dir_util import copy_tree

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
        self.marker_list = []
        self.stopped = False
        for i in self.robot_ids:
            self.marker_list.append(Marker(i))

    def start_live_track(self,participant_id, aruco_size):
        threading._start_new_thread(self.live_track, (participant_id, aruco_size,))
        #Thread(target = self.live_track, args = (participant_id, aruco_size, )).start()
        return self

    def stop_live_track(self):
        self.stopped = True

    def read(self, m_id):
        for i in self.marker_list:
            if m_id == i.ID:
                content = [i.pos_x, i.pos_y, i.pos_z, i.rot_x, i.rot_y]
                return content

#main functions ----------------------------------------------------------------------------#
    def project_tracker(self, participant_id, aruco_size):
        #test to see if all cameras have a relationship to the global frame
        if self.global_test():
            #source = ["trial_cache/dataRead.m", "trial_cache/PostTrialAnalysis.m"]
            #creates a file for each participant to store all of their trials
            count = 0
            if not os.path.isdir("trial_cache/" + participant_id):
                os.mkdir("trial_cache/" + participant_id)
            while True:
                if not os.path.isdir("trial_cache/" + participant_id + "/trial_" + str(count + 1).zfill(4)):
                    break
                else: count += 1
            fps = FPS().start()
            #Begins trial looping
            while True:
                count += 1
                while True:
                    if os.path.isdir("trial_cache/" + participant_id + "/trial_" + str(count).zfill(4)):
                        count += 1 #counter for trial number
                    else: break
                #input("Press [ENTER] to begin Trial # " + str(count) + "\n")
                #path creation for different trials
                trial_name = "/trial_" + str(count).zfill(4)
                path_csv = "trial_cache/" + participant_id + trial_name + "/data.csv"
                #dest = ["trial_cache/" + participant_id + trial_name +"/dataRead.m", "trial_cache/" + participant_id + trial_name + "/PostTrialAnalysis.m"]

                #folder creation for different trials
                os.mkdir("trial_cache/" + participant_id + trial_name)
                os.mkdir("trial_cache/" + participant_id + trial_name + "/calib_cache")
                if self.write_enable:
                    for camera in self.cam_list:
                        os.mkdir("trial_cache/" + participant_id + trial_name + "/camera_" + camera.ID)
                        camera.set_path("trial_cache/" + participant_id + trial_name + "/camera_" + camera.ID)
                #copyfile(source[0], dest[0])
                #copyfile(source[1], dest[1])
                csv = open(path_csv, "w+")
                csv.close()
                copy_tree("calib_cache", "trial_cache/" + participant_id + "/trial_" + str(count).zfill(4) + "/calib_cache")
                #setting the base time function to reset with each trial
                clock = 0
                start_time = time.time()
                current_time = time.time() - start_time
                img_count = 0

                #creates data frames for different ids so that each object can be tracked uniquely
                empty_df_list =[]

                for i in self.robot_ids:
                    df = pd.DataFrame({'Aruco_ID':[i], 'Cam_ID': 0,'Time':0, 'X': 0, 'Y': 0, 'Z': 0, 'x_rot':0, 'y_rot':0, 'z_rot':0})
                    empty_df_list.append(df)
                empty_df_list = pd.concat(empty_df_list, axis = 1)
                with open(path_csv, 'a') as f:
                    empty_df_list.to_csv(f, header = True, index = False, line_terminator = '\n')

                try:

                    print("Trial#: " + str(count).zfill(4) + " running...")

                    if self.write_enable:
                        for camera in self.cam_list:
                            camera.start_save()
                    while True: 
                        df_list = []
                        something_happend = False
                        for i in self.robot_ids:
                            df = pd.DataFrame({'Aruco_ID': [i], 'Cam_ID': 0, 'Time': 0, 'X': 0, 'Y': 0, 'Z': 0, 'x_rot': 0, 'y_rot': 0, 'z_rot': 0})
                            df_list.append(df)
                        #sets time state
                        previous_time = current_time
                        current_time = time.time() - start_time

                        #checks each camera from ids
                        t_a = [] #the marker relation to global frame temp storage
                        t_b = [] #the marker id number
                        t_c = [] #the camera id whcih detected the marker
                        for camera in self.cam_list:
                            flag, frame, ids, t_vects, r_vects = self.detect_arucos(camera, aruco_size)

                            #shows/saves the frames based on setting chosen in config
                            if self.stream_enable:
                                cv.imshow("cam"+camera.ID, frame)

                            #if aruco markers were found
                            if flag:
                                for(m_id, m_t_vec, m_r_vec) in zip(ids, t_vects, r_vects):
                                    if m_id[0] in self.robot_ids:
                                        t_mat = self.vect_to_tmat(m_t_vec[0], m_r_vec[0])
                                        tempe = camera.global_relationship @ t_mat
                                        self.set_marker_prop(m_id[0], tempe)
                                        t_a.append(tempe)
                                        t_b.append(m_id[0])
                                        t_c.append(camera.ID)
                                        if not something_happend: something_happend = True
                        if len(t_b) == len(set(t_b)):
                            for index in range(len(t_a)):
                                df_list = self.to_df(df_list, t_a[index], t_b[index], clock, t_c[index])
                        else:
                            todf_list = []
                            dup_ids = []
                            for i in range(len(t_b)):
                                int_l = [i]
                                if not (t_b[i] in dup_ids):
                                    for j in range(len(t_b[i:])):
                                        if t_b[i] == t_b[j]:
                                            flag = True
                                            int_l.append(j)
                                            if not (t_b[i] in dup_ids): dup_ids.append(t_b[i])
                                        else: flag  = False
                                #at this point all duplicates are found
                                    if len(int_l) > 2:
                                        current_index = int_l[0]
                                        for index, p in enumerate(int_l):
                                            #favoring the camera with the lowest id
                                            for q in int_l[index:]:
                                                if t_c[q] < t_c[p]: current_index = q
                                        todf_list.append(current_index) 
                                    else: todf_list.append(i)
                            for m in todf_list:
                                df_list = self.to_df(df_list, t_a[m], t_b[m], clock, t_c[m])
                        t_a.clear()
                        t_b.clear()
                        t_c.clear()
                        del t_a, t_b, t_c
                        clock += current_time - previous_time
                        if something_happend: self.store_results(df_list, path_csv)
                        if cv.waitKey(1) == 13:
                            break
                        img_count += 1
                        fps.update()
                except KeyboardInterrupt:
                    pass
                if self.write_enable:
                    for camera in self.cam_list:
                        camera.stop_save()
                fps.stop()
                #print("Trial #" + str(count) + " has completed...")
                print("[INFO]: approx. loops per second: {:.2f}".format(fps.fps()))
                cv.destroyAllWindows()
 #               huel = input("Press [Enter] to continue or type STOP to stop: ")
#                if huel == "STOP": # -- if wanting to come back to running constant, remove # nad push return ->
                return # -- use spaces and not tabs!
        else: print("Please get the specified camera relationships to the global frame")

    def live_track(self, participant_id, aruco_size):
        #test to see if all cameras have a relationship to the global frame
        if self.global_test():
            #creates a file for each participant to store all of their trials
            count = 0
            if not os.path.isdir("trial_cache/" + participant_id):
                os.mkdir("trial_cache/" + participant_id)
            while True:
                if not os.path.isdir("trial_cache/" + participant_id + "/trial_" + str(count + 1).zfill(4)):
                    break
                else: count += 1
            fps = FPS().start()
            #Begins trial looping
            while True:
                count += 1
                while True:
                    if os.path.isdir("trial_cache/" + participant_id + "/trial_" + str(count).zfill(4)):
                        count += 1 #counter for trial number
                    else: break
                #path creation for different trials
                trial_name = "/trial_" + str(count).zfill(4)
                path_csv = "trial_cache/" + participant_id + trial_name + "/data.csv"

                #folder creation for different trials
                os.mkdir("trial_cache/" + participant_id + trial_name)
                os.mkdir("trial_cache/" + participant_id + trial_name + "/calib_cache")
                if self.write_enable:
                    for camera in self.cam_list:
                        os.mkdir("trial_cache/" + participant_id + trial_name + "/camera_" + camera.ID)
                        camera.set_path("trial_cache/" + participant_id + trial_name + "/camera_" + camera.ID)

                csv = open(path_csv, "w+")
                csv.close()
                copy_tree("calib_cache", "trial_cache/" + participant_id + "/trial_" + str(count).zfill(4) + "/calib_cache")
                #setting the base time function to reset with each trial
                clock = 0
                start_time = time.time()
                current_time = time.time() - start_time
                img_count = 0

                #creates data frames for different ids so that each object can be tracked uniquely
                empty_df_list =[]

                for i in self.robot_ids:
                    df = pd.DataFrame({'Aruco_ID':[i], 'Cam_ID': 0,'Time':0, 'X': 0, 'Y': 0, 'Z': 0, 'x_rot':0, 'y_rot':0, 'z_rot':0})
                    empty_df_list.append(df)
                empty_df_list = pd.concat(empty_df_list, axis = 1)
                with open(path_csv, 'a') as f:
                    empty_df_list.to_csv(f, header = True, index = False, line_terminator = '\n')

                print("Trial#: " + str(count).zfill(4) + " running...")
                if self.write_enable:
                    for camera in self.cam_list:
                        camera.start_save()
                while True: 
                    df_list = []
                    something_happend = False
                    for i in self.robot_ids:
                        df = pd.DataFrame({'Aruco_ID': [i], 'Cam_ID': 0, 'Time': 0, 'X': 0, 'Y': 0, 'Z': 0, 'x_rot': 0, 'y_rot': 0, 'z_rot': 0})
                        df_list.append(df)
                    #sets time state
                    previous_time = current_time
                    current_time = time.time() - start_time

                    #checks each camera from ids
                    t_a = [] #the marker relation to global frame temp storage
                    t_b = [] #the marker id number
                    t_c = [] #the camera id whcih detected the marker
                    for camera in self.cam_list:
                        flag, frame, ids, t_vects, r_vects = self.detect_arucos(camera, aruco_size)
                        #shows/saves the frames based on setting chosen in config
                        if self.stream_enable:
                            cv.imshow("cam"+camera.ID, frame)
                        #if aruco markers were found
                        if flag:
                            for(m_id, m_t_vec, m_r_vec) in zip(ids, t_vects, r_vects):
                                if m_id[0] in self.robot_ids:
                                    t_mat = self.vect_to_tmat(m_t_vec[0], m_r_vec[0])
                                    tempe = camera.global_relationship @ t_mat
                                    self.set_marker_prop(m_id[0], tempe)
                                    t_a.append(tempe)
                                    t_b.append(m_id[0])
                                    t_c.append(camera.ID)
                                    if not something_happend: something_happend = True
                    if len(t_b) == len(set(t_b)):
                        for index in range(len(t_a)):
                            df_list = self.to_df(df_list, t_a[index], t_b[index], clock, t_c[index])
                    else:
                        todf_list = []
                        dup_ids = []
                        for i in range(len(t_b)):
                            int_l = [i]
                            if not (t_b[i] in dup_ids):
                                for j in range(len(t_b[i:])):
                                    if t_b[i] == t_b[j]:
                                        flag = True
                                        int_l.append(j)
                                        if not (t_b[i] in dup_ids): dup_ids.append(t_b[i])
                                    else: flag  = False
                                #at this point all duplicates are found
                                if len(int_l) > 2:
                                    current_index = int_l[0]
                                    for index, p in enumerate(int_l):
                                        #favoring the camera with the lowest id
                                        for q in int_l[index:]:
                                            if t_c[q] < t_c[p]: current_index = q
                                    todf_list.append(current_index) 
                                else: todf_list.append(i)
                        for m in todf_list:
                            df_list = self.to_df(df_list, t_a[m], t_b[m], clock, t_c[m])
                    t_a.clear()
                    t_b.clear()
                    t_c.clear()
                    del t_a, t_b, t_c
                    clock += current_time - previous_time
                    if something_happend: self.store_results(df_list, path_csv)
                    if cv.waitKey(1) == 13:
                        break
                    img_count += 1
                    fps.update()
                    if self.stopped:
                        break

                if self.write_enable:
                    for camera in self.cam_list:
                        camera.stop_save()
                fps.stop()

                print("[INFO]: approx. loops per second: {:.2f}".format(fps.fps()))
                cv.destroyAllWindows()
                return # -- use spaces and not tabs!
        else: print("Please get the specified camera relationships to the global frame")

#Calibration
#----------------------------------------------------------------------------------------------------------------------------------------------
    #gets the camera relationships to the global frame.
    #stage represents the stage of extrinsic calibration:
    #----Stage 0 = get all camera relationships to the global frame (fresh start)
    #----Stage 1 = get new camera relationship to the global frame (additive)
    #----Stage 2 = re-do a specific camera extrinsic calibration
    #----Stage 3 = print current calib
    def get_global_relations(self, global_id, id_a, id_b, aruco_size, length, num_samp, stage, cameras_to_change):
        #sets up the necessary calibration components
        if stage != 3:
            if not osp.isfile('./calib_cache/log.txt'):
                log_file = open('calib_cache/log.txt', 'w')
                a = osp.basename(osp.dirname(osp.realpath(__file__)))
                L = ['current_version: ' + a[11:16] + '.000', 'path: config_cache/' + a[11:16] + '.000']
                log_file.writelines(i+'\n' for i in L)
            file = open('calib_cache/log.txt', 'r')
            a = file.readlines()
            file.close()
            b = osp.basename(osp.dirname(osp.dirname(osp.realpath(__file__))))
            proj_ver = b[11:16]
            prev_path = a[1][6:29]
            last_proj_ver = a[0][17:22]
            last_calib = a[0][17:26]
            if proj_ver == last_proj_ver:
                calib_it = str(int(a[0][23:26]) + 1).zfill(3)
            else: calib_it = '000'
            new_calib = proj_ver + '.' + calib_it
            new_path = './calib_cache/' + new_calib
            file = open('calib_cache/log.txt', 'w')
            L = ['current_version: ' + new_calib, 'path: ' + new_path]
            file.writelines(i + '\n' for i in L)
            file.close()

            copy_tree(prev_path, new_path)
            copy_tree(prev_path, 'calib_cache/lastver/' + last_calib)
            shutil.rmtree(prev_path)
            path = new_path + '/cam_'

            b_wrt_a = np.identity(4)
            b_wrt_a[1, 3] = length
        if stage == 0:
            for i in range(len(self.cam_list)):
                if osp.isfile(path + str(i) + '/global_relation.txt'):
                    os.remove(path + str(i) + '/global_relation.txt')
            for i in self.cam_list:
                i.global_relationship_flag = False
            #get location of the global frame with respect to one of the cameras
            print("Place the global marker in one of the camera frames")
            print("Then press [ENTER] to set the global frame")
            while True:
                bool = False
                for camera in self.cam_list:
                    frame = camera.read()
                    frame = cv.resize(frame, (640, int(640 / camera.aspect_ratio)))
                    cv.imshow("Camera_" + camera.ID, frame)

                if cv.waitKey(10) == 13:
                    glob_camera = None
                    count = 0
                    t_list = []
                    print('Looking for markers')
                    while(count < num_samp):
                        for camera in self.cam_list:
                            flag, frame, ids, t_vects, r_vects = self.detect_arucos(camera, aruco_size)
                            if flag:
                                for (m_id, m_tvect, m_rvect) in zip(ids, t_vects, r_vects):
                                    if m_id == global_id:
                                        camera.global_relationship_flag = True
                                        t_mat = self.vect_to_tmat(m_tvect[0], m_rvect[0])
                                        temp = np.linalg.inv(t_mat) #gives the position and pose of the camera relative to the marker 
                                        camera.global_relationship = temp
                                        count += 1
                                        t_list.append(temp)
                                        glob_camera = camera.ID
                        if count == 0:
                            print('Marker not found: reposition or stop and switch out markers')
                            print("Press [ENTER] to try again!")
                            break
                    if count >= num_samp:
                        a = self.avg_tmat(t_list)
                        np.savetxt(path + glob_camera + "/global_relation.txt", a)
                        bool = True
                if bool:
                    cv.destroyAllWindows()
                    break
            #get relationships between all cameras by dragging the test board between cameras
            input("Press [ENTER] to continue!")
            self.global_relations_work(b_wrt_a, length, aruco_size, id_a, id_b, path, num_samp)

        elif stage == 1:
            self.global_relations_work(b_wrt_a, length, aruco_size, id_a, id_b, path, num_samp)

        elif stage == 2:
            print(cameras_to_change)
            for camera in self.cam_list:
                if int(camera.ID) in cameras_to_change:
                    camera.global_relationship_flag = False
            self.global_relations_work(b_wrt_a, length, aruco_size, id_a, id_b, path, num_samp)
        
        elif stage == 3:
            counts = 0
            for camera in self.cam_list:
                if camera.global_relationship_flag:
                    counts += 1

            if counts is len(self.cam_list):
                cv.destroyAllWindows()
                print("Calibration Complete...")
                self.plot_results()

    def print_updt_msg(self, updt_msg, count):
        self.clear()
        print("Global Relations List:")
        print("----------------------")
        for i in self.cam_list:
            if i.global_relationship_flag: 
                print("Camera_" + i.ID + ": Found")
            else: print("Camera_" + i.ID + ": Missing")
        print('----------------------')
        print(updt_msg)
        if count != 0:
            print('current sample:', count)

    def global_relations_work(self, b_wrt_a, length, aruco_size, id_a, id_b, path, num_samp):
        test = 0
        updt_msg = 'Looking for matches'
        match = False
        cam_a_index = None
        cam_b_index = None
        cams_in_relation = []
        t_list = []
        while True:
            self.print_updt_msg(updt_msg, 0)
            if match:
                updt_msg = 'Match found between camera ' + self.cam_list[cams_in_relation[0]].ID + ' and camera ' + self.cam_list[cams_in_relation[1]].ID
                self.print_updt_msg(updt_msg, 0)
                match = False
                count = 0
                cam_a_flag = self.cam_list[cams_in_relation[0]].global_relationship_flag
                cam_b_flag = self.cam_list[cams_in_relation[1]].global_relationship_flag
                while(count <= num_samp):
                    self.print_updt_msg(updt_msg, count)
                    time.sleep(0.1)
                    flag_a, _, ids_a, t_vects_a, r_vects_a = self.detect_arucos(self.cam_list[cams_in_relation[0]], aruco_size)
                    flag_b, _, ids_b, t_vects_b, r_vects_b = self.detect_arucos(self.cam_list[cams_in_relation[1]], aruco_size)
                    if flag_a and flag_b:
                        if ((id_a in ids_a) and not (id_b in ids_a)) and ((id_b in ids_b) and not (id_a in ids_b)):
                            count += 1
                            index_a = np.where(ids_a == id_a)
                            index_b = np.where(ids_b == id_b)
                            tmat_a = self.vect_to_tmat(t_vects_a[index_a][0], r_vects_a[index_a][0])
                            tmat_b = self.vect_to_tmat(t_vects_b[index_b][0], r_vects_b[index_b][0])
                            t_list.append(self.extrap(tmat_a, tmat_b, cam_a_flag, cam_b_flag, cams_in_relation[0], cams_in_relation[1], length))
                        elif ((id_b in ids_a) and not (id_a in ids_a)) and ((id_a in ids_b) and not (id_b in ids_b)):
                            count += 1
                            index_a = np.where(ids_a == id_b)
                            index_b = np.where(ids_b == id_a)
                            tmat_a = self.vect_to_tmat(t_vects_a[index_a][0], r_vects_a[index_a][0])
                            tmat_b = self.vect_to_tmat(t_vects_b[index_b][0], r_vects_b[index_b][0])
                            t_list.append(self.extrap(tmat_a, tmat_b, cam_a_flag, cam_b_flag, cams_in_relation[0], cams_in_relation[1], length))
                if count >= num_samp:
                    if cam_a_flag:
                        a = self.avg_tmat(t_list)
                        np.savetxt(path + self.cam_list[cams_in_relation[1]].ID + "/global_relation.txt", a)
                        self.cam_list[cams_in_relation[1]].global_relationship_flag = True
                        self.cam_list[cams_in_relation[1]].global_relationship = a
                    elif cam_b_flag:
                        a = self.avg_tmat(t_list)
                        np.savetxt(path + self.cam_list[cams_in_relation[0]].ID + "/global_relation.txt", a)
                        self.cam_list[cams_in_relation[0]].global_relationship_flag = True
                        self.cam_list[cams_in_relation[0]].global_relationship = a
                updt_msg = 'Looking for matches'
                t_list = []

            else:
                tmat_a_flag = False
                tmat_b_flag = False
                cam_a_flag = False
                cam_b_flag = False
                bool = False
                for location, camera in enumerate(self.cam_list):
                    if tmat_a_flag and tmat_b_flag: break
                    flag, frame, ids, t_vects, r_vects = self.detect_arucos(camera, aruco_size)
                    #frame = cv.resize(frame, (640, int(640 / camera.aspect_ratio)))
                    #cv.imshow("Cam_" + camera.ID, frame)
                    if flag:
                        if (id_a in ids) and not (id_b in ids):
                            index = np.where(ids == id_a)
                            t = t_vects[index]
                            r = r_vects[index]
                            tmat_a = self.vect_to_tmat(t[0], r[0])
                            tmat_a_flag = True
                            cam_a_flag = camera.global_relationship_flag
                            cam_a_index = location
                            #print("TMAT A: ")
                            #print(tmat_a)

                        elif (id_b in ids) and not (id_a in ids):
                            index = np.where(ids == id_b)
                            t = t_vects[index]
                            r = r_vects[index]
                            tmat_b = self.vect_to_tmat(t[0], r[0])
                            tmat_b_flag = True
                            cam_b_flag = camera.global_relationship_flag
                            cam_b_index = location
                            #print("TMAT B: ") 
                            #print(tmat_b)
                if tmat_a_flag and tmat_b_flag and cam_a_flag and not cam_b_flag:
                    match = True 
                    cams_in_relation = [cam_a_index, cam_b_index]
                    t_list.append(self.extrap(tmat_a, tmat_b, cam_a_flag, cam_b_flag, cam_a_index, cam_b_index, length))
                elif tmat_a_flag and tmat_b_flag and cam_b_flag and not cam_a_flag:
                    match = True
                    cams_in_relation = [cam_a_index, cam_b_index]
                    t_list.append(self.extrap(tmat_a, tmat_b, cam_a_flag, cam_b_flag, cam_a_index, cam_b_index, length))

            counts = 0
            for camera in self.cam_list:
                if camera.global_relationship_flag:
                    counts += 1

            if counts is len(self.cam_list):
                cv.destroyAllWindows()
                print("Calibration Complete...")
                self.plot_results()
                return
        time.sleep(1)

    def extrap(self, tmat_a, tmat_b, cam_a_flag, cam_b_flag, cam_a_index, cam_b_index, length):
        if cam_a_flag and not cam_b_flag: #id_a is in a camera with a global relationship while id_b is in a camera w/o a global relationship
            b_wrt_a = np.identity(4)
            b_wrt_a[1, 3] = length
            global_relation = self.cam_list[cam_a_index].global_relationship @ tmat_a @ b_wrt_a @ np.linalg.inv(tmat_b)

        elif cam_b_flag and not cam_a_flag: #id_b is in a camera with a global relationship while id_a is in a camera w/o a global relationship
            a_wrt_b = np.identity(4)
            a_wrt_b[1, 3] = length*-1
            global_relation = self.cam_list[cam_b_index].global_relationship @ tmat_b @ a_wrt_b @ np.linalg.inv(tmat_a)

        return global_relation

    def plot_results(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection = '3d')
        x = np.empty((0,3), float)
        y = np.empty((0,3), float)
        z = np.empty((0,3), float)
        for camera in self.cam_list:
            t_mat = camera.global_relationship
            t_vect, r_mat = self.tmat_to_comp(t_mat)
            t = t_vect[0]
            x_vect = np.array([[1],[0],[0]])
            y_vect = np.array([[0],[1],[0]])
            z_vect = np.array([[0],[0],[1]])
            x_vect = r_mat @ x_vect
            y_vect = r_mat @ y_vect
            z_vect = r_mat @ z_vect
            x_ax = Arrow3D([t[0],x_vect[0,0]+t[0]], [t[1],x_vect[1,0]+t[1]], [t[2],x_vect[2,0]+t[2]], mutation_scale = 10, lw = 1, arrowstyle = "-|>", color = "r")
            y_ax = Arrow3D([t[0],y_vect[0,0]+t[0]], [t[1],y_vect[1,0]+t[1]], [t[2],y_vect[2,0]+t[2]], mutation_scale = 10, lw = 1, arrowstyle = "-|>", color = "g")
            z_ax = Arrow3D([t[0],z_vect[0,0]+t[0]], [t[1],z_vect[1,0]+t[1]], [t[2],z_vect[2,0]+t[2]], mutation_scale = 10, lw = 1, arrowstyle = "-|>", color = "b")
            ax.add_artist(x_ax)
            ax.add_artist(y_ax)
            ax.add_artist(z_ax)
            label = camera.ID
            ax.text(t[0], t[1], t[2] + 0.15, label) 
            x = np.append(x, t[0])
            y = np.append(y, t[1])
            z = np.append(z, t[2])
        ax.scatter(x, y, z, c='r', marker='o')
        ax.set_xlabel('X(m)')
        ax.set_ylabel('Y(m))')
        ax.set_zlabel('Z(m)')
        ax.axes.set_zlim(0, 5)
        ax.axes.set_xlim(0, 14)
        ax.axes.set_ylim(0, 6)
        plt.show()

#------------------------------------------------------------------------------------------------------------------------------------------------

#assisting functions -----------------------------------------------------------------------#
    def detect_arucos(self, camera, aruco_size):
        frame = camera.read()
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, ids, _  = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters = self.aruco_params)
        if len(corners) > 0:
            r_vects, t_vects, mpoints = cv.aruco.estimatePoseSingleMarkers(corners, aruco_size, camera.mtx, camera.dist)
            #(r_vects-t_vects).any()
            #ids = ids.flatten()
            if self.stream_enable or self.write_enable:
                for (r, t) in zip(r_vects, t_vects):
                    cv.aruco.drawAxis(frame, camera.mtx, camera.dist, r, t, aruco_size)
            return True, frame, ids, t_vects, r_vects
        else: return False, frame, None, None, None

    def set_marker_prop(self, m_id, t_mat):
        for i in self.marker_list:
            if m_id == i.ID:
                i.set_pose(t_mat)
                return

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
                elif j == 3 and i < 3: 
                    t_mat[i, 3] = t_vect[i]
                elif i == 3 and j < 3: t_mat[3, j] = 0
                else: t_mat[3, 3] = 1
        return t_mat

    def comp_to_tmat(self, t_vect, r_mat):
        t_mat = np.zeros((4,4))
        for i in range(4):
            for j in range(4):
                if i < 3 and j < 3: t_mat[i, j] = r_mat[i, j]
                elif j == 3 and i < 3: 
                    t_mat[i, 3] = t_vect[i]
                elif i == 3 and j < 3: t_mat[3, j] = 0
                else: t_mat[3, 3] = 1
        return t_mat

    def tmat_to_comp(self, t_mat):
        r_mat = np.zeros((3, 3))
        t_vect = np.zeros((1,3))
        for i in range(3):
            for j in range(4):
                if j < 3: r_mat[i, j] = t_mat[i, j]
                elif j == 3: t_vect[0, i] = t_mat[i, 3]
        return t_vect, r_mat

    def avg_tmat(self, t_list):
        tvect_list = []
        rmat_list = []
        for t_mat in t_list:
            t_vect, r_mat = self.tmat_to_comp(t_mat)
            tvect_list.append(t_vect)
            rmat_list.append(r_mat)
        x_sum = 0
        y_sum = 0
        z_sum = 0

        for i in tvect_list:
            x_sum += i[0][0]
            y_sum += i[0][1]
            z_sum += i[0][2]

        x_avg = x_sum / len(tvect_list)
        y_avg = y_sum / len(tvect_list)
        z_avg = z_sum / len(tvect_list)
        avgd_tvect = np.array([x_avg, y_avg, z_avg])

        r_mats = np.stack(rmat_list)
        [N,_,_] = np.shape(r_mats)
        bR = np.transpose(1/N*sum(r_mats,0))
        u, s, vh = np.linalg.svd(bR, full_matrices = True)
        s = np.array([[s[0], 0, 0], [0, s[1], 0], [0, 0, s[2]]])
        vh = np.transpose(vh)
        p = vh @ s @ np.transpose(vh)
        other_u = u @ np.transpose(vh)

        if np.linalg.det(bR) > 0:
            mR = other_u @ p
        else: mR = other_u @ np.array([[1, 0, 0],[0, 1, 0],[0,0,-1]]) @ p
        
        mR = np.transpose(mR)

        avgd_tmat = self.comp_to_tmat(avgd_tvect, mR)

        return avgd_tmat

    def to_df(self, df_list, t_mat, id, clock, cID):
        t_vect, r_mat = self. tmat_to_comp(t_mat)
        t_vect = t_vect[0]
        r_x, r_y, r_z = self.rmat_to_angles(r_mat)
        for index, df in enumerate(df_list):
            if df.at[0,'Aruco_ID'] == id:
                df_list[index] = pd.DataFrame({'Aruco_ID':[id], 'Cam_ID': cID, 'Time': clock, 'X': t_vect[0], 'Y': t_vect[1], 'Z': t_vect[2], 'x_rot': r_x, 'y_rot': r_y, 'z_rot': r_z})
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

    def store_results(self, df_list, path):
        df_list = pd.concat(df_list, axis = 1)
        with open(path, 'a') as f:
            df_list.to_csv(f, header=False, index=False, line_terminator='\n')

    def clear(self):
  
        # for windows
        if os.name == 'nt':
            _ = os.system('cls')
  
        # for mac and linux(here, os.name is 'posix')
        else:
            _ = os.system('clear')

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)

class Marker:
    def __init__(self, marker_id):
        self.ID = marker_id
        self.pos_x = None
        self.pos_y = None
        self.pos_z = None
        self.rot_x = None
        self.rot_y = None
        self.rot_z = None
        self.r_mat = None

    def set_pose(self, t_mat):
        t_vect, r_vect, r_mat = self.tmat_to_comp(t_mat)
        t_vect = t_vect[0]
        self.pos_x = t_vect[0]
        self.pos_y = t_vect[1]
        self.pos_z = t_vect[2]
        self.rot_x = r_vect[0]
        self.rot_y = r_vect[1]
        self.rot_z = r_vect[2]
        self.r_mat = r_mat

    def tmat_to_comp(self, t_mat):
        r_mat = np.zeros((3, 3))
        t_vect = np.zeros((1,3))
        for i in range(3):
            for j in range(4):
                if j < 3: r_mat[i, j] = t_mat[i, j]
                elif j == 3: t_vect[0, i] = t_mat[i, 3]
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
        return t_vect, np.array([x, y, z]), r_mat
