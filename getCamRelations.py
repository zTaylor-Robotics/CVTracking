from camera import camObjThreaded
from tracker import Track
import configparser
import json
import os
import io

configfile_name = "relations_config.ini"

if not os.path.isfile(configfile_name):
    cfgfile = open(configfile_name, "w")

    Config = configparser.ConfigParser()
    Config.add_section('Cameras')
    Config.set('Cameras', 'camera_list', '[0, 1, 2, 3, 4]')
    Config.set('Cameras', 'default_camera_order', '[L,CL,C,CR,R]')
    Config.set('Cameras', 'corresponding_aruco_ids', '[50,51,52,53,54]')
    Config.set('Cameras', 'width', '1280')
    Config.set('Cameras', 'height', '0')

    Config.add_section('Tracking')
    Config.set('Tracking','global_aruco_id', '7')
    Config.set('Tracking','marker1_aruco_id', '15')
    Config.set('Tracking', 'marker2_aruco_id', '20')
    Config.set('Tracking', 'calib_board_Length', '.72')
    Config.set('Tracking', 'Calib_method_[1 or 2]', '1')
    Config.set('Tracking', 'aruco_size', '0.1651')

    Config.write(cfgfile)
    cfgfile.close()
    print("Config file has been added, please restart the program")
else:
    config = configparser.ConfigParser()
    config.read(configfile_name)

    camList = json.loads(config["Cameras"]["camera_list"])
    camWidth = config['Cameras'].getint('width')
    camHeight = config['Cameras'].getint('height')
    camOrder = ['L', 'CL', 'C', 'CR', 'R']
    camIDList = json.loads(config["Cameras"]["corresponding_aruco_ids"])

    globalArucoID = config['Tracking'].getint('global_aruco_id')
    A1ID = config['Tracking'].getint('marker1_aruco_id')
    A2ID = config['Tracking'].getint('marker2_aruco_id')
    L = config['Tracking'].getfloat('calib_board_length')
    calibMethod = config['Tracking'].getint('Calib_method_[1 or 2]')
    arucoSize = config['Tracking'].getfloat('aruco_size')

    camObjList = []
    for i in camList:
        camObjList.append(camObjThreaded(i, arucoSize, camOrder, camIDList, width = camWidth, height = camHeight))

    tr = Track(camObjList, False, False, [0], targetID=0)

    tr.getCameraRelations2Global(globalArucoID,A1ID,A2ID,arucoSize,L, calibMethod)

    for obj in camObjList:
        obj.stop()
