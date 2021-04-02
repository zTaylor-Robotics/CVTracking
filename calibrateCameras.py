from camera import camObjThreaded
from tracker import Track
import configparser
import json
import os
import io

configfile_name = "calib_config.ini"

if not os.path.isfile(configfile_name):
    cfgfile = open(configfile_name, "w")

    Config = configparser.ConfigParser()
    Config.add_section('Cameras')
    Config.set('Cameras', 'cameras_to_calibrate', '[0, 1, 2, 3, 4]')
    Config.set('Cameras', 'default_camera_order', '[L,CL,C,CR,R]')
    Config.set('Cameras', 'corresponding_aruco_ids', '[50,51,52,53,54]')
    Config.set('Cameras', 'aruco_size', '0.1651')
    Config.set('Cameras', 'width', '1280')
    Config.set('Cameras', 'height', '0')

    Config.write(cfgfile)
    cfgfile.close()
    print("Config file has been added, please restart the program")
else:
    config = configparser.ConfigParser()
    config.read(configfile_name)

    camList = json.loads(config["Cameras"]["cameras_to_calibrate"])
    camWidth = config['Cameras'].getint('width')
    camHeight = config['Cameras'].getint('height')
    camOrder = ['L', 'CL', 'C', 'CR', 'R']
    camIDList = json.loads(config["Cameras"]["corresponding_aruco_ids"])
    arucoSize = config['Cameras'].getfloat('aruco_size')

    camObjList = []
    for i in camList:
        camObjList.append(camObjThreaded(i, arucoSize, camOrder, camIDList, width = camWidth, height = camHeight))

    for j in camObjList:
        j.calibrate()

    for obj in camObjList:
        obj.stop()
