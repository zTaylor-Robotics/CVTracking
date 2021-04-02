from camera import camObjThreaded
from tracker import Track
import configparser
import json
import os
import io

configfile_name = "track_config.ini"

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
    Config.set('Tracking','object_aruco_ids', '[20, 21]')
    Config.set('Tracking','target_aruco_id', '20')
    Config.set('Tracking', 'aruco_size', '0.1651')
    Config.set('Tracking', 'stream_enable', 'False')
    Config.set('Tracking', 'write_enable', 'True')


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

    robotArucoIDs = json.loads(config["Tracking"]["object_aruco_ids"])
    targetID = config['Tracking'].getint('target_aruco_id')
    arucoSize = config['Tracking'].getfloat('aruco_size')
    stream_enable = config['Tracking'].getboolean('stream_enable')
    write_enable = config['Tracking'].getboolean('write_enable')

    camObjList = []
    for i in camList:
        camObjList.append(camObjThreaded(i, arucoSize, camOrder, camIDList, width = camWidth, height = camHeight))

    tr = Track(camObjList, stream_enable, write_enable, robotArucoIDs, targetID=targetID)

    input("Press [Enter] to Begin")
    while True:
        trialName = input("Please input trial name: ")
        tr.projectTracker(trialName, arucoSize)
        temp = input("Would you like to continue? [Y or N]: ")
        if temp != 'Y' and temp != 'y':
            break
    print("#---------------[Trials Have Concluded]---------------")
    for obj in camObjList:
        obj.stop()
