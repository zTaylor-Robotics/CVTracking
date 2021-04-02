from camera import camObjThreaded
from tracker import Track
import configparser
import json
import os
import io

configfile_name = "camError_config.ini"

if not os.path.isfile(configfile_name):
    cfgfile = open(configfile_name, "w")

    Config = configparser.ConfigParser()
    Config.add_section('Camera')
    Config.set('Camera', 'camera_ID', '0')
    Config.set('Camera', 'default_camera_order', '[L,CL,C,CR,R]')
    Config.set('Camera', 'corresponding_aruco_ids', '[50,51,52,53,54]')
    Config.set('Camera', 'width', '1280')
    Config.set('Camera', 'height', '0')

    Config.add_section('camError')
    Config.set('camError','aruco_1ID', '1')
    Config.set('camError','aruco_2ID', '2')
    Config.set('camError', 'aruco_size', '0.1651')
    Config.set('camError', 'length_btw_markers', '0.762')
    Config.set('camError', 'stream_enable', 'False')
    Config.set('camError', 'write_enable', 'True')
    Config.set('camError', 'sort_enable', 'False')

    Config.write(cfgfile)
    cfgfile.close()
    print("Config file has been added, please restart the program")

else:
    config = configparser.ConfigParser()
    config.read(configfile_name)

    camID = config['Camera'].getint('camera_ID')
    camWidth = config['Camera'].getint('width')
    camHeight = config['Camera'].getint('height')
    camOrder = ['L', 'CL', 'C', 'CR', 'R']
    camIDList = json.loads(config["Camera"]["corresponding_aruco_ids"])

    A1ID = config['camError'].getint('aruco_1ID')
    A2ID = config['camError'].getint('aruco_2ID')
    L = config['camError'].getfloat('length_btw_markers')
    arucoSize = config['camError'].getfloat('aruco_size')
    stream_enable = config['camError'].getboolean('stream_enable')
    write_enable = config['camError'].getboolean('write_enable')
    sort_enable = config['camError'].getboolean('sort_enable')

    camObj = camObjThreaded(camID, arucoSize, camOrder, camIDList, width = camWidth, height = camHeight)

    tr = Track([camObj], stream_enable, write_enable, [100], targetID=100, sortEnable = sort_enable)
    tr.getCamError(A1ID, A2ID, L, arucoSize)

    camObj.stop()
