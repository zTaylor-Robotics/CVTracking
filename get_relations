from camera import  Cam
from tracker import Track
import configparser
import json
import os
import io

configfile_name = "config_cache/relations_config.ini"

if not os.path.isfile(configfile_name):
    cfgfile = open(configfile_name, "w")

    Config = configparser.ConfigParser()
    Config.add_section('Cameras')
    Config.set('Cameras', 'number_of_cameras', '1')
    Config.set('Cameras', 'camera_id_list', '[99]')
    Config.set('Cameras', 'width', '1280')
    Config.set('Cameras', 'height', '720')

    Config.add_section('Tracking')
    Config.set('Tracking','global_aruco_id', '7')
    Config.set('Tracking','marker1_aruco_id', '15')
    Config.set('Tracking', 'marker2_aruco_id', '20')
    Config.set('Tracking', 'calib_board_Length', '.72')
    Config.set('Tracking', 'aruco_size', '0.1651')
    Config.set('Tracking', 'calib_stage', '0')

    Config.write(cfgfile)
    cfgfile.close()
    print("Config file has been added, please restart the program")
else:
    config = configparser.ConfigParser()
    config.read(configfile_name)

    cam_num = config['Cameras'].getint('number_of_cameras')
    cam_id_list = json.loads(config["Cameras"]["camera_id_list"])
    cam_width = config['Cameras'].getint('width')
    cam_height = config['Cameras'].getint('height')

    global_id = config['Tracking'].getint('global_aruco_id')
    id_a = config['Tracking'].getint('marker1_aruco_id')
    id_b = config['Tracking'].getint('marker2_aruco_id')
    length = config['Tracking'].getfloat('calib_board_length')
    aruco_size = config['Tracking'].getfloat('aruco_size')

    cam_list = []
    for i in range(cam_num):
        cam_list.append(Cam(i, cam_id_list, width = cam_width, height = cam_height))

    tr = Track(cam_list, False, False, [0])

    tr.get_global_relations(global_id,id_a,id_b,aruco_size,length,0)

    for obj in cam_list:
        obj.stop()
