import sys
sys.path.append('./source')

from camera import  Cam
from tracker import Track
from LT_utils import lt_server
import cv2 as cv
import configparser
import json
import os
import io


#----------[COMPONENT FUNCTIONS]---------------------------------------------------------------
def printProgressBar (iteration, total, prefix = '', suffix = '', decimals = 1, length = 100, fill = '█', printEnd = "\r"):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
        printEnd    - Optional  : end character (e.g. "\r", "\r\n") (Str)
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    if iteration == total:
        print("Success!")
    else: print("Tracker Loading...")
    print(f'\r{prefix} |{bar}| {percent}% {suffix}')
    #print("_"*77)
    #print("")
    # Print New Line on Complete
    if iteration == total: 
        print()

def clear():
  
    # for windows
    if os.name == 'nt':
        _ = os.system('cls')
  
    # for mac and linux(here, os.name is 'posix')
    else:
        _ = os.system('clear')
def sort_list(cam_list):
    count = 0
    list = []
    while len(list) != len(cam_list):
        for cam in cam_list:
            if cam.ID == str(count):
                list.append(cam)
                count += 1
    return list

#---------[CONFIG FILE READ AND WRITE]---------------------------------------------------------

configfile_name = "config_cache/LT_config.ini"

if not os.path.isfile(configfile_name):
    cfgfile = open(configfile_name, "w")

    Config = configparser.ConfigParser()
    Config.add_section('Cameras')
    Config.set('Cameras', 'camera_sources', '[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]')
    Config.set('Cameras', 'camera_id_list', '[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]')
    Config.set('Cameras', 'width', '1280')
    Config.set('Cameras', 'height', '720')

    Config.add_section('Tracking')
    Config.set('Tracking', 'object_aruco_ids', '[15]')
    Config.set('Tracking', 'aruco_size', '0.2667')
    Config.set('Tracking', 'stream_enable', 'False')
    Config.set('Tracking', 'write_enable', 'False')
    Config.set('Tracking', 'participant_id', '12345678')
    
    Config.add_section('Server')
    Config.set('Server', 'host_addr', '192.168.1.13')
    Config.set('Server', 'port', '12345')
    Config.set('Server', 'time_step', '0.1')

    Config.write(cfgfile)
    cfgfile.close()
    print("Config file has been added, please restart the program")
else:
    config = configparser.ConfigParser()
    config.read(configfile_name)

    cam_id_list = json.loads(config["Cameras"]["camera_id_list"])
    cam_sources = json.loads(config["Cameras"]["camera_sources"])
    cam_width = config['Cameras'].getint('width')
    cam_height = config['Cameras'].getint('height')

    object_aruco_ids = json.loads(config["Tracking"]["object_aruco_ids"])
    aruco_size = config['Tracking'].getfloat('aruco_size')
    stream_enable = config['Tracking'].getboolean('stream_enable')
    write_enable = config['Tracking'].getboolean('write_enable')
    participant_id = config['Tracking']['participant_id']

    host_addr = config['Server'].get('host_addr')
    port = config['Server'].getint('port')
    time_step = config['Server'].getfloat('time_step')
    #clear()

#----------[EXECUTED CODE]---------------------------------------------------------------------
    #Create a list of camera obects based on config settings
    cam_list = []
    for index, i in enumerate(cam_sources):
        cam_list.append(Cam(i, cam_id_list, width = cam_width, height = cam_height))
    
    #Sort the cam list based on camera self-identified ID#
    cam_list = sort_list(cam_list)
    
    #Create a tracking object
    args = [cam_list, stream_enable, write_enable, object_aruco_ids, participant_id, aruco_size, host_addr, port]
    #Create a Server to have clients connected to
    srvr = lt_server(args, time_step).start()

    for obj in cam_list:
        obj.stop()
    cv.waitKey(500)
    #clear()
