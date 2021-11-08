import sys
sys.path.append('./source')

from camera import  Cam
from tracker import Track
import cv2 as cv
import configparser
import json
import os
import io

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
    else: print("Loading...")
    print(f'\r{prefix} |{bar}| {percent}% {suffix}')
    print("_"*77)
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
    
items = list(range(0, 9))

configfile_name = "config_cache/relations_config.ini"

if not os.path.isfile(configfile_name):
    cfgfile = open(configfile_name, "w")

    Config = configparser.ConfigParser()
    Config.add_section('Cameras')
    Config.set('Cameras', 'camera_sources', '[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]')
    Config.set('Cameras', 'camera_id_list', '[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]')
    Config.set('Cameras', 'width', '1280')
    Config.set('Cameras', 'height', '720')

    Config.add_section('Tracking')
    Config.set('Tracking','global_aruco_id', '7')
    Config.set('Tracking','marker1_aruco_id', '15')
    Config.set('Tracking', 'marker2_aruco_id', '20')
    Config.set('Tracking', 'calib_board_Length', '.72')
    Config.set('Tracking', 'aruco_size', '0.1651')
    Config.set('Tracking', 'number_of_samples', '15')
    Config.set('Tracking', 'calib_stage', '0')
    Config.set('Tracking', 'cameras_to_change', '[]')

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

    global_id = config['Tracking'].getint('global_aruco_id')
    id_a = config['Tracking'].getint('marker1_aruco_id')
    id_b = config['Tracking'].getint('marker2_aruco_id')
    length = config['Tracking'].getfloat('calib_board_length')
    aruco_size = config['Tracking'].getfloat('aruco_size')
    num_samp = config['Tracking'].getint('number_of_samples')
    stage = config['Tracking'].getint('calib_stage')
    cameras_to_change = json.loads(config["Tracking"]["cameras_to_change"])

    clear()
    cam_list = []
    for index, i in enumerate(cam_sources):
        clear()
        printProgressBar(index, len(cam_sources), prefix = 'Progress:', suffix = 'Complete', length = 50)
        cam_list.append(Cam(i, cam_id_list, width = cam_width, height = cam_height))
        cv.waitKey(250)
    clear()
    printProgressBar(len(cam_sources), len(cam_sources), prefix = 'Progress:', suffix = 'Complete', length = 50)
    cv.waitKey(250)
    cam_list = sort_list(cam_list)
    clear()

    tr = Track(cam_list, False, False, [0])

    tr.get_global_relations(global_id, id_a, id_b, aruco_size, length, num_samp, stage, cameras_to_change)

    for obj in cam_list:
        obj.stop()
    cv.waitKey(500)
    clear()
