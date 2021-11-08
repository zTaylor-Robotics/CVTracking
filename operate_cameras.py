import sys
sys.path.append('./source')

from camera import Cam
import configparser
import json
import os
import cv2 as cv
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

file_name = "config_cache/operate_config.ini"

if not os.path.isfile(file_name):
    cfg_file = open(file_name, "w")

    Config = configparser.ConfigParser()
    Config.add_section('Camera')
    Config.set('Camera', 'camera_sources', '[1, 2]')
    Config.set('Camera', 'camera_id_list', '[99]')
    Config.set('Camera', 'width', '1280')
    Config.set('Camera', 'height', '720')

    Config.write(cfg_file)
    cfg_file.close()
    print("Config file has been added, please restart the program")
else:
    config = configparser.ConfigParser()
    config.read(file_name)

    cam_id_list = json.loads(config["Camera"]["camera_id_list"])
    cam_sources = json.loads(config["Camera"]["camera_sources"])
    cam_amount = config['Camera'].getint('number_of_cameras')
    cam_width = config['Camera'].getint('width')
    cam_height = config['Camera'].getint('height')

    #os.system('cls||clear')
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

    #Insert Functions Here
    # A List of Items
 #---------------------------------------------------------------------------------------------------------------------
    while True:
        for i in cam_list:
            frame = i.read()
            frame = cv.resize(frame, (240, int(240 / i.aspect_ratio)))
            cv.imshow("Cam_"+i.ID, frame)
        if cv.waitKey(1) == 13:
            cv.destroyAllWindows()
            break
    #---------------------------------------------------------------------------------------------------------------------
    for j in cam_list:
        j.stop()
    cv.waitKey(500)
    clear()
