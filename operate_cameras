from camera import Cam
import configparser
import json
import os
import io

file_name = "config_cache/operate_config.ini"

if not os.path.isfile(file_name):
    cfg_file = open(file_name, "w")

    Config = configparser.ConfigParser()
    Config.add_section('Camera')
    Config.set('Camera', 'number_of_cameras', '1')
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
    cam_amount = config['Camera'].getint('number_of_cameras')
    cam_width = config['Camera'].getint('width')
    cam_height = config['Camera'].getint('height')

    cam_list = []
    for i in range(cam_amount):
        cam_list.append(Cam(i, cam_id_list, width = cam_width, height = cam_height))

    #Insert Functions Here
    #---------------------------------------------------------------------------------------------------------------------
    cam_list[0].show_feed()
    #cam_list[1].show_feed()
    print(cam_list[0].aspect_ratio)
    #---------------------------------------------------------------------------------------------------------------------
    for j in cam_list:
        j.stop()
