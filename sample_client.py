import sys
sys.path.append('./source')

from LT_utils import lt_client

import time

#env variable to slow the loop down to prevent an infinite loop crash
TIMESTEP = 0.1
  
#Server addr and port
host = '192.168.1.13'
port = 12345

#id you wish to recieve information about
track_id = 22

other_id = 23

#create the client object, initialized with the srvvr address, port, aurco id #
client = lt_client(host, port, track_id, TIMESTEP)

second_client = lt_client(host, port, other_id, TIMESTEP)

#start the client
client.start()
second_client.start()


#Sample Event Loop, Insert desired codes within this style of event loop
try:
    while True:
        time.sleep(TIMESTEP)

        #read information about the marker from the client
        #data = [pos_x, pos_y, pos_z, rot_x, rot_y, rot_z]
        data = client.read()
        data_of_other = second_client.read()
        robots_x = data[0]
        robots_y = data[1]
        robots_z = data[2]
        
        print(data)
except KeyboardInterrupt:
    client.stop()


