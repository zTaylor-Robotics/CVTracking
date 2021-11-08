import sys
sys.path.append('./source')

from tracker import Track
import socket
import selectors
import threading
import pickle
import time

class lt_server:
    def __init__(self, args, time_step):
        self.time_step = time_step
        self.args = args
        self.stopped = False
        self.tr = None

    def start(self):
        self.tr = Track(self.args[0], self.args[1], self.args[2], self.args[3])
        self.tr.start_live_track(self.args[4], self.args[5])
        srvsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srvsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srvsock.bind(('', self.args[7]))
        srvsock.listen(5)
        while True:
            if self.stopped == True:
                time.sleep(1)
                srvsock.close()
                self.tr.stop_live_track()
                break
            else:
                try:
                    cli, ip = srvsock.accept()
                    threading._start_new_thread(self.NewClientSocketHandler, (cli, ip,))
                except KeyboardInterrupt:
                    self.stopped = True
                    time.sleep(1)
                    srvsock.close()
                    self.tr.stop_live_track()
            
    def NewClientSocketHandler(self, cli, ip):
        m_id = int(cli.recv(1024).decode())
        print('['+ip[0]+' @ '+str(ip[1])+']: connection established')
        print('....Sending information about marker', m_id)
        try:
            while True:
                time.sleep(self.time_step)
                if self.stopped:
                    msg = ['stop']
                    packaged_msg = pickle.dumps(msg)
                    cli.send(packaged_msg)
                    cli.close()
                    break
                else:
                    msg = self.tr.read(m_id)
                    packaged_msg = pickle.dumps(msg)
                    cli.send(packaged_msg)
        except ConnectionResetError:
            print('['+ip[0]+' @ '+str(ip[1])+']: connection terminated') 
            cli.close()

class lt_client:
    def __init__(self, host, port, id_num, time_step):
        self.time_step = time_step
        self.host = host
        self.port = port
        self.id_num = id_num
        self.data = None
        self.stopped = False
        self.clisock = None
   
    def start(self):
        self.clisock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.clisock.connect((self.host, self.port))
        threading._start_new_thread(self.update, ())

    def update(self):
        print('['+self.host+' @ '+str(self.port)+']: connection established')
        print('....Receiving information about marker', self.id_num)
        
        self.clisock.send(str(self.id_num).encode())
        try:
            while True:
                time.sleep(self.time_step)
                rcvd_msg = self.clisock.recv(4096)
                unpacked_msg = pickle.loads(rcvd_msg)
                if unpacked_msg[0] == 'stop':
                    raise ValueError
                        break
                else:
                    self.data = unpacked_msg
        except (EOFError, ValueError) as e:
            print('['+self.host+' @ '+str(self.port)+']: connection terminated')
            self.stop()
            self.clisock.close()
           
    def stop(self):
        self.stopped = True

    def read(self):
        if self.stopped: raise KeyboardInterrupt
        if self.data is None:
            return None
        else: return self.data
