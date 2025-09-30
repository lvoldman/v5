__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "1.0.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Tool"

import zmq
from zmq.devices import monitored_queue
import time, sys

from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32 
import json

class anim_0MQ:
    def __init__(self):
        self.subscriber = None
        self.publisher = None
        self.ctx = zmq.Context.instance()
        self.url = None
        self.port = None
        
        pass
    
    def __del__ (self):
        pass
    
    def _subscriber(self, url):
        try:
            self.url = url
            self.subscriber = self.ctx.socket(zmq.SUB)
            self.subscriber.connect(url)
            # self.subscriber.setsockopt(zmq.SUBSCRIBE, b"SUBSCRIBER")
            # self.subscriber.setsockopt(zmq.SUBSCRIBE, b"biosense")
            self.subscriber.subscribe("")

            print_log(f'ZMQ subscriber created. URL = {url}')
        except Exception as ex:
                e_type, e_filename, e_line_number, e_message = exptTrace(ex)
                print_log(f'ZMQ subscriber initiation failed')


    def _publisher(self, port):
        try:
            self.port = port
            self.publisher = self.ctx.socket(zmq.PUB)

            self.publisher.bind(f'tcp://*:{port}')
            # self.publisher.bind(f'tcp://127.0.0.1:{port}')    

            print_log(f'ZMQ publisher created. port = {port}')
        except Exception as ex:
                e_type, e_filename, e_line_number, e_message = exptTrace(ex)
                print_log(f'ZMQ publisher initiation failed')


    def getMsg(self):
        msg = None

        if not self.subscriber:
            print_log(f'Subscriber is not initiated')
            return None
        
        try:
            # msg = self.subscriber.recv_multipart()
            # msg = self.subscriber.recv()
            msg = self.subscriber.recv_string(flags = zmq.NOBLOCK)
        except zmq.ZMQError as ex:
            if ex.errno == zmq.ETERM:
                print_log(f'Connection is terminated. URL = {self.url}. Exception = {ex}')
            else:
                exptTrace(ex)

        except Exception as ex:
            exptTrace(ex)
            

        
        print_log(f'Received msg from {self.url} - {msg}')
        return msg
    
    def publishMsg(self, data):
        try:
            print_log(f'publishing on port {self.port} = {data}')
            # msg = f"biosense {data}".encode('utf-8')
            msg = data.encode('utf-8')
            self.publisher.send_string(str(msg), flags = zmq.NOBLOCK)

        except Exception as ex:
            exptTrace(ex)
            
    def publishJsonMsg(self, data):
        try:
            # print_log(f'publishing on port {self.port} = {data}')

            msg = json.dumps(data).encode('utf-8')
            self.publisher.send_string(str(msg), flags = zmq.NOBLOCK)

        except Exception as ex:
            exptTrace(ex)



if __name__ == "__main__":


    if (len(sys.argv) != 3) or \
        (len(sys.argv[1]) != 1) or \
        ((sys.argv[1][0].upper() != "S" ) and ((sys.argv[1][0].upper() != "P" ))) or \
        (not sys.argv[2].isdecimal()):

        print (f'Usage: python {sys.argv[0]} role port (where role is S for subscriber or P for publisher )')
        sys.exit()
        
    port = sys.argv[2]

    try:
        _Q = anim_0MQ()
        if sys.argv[1][0].upper() == "S":
            _Q._subscriber(f'tcp://127.0.0.1:{port}')
            while True:
                msg = str(_Q.getMsg())
                if msg:
                    print_log(f'msg received = {msg}')
                else:
                    print_log(f'no msg so far')
                time.sleep(1)
        else:
            _Q._publisher(port)
            while True:
                msg = str(input("Enter msg:  "))
                print_log(f'Msg to send = {msg}')
                _Q.publishMsg(msg)
                pass
    except KeyboardInterrupt:
        print('\nExiting....')
        pass

    except Exception as ex:
        print(f"Exception: {ex} of type: {type(ex)}")
        exptTrace(ex)

    
