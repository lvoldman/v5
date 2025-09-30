__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "1.0.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Tool"


import re
from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, set_parm, get_parm, assign_parm
from bs1_ni6002 import NI6002
import threading
import PySimpleGUI as sg

import sys, time

DEFAULT_TIMEOUT = 1

class IOcontrol:
    # blue/brown pair: Normally closed, 0->1 when the door is opened 
    # def __init__(self, _dname:str, _device, parms:dict,  __NO = True):
    def __init__(self, _dname:str, _device, __line:int, __port:int, parms:dict, __NO:bool = True):
        self.__port:int = __port                 # default
        self.__line:int = __line
        self.__device = _device
        self.devName:str = _dname
        self.wd = None
        self.stop = True
        self.__opDev = None
        self.__window:sg.Window = None
        self.__stored_value:bool = False                     
        self.__pollingTimeout = DEFAULT_TIMEOUT
        self.__NO:bool =  __NO                               # NO/NC, __NO = TRUE->NO, __NO = FALSE->NC

        self.set_parms(parms)

        try:
            self.wd = threading.Thread(target=self.__opThread)
            self.wd.start()
        
        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'ERROR starting interlock thread. Exception: {ex} of type: {type(ex)}.')

    def __repr__(self):
        return f'I/O control: port{self.__port}.line{self.__line}, run on dev = {self.__device}, name = {self.devName} '

    def getIODev(self)->str:
        return self.__device    

    def set_parms(self, parms):       
        if len(parms) == 0:
            return
        
        self.__pollingTimeout = assign_parm(self.__device, parms, 'TIMEOUT', DEFAULT_TIMEOUT)

        print_log(f'I/O control {self.devName} on dev of type {self.__device} defined on port={self.__port}, line={self.__line} ,type = {"NO" if self.__NO else "NC" }')

    def startOp(self, window:sg.Window, opDev:NI6002):
        self.__window = window
        self.__opDev = opDev
        print_log(f'Starting operation on IO device = {self}')
        _res, self.__stored_value = opDev.getSignal(self.__port, self.__line)
        self.__window.write_event_value(f'-{self.devName}-', self.__stored_value)   
                                                                    # check status when started

    def __opThread(self):
        self.stop = False
        print_log(f'Starting watchdog thread for I/O control {self.devName} dev of type {self.__device} (port{self.__port}.line{self.__line})')
        while not self.stop and time.sleep(self.__pollingTimeout) is None:
            if self.__opDev is None or self.__window is None:          # Op is not armed
                continue
            
            _res, _signal = self.__opDev.getSignal(self.__port, self.__line)

            if not _res:
                print_err(f'Failed read signal value. I/O control={self.devName} dev={self.__device} (port{self.__port}.line{self.__line})')
                continue

            if not (_signal == self.__stored_value):                    # status changed
                print_log(f'I/O control = {self.devName} dev={self.__device} (port{self.__port}.line{self.__line}): {self.__stored_value} -> {_signal}')
                self.__stored_value = _signal
                self.__window.write_event_value(f'-{self.devName}-', _signal)
            

        print_log(f'Exiting watchdog thread for I/O control={self.devName} dev of type {self.__device} (port{self.__port}.line{self.__line})')
            

    def getStatus(self) ->bool:             # according to NO/NC setting
                                            # False when NO and signal is 0 // NC and signal is 1
                                            # True when NO and signal is 1 // NC and signal is 0
        return True if self.__NO and self.__stored_value or not self.__NO and not self.__stored_value else False
    
    def stopOp(self):
        print_log(f'Stopping IO control on : {self} / {self.devName} dev of type {self.__device} (port{self.__port}.line{self.__line})')
        self.__opDev = None
        self.__window = None

    def mDev_stop(self):
        self.stopOp()
        self.stop = True
        

    def __del__(self):
        self.mDev_stop()
        time.sleep(self.__pollingTimeout)



#---------------  UNIT TEST------------------
if __name__ == "__main__":
    import re
    

    if not len(sys.argv) == 2:
        print_log(f'Usage: python {sys.argv[0]} S/N')
        sys.exit()
    _re =  re.compile(r'[0-9a-fA-F]+')
    if not _re.match(sys.argv[1]):
        print_log (f'Invalid S/N: {sys.argv[1]}')
        sys.exit()


    _ni = NI6002('test', int(sys.argv[1]), dict())
    if _ni == None:
        print_err(f'Error initiating NI: {_ni}')
        sys.exit()

    _pi = InterLock('P-test', 'DAQ NI', dict())
    if _pi == None:
        print_err(f'Error initiating Interlock: {_pi}')
        sys.exit()

    _pi.startOp(None, _ni, 0, 1)

    try:
        while True:
            pass

    except KeyboardInterrupt as ex:
        print(f'Exiting by ^C \n{ex}')
        _pi.stopOp()
        sys.exit()
    except Exception as ex:
        print(f'Exception operation error on {ex}')   
