__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "5.0.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Tool"


from abc import ABC, abstractmethod
import sys, time
import re
import threading
from enum import Enum


from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, set_parm, get_parm, assign_parm
from bs1_base_motor import BaseDev


DEFAULT_TIMEOUT = 1
from bs1_ni6002 import NI6002

# GPIO_Dev class is a base class for GPIO devices like NI DAQ, NCT6102D, PLC IO modules etc
# It defines the basic GPIO operations interface
# Each of the derived classes should implement the methods below
# GPIO_Dev derived classes should be able to implement the low level device specific commands   
# The GPIOcontrol class uses GPIO_Dev derived class object to operate a single IO line on the given device
# No dirrect device commands should be implemented in GPIOcontrol class
# GPIO_Dev derived classes have no dirrect inteface open for user (via GUI or script), only via GPIOcontrol class objects
class GPIO_Dev(BaseDev):

    def __init__(self, devName:str, parms:dict=None):
        super().__init__(devName, parms)


    def devQuery(self, query:str, timeout:float=0)-> str:
        pass
    
    # operateDevice command: str -> (result:bool, blocked:bool )
    # blocked - True if the device is in async mode and notification should be awaited
    # result - True if the command run was successful (in case of synchronous mode) or operation started successfully (in case of async mode)
    def operateDevice(self, command:str, **kwards)-> tuple[bool, bool]:
                                                # various parameters possible
                                                # using: _command = kwards['window']
        pass

    # each of the methods below should return (result:bool, blocked:bool)
    @abstractmethod
    def setIO(self, port:int, line:int, value:bool) -> tuple[bool, bool]:
        pass

    # getIO return input gpIO value (value:bool, blocked:bool)
    @abstractmethod
    def getIO(self, port:int, line:int) -> tuple[bool, bool]:
        pass

    # blocking wait for IO line event: line state or state change according to waitingMethod
    @abstractmethod
    def waitForIO(self, port:int, line:int, waitingMethod:waitingType) -> tuple[bool, bool]:
        pass

''' GPIOcontrol class
    I/O control object representing a single input/ output line on a given device 

    
    
'''
class GPIOcontrol(BaseDev):
    class waitingType(Enum):
        waitL = 0                   # wait for FALSE signal on line(low)
        waitRE = 1                  # wait for RISING EDGE on line
        waitFE = 2                  # wait for FALLING EDGE on line
        waitH = 3                # wait for TRUE signal on line (high)
    class IOType(Enum):
        GPI = 0             # input
        GPO = 1             # output

    # device: 'DAQ NI'  // 'GPIO NCT6102D' // PLC SingleIoControl // 'PLC Tc_IOSequencer' // 'Virtual IO' // etc
    def __init__(self, _dname:str, _device:GPIO_Dev, __line:int, __port:int, parms:dict, _io:IOType=IOType.GPI, __NO:bool = True):
        super().__init__(devName=_dname, parms=parms)
        self.__io:GPIOcontrol.IOType = _io
        self.__port:int = __port                 # default
        self.__line:int = __line
        self.__device: GPIO_Dev = _device
        self.wd = None
        self.stop: threading.Event = threading.Event()
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
        return f'I/O control: port{self.__port}.line{self.__line}, run on dev = {self.__device}, name = {self.devName}, NO/NC = {"NO" if self.__NO else "NC" }, IO type = {self.__io.name} '

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
        self.stop.clear()
        print_log(f'Starting watchdog thread for I/O control {self.devName} dev of type {self.__device} (port{self.__port}.line{self.__line})')
        while not self.stop.is_set() and time.sleep(self.__pollingTimeout) is None:
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
        self.stopOp()                   # stop operation
        self.stop.set()                 # stop thread
        if self.wd is not None:         # wait for thread termination    
            self.wd.join()
            self.wd = None  
    
    def getGPIValue(self)-> bool:
        return self.__stored_value

    def setGPOValue(self, value:bool)-> bool:
        _res, _blocked = self.__device.setIO(self.__port, self.__line, value)
        if not _res:
            print_err(f'Failed set GPO value={value} on I/O control={self.devName} dev={self.__device} (port{self.__port}.line{self.__line})')
            return False
        return True
    

    
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
