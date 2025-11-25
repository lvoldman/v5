__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["Leonid Voldman","Sasha Schechtman"]
__license__ = "SLA"
__version__ = "2.0.1"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Prototype"

from abc import ABC, abstractmethod

from curses.ascii import isdigit
from weakref import finalize
import serial as serial
import sys, os
import time
import threading
import ctypes
from threading import Lock
from collections import namedtuple

from inputimeout  import inputimeout , TimeoutOccurred
from dataclasses import dataclass
from queue import Queue 


from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, num2binstr, set_parm, get_parm, void_f

print_DEBUG = void_f

from ctypes import *
from ctypes import wintypes
from errorH import ErrTxt
import threading


class BaseDev(ABC):
    def __init__(self, devName:str, parms:dict):
        self._parms:dict = parms
        self._devName:str = devName
        self._dev_lock:Lock = Lock()
        self._wd:threading.Thread = None
        self.devNotificationQ:Queue = Queue()

    def __del__(self):
        pass

    @abstractmethod
    def devQuery(self, query:str, timeout:float=0)-> str:
        pass
    
    @abstractmethod
    def operateDevice(self, command:str)-> tuple(bool, bool):
        pass
    


class BaseMotor(BaseDev):
    def __init__(self, port:str, devName:str, parms:dict):
        super().__init__(devName=devName, parms=parms)
        self._mDev_port:str = port

        self._mDev_type:str = None                           # devise type (DevType.ROTATOR / DevType.GRIPPERv3 / DevType.DIST_ROTATOR / DevType.TIME_ROTATOR (SPINNER))/ DevType.DHGRIPPER
        self._mDev_pos:int = 0                               #  current position 
        self._el_current_limit:int = 0                       # electrical current limit to stop 
        self._el_current_on_the_fly:int = 0                  # On-the-fly current         
        self._wd:threading.Thread = None                     # watch dog identificator
        self._mDev_SN:str = None                                 # Serial N (0x1018:0x04)
        self._mDev_status:bool = False                            # device status (bool) / used for succesful initiation validation
        self._mDev_in_motion:bool = False                         # is device in motion
        self._possition_control_mode:bool = False                 # TRUE - control possition, FALSE - don't
        self._time_control_mode:bool = False                      # TRUE - time control
        self._mDev_pressed:bool = False                           # is motion button pressed (OBSOLET)
        self._gripper_onoff:bool = False                            # True means open
        self._start_time: float = 0                                   # Start thread time
        self._success_flag:bool = True                            # end of op flag
        self._rotationTime:float = 0                               # rotation time
        self._title:str = None

                

    def __del__(self):
        pass

    @abstractmethod
    def init_dev(self, dev_type) -> bool:
        pass

    @abstractmethod
    def  mDev_watch_dog_thread(self):
        pass

    @property
    def OnOff(self)->bool:
        return self._gripper_onoff

    def  mDev_watch_dog(self):
        self._start_time = time.time()
        print_log(f'Running whatch dog thread')
        self._wd = threading.Thread(target=self.mDev_watch_dog_thread)
        self._wd.start()
        return self._wd

    @abstractmethod
    def mDev_stop(self)-> bool:
        pass

    @abstractmethod
    def gripper_on(self)-> bool:
        pass

    @abstractmethod
    def gripper_off(self)-> bool:
        pass

    @abstractmethod
    def go2pos(self, new_position, velocity = None, stall=None)->bool:
        pass

    @abstractmethod
    def mDev_stored_pos(self)->int:
        pass

    @abstractmethod
    def  mDev_reset_pos(self)->bool:
        pass

    @abstractmethod
    def mDev_get_cur_pos(self)->int:
        pass

    @abstractmethod
    def set_parms(self, parms:dict):
        pass

    def mutualControl(self):
        if self._dev_lock.locked():
            print_err(f'ERROR- The device {self.devName} (port:{self._mDev_port}) is active. Cant allow multiply activations')

            return False
        else:
            self._dev_lock.acquire()
            return True
        
    def getTitle(self)->str:
        return self._title