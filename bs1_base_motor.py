from __future__ import annotations

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
import re

from inputimeout  import inputimeout , TimeoutOccurred
from dataclasses import dataclass, field
from queue import Queue 

import shlex
from typing import Any
import ast



from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, num2binstr, set_parm, get_parm, void_f

print_DEBUG = void_f

from ctypes import *
from ctypes import wintypes
from errorH import ErrTxt
import threading


# command in format DEV.OP, where DEV is device name, OP is operation
# like 'PHG.UV param1:val1 param2:val2'
@dataclass
class Command:
    device: str                     # device name
    op: str                        # operation name                     
    args: dict[str, Any] = field(default_factory=dict)    # parameters dictionary

    # coerce string value to int/float/bool/None/list/dict/string
    @staticmethod
    def coerce(value: str) -> Any:
        # For types int/float/bool/None/list/dict/string in quotes
        try:
            return ast.literal_eval(value)
        except Exception:
            return value  # as is (string without quotes)

    @staticmethod 
    def parse_cmd(s: str) -> Command:
        s = re.sub(r'\s*:\s*', ':', s)    # remove spaces around ':'

        CMD_STR_RE = re.compile(
            r'^(?P<dev>[A-Za-z_]\w*)\.(?P<op>[A-Za-z_]\w*)(?:\s+(?P<args>.*))?$'
        )                                   # split into DEV.OP and rest (args)

        KV_RE = re.compile(r'^(?P<key>[A-Za-z_][\w-]*):(?P<val>.*)$')
                                            # key:value pairs regex

        m = CMD_STR_RE.match(s.strip())     # match the command string

        if not m:
            raise ValueError("Expected: DEV.OP par:val ...")
        
        dev = m.group("dev")
        op  = m.group("op")
        tail = m.group("args") or ""

        # print_DEBUG(f'Parsing command: dev={dev}, op={op}, tail={tail}')

        tokens = shlex.split(s)            # supportes quoted strings
        if not tokens:                      # list is empty
            raise ValueError("Empty command string")
        head, *rest = tokens                # split an iterable into its head (DEV.OP) and rest (parameters)

        cmd_re = re.compile(r'^[A-Za-z0-9_]+\.[A-Za-z0-9_]+$')
        if not cmd_re.match(head):
            raise ValueError(f"Invalid command format: {head!r}, expected DEV.OP")  # DEV.OP format prints head representation !r  (same as repr(head))
        
        # if "." not in head:
        #     raise ValueError("Expected DEV.OP at the beginning")
        
        dev, op = head.split(".", 1)

        args: dict[str, Any] = {}

        for _token in rest:
            km = KV_RE.match(_token)
            if not km:
                raise ValueError(f"Expected key:value, got: {_token!r}")
            
            k = km.group("key")
            v = km.group("val")

            if ":" not in _token:
                raise ValueError(f"Expected key=value, got: {t!r}")
            # k, v = t.split(":", 1)


            args[k] = Command.coerce(v)

        return Command(device=dev, op=op, args=args)
    
    @staticmethod
    def validate_cmd_str(s: str) -> bool:
        try:
            Command.parse_cmd(s)
            return True
        except ValueError:
            return False



class BaseDev(ABC):
    def __init__(self, devName:str, parms:dict=None):
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
    
    # operateDevice command: str -> (blocked:bool, result:bool)
    # blocked - True if the device is in async mode and notification should be awaited
    # result - True if the command run was successful (in case of synchronous mode) or operation started successfully (in case of async mode)
    
    @abstractmethod
    def operateDevice(self, command, **kwards)-> tuple[bool, bool]:
                                                # various parameters possible
                                                # using: _command = kwards['window']
        pass
    
    # parsing the command into command name and parameters dictionary
    # all command in format DEV.OP, where DEV is device name, OP is operation
    # like 'PHG.UV param1:val1 param2=val2'
    def parseCommand(self, command:str)-> Command:
        _command =  Command.parse_cmd(command)
        if _command.device != self.devName:
            raise ValueError(f'Command device name { _command.device } does not match the device instance name { self.devName }')   
        return _command

    
    # def parseCommand(self, command:str)-> tuple[str, dict]:
    #     cmd_parts = command.split(' ')
    #     cmd_name = cmd_parts[0].split('.')[0]
    #     cmd_params = {}
    #     for param in cmd_parts[1:]:
    #         if '=' in param:
    #             key, value = param.split(':', 1)
    #             cmd_params[key] = value
    #     return cmd_name, cmd_params
    
        

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
    

#------------------------   UNITEST SECTION -----------------
if __name__ == "__main__":


    def test(_str:str):
        try:
            cmd = Command.parse_cmd(_str)
            print(f'Command parsed: device={cmd.device}, op={cmd.op}, args={cmd.args}')
        except Exception as ex:
            print(f'Exception : {ex}')
            
    print("Running bs1_base_motor.py unitest ...")
    _cmd = "PHG.UV param1:val1 param2:val2 param3:123 param4:45.67 param5:True param6:None param7:[1,2,3] param8:{'key':'value'}"
    test(_cmd)