from __future__ import annotations

__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["Leonid Voldman","Sasha Schechtman"]
__license__ = "SLA"
__version__ = "5.0.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Prototype"


from abc import ABC, abstractmethod

from curses.ascii import isdigit
from weakref import finalize
import serial as serial

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
from enum import Enum

class DevType(Enum):
    TIME_ROTATOR = 'SP'
    DIST_ROTATOR = 'RT'
    TROLLEY = 'TR'
    GRIPPER = 'OBSOLETE'            # replaced by GRIPPERv3
    GRIPPERv3 = 'GR'
    ZABER = 'ZB'                    # stepper actuator
    DAQ_NI = 'DAQ_NI'
    HMP = 'HMP'
    PHG = 'PHG'
    CAM = 'CAM'
    DH = 'DH'
    MCDMC = 'MCDMC'
    MARCO = 'MARCO'
    INTERLOCK = 'INTERLOCK'
    IOCONTROL = 'IOCONTROL'
    JTSE = 'JTSE'
    DB = 'DB'
    PLCDEV = 'PLCDEV'
    SYS = 'SYS'

class pType(Enum):
    OP:str = 'OP'          # operation
    PARM:str = 'PARM'         # parameter
    RET:str = 'RET'          # return value

# device command configuration dictionary type
# devCmdCnfg: devType -> { 'OP': [op1, op2, ...], 'PARM': [parm1, parm2, ...] }, wher OP is an allowed operation list, 
# PARM is a possible parameters list

devCmdCnfg: dict[DevType, dict[pType, list[str]]] = {
    DevType.TROLLEY: {
        pType.OP: ['ML', 'MR', 'HO', 'REL', 'STALL'],
        pType.PARM: ['position', 'velocity'],
        pType.RET: ['success', 'message', 'position']
    },
    DevType.ZABER: {
        pType.OP: ['MA', 'MR', 'HO', 'VIBRATE'],
        pType.PARM: ['position', 'velocity', 'distance'],
        pType.RET: ['success', 'message', 'position']
    },
    DevType.CAM: {
        pType.OP: ['CHECK'],
        pType.PARM: ['profile'],
        pType.RET: ['success', 'message', 'action']
    },
    DevType.DB: {
        pType.OP: ['ADD', 'RESET'],
        pType.PARM: ['good'],
        pType.RET: ['success', 'message', 'good', 'bad']
    },
    DevType.DH: {
        pType.OP: ['MA', 'MR', 'HO','OPEN', 'CLOSE'],
        pType.PARM: ['position', 'distance', 'velocity'],
        pType.RET: ['success', 'message', 'state', 'position']
    },
    DevType.GRIPPERv3: {
        pType.OP: ['OPEN', 'CLOSE'],
        pType.PARM: [],
        pType.RET: ['success', 'message', 'state']
    },
    DevType.JTSE: {
        pType.OP: ['UPDATE'],
        pType.PARM: [],
        pType.RET: ['success', 'message', 'temp']
    },
    DevType.MCDMC: {
        pType.OP: ['PUT', 'PUTPCB', 'INSERTPCB', 'MOVESAFE', 'MOVEREL', 'MOVEABS', 'VACUUM', 'ACTIVATE', 'DEACTIVATE', 'VALIDATE' ],
        pType.PARM: ['p3d', 'vacuum', 'duration', 'velocity', 'x', 'y', 'z', 'alpha', 'beta', 'gamma'],
        pType.RET: ['success', 'message', 'x', 'y', 'z', 'alpha', 'beta', 'gamma', 'vacuum', 'valid_products']
    },
    DevType.PHG: {
        pType.OP: ['*'],            # dev name defined in params configuration 
        pType.PARM: ['onoff'],
        pType.RET: ['success', 'message', 'state', 'device']
    }, 
    DevType.DIST_ROTATOR: {
        pType.OP: ['MA', 'ML', 'MR', 'MLC', 'MRC', 'HO', 'REL'],            
        pType.PARM: ['position', 'velocity', 'duration', 'gpio_quickstop_polarity'],
        pType.RET: ['success', 'message', 'state', 'device']
    },
    DevType.TIME_ROTATOR: {
        pType.OP: ['ML', 'MR'],
        pType.PARM: ['duration'],
        pType.RET: ['success', 'message', 'position']
    },
}   
# parameter type coercion dictionary

vType: dict[str, type] = {
    'velocity': float,
    'position': float,
    'distance': float,
    'success': bool,
    'message': str,
    'action': int,
    'profile': str,
    'temp': float,
    'p3d': str,
    'vacuum': float,
    'duration': float,
    'x': float, 
    'y': float,
    'z': float,
    'alpha': float,
    'beta': float,
    'gamma': float,
    'valid_products': int,
    'good': bool,
    'bad': bool,
    'state': str,
    'device': str,
    'onoff': str,       # "" for trigger/toggle | "ON" | "OFF"
    'NoneType': type(None)
}

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
            return ast.literal_eval(value)   # safe evaluation of string to Python literal
                                            # all other (non-literals) will be interpreted as string
        except Exception:
            return value  # as is (string without quotes)

    @staticmethod 
    def parse_cmd(com_str: str) -> Command:
        com_str = re.sub(r'\s*:\s*', ':', com_str)    # remove spaces around ':'

        CMD_STR_RE = re.compile(
            r'^(?P<dev>[A-Za-z_]\w*)\.(?P<op>[A-Za-z_]\w*)(?:\s+(?P<args>.*))?$'
        )                                   # split into DEV.OP and rest (args)
                                            # capturing groups:
                                            # dev - device name
                                            # op  - operation name
                                            # args - rest of the string (parameters)

        KV_RE = re.compile(r'^(?P<key>[A-Za-z_][\w-]*):(?P<val>.*)$')
                                            # capturing groups: 
                                            # key:value pairs regex

        _match_groups = CMD_STR_RE.match(com_str.strip())     # match the command string

        if not _match_groups:
            raise ValueError('Expected: DEV.OP par:val ...')
        
        dev = _match_groups.group('dev')
        op  = _match_groups.group('op')
        tail = _match_groups.group('args') or ''

        # dev, op = head.split('.', 1)

        # print_DEBUG(f'Parsing command: dev={dev}, op={op}, tail={tail}')

        tokens = shlex.split(com_str)            # supportes quoted strings
        if not tokens:                      # list is empty
            raise ValueError('Empty command string')
        head, *rest = tokens                # split an iterable into its head (DEV.OP) and rest (parameters)

        print_DEBUG(f'Parsing command: head={head}, rest={rest}, tail={tail}')

        cmd_re = re.compile(r'^[A-Za-z0-9_]+\.[A-Za-z0-9_]+$')
        if not cmd_re.match(head):
            raise ValueError(f'Invalid command format: {head!r}, expected DEV.OP')  # DEV.OP format prints head representation !r  (same as repr(head))
        
        # if '.' not in head:
        #     raise ValueError('Expected DEV.OP at the beginning')
        

        args: dict[str, Any] = {}

        for _token in rest:                 # parse key:value pairs
            km = KV_RE.match(_token)
            if not km:                      # invalid format
                raise ValueError(f'Expected key:value, got: {_token!r}')
            
            _parm = km.group('key')         # parameter name
            _val = km.group('val')          # parameter value
            # _parm, _val = _token.split(':', 1)

            if ':' not in _token:
                raise ValueError(f'Expected key=value, got: {_token!r}')

            args[_parm] = Command.coerce(_val)

        return Command(device=dev, op=op, args=args)
    
    @staticmethod
    def validate_cmd_str(com_str: str) -> bool:
        try:
            Command.parse_cmd(com_str)
            return True
        except ValueError:
            return False
    
    @staticmethod
    def validate_device_cmd(dType:DevType, cmd: Command,  parms:dict=None) -> bool:
        try:
            device_type: DevType = DevType(dType)
            print_log(f'Validating command {cmd} for device type {device_type}')
            if  cmd.op not in devCmdCnfg[device_type][pType.OP] and \
                '*' not in devCmdCnfg[device_type][pType.OP]:   # wildcard for any operation    

                raise ValueError(f'Operation {cmd.op} not allowed for device type {device_type}')
            
            for parm in cmd.args.keys():            # check that all parameters are allowed
                if parm not in devCmdCnfg[device_type][pType.PARM]:
                    raise ValueError(f'Parameter {parm} not allowed for device type {device_type}')
                
                if cmd.args[parm] is not None:    # check type coercion
                    expected_type = vType.get(parm, str)   # default to str if type not found

                    if type(cmd.args[parm]) == int and expected_type == float:
                        cmd.args[parm] = float(cmd.args[parm])   # allow int to float conversion

                    if not isinstance(cmd.args[parm], expected_type):
                        raise ValueError(f'Parameter {parm} expected type {expected_type}, got {type(cmd.args[parm])} in command {cmd} for device type {device_type}')
            # device specific validation
            if device_type == DevType.CAM and cmd.op == 'CHECK':
                profile = cmd.args.get('profile', '')
                if parms is not None and cmd.device in parms and 'profiles' in parms[cmd.device]:
                    allowed_profiles = parms[cmd.device]['profiles']
                    if profile not in allowed_profiles:
                        raise ValueError(f'Camera profile {profile} not in allowed profiles {allowed_profiles}')
                
                else:
                    raise ValueError(f'No appropriate profiles configuration provided for camera device validation')
                
            if device_type == DevType.MCDMC:
                if '3DPOSITIONS' in parms[cmd.device].keys() and (_coord3D := parms[cmd.device]['3DPOSITIONS']) is not None:
                    _subcmd = cmd.op
                    print_log(f'cmd = {cmd}, subcmd = {_subcmd}')
                    if  (_subcmd == 'MOVEABS' or _subcmd == 'MOVESAFE') and  cmd.args.get('p3d', '') not in _coord3D.keys():
                        raise ValueError(f"ERROR: Invalid POSITION in MCDMC MOVEABS/MOVESAFE/PUT  - {cmd.args.get('p3d', '')} is not listed in {_coord3D}. Cmd = {cmd}")
                    elif  _subcmd == 'PICKUP' and not (cmd.args.get('p3d', '') in _coord3D.keys() or cmd.args.get('p3d', '') == 'ASYRIL' or cmd.args.get('p3d', '') == 'COGNEX') :
                        raise ValueError(f"ERROR: Invalid POSITION in PICKUP  - {cmd.args.get('p3d', '')} is not listed in {_coord3D}. Cmd = {cmd}")
                    elif (_subcmd == 'PUTPCB' or _subcmd == 'PUT') and not cmd.args.get('p3d', '') in _coord3D.keys() and not _put_re.match(cmd):
                        raise ValueError(f"ERROR: Invalid POSITION in PUTPCB/PUT  - {cmd.args.get('p3d', '')} is not listed in {_coord3D}. Cmd = {cmd}")
                    else:
                        print_log (f" Cmd - {cmd}. Subcmd - {cmd.op} , list - {_coord3D.keys()}. ")

                    
        except ValueError as ve:
            print_err(f'Command validation failed: {cmd} for device type {device_type}: {ve}')
            return False
    
        except Exception as ex:
            exptTrace(ex)
            print_err(f'Function validation failed: {cmd} for device type {device_type}: {ex}')
            return False
        
        return True
        


#------------------------   UNITEST SECTION -----------------
if __name__ == '__main__':


    def test(_str:str):
        try:
            cmd = Command.parse_cmd(_str)
            print(f'Command parsed: device={cmd.device}, op={cmd.op}, args={cmd.args}')
            res = Command.validate_device_cmd(DevType.PHG, cmd)
            print(f'Command validation result: {res}')
        except Exception as ex:
            exptTrace(ex)
            print(f'Exception : {ex}')
            
    print('Running bs2_DSL_cmd.py unitest ...')
    _cmd = 'PHG.UV param1:val1 param2:val2 param3:123 param4:45.67 param5:True param6:None param7:[1,2,3] param8:{"key":"value"}'
    
    test(_cmd)

