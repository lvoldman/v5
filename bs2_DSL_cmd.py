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

# print_DEBUG = void_f

from ctypes import *
from ctypes import wintypes
from errorH import ErrTxt
import threading
from enum import Enum

#  types of physical devices by vendor provided functionality

class VendorDevTypes(Enum):
    FAULHABBER = 'FAULHABBER'
    MAXON = 'MAXON'
    ZABER = 'ZABER'                    
    NI_DAQ = 'NI_DAQ'
    HMP = 'HMP'
    PHIDGETS = 'PHIDGETS'
    IP_CAMERA = 'IP_CAMERA'
    DH_ROBOTICS = 'DH_ROBOTICS'
    MECADEMIC = 'MECADEMIC'
    MARCO = 'MARCO'
    JTSE = 'JTSE'
    DB = 'DB'
    PLCDEV = 'PLCDEV'
    SYS = 'SYS'
    SQLITE = 'SQLITE'
    POSTGRESQL = 'POSTGRESQL'
    NCT6102D = 'NCT6102D'

# device logical types by target functionality 
class DevType(Enum):
    MOTOR = 'MOTOR'# replaced by GRIPPERv3
    CAM = 'CAM'
    DISP = 'DISP'
    ROBOT = 'ROBOT'
    HOTAIR = 'HOTAIR'
    ROT_GRIPPER = 'ROT_GRIPPER'       # like DH Robotics 
    GRIPPER = 'GRIPPER'
    DB = 'DB'
    GPI = 'GPI'
    GPO = 'GPO'
    SYS = 'SYS'
      
class confParm(Enum):              # configuration type
    NODE:str = 'node'      # node name    
    DEV:str = 'dev'               # device (name, spec, etc)  
    OP:str = 'op'               # operation (trigger, on/off, etc)
    DISP:str = 'disp'       # show in GUI display (True/False)

class pType(Enum):        # parameter type
    OP:str | bool = 'OP'      # operation / boolean in case of PHG ON/OFF/True/False
    PARM:str = 'PARM'         # parameter
    RET:str = 'RET'          # return value

# device command configuration dictionary type
# devCmdCnfg: devType -> { 'OP': [op1, op2, ...], 'PARM': [parm1, parm2, ...] }, wher OP is an allowed operation list, 
# PARM is a possible parameters list
# RET is a possible return values list
# operation codes:
# MF - Move forward
# MB - Move backward
# MA - Move absolute
# MR - Move relative
# HO - Home
# REL - Release
# STALL - stall device until 
# STOP - stop device motion
# OPEN - open gripper
# CLOSE - close gripper
# CHECK - check camera detected object
# CALIBRE - calibrate device with camera
# TERM - terminate camera operation
# ADD - add good part to database
# RESET - reset parts database counters
# UPDATE - update JTSE temperature status
# ROBOT commands: Will be defined later

devCmdCnfg: dict[DevType, dict[pType, list[str]]] = {
    DevType.MOTOR: {
        pType.OP: ['MF', 'MB', 'MA', 'MR', 'HO', 'REL', 'STALL', 'STOP'],
        pType.PARM: ['position', 'velocity', 'stall'],
        pType.RET: ['success', 'message', 'position', 'velocity', 'state']
    },
    
    DevType.CAM: {
        pType.OP: ['CHECK', 'CALIBRE', 'TERM'],
        pType.PARM: ['profile', 'abs', 'device'],
        pType.RET: ['success', 'message', 'action']
    },
    DevType.DB: {
        pType.OP: ['ADD', 'RESET'],
        pType.PARM: ['good'],
        pType.RET: ['success', 'message', 'good', 'bad']
    },
    DevType.ROT_GRIPPER: {
        pType.OP: ['MA', 'MR', 'HO','OPEN', 'CLOSE', 'STOP'],
        pType.PARM: ['position', 'distance', 'velocity'],
        pType.RET: ['success', 'message', 'state', 'position']
    },
    DevType.GRIPPER: {
        pType.OP: ['OPEN', 'CLOSE', 'STOP'],
        pType.PARM: [],
        pType.RET: ['success', 'message', 'state']
    },
    DevType.HOTAIR: {
        pType.OP: ['UPDATE'],
        pType.PARM: [],
        pType.RET: ['success', 'message', 'temp']
    },
    DevType.ROBOT: {
        pType.OP: ['PUT', 'PUTPCB', 'INSERTPCB', 'MOVESAFE', 'MOVEREL', 'MOVEABS', 'VACUUM', 'ACTIVATE', 'DEACTIVATE', 'VALIDATE', 'STOP' ],
        pType.PARM: ['p3d', 'vacuum', 'duration', 'velocity', 'x', 'y', 'z', 'alpha', 'beta', 'gamma'],
        pType.RET: ['success', 'message', 'x', 'y', 'z', 'alpha', 'beta', 'gamma', 'vacuum', 'valid_products']
    },
    DevType.GPO: {                              # dev name is defined in devices configuration file (serials) 
        pType.OP: ['TRUE', 'FALSE','TRIG'],   # ON/OFF for set state, TRIG/True/False for trigger/toggle
        pType.PARM: ['onoff'],
        pType.RET: ['success', 'message', 'state']
    }, 
    DevType.GPI: {                              # GP input device
        pType.OP: ['GET', 'WAIT_RE', 'WAIT_FE'],   # Get current state, WAIT_RE - wait for rising edge, WAIT_FE - wait for falling edge
        pType.PARM: [],
        pType.RET: ['success', 'message', 'state']
    },
    DevType.SYS: {
        pType.OP: ['DELAY', 'PLAY_MEDIA'],
        pType.PARM: ['duration', 'file'],
        pType.RET: ['success', 'message']
    }
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
    'onoff': str  | bool,       # "" for trigger/toggle | "ON" | "OFF"
    'file': str,
    'abs': bool,
    'device': str,
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
        op = Command.coerce(op)      # coerce operation to bool if needed (for PHG ON/OFF/True/False)
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
    def validate_device_cmd(dType:DevType, cmd: Command,  parms:dict=None, confDevs:dict = None) -> bool:
         
        try:
            device_type: DevType = DevType(dType)
            print_log(f'Validating command {cmd} for device type {device_type}')

            if device_type not in devCmdCnfg.keys():
                raise ValueError(f'Device type {device_type} not recognized in device command configuration. Allowed types: {list(devCmdCnfg.keys())}')
        
            if  cmd.op not in devCmdCnfg[device_type][pType.OP] and \
                '*' not in devCmdCnfg[device_type][pType.OP]:   # wildcard for any operation    

                raise ValueError(f'Operation {cmd.op} not allowed for device type {device_type} (not listed in {devCmdCnfg[device_type][pType.OP]})')
            
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
            # if device_type == DevType.CAM and cmd.op == 'CHECK':
            if device_type == DevType.CAM and cmd.op in ['CHECK', 'CALIBRE']:
                profile = cmd.args.get('profile', '')
                if parms is not None and cmd.device in parms and 'profiles' in parms[cmd.device]:
                    allowed_profiles = parms[cmd.device]['profiles']
                    if profile not in allowed_profiles:
                        raise ValueError(f'Camera profile {profile} not in allowed profiles {allowed_profiles}')
                
                else:
                    raise ValueError(f'No appropriate profiles configuration provided for camera device validation')
                
            elif device_type == DevType.MCDMC:
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
            elif device_type == DevType.PHG:
                # operation is the device name for PHG
                if confDevs is not None and confDevs[dType] is not None and cmd.device not in confDevs[dType].keys():   
                    raise ValueError(f'PHG device {cmd.device} not found in configuration devices list {list(confDevs[dType].keys())} of type {dType} for command {cmd}')
                
                    
        except ValueError as ve:
            print_err(f'Command validation failed: {cmd} for device type {device_type}: {ve}')
            return False
    
        except Exception as ex:
            exptTrace(ex)
            print_err(f'Function validation failed: {cmd} for device type {device_type}: {ex}')
            return False
        
        return True
        
@dataclass
class confDevCmd:
    devType: DevType
    cnfg: dict[confParm, str]


    # _type: device type string
    # nodes: list of node names (strings)
    # cfg_str: configuration string in format 'par1:val1, par2:val2, ...'
    @classmethod 
    def parse_cmd(cls, _type:str, _name:str, nodes:list[str], cfg_str: str) -> confDevCmd:
        print_DEBUG(f'confDevCmd.parse_cmd: type={_type}, name={_name}, nodes={nodes}, cfg_str={cfg_str}')
        try:
            if _type not in DevType.__members__:
                raise ValueError(f'Invalid device type: {_type}, expected one of {list(DevType._member_names_)}')
        
            # if _name not in nodes:
            #     raise ValueError(f'Invalid node name: {_name}, expected one of {nodes}')
            
            cfg_str = re.sub(r'\s*=\s*', '=', cfg_str)    # remove spaces around '='
            cfg_str = re.sub(r'\s*,\s*', ' ', cfg_str)    # replace ',' with ' ' and remove spaces around ','
            print_DEBUG(f'Parsing configuration command: type={_type}, name={_name}, cfg_str -> {cfg_str}')

            dev_type = DevType(_type)

            KV_RE = re.compile(r'^(?P<key>[A-Za-z_][\w-]*)=(?P<val>.*)$')
                                            # capturing groups: 
                                            # key:value pairs regex

            tokens = shlex.split(cfg_str)         # supportes quoted strings
            
            print_DEBUG(f'Parsing config string: {cfg_str}, tokens={tokens}')

            _cfg_dict: dict[str, dict[str, Any]] = dict()
            _cfg_dict[_name] = dict()
            for _token in tokens:                 # parse key:value pairs
                km = KV_RE.match(_token)
                if not km:                      # invalid format
                    raise ValueError(f'Expected key=value, got: {_token!r}')
                
                _parm = km.group('key')         # parameter name
                _val = km.group('val')          # parameter value

                if '=' not in _token:
                    raise ValueError(f'Expected key=value, got: {_token!r}')
                
                print_DEBUG(f'Parsed config token: parm={_parm}, val={_val}')

                _coerce_value = Command.coerce(_val)
                print_DEBUG(f'Coerced config value: parm={_parm}, val={_val} -> {_coerce_value} ({type(_coerce_value)})')
                _cfg_dict[_name][_parm]  = _coerce_value

                if not confParm.NODE.value in list(_cfg_dict[_name].keys()) or not _cfg_dict[_name][confParm.NODE.value] in nodes:   # validate node name
                    print_DEBUG(f'record: {_cfg_dict[_name]}  keys: {list(_cfg_dict[_name].keys())}, value: {_cfg_dict[_name].get(confParm.NODE.value, None)}, nodes = {nodes}  ')
                    raise ValueError(f'Invalid or missing node name: {_cfg_dict[_name].get(confParm.NODE.value, None)}, expected one of {nodes}')
                
                print_DEBUG(f'Parsed configuration dictionary: {_cfg_dict}')

            
        except Exception as ex:
            exptTrace(ex)
            raise ValueError(f'Error parsing configuration command: {_type}:{cfg_str}: {ex}')
        
        return confDevCmd(devType=dev_type, cnfg=_cfg_dict)

    @classmethod 
    def validate(cls, _type:str, _name:str, nodes:list[str], cfg_str: str) -> bool:
        try:
            _cmd = cls.parse_cmd(_type, _name, nodes, cfg_str)
            print_log(f'Configuration command validated: {_cmd}')
            return True
        except Exception as ex:
            exptTrace(ex)
            print_err(f'Configuration command validation failed: {_type}, {_name}, {nodes}, {cfg_str}: {ex}')
            return False

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
            
    def test2(_type:str, _name:str, nodes:list[str], _cfg_str:str):
        try:
            cmd = confDevCmd.parse_cmd(_type, _name, nodes, _cfg_str)
            print(f'Config Command parsed: devType={cmd.devType}, cnfg={cmd.cnfg}')
            print(f'{cmd}')
        except Exception as ex:
            exptTrace(ex)
            print(f'Exception : {ex}')

    print('Running bs2_DSL_cmd.py unitest ...')
    # _cmd = 'PHG.UV param1:val1 param2:val2 param3:123 param4:45.67 param5:True param6:None param7:[1,2,3] param8:{"key":"value"}'
    # _cmd = 'PHG.UV'
    
    # test(_cmd)

    # _cfg1 = 'node=DAQ1, dev=port1.line1, op=TRIGGER, disp=FALSE'
    # test2('GPI', 'DOOR', ['DAQ1', 'DAQ2'], _cfg1)
    _cfg2 = 'node=COINING_PLC, dev=PNEUMATIC_1, op=TRIGGER, disp=FALSE'
    test2('GPO', 'PNEUMATIC_VALVE_1', ['COINING_PLC'], _cfg2)