from __future__ import annotations


__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "1.0.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Tool"

from typing import TYPE_CHECKING
import serial.tools.list_ports
from queue import Queue 
from abc import ABC, abstractmethod
from collections import namedtuple
from dataclasses import dataclass
import yaml, sys
from zaber_motion.ascii import Connection, Device
from enum import Enum
from zaber_motion import Units, MotionLibException, MovementFailedException
import datetime, re

from bs1_ads import commADS, symbolsADS
from bs1_plc_dev import PLCDev

if TYPE_CHECKING:
    from bs1_base_motor import BaseDev
    from bs1_faulhaber import FH_Motor, FH_cmd, fh_baudrate, fh_timeout, fh_break_duration
    from bs1_zaber import Zaber_Motor
    from bs1_FHv3 import FH_Motor_v3
    from bs1_maxon import MAXON_Motor
    from bs1_ni6002 import NI6002
    from bs1_cam_modbus import Cam_modbus
    from bs1_HAMEG import HMP_PS
    from bs1_DH_RB_modbus import DH_ROB_RGI_modbus
    from bs1_phidget import PhidgetRELAY
    from bs1_mecademic import robotMecademic
    from bs1_meca500 import robotMeca500
    from bs1_marco_modbus import Marco_modbus
    from bs1_interlock import InterLock
    from bs1_io_control import IOcontrol
    from bs1_festo_modbus import Festo_Motor
    from bs1_sqlite import StatisticSQLite 
    from bs1_jtse_serial import  JTSEcontrol

from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, void_f, \
                        assign_parm, get_parm  
print_DEBUG = void_f

# DEV_API_SIZE = 4000  # bytes
# DEV_NAME_SIZE = 80  # chars
# DEV_INFO_SIZE = 1024  # bytes

# class plcDataPTR(Enum):
#     API = "API"
#     DeviceName = "DeviceName"
#     DeviceInfo = "DeviceInfo"
#     State = "State"

plcDataPTR = Enum('plcDataPTR', [('API', "API"), ('DeviceName', "DeviceName"), \
                           ('DeviceInfo', "DeviceInfo"), ('State', "State")])

DevType = Enum("DevType", ["TIME_ROTATOR", "DIST_ROTATOR", "TROLLEY", "GRIPPER", "GRIPPERv3", "ZABER", "DAQ_NI", "HMP", "PHG", "CAM", "DH", "MCDMC", "MARCO", "INTERLOCK", "IOCONTROL", "JTSE", "DB", "PLCDEV"])


ZABER_ID = 1027
FAULHABER_ID = 2134
#==============================

#==============================

getDevbySN = lambda devs, sn: next((_dev for _dev, _sn in devs.items() if _sn == sn), None)



'''
CDev - data class

C_type = device type: DevType.TROLLEY / DevType.GRIPPER / DevType.GRIPPERv3/ DevType.ZABER/ DevType.DIST_ROTATOR / DevType.TIME_ROTATOR (SPINNER) / DevType.CAM / DevType.DAQ-NI / DevType.HMP / DevType.PHG / DevType.DH / DevType.INTERLOCK / DevType.IOCONTROL
C_port = COM port (COM1/COM2/.../COMn) /  Port number for FHv3 / IP for cam / None for DAQ NI-6002 / USB port for MAXON / COMn for HMP
c_id = vendor ID for COM port devices (ZABER_ID, HMP = 1027 / FAULHABER_ID = 2134) / device info for FHv3 / none for CAM  / model for DAQ / dev info for MAXON/ None for DH
c_serialN = device serial number (when available)
c_gui = GUI_ID ( number of the device in the GUI sequensce: 1,2,3,...) (when available)

c_dev / __device = reference to the actual device class (Zaber_Motor / FH_Motor / FH_Motor_v3 / NI6002 / HMP_PS / PhidgetRELAY / Cam_modbus / DH_ROB_RGI_modbus / robotMecademic / robotMeca500 / Marco_modbus / InterLock / IOcontrol / StatisticSQLite / JTSEcontrol )


----
separate for ecah device - for compatability/debugging only
For final version will be the single refernce.



'''

class CDev:

    def __init__(self, C_type, C_port, c_id, c_serialN, c_dev = None, c_gui=None):
        self.C_type = C_type
        self.C_port = C_port
        self.c_id = c_id
        self.c_serialN = c_serialN
        self.c_gui = c_gui

        self.__device:BaseDev = c_dev

    def get_device(self):                       # for backward compatability 
        return self.__device

    @property
    def cDevice(self):
        return self.__device



    def __repr__(self) -> str:
        # r_str = f'[CDev]: Type={self.C_type}, port={self.C_port}, serialN={self.c_serialN}, GUI ID={self.c_gui}, vendorID={self.c_id}'
        r_str = f'[CDev: {self.C_type}{self.c_gui}]'
        return (r_str)
        
    def __del__(self):

        print_log(f'Deleteing device {self} on port {self.C_port}')
        _dev = self.get_device()
        if _dev is not None:
            del _dev

    def operateDevice(self, cmd:str) -> tuple[bool, bool] :

        if self.__device is not None:
            return self.__device.operateDevice(cmd)
        else:
            print_err(f'ERROR operating cmd = {cmd}, unkmown device - {self}')
            return False, False




class systemDevices:
    def __init__(self, _conf_file:str='serials.yml', _params_file:str='params.yml'):

        self.__devs:dict = dict()    #  Lists all available devs {dev_name1: CDev1, dev_name2: CDev2, ...} 
        self.__conf_file = _conf_file
        self.__params_file = _params_file   
        self.__platform_devs:abstractPlatformDevs = None

        try:
            with open(self.__conf_file) as p_file:
                doc = yaml.safe_load(p_file)
                _keys = list(doc.keys())
                print_log(f'Config file = {doc}, keys = {_keys}' )
                if doc is not None:
                    if 'ADS_NETID' in _keys or 'REMOTE_IP' in _keys:
                        print_log(f'PLC platform detected in configuration file {self.__conf_file}')
                        self.__platform_devs = plcPlatformDevs(self.__conf_file, self.__params_file)
                    else:
                        print_log(f'PC platform detected in configuration file {self.__conf_file}')
                        self.__platform_devs = pcPlatformDevs(self.__conf_file, self.__params_file) 
        

        except Exception as ex:
            print_err(f'Error reading parameters file, exception = {ex}')
            exptTrace(ex)
            raise ex
        
        try:
            # scan ports and add devices defined in the configuration file
            self.__platform_devs.loadConf(self)
        except Exception as ex:
            print_err(f'Error scanning ports and loading devices, exception = {ex}')
            exptTrace(ex)        
            raise ex
        
    # bracket notation for getting/setting devices by name (example: sysDevs['T1'] / sysDevs['G1'] = CDev(...) )
    def __getitem__(self, _devName:str) -> CDev:
        return None if _devName not in self.self.__devs.keys() else self.self.__devs[_devName] 
    
    def __setitem__(self, _devName:str, _dev:CDev):
        if isinstance(_dev, CDev):
            self.__devs[_devName] = _dev
        else:
            raise Exception(f'Wrong device [{_dev}] type ({type(_dev)}), expected CDev type')
        
    
    # for backward compatability    
    def port_scan(self) -> list:
        return self.__devs.values()             # list of CDev objects
    
    # for backward compatability    
    def append(self, _devName:str, _dev:CDev):
        self[_devName] = _dev


class abstractPlatformDevs(ABC):

    def __init__(self, __conf_file:str=None, _params_file:str=None):

        ###  allDevs dict: 
        # for PLC platform: 
        #       {'ADS_NETID': '123.456.78.90.1.1'}
        # for PC platform:
        #       {'TR': {"T1": 12345, "T2": 23456}, 'GR': {"G1": 34567}, 'ZB': {"Z1": 45678, "Z2": 56789}, 'RT': {"R1": 67890}, \
        #       'SP': {"S1": 78901}, 'CAM': {"CAM1": "124.45.67.89:123"}, 'NI': {"DAQ1": 4098}, 'HMP': {"H1": "12345"}, 'PHG': {"RELAY_NAME_1": "123456/7, TOGGLE|TRIGGER|ONOFF, TRUE|FALSE"}, 'DH': {"D1": 12345}, 'MCDMC': {"MCDMC1": "12345"}, 'MARCO': {"DISP1": "123.45.67.89:123"}, 'INTERLOCK': {"LCK1": "DAQ1"}, 'IOCONTROL': {"IO_NAME_1": "provider, port.line, NO/NC"}}
        self.allDevs:dict = None
        self.allParams:dict = None
    
    @abstractmethod
    def loadConf(self, _sysDevs:systemDevices):
        pass
    
    def __getitem__(self, _devType):
        return None if _devType not in self.allDevs.keys() else self.allDevs[_devType]
    
    def __setitem__(self, _devType, _devList):
        raise Exception(f'Unsupported bracket notation ([]) operation')
    
    def getDevs(self):
        return self.allDevs
    

class plcPlatformDevs(abstractPlatformDevs):
    _max_num_of_devs:int = None

    def __init__(self, _config_file:str = None, _params_file:str = None):
        super().__init__()
        self.ADS_NETID = None
        self.REMOTE_IP = None
        self._ads:commADS = None
        self._plcDevs:dict = dict()   # {dev_name1: PLCDev1, dev_name2: PLCDev2, ...}
        self.number_of_devs:int = 0

        self.__config_file = _config_file
        try:
            with open(self.__config_file) as p_file:
                doc = yaml.safe_load(p_file)
                _keys = doc.keys()
                if 'ADS_NETID' in  _keys or 'REMOTE_IP' in _keys:
                    self.ADS_NETID = doc['ADS_NETID']
                    self.REMOTE_IP = doc['REMOTE_IP']
                else:
                    raise Exception(f'Error: ADS_NETID or REMOTE_IP are not defined in the configuration file {_config_file}')
                
                _ams_re = r'\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)(?:\.1\.1)\b'
                _remip_re  = r'\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\b'
                _ams_re_compiled = re.compile(_ams_re)
                _remip_re_compiled = re.compile(_remip_re)
                if not _ams_re_compiled.match(self.ADS_NETID) or not _remip_re_compiled.match(self.REMOTE_IP):
                    raise Exception(f'Error: Wrong ADS_NETID ({self.ADS_NETID}) or REMOTE_IP ({self.REMOTE_IP}) format in the configuration file {_config_file}')
                
                self._ads = commADS(self.ADS_NETID, self.REMOTE_IP)
                plcPlatformDevs._max_num_of_devs = doc['MAX_NUM_OF_DEVS'] if 'MAX_NUM_OF_DEVS' in _keys else self._ads.readVar(symbol_name=symbolsADS._max_num_of_devs, var_type=int)
                


        except Exception as ex:
            print_err(f'Error reading configuration file, exception = {ex}')
            exptTrace(ex)
            raise ex
    
    def loadConf(self, _sysDevs:systemDevices):
        try: 
            print_log(f'PLC based devices operation. Load configuration from PLC at {self.REMOTE_IP}, AMS NET ID = {self.ADS_NETID}')
            
            _dev_idx = 0

            _devLst = PLCDev.enum_devs(_comADS=self._ads)
            print_log(f'Found {_devLst} ({len(_devLst)}) devices in PLC configuration')

            for _dev_idx, _dev_name in enumerate(_devLst):
                _dev_name = _dev_name
                if _dev_name is  None:
                    print_err(f'Unexpexted end of devices list. Error reading device name for device index {_dev_idx}')
                    break
                _plcDev = PLCDev(dev_name=_dev_name, _comADS=self._ads)
                _cdev = CDev(C_type=DevType.PLCDEV, C_port=None, c_id=None, \
                                                            c_serialN=None, c_dev=_plcDev, c_gui=_dev_idx+1)


                print_log(f'Added PLC device: {_cdev}')
            if _dev_idx == 0:
                print_err(f'No devices found in PLC configuration')
        except Exception as ex:
            print_err(f'Error loading PLC configuration, exception = {ex}')
            exptTrace(ex)

'''
params file (params.yml) format w/examples:
#Grippers:
G1: 
    TITLE: 'Electrod Stage'
    MEASUREMENT_DELAY:  0.25
    MINIMAL_OP_DURATION: 0.1
    GRIPPER_TIMEOUT:  5
    DEAFULT_VELOCITY_EV_VOLTAGE: 10000
    DevMaxSPEED:  15005
    DevOpSPEED:  648
    GRIPPER_DEFAULT_CURRENT_LIMIT: 13

#Rotators:
R1:
    TITLE: 'PCB Driver'
    MEASUREMENT_DELAY:  0.25
    MINIMAL_OP_DURATION: 0.25
    DEFAULT_CURRENT_LIMIT: 500
    DEAFULT_VELOCITY_EV_VOLTAGE: 5005
    DevMaxSPEED:  15009
    DevOpSPEED:  649
    DIAMETER: 6
    GEAR: 64

#Trolleys:
T1:
    MEASUREMENT_DELAY:  0.25
    MINIMAL_OP_DURATION: 0.25
    DEAFULT_TROLLEY_RPM: 841
    TROLLEY_DEFAULT_CURRENT_LIMIT: 1105
    TROLLEY_PARKING_VELOCITY: 25   

#Spinners:
S1:
    MEASUREMENT_DELAY: 0.25
    MINIMAL_OP_DURATION: 0.25
    DEFAULT_ROTATION_TIME: 5
    SPINNER_DEFAULT_CURRENT_LIMIT: 125
    DEAFULT_VELOCITY_EV_VOLTAGE: 20000

# Zaber (stepper actuators):
    TITLE: 'BACK/FWRD'
    MAX_ACCEL: 199                 
    ACCEL_PERCENT: 100
    RAMP_TIME: 50
    CURRENT_RUN: 17
    DEFAULT_VELOCITY_PERCENTAGE: 100
    Z_MAXSPEED: 25
    RND_STEP: 1

# Cameras:
CAM1:
    TIMEOUT_ERROR: 20
    MAX_MOVEMENT: 100000 
    MOTORS: Z3, Z4, D1/D2
    PROFILES:
        ELEC_FRONT: 1
        ELEC_BACK: 2
        PCB_FRONT: 3
        PCB_BACK: 4

# Mecademic robots:        
MCDMC1:
    TYPE: MCS500                        # MECA500 / MCS500
    PUT_H_ASYRIL: 92.0                 # put part height 
    PICKUP_H_ASYRIL: 79.1403             # pich part height (asyril) 
    PICK_PRESSURE_EL: -84.5
    VACUUM_PURGE_DURATION_EL: 0.5


    VACUUM_PURGE_DURATION_PCB: 2.5
    PICKUP_PRESSURE_PCB: -84.5
    PICKUP_H_COGNEX: 79.1403             # pich part height (coxnex) 
    PUT_H_COGNEX: 92.0
    PICKUP_OFFSET_PCB: 0.5, 0

    DEFAULT_PICKUP_PRESSURE: -85

    WORKING_H: 44               # movement height 
    VELOCITY: 25
    PICK_DELAY: 0.5
    PUT_DELAY: 0.5
    WORK_ZONE_LIMIT_XYZ_MIN: 0, 0, 0   #  SetWorkZoneLimits - xmin,ymin,zmin,
    WORK_ZONE_LIMIT_XYZ_MAX: 0, 0, 0    # xmax,ymax,zmax
    START_POSITION: 0, 0, 0, 0          # x, y, z, gamma (coordinates)
    RELIEF_POSITION: 0, 0, 0, 0         # q1,q2,q3,q4 for MoveJoints(q1,q2,q3,q4)
    MID_JOINTS_POSITION: -67.69506, -55.04702, -65.00323, -1548.44055     
                                        # MoveJoints(q1,q2,q3,q4)
                                        # -140° ≤ q1 ≤ 140°, -145° ≤ q2 ≤ 145°, -102 mm ≤ q3 ≤ 0 mm, -3,600° ≤ q4 ≤ 3,600°.

    REPEATS: 3
    PUT_ANGLE: 180                      
    PICK_ANGLE_RANGE: 0, 180            #  must be more or equal 180 for symetric parts, = 360 for non symetric  
    ASYRIL: 192.168.1.50:7171/61013     # ip:port/recipe
    COGNEX: 192.168.10.10:502

    PICKUP_ANGLE: 0
    3DOFFSET:
        PCB_INSERT: 0, 6, 0, 0                   # offset to move PCB (x,y,z,gamma)

    3DPOSITIONS:
        STAGEBACKLEFT: ..., -52.03, -52.03, -159.01
        P1: -52.03, -159.01, ..., 52
        P2: -52.03, ..., -159.01, ...
        P3: ..., ..., -52.03, -159.01
        P4: -52.03, -159.01, -159.01, -159.01

# DH Robotics
DHRB:
    ROTATION_FORCE: 50              # 20 - 100%
    GRIPPER_FORCE: 50               # 20 -100 %
    DEFAULT_ROTATION_SPEED: 100      # 1 - 100 
    DEFAULT_GRIPPER_SPEED: 50       # 1 - 100
    DH_TOLERANCE_LIMIT: 0              # target error tolerance 

# RelayPhidgets    
PHG:
    WELDER:  "Welder"
    JTSE_HOTAIR:  "JTSE HotAir"
    DISP:  "Dispenser"
    UV:  "UV"
    UV1:  "UV1"
    BACKLIGHT:  "Back" 
    FRONTLIGHT:  "Front"
    HOLELIGHT:  "Hole"
    RED: "RED"
    BLUE: UV alarm, 5000              # name, trigger time in ms
    ALARM: Alarm, 1500

# DAQ data acquisition devices:
DAQ1:
    TIMEOUT: 0.5
    LCK1: port0.line1
    LCK2: port0.line2

# DataBase
DB:
    TIMEOUT: 5
    DB_NAME: production.db
    TBL_NAME: statistic

OMNIPULSE:
    FRONTLEFT: 4, -224, 40, 90 

'''   

'''
Config file: 
ADS_NETID: 123.456.78.90
OR if it's not defined (empty) in the system enviroment variables, it will be read from the config file
Config file (serials.yml) format w/examples:
# Grippers
GR: (Gn: s/n)
    G1: 512000099                          
# Trolleys (Tn: s/n )
TR:
    T1: 162400865                     
# Zaber -- stepper actuator(Zn: s/n)
ZB:
    Z1: 123883
# Rotators (Rn: s/n in decimal)
RT: 
    R1: 1073807751                          # 0x40010187
# Spinners (Sn: s/n)    
SP:
    S1: 1073807704     
# Cameras (CAMn: IP:port)
CAM:
    CAM1: 192.168.230.54:1502
# DH Robotics grippers (Dn: s/n)
DH:
    D1: 8838
# Phidgets relay (Pn: sn/port, OpType=TRIGGER/ONOFF/TOGGLE, PresentAtGUI=FALSE/TRUE)
PHG:
    DISP: 700440/0, TRIGGER
    UV: 700440/1, ONOFF
    FRONTLIGHT: 700440/4, TOGGLE, FALSE
# MARCO dispenser systems (DISPn: IP:port)
MARCO:
    DISP1: 192.168.1.214:5020
# DAQ (data acquisition) devices (DAQn: s/n in HEX)
NI:
    DAQ1: 020DC90D              #SN in DEC:34457869
# IO control devices (provider, port.line, NO/NC)
IOCONTROL:
    DOOR: DAQ1, port0.line0
    EMERG: DAQ1, port0.line1, NC
    ON_OFF_BUTTON: DAQ1, port0.line2
    D1_D2_GRIPERS_TOGGLE:  DAQ1, port0.line3
# JTSE blower
JTSE:
    BL: JTSE

'''   

class pcPlatformDevs(abstractPlatformDevs):
    devTypesTbl:dict = {'TR':'T', 'GR':'G', 'ZB':'Z', 'RT':'R', 'SP':'S', 'NI':'DAQ', 'CAM':'CAM', 'HMP':'H', \
                   'DH':'D', 'PHG':'*', 'MCDMC':'MCDMC', 'MARCO':'DISP', 'INTERLOCK':'LCK', 'IOCONTROL':'*', \
                    'JTSE':'*'}
    freeStyleDevs:set = set()
    def __init__(self, _config_file:str = 'serials.yml', _params_file:str = 'params.yml'):
        super().__init__(_config_file, _params_file)
        self.__config_file = _config_file
        self.__params_file = _params_file
        self._serialPorts:dict = dict()   # {port: {'vid':, 'description':, 'hwid':, 'serial_number':, 'location':, 'manufacturer':, 'product':, 'interface':} }
        
        self.__params_table:dict =  pcPlatformDevs.read_params(self.__params_file)  
                                                        # {devName: {parm1: value1, parm2: value2, ...}, ...}
        

    def loadConf(self, _sysDevs:systemDevices):
        
        print_log(f'PC based devices operation. Load configuration from {self.__config_file} configuration file')
       
        self.allDevs = pcPlatformDevs.read_configuration(self.__config_file)
        self.__addSerialDevs(_sysDevs)
        self.__addFHv3Devs(_sysDevs)
        self.__addMaxonDevs(_sysDevs)
        self.__addDAQDevs(_sysDevs)
        self.__addIODevs(_sysDevs)
        self.__addPHGDevs(_sysDevs)
        self.__addCamDevs(_sysDevs)
        self.__addMecademicDevs(_sysDevs)
        self.__addMarcoDevs(_sysDevs)
        


    @staticmethod
    def read_configuration(configuration_file:str) -> dict:
        _dev_class = None
        _devices = None
        _allDevs = dict()
        
        try:
            with open(configuration_file) as conf_file:

                _allDevs = yaml.safe_load(conf_file)
                print_log(f'conf. file = {_allDevs}')
                for _dev_class, _devices in _allDevs.items():       # devs is a dictionary 
                    print_log(f'_dev_class = {_dev_class}, _devices = {_devices}')
                    if _devices == None:
                        print_err(f'Error: Empty device list for dev type {_dev_class},  removed')
                        _allDevs.pop(_dev_class)
                        
                    elif not isinstance(_devices, dict) or len(_devices) == 0:
                        print_err(f'Warning: Wrong device list format (or empty) for dev type {_dev_class},  removed')
                        _allDevs.pop(_dev_class)
                        
                    elif pcPlatformDevs.devTypesTbl[_dev_class] == '*':
                        # for IO control and Phidgets relay we don't know the device ID format, 
                        # so we just verify name uniqueness
                        for _dev in _devices.items():
                            if _dev in pcPlatformDevs.freeStyleDevs:   
                                print_err(f'Error: The device {_dev} of dev class {_dev_class} is already defined, removed')
                                _devices.pop(_dev)
                            else:       
                                pcPlatformDevs.freeStyleDevs.add(_dev_class)
                        if len(_devices) == 0:
                            print_err(f'No valid devices for dev class {_devices}. Removed')
                            _allDevs.pop(_devices)
                    else:
                        _dev_re += fr'{pcPlatformDevs.devTypesTbl[_dev_class]}\d*'
                        _dev_re_compiled = re.compile(_dev_re)
                        for _dev in _devices.items():
                            if not _dev_re_compiled.match(_dev):
                                print_err(f'Error. Wrong format dev name {_dev} in dev class {_devices}')
                                _devices.pop(_dev)
                        if len(_devices) == 0:
                            print_err(f'No valid devices for dev class {_devices}. Removed')
                            _allDevs.pop(_devices)

                print_log(f'All valid devices in confiration: {_allDevs}')
                        
            
        except Exception as ex:
            print_log(f'Exception -> {ex} of type {type(ex)}')
            print_log(f'Wrong configuration format in class: {_dev_class} [list: {_devices}]')
            exptTrace(ex)
            
        else:
            print_log(f'')

        return _allDevs

    @staticmethod
    def read_params(params_file) -> dict:
        print_inf(f'Read params from file {params_file}')
        params_table = dict()
        
        try:
            with open(params_file) as p_file:
                doc = yaml.safe_load(p_file)
                if doc is None:
                    print_log(f'No parameters defined in the parameters file {params_file}')
                    return params_table
                
                for dev, dev_parms in doc.items():       # devivce parm is a dictionary 
                    params_table[dev] = dict()
                    for i, parm in enumerate(dev_parms):
                        params_table[dev][parm] = dev_parms[parm]

                print_log(f'params_table={params_table}')

                    

        except Exception as ex:
            print_err(f'Error reading parameters file, exception = {ex}')
            exptTrace(ex)
            pass
        
        return params_table


    '''
    Serial devices:
    VID = 2134 (FAULHABER) 
    VID = 1027 (ZABER, HMP, DH)
    JTSE
    '''
    def __addSerialDevs(self, _sysDevs:systemDevices):
        try:

            # Seial port devices: ZB, FAULHABER (TR, GR, SP), HMP, DH, JTSE
            serialDevsClasses = set(['ZB', 'TR', 'GR', 'SP', 'HMP', 'DH', 'JTSE'])
            faulhaberTypes = set([ 'TR', 'GR', 'SP'])
            configuredDevs = set(self.allDevs.keys())

            # if 'ZB'  not in configuredDevsList and 'TR' not in configuredDevsList and \
            #       'GR'  not in configuredDevsList and 'SP' not in configuredDevsList and \
            #         'HMP' not in configuredDevsList and 'DH' not in configuredDevsList and \
            #             'JTSE' not in configuredDevsList:
            if len(serialDevsClasses & configuredDevs) == 0:
                print_log(f'No serial port devices defined')
                return
            
            print_log(f'Scanning serial (COM) ports for Zaber, Faulhaber, HAMEG, DH Robotics, JTSE devices')

            for ps in list(serial.tools.list_ports.comports()):
                print_log ("============ Serial Ports available ==================")
                print_log (f"Name/device={ps.name}/{ps.device} ** description={ps.description} ** hwid={ps.hwid} ** vid={ps.vid} ")
                print_log (f" serial_number={ps.serial_number} ** location={ps.location} ** manufacturer={ps.manufacturer} ** product={ps.product} ** interface={ps.interface} ")
                self._serialPorts[ps.device] = {'vid': ps.vid, 'description': ps.description, 'hwid': ps.hwid, \
                                                'serial_number': ps.serial_number, 'location': ps.location,
                                                'manufacturer': ps.manufacturer, 'product': ps.product, 'interface': ps.interface}           

                if ps.vid == FAULHABER_ID:                      # vid = 2134 // Faulhaber
                    if len(faulhaberTypes & configuredDevs) > 0:        #  if Faulhaber devices are configured
                        _fhSN = FH_Motor.recognizeDev(ps)
                        if _fhSN is not None:
                            print_log (f'Faulhaber device with S/N = {_fhSN} found on port {ps.device}')    

                            if _fhSN in self.allDevs['TR'].values():
                                _devName = getDevbySN(self.allDevs['TR'], _fhSN)
                                if _devName is None:
                                    print_err(f'ERROR: Something went wrong while retriving Faulhaber TROLLEY dev name with S/N = {serN} ')
                                    continue


                                dev_trolley = FH_Motor(ps.device, fh_baudrate, fh_timeout, self.__params_table, _devName)
                                i_dev = CDev(DevType.TROLLEY, ps.device, ps.vid, _fhSN, dev_trolley, list(self.allDevs['TR'].keys()).index(_devName)+1)

                                if not dev_trolley.init_dev(i_dev.C_type):
                                    print_log(f'Trolley initiation failed on port {ps.device}')
                                    del dev_trolley
                                    continue

                                _sysDevs[_devName] = i_dev
                                # devs.append(i_dev)

                            elif _fhSN in self.allDevs['GR'].values():
                                _devName = getDevbySN(self.allDevs['GR'], _fhSN)
                                if _devName is None:
                                    print_err(f'ERROR: Something went wrong while retriving Faulhaber GRIPPER dev name with S/N = {serN} ')
                                    continue


                                dev_gripper = FH_Motor(ps.device, fh_baudrate, fh_timeout, self.__params_table, _devName)
                                i_dev = CDev(DevType.GRIPPER, ps.device, ps.vid, _fhSN, dev_gripper, list(self.allDevs['GR'].keys()).index(_devName)+1)

                                if not dev_gripper.init_dev(i_dev.C_type):
                                    print_log(f'Gripper initiation failed on port {ps.device}')
                                    del dev_gripper
                                    continue

                                _sysDevs[_devName] = i_dev
                                # devs.append(i_dev)

                            elif _fhSN in self.allDevs['SP'].values():
                                _devName = getDevbySN(self.allDevs['SP'], _fhSN)
                                if _devName is None:
                                    print_err(f'ERROR: Something went wrong while retriving Faulhaber SPINNER dev name with S/N = {serN} ')
                                    continue


                                dev_spinner = FH_Motor(ps.device, fh_baudrate, fh_timeout, self.__params_table, _devName)
                                i_dev = CDev(DevType.TIME_ROTATOR, ps.device, ps.vid, _fhSN, dev_gripper, dev_spinner, list(self.allDevs['SP'].keys()).index(_devName)+1)

                                if not dev_spinner.init_dev(i_dev.C_type):
                                    print_log(f'Spinner initiation failed on port {ps.device}')
                                    del dev_spinner
                                    continue

                                _sysDevs[_devName] = i_dev
                                # devs.append(i_dev)

                            else :
                                print_log (f'Undefined device on port {ps.device} with s/n {serN}')
                                continue
                            
                            print_log(f'Added FAULHABER device on port {ps.device} with s/n {serN}')

                        else:           # Faulhaber device on port  can not be recognized
                            pass

                    else:
                        print_log(f'No Faulhaber devices configured in the system')        

                
                elif ps.vid == ZABER_ID:
                    if ps.description.split()[0] == 'HAMEG':
                        print_log(f'Found HAMEG device on port {ps.device}')
                        i_dev =  None
                        if len(self.allDevs['HMP'].values()) > 0:
                            try:
                                for sn in enumerate(self.allDevs['HMP'].values()):
                                    # if ps.serial_number == sn:
                                    if ps.description.split()[0] == sn:
                                        _devName = getDevbySN(self.allDevs['HMP'], sn)
                                        guiIND = list(self.allDevs['HMP'].keys()).index(_devName)+1
                                        hmp_dev = HMP_PS(sn=sn, port=ps.device, parms=self.__params_table)
                                        i_dev = CDev(DevType.HMP, ps.device, ps.description,
                                                        ps.serial_number, hmp_dev, guiIND)

                                        print_log(f'HMP device added on port {ps.device}, s/n = {sn}')
                                        _sysDevs[_devName] = i_dev
                                        # devs.append(i_dev)
                                        break
                                if i_dev is None:
                                    print_log(f'HAMEG device found on port {ps.device} is not configured in the system')                                        

                            except Exception as ex:
                                print_log(f"Fail to add HAMEG device. Exception: {ex}")
                                exptTrace(ex)
                        else:
                            print_log(f'No HAMEG devices configured in the system')

                    elif (len(self.allDevs['DH'].values()) > 0) and (sn := DH_ROB_RGI_modbus.find_server(ps.device)) is not None:

                        print_log(f'Trying add DH Robotics device on port {ps.device}, s/n = {sn}')
                        if not sn in self.allDevs['DH'].values():
                            print_log(f'Found device with sn = {sn} is not configured in the system')
                        else:
                            _devName = getDevbySN(self.allDevs['DH'], sn)
                            guiIND = list(self.allDevs['DH'].keys()).index(_devName)+1

                            dh_dev = DH_ROB_RGI_modbus(sn=sn, port=ps.device, d_name = _devName, parms=self.__params_table)
                            i_dev = CDev(DevType.DH, ps.device, None, \
                                            sn, dh_dev, guiIND)
                            if not dh_dev.init_dev(i_dev.C_type):
                                print_err(f'Can not initiate DH Robotics devivce on port {ps.device}')
                            else:
                                _sysDevs[_devName] = i_dev
                                # devs.append(i_dev)
                                print_log(f'DH Robotics device with SN = {sn} succesfully added on port {ps.device}')

                    else:               # ZABER
                        try:
                            if len(self.allDevs['ZB'].values()) > 0 and Zaber_Motor.init_devs(ps.device):        
                                                                                # if ZABER devices are configured and initilized
                                print_log(f'ZABER DEVICES on port {ps.device} have been initialized')

                                z_len = len(Zaber_Motor.device_port_list[ps.device]["device_list"])
                                print_log (f'{z_len} ZABER devices will be added')

                                for ind in range (z_len):
                                        
                                    axes, device_id, identity, d_name, d_serial_number = Zaber_Motor.GetInfo(ind, ps.device)


                                    print_log (f'Device has {axes} axes')
                                    print_log (f'DeviceID: {device_id}')
                                    print_log (f'Device identity: {identity}')
                                    print_log (f'Device name: {d_name}')
                                    serN = int(d_serial_number)
                                    print_log (f'Device serialNumber: {d_serial_number}/{serN}')
                                    
                                    devName = getDevbySN(self.allDevs['ZB'], serN)
                                    if devName is None:
                                        print_err(f'ERROR: ZABER dev name with S/N = {serN} is not configured in the system')
                                        continue
                                    
                                    guiIND = list(self.allDevs['ZB'].keys()).index(_devName)+1
                                    dev_zaber = Zaber_Motor(ind, ps.device, self.__params_table, devName)
                                    i_dev = CDev(DevType.ZABER, ps.device, device_id, serN, \
                                                        dev_zaber, guiIND)
                                    
                                    _sysDevs[_devName] = i_dev
                                    # devs.append(i_dev)
                                    print_log(f'Added ZABER device on port {ps.device} with s/n {serN}')
                                
                        
                        
                        except MotionLibException as ex:
                            print_log(f'There was error adding ZABER devices on port {ps.device}')
                            print_log(f"Exception: {ex}")
                            
                if len(self.allDevs['JTSE'].values()) > 0 and isinstance(self.allDevs['JTSE'], dict):
                    devName = list(self.allDevs['JTSE'].keys())[0]
                    _the_first_device = list(self.allDevs['JTSE'].values())[0]

                    print_log(f'Looking for devName /{_the_first_device}/ blower on port {ps.device}')
                    try:
                        _com = JTSEcontrol.findDev(_the_first_device, ps.device)
                        if _com is None or _com == '':
                            print_log('No JTSE found')
                        else:
                            dev_jtse = JTSEcontrol(devName, _com)
                            i_dev = CDev(DevType.JTSE, _com, None, \
                                                None, dev_jtse, None)
                           
                            _sysDevs[_devName] = i_dev
                            # devs.append(i_dev)
                            print_log(f'JBC/JTSE dev ({devName}) at port {_com} succesfully added')

                    except Exception as ex:
                        print_log(f"Fail to adding JTSE. Exception: {ex}")
                        exptTrace(ex)
                else:
                    print_log(f'No JTSE blower configured')


        except Exception as ex:
            print_log(f"Unexpected Exception: {ex}")
        else:
            pass

        finally:
            pass
                


    def __addFHv3Devs(self, _sysDevs:systemDevices):
        FHv3Types = set([ 'TR', 'GR', 'SP', 'RT'])
        configuredDevs = set(self.allDevs.keys())

        if len(FHv3Types & configuredDevs) == 0:
            print_log(f'No Faulhaber v3 devices defined in the configuration')
            return
                
        print_log('Scanning FHv3 comunnication (USB) ports')
        
        try:
            FHv3Devs = FH_Motor_v3.init_devices()
            print_log(f'FHv3 devs - {FHv3Devs}')

            if FHv3Devs == None or len(FHv3Devs) == 0:
                print_log(f'No Faulhaber v3 devices were found')
                return

            FHv3_len = len(FHv3Devs)

            print_log (f'{FHv3_len} Faulhaber v3 devices could be added')

            for ind in range(len(FHv3Devs)):
                devFHv3 = FHv3Devs[ind]
                print_log (f'{ind} -> {devFHv3.devinfo}, s/n = {devFHv3.serialN}')

                
                if devFHv3.serialN in self.allDevs['RT'].values():
                    devType = DevType.DIST_ROTATOR
                    devName = getDevbySN(self.allDevs['RT'], devFHv3.serialN)
                    guiIND = list(self.allDevs['RT'].keys()).index(devName)+1
                    
                elif devFHv3.serialN in self.allDevs['SP'].values():
                    devType = DevType.TIME_ROTATOR
                    devName = getDevbySN(self.allDevs['SP'], devFHv3.serialN)
                    guiIND = list(self.allDevs['SP'].keys()).index(devName)+1

                elif devFHv3.serialN in self.allDevs['GR'].values():
                    devType = DevType.GRIPPERv3
                    devName = getDevbySN(self.allDevs['GR'], devFHv3.serialN)
                    guiIND = list(self.allDevs['GR'].keys()).index(devName)+1

                elif devFHv3.serialN in self.allDevs['TR'].values():
                    devType = DevType.TROLLEY
                    devName = getDevbySN(self.allDevs['TR'], devFHv3.serialN)
                    guiIND = list(self.allDevs['TR'].keys()).index(devName)+1

                else :
                    print_log (f'Undefined FHv3 device: {devFHv3}')
                    continue

                dev_mDC = FH_Motor_v3(int(devFHv3.port), int(devFHv3.channel), self.__params_table, devName)
                i_dev = CDev(devType, devFHv3.port, devFHv3.devinfo, \
                                devFHv3.serialN, dev_mDC, guiIND)
                
                if not dev_mDC.init_dev(i_dev.C_type):
                    print_log(f'FHv3 initiation failed for device: {i_dev}')
                    del i_dev
                    continue
                
                _sysDevs[devName] = i_dev
                # devs.append(i_dev)
                
                print_log(f'Added FHv3 device: {devFHv3}')

        except Exception as ex:
            print_log(f"Fail to add FHv3 device. Unexpected Exception: {ex}")
            exptTrace(ex)
        
    
    def __addMaxonDevs(self, _sysDevs:systemDevices):

        maxonTypes = set(['GR', 'RT'])
        configuredDevs = set(self.allDevs.keys())

        if len(maxonTypes & configuredDevs) == 0:
            print_log(f'No MAXON compitable devices defined in the configuration')
            return

        print_log('Scanning MAXON devs')

        try:
            # mxnDev = bytes(self.__params_table['DEAFULT']['MAXON_DEV'], 'utf-8')
            # mxnIntfc = bytes(self.__params_table['DEAFULT']['MAXON_INTERFACE'], 'utf-8')

            mxnDev = bytes(assign_parm('DEFAULT', self.__params_table['DEAFULT']['MAXON_DEV']), 'utf-8')
            mxnIntfc = bytes(assign_parm('DEFAULT', self.__params_table['DEAFULT']['MAXON_INTERFACE']), 'utf-8')
            
            if mxnDev is None or mxnIntfc is None:
                print_log(f'MAXON device type or interface is not correctly defined in the parameters table')
                return
            
            MAXONDevs = MAXON_Motor.init_devices(mxnDev, mxnIntfc)
            print_log(f'MAXON devs - {MAXONDevs}')

            if MAXONDevs == None or len(MAXONDevs) == 0:
                print_log(f'No MAXON devices were found')
                return
            MAXON_len = len(MAXONDevs)

            print_log (f'{MAXON_len} MAXON devices could be added')

            for ind in range(len(MAXONDevs)):
                devMAXON = MAXONDevs[ind]
                print_log (f'{ind} -> {devMAXON}, s/n = {devMAXON.sn}')

                if devMAXON.sn in self.allDevs['RT'].values():
                    devType = DevType.DIST_ROTATOR
                    devName = getDevbySN(self.allDevs['RT'], devMAXON.sn)
                    guiIND = list(self.allDevs['RT'].keys()).index(devName)+1

                elif devMAXON.sn in self.allDevs['GR'].values():
                    devType = DevType.GRIPPER
                    devName = getDevbySN(self.allDevs['GR'], devMAXON.sn)
                    guiIND = list(self.allDevs['GR'].keys()).index(devName)+1

                else :
                    print_log (f'Undefined MAXON device: {devMAXON}')
                    continue

                dev_mDC = MAXON_Motor(devMAXON, self.__params_table, devName)
                i_dev = CDev(devType, devMAXON.port, devMAXON, \
                                devMAXON.sn, dev_mDC, guiIND)
                
                if not dev_mDC.init_dev(i_dev.C_type):         
                    print_log(f'MAXON initiation failed for device: {i_dev}')
                    continue

                _sysDevs[devName] = i_dev
                # devs.append(i_dev)
                print_log(f'Added MAXON device: {devMAXON}')




        except Exception as ex:
            print_log(f"Fail to add MAXON device. Unexpected Exception: {ex}")
            exptTrace(ex)

    
    def __addDAQDevs(self, _sysDevs:systemDevices):

        if 'NI' not in self.allDevs.keys():
            print_log(f'No NI DAQ devices defined in the configuration')
            return
        
        NI = list(self.allDevs['NI'].values())      # serial numbers list

        print_log('Looking for NI devices')
        
        try:
            
            for ind, sn in enumerate(NI):
                found_dev = NI6002.find_devs(sn)
                if found_dev:

                    devType = DevType.DAQ_NI
                    devName = getDevbySN(self.allDevs['NI'], sn)
                    guiIND = list(self.allDevs['NI'].keys()).index(devName)+1
                    
                    dev_ni = NI6002( devName, sn, self.__params_table)
                    i_dev = CDev(devType, found_dev.device, found_dev.model, \
                                    found_dev.sn, dev_ni, guiIND)
                    
                    # devs.append(i_dev)
                    _sysDevs[devName] = i_dev

                    print_log(f'NI device with SN = {hex(sn)} ({sn}) succesfully added')


    ##########################                

                    print_log(f'Adding Interlock for NI DAQ{guiIND}')
                    INTERLOCK = list(self.allDevs['INTERLOCK'].values()) 
                    for interIndex, interDev in enumerate(INTERLOCK):
                        if interDev == f'DAQ{guiIND}':
                            iLockName = getDevbySN(self.allDevs['INTERLOCK'], interDev)
                            dev_interlock = InterLock(iLockName, interDev, self.__params_table)
                            i_dev = CDev(DevType.INTERLOCK, None, None, \
                                    None, dev_interlock, interIndex+1)
                            
                            _sysDevs[iLockName] = i_dev
                            # devs.append(intlck_dev)
                            print_log(f'Interlock device {dev_interlock} assosiated with NI {i_dev} was added')

    ##########################                
                else:
                    print_err(f'NI device with SN = {hex(sn)} ({sn}) can not be detected')

        
        except Exception as ex:
            print_log(f"Fail to add NI device. Exception: {ex}")
            exptTrace(ex)


    def __addFESTODevs(self, _sysDevs:systemDevices):

        if 'ZABER' not in self.allDevs.keys():
            print_log(f'No stepper group including FESTO devices defined in the configuration')
            return

        print_log(f'Looking for FESTO in stepper group: \n{self.allDevs["ZABER"]}  ')
        try:
            print_log(f'Looking for  FESTO devs in neighbour nodes ')
            Festo_Motor.enum_devices()

            ZABER = list(self.allDevs['ZABER'].values())      # SN/IP list
            for ind, ip in enumerate(ZABER):

                _f_rex = re.compile(r'(?:\d{1,3}\.){3}\d{1,3}')
                if _f_rex.match(str(ip)):
                    _sn = Festo_Motor.find_dev(ip)
                    if _sn is None:
                        print_err(f'No FESTO device found at IP = {ip}')
                        continue

                    devType = DevType.ZABER
                    devName = getDevbySN(self.allDevs['ZABER'], ip)
                    guiIND = list(self.allDevs['ZABER'].keys()).index(devName)+1

                    dev_mDC = Festo_Motor(_sn, ip, devName, self.__params_table)
                    i_dev = CDev(devType, ip, '502', _sn, dev_mDC, \
                                                    guiIND)
                    _sysDevs[devName] = i_dev
                    # devs.append(i_dev)
                    print_log(f'Added FESTO device on ip {ip} with s/n {_sn}, name = {devName}')


        except Exception as ex:
            exptTrace(ex)
            print_log(f"Fail while adding FESTO devs. Exception: {ex}")

# BUGBUG 

    def __addIODevs(self, _sysDevs:systemDevices):
        io_dev_dict = self.allDevs['IOCONTROL']      # {dev_name: 'provider, port.line, NO/NC', ...}
    
        if len(io_dev_dict) == 0:
            print_log(f'No IO control (IOCONTROL)  devices defined in the configuration')
            return  
    
        print_log(f'Looking for IO devs: \n{io_dev_dict} ')
        
        try:
            
            for dev_name, _io_ctl in io_dev_dict.items():
                io_dev_cnf = re.compile(r'^\s*\w+\s*,\s*port\d.line\d\s*(,\s*(NO|NC)\s*)?$')
                if  not io_dev_cnf.match(_io_ctl):
                    raise Exception(f'Wrong format configuring IO control: {_io_ctl}')
                
                _pars = _io_ctl.split(',')
                __io_provider = _pars[0].strip()
                port_line = _pars[1].strip()
                _pl = port_line.split('.')
                __port = int(_pl[0][4])
                __line = int(_pl[1][4])

                _NO = True
                _nonc = None
                if len(_pars) == 2:
                    _NO = True
                elif len(_pars) == 3:
                    _nonc = _pars[2].strip()
                    if  _nonc.upper() == 'NC':
                    # if  _nonc == 'NC':
                        _NO = False
                    else:
                        _NO = True
                
                dev_iocontrol = IOcontrol(dev_name, __io_provider, __line, __port, self.__params_table, _NO)
                i_dev = CDev(C_type=DevType.IOCONTROL, C_port=_pl, c_id=__io_provider, c_serialN=0, \
                                        c_dev = dev_iocontrol, c_gui=None)
                
                # devs.append(i_dev)

                print_log(f'IO control ({_pars}/{__io_provider}, {__port}.{__line}, {_NO} [{_nonc}] len = {len(_pars)}) was added for name= {dev_name} dev = {__io_provider}, port={__port}, line = {__line}, NO/^NC = {_NO}')
        
        except Exception as ex:
            exptTrace(ex)
            print_log(f"Fail to add IO CONTROL. Exception: {ex}")

    
    
    def __addPHGDevs(self, _sysDevs:systemDevices):

        phg_dev_dict = self.allDevs['PHG']      # {dev_name: 'sn/channel, type, TRUE/FALSE', ...}
        if len(phg_dev_dict) == 0:
            print_log(f'No Phidgets (PHG) devices defined in the configuration')
            return      
        
        print_log(f'Looking for Phidgets:  \n{phg_dev_dict} ')
        
        try:
            chk_lst = list()
            for _dName, connection in phg_dev_dict.items():
                _dType = None
                dev_chan = re.compile(r'^\d+/\d\s*,\s*(TRIGGER|ONOFF|TOGGLE)\s*(,\s*(TRUE|FALSE))?\s*$')
                if  dev_chan.match(connection):
                    conf_con = connection.split(',')
                    _sw_type = conf_con[1].strip() 
                    sCon = conf_con[0].split('/')
                    sn = sCon[0]
                    chan = sCon[1]
                    print_log(f'Looking dev. sn = {sn}, channel = {chan}')
                    if connection in chk_lst:
                        print_err(f'-ERROR -- SN:Channel {connection} appearce more than once for Phidget device. ')
                        break
                    else:
                        chk_lst.append(connection)

                    found_dev = PhidgetRELAY.find_devs(sn, chan)
                    if found_dev: 
                        guiIND = list(phg_dev_dict.keys()).index(_dName)+1
                        match _sw_type:
                            case 'TRIGGER':
                                _dType = PhidgetRELAY.devType.trigger
                                
                            case 'TOGGLE':
                                _dType = PhidgetRELAY.devType.toggle
                                
                            case 'ONOFF':
                                _dType = PhidgetRELAY.devType.onoff
                                
                            case _:
                                print_err(f'ERROR -- unknown type: *{_sw_type}*. Device will not be added')
                                

                        if _dType:
                            dev_phg = PhidgetRELAY(sn, chan, _dType, _dName, self.__params_table)
                            i_dev = CDev(DevType.PHG, found_dev.channelName, found_dev.channelClassName, \
                                            found_dev.devSN, dev_phg, guiIND)
                            _sysDevs[_dName] = i_dev
                            # devs.append(i_dev)
                            print_log(f'PHG device with SN/channel = {connection}, name = {_dName}, type = {_dType}/{_itm} succesfully added')
                        else:
                            print_err(f'Error adding device:  SN/channel = {connection}, name = {_dName}, type = {_dType}/{_itm}')
                    else:
                        print_err(f'PHG device with SN/channel = {connection}, SN = {sn}, channel = {chan} was not found')
                else:
                    print_log(f'Wrong format declaring Phidget: {connection}')
        
        except Exception as ex:
            print_log(f"Fail to add PNG device. Exception: {ex}")
            exptTrace(ex)



    def __addCamDevs(self, _sysDevs:systemDevices):
        if 'CAM' not in self.allDevs.keys():
            print_log(f'No ModBus Camera devices defined in the configuration')
            return
        CAM = self.allDevs['CAM']     # IP:port dictionary

        print_log('Looking for ModBus servers / Camera in {CAM}')
        
        try:
            for _devName, connection in CAM.items():
                ip_port = re.compile(r'(?:\d{1,3}\.){3}\d{1,3}:\d{1,5}')
                if  ip_port.match(connection):
                    sCon = connection.split(':')
                    ip = sCon[0]
                    port = sCon[1]
                    found_dev = Cam_modbus.find_server(ip, port)
                    if found_dev:
                        # guiIND = ind + 1
                        # guiIND = CAM.index(connection)+1
                        guiIND = list(CAM.values()).index(connection)+1
                        dev_cam = Cam_modbus(ip, port, self.__params_table, _devName)
                        i_dev = CDev(DevType.CAM, connection, None, None,dev_cam, guiIND)

                        _sysDevs[_devName] = i_dev
                        # devs.append(i_dev)
                        print_log(f'ModBus device {_devName} with ip = {ip}, port = {port} succesfully added')
                        
                    else:
                        print_err(f'ModBus device {_devName} at {connection} can not be detected')
                else:
                    print_err(f'Wrong configuration format for {_devName}  to connect ModBus server / Camera = {connection}')
        
        except Exception as ex:
            print_log(f"Fail to add ModBus device. Exception: {ex}")
            exptTrace(ex)


    def __addMecademicDevs(self, _sysDevs:systemDevices):

        if 'MCDMC' not in self.allDevs.keys():
            print_log(f'No Mecademic / Asyril  devices defined in the configuration')
            return

        print_log('Looking for Mecademic / Asyril')
        MCDMC = self.allDevs['MCDMC']                 # Mecademic devices dictionary  {dev_name: ip_address, ...}

        try:
            for _devName, _ipAddr in MCDMC.items():
                _ip_format = re.compile(r'(?:\d{1,3}\.){3}\d{1,3}')
                # parm_re = re.compile(r'\b(?:\d{1,3}\.){3}\d{1,3}:\d{1,5}\b')
                parm_re = re.compile(r'\b(?:\d{1,3}\.){3}\d{1,3}:\d{1,5}/\d{1,5}\b')
                # _asyrilIPStr = get_parm(_devName, params_table, 'ASYRIL')

                _asyrilIPStr = assign_parm(_devName, self.__params_table, 'ASYRIL',None)
                __asyrilIPort = None if _asyrilIPStr is None else _asyrilIPStr.split('/')[0]
                _robot_type = assign_parm(_devName, self.__params_table, 'TYPE', 'MECA500')

                if _asyrilIPStr is not None and _robot_type == 'MCS500':
                                                                        # Asyril is defined and SCARA robot (MCS500)
                    print_log(F'Checking  for Mecademic / MCS500 ({_ipAddr}) / Asyril {_asyrilIPStr} together')
                    if  _ip_format.match(_ipAddr) and parm_re.match(_asyrilIPStr):
                        found_dev = robotMecademic.find_avilable(_ipAddr, __asyrilIPort)
                        if found_dev:
                            guiIND = list(MCDMC.keys()).index(_devName)+1
                            # guiIND = MCDMC.index(_ipAddr)+1
                            dev_mcdmc = robotMecademic(_devName, _ipAddr,  self.__params_table)
                            i_dev = CDev(DevType.MCDMC, _ipAddr, None, \
                                            None, dev_mcdmc, guiIND)
                            
                            _sysDevs[_devName] = i_dev
                            # devs.append(i_dev)
                            print_log(f'Mecademic robot at ip = {_ipAddr}, assosiated with Asyril at {_asyrilIPStr}  were succesfully added')
                            
                        else:
                            print_err(f'Mecademic robot at ip = {_ipAddr}  or assosiated with Asyril at {_asyrilIPStr} can not be detected')
                    else:
                        print_err(f'Wrong configuration format: Mecademic robot = {_ipAddr} or Asyril = {_asyrilIPStr}')
                elif _robot_type == 'MECA500':
                    print_log(F'Checking  for Mecademic / MECA500 ({_ipAddr}) robot availability')
                    if  _ip_format.match(_ipAddr):
                        found_dev = robotMeca500.find_avilable(_ipAddr)
                        if found_dev:
                            guiIND = list(MCDMC.values()).index(_ipAddr)+1
                            # guiIND = MCDMC.index(_ipAddr)+1
                            dev_mcdmc = robotMeca500(_devName, _ipAddr,  self.__params_table)
                            i_dev = CDev(DevType.MCDMC, _ipAddr, None, \
                                            None, dev_mcdmc, guiIND)
                            _sysDevs[_devName] = i_dev
                            # devs.append(i_dev)
                            print_log(f'Mecademic robot / MECA500 at ip = {_ipAddr} was succesfully added')
                            
                        else:
                            print_err(f'Mecademic robot at ip = {_ipAddr} can not be detected')
                    else:
                        print_err(f'Wrong configuration format: Mecademic robot = {_ipAddr} ')                
        
        except Exception as ex:
            print_log(f"Fail to add Mecademic device. Exception: {ex}")
            exptTrace(ex)

        
    def __addMarcoDevs(self, _sysDevs:systemDevices):  
        if 'MARCO' not in self.allDevs.keys():
            print_log(f'No ModBus Marco devices defined in the configuration')
            return
        MARCO = self.allDevs['MARCO']     # {devName: 'IP:port'} dictionary  
        print_log('Looking for ModBus Marco')
        
        try:
            for _devName, connection in MARCO.items():  # 
                ip_port = re.compile(r'(?:\d{1,3}\.){3}\d{1,3}:\d{1,5}')
                if  ip_port.match(connection):
                    sCon = connection.split(':')
                    ip = sCon[0]
                    port = sCon[1]
                    found_dev = Marco_modbus.find_server(ip, port)
                    if found_dev:

                        # guiIND = MARCO.index(connection)+1
                        guiIND = list(MARCO.values()).index(connection)+1
                        dev_marco = Marco_modbus(ip, port, self.__params_table, _devName )
                        i_dev = CDev(DevType.MARCO, connection, None, None, dev_marco, guiIND)
                        _sysDevs[_devName] = i_dev
                        # devs.append(i_dev)
                        print_log(f'Marco ModBus device {_devName} with ip = {ip}, port = {port}, i_dev={i_dev}, guiIND={guiIND}, C_port = {connection}  succesfully added')
                        
                    else:
                        print_err(f'Marco ModBus device {_devName} at {connection} can not be detected')
                else:
                    print_err(f'Wrong configuration format to connect ModBus server / Marco dispenser = {connection}')
        
        except Exception as ex:
            print_log(f"Fail to add ModBus device. Exception: {ex}")
            exptTrace(ex)


    def __addDBDev(self, _sysDevs:systemDevices):

        print_log('Adding DB')
        try:
            _db = assign_parm('DB', self.__params_table, 'DB_NAME', 'production.db')
            _tbl = assign_parm('DB', self.__params_table, 'TBL_NAME', 'statistic')
            dev_DB = StatisticSQLite('DB', _db, _tbl)
            i_dev = CDev(DevType.DB, C_port=None, c_id=None, c_serialN=None, dev = dev_DB, c_gui=None)
            
            if not dev_DB.StartDB():
                raise Exception(f'Error initiation DB operation')
            
            _sysDevs['DB'] = i_dev
            # devs.append(i_dev)
            print_log(f'Added DB = {_db}, Table - {_tbl}')

        except Exception as ex:
            print_log(f"Fail to adding DB. Exception: {ex}")
            exptTrace(ex)




def get_dev(dev_descr:str, devs_list: list[CDev]):
    for dev in devs_list:
        if dev_descr == dev.C_type:
            return dev.get_device()
    return None


def port_scan(configuration_file = 'serials.yml', params_file = 'params.yml')->list[CDev]: 

    
    devs: list[CDev] = list()
    

    # load_dev_config(configuration_file)

    params_table = read_params(params_file)

    print_log('Scanning serial comunnication (COM) ports')

      

                                    # Serial ports scanning for Zaber & Faulhaber v 2.5
    for ps in list(serial.tools.list_ports.comports()):
        

        print_log ("========================================")
        print_log (f"name={ps.name}")
        print_log (f"device={ps.device}")
        print_log (f"description={ps.description}")
        print_log (f"hwid={ps.hwid}")
        print_log (f"vid={ps.vid}")
        print_log (f"serial_number={ps.serial_number}")
        print_log (f"location={ps.location}")
        print_log (f"manufacturer={ps.manufacturer}")
        print_log (f"product={ps.product}")
        print_log (f"interface={ps.interface}")

        if not ((ps.vid == FAULHABER_ID) or (ps.vid == ZABER_ID)):
            continue 

        vendor = 'ZABER' if ps.vid == ZABER_ID else 'FAULHABER'
        type = None
        start_pos = 0

        if ps.vid == FAULHABER_ID:

            try:
                ser = serial.Serial(port=ps.device, baudrate = fh_baudrate, timeout = fh_timeout)
                name = ser.name
                print_log (f"name = {name}")
                ser.send_break(duration = fh_break_duration)                   
                answ = ser.readline()
                print_log(f'Serial port init returns - {answ}')
                if not answ.__str__().__contains__("FAULHABER"):
                    print_log(f'FAULHABBER on port {ps.device} is NOT ACTIVE')
                    ser.close()
                    ser.__del__()
                    continue
                res, answ = FH_cmd(ser, 'en')
                res, answ = FH_cmd(ser, 'GRC')
                current_A = int(answ)
                print_log(f'Electric current read result = {answ}/{current_A}')
                
                print_log(f" digital GRC={current_A}")

                res, answ = FH_cmd(ser, 'GSER')
                serN = int(answ)
                print_log(f'Serial number = {answ}/{serN}')
                
                res, answ = FH_cmd(ser, 'POS')
                start_pos = int (answ)
                print_log(f'Current actual position = {answ}/{start_pos}')
                res, answ = FH_cmd(ser, 'GU')
                print_log(f'PWM value = {answ}')



                res, answ = FH_cmd(ser, 'di')
                ser.close()
                ser.__del__()

            except Exception as ex:
                print_log(f"Connection to port {ps.device} was lost")
                print_log(f"Unexpected Exception: {ex}")
                continue
            else:
                if serN in TROLLEY:
                    # i_dev = CDev(DevType.TROLLEY, ps.device, ps.vid, ps.serial_number, TROLLEY.index(serN)+1)
                    i_dev = CDev(DevType.TROLLEY, ps.device, ps.vid, serN, TROLLEY.index(serN)+1)
                    dev_trolley = FH_Motor(ps.device, fh_baudrate, fh_timeout, params_table, f'T{TROLLEY.index(serN)+1}',  cur_pos = start_pos)
                    i_dev.dev_mDC = dev_trolley

                    if not dev_trolley.init_dev(i_dev.C_type):
                        print_log(f'Trolley initiation failed on port {ps.device}')
                        del dev_trolley
                        continue

                    devs.append(i_dev)

                elif serN in GRIPPER:
                    # i_dev = CDev(DevType.GRIPPER, ps.device, ps.vid, ps.serial_number, GRIPPER.index(serN)+1)
                    i_dev = CDev(DevType.GRIPPER, ps.device, ps.vid, serN, GRIPPER.index(serN)+1)
                    dev_gripper = FH_Motor(ps.device, fh_baudrate, fh_timeout, params_table, f'G{GRIPPER.index(serN)+1}')
                    i_dev.dev_mDC = dev_gripper

                    if not dev_gripper.init_dev(i_dev.C_type):
                        print_log(f'Gripper initiation failed on port {ps.device}')
                        del dev_gripper
                        continue

                    devs.append(i_dev)

                elif serN in SPINNER:
                    i_dev = CDev(DevType.TIME_ROTATOR, ps.device, ps.vid, serN, SPINNER.index(serN)+1)
                    i_dev.dev_mDC = FH_Motor(ps.device, fh_baudrate, fh_timeout, params_table, f'S{SPINNER.index(serN)+1}')

                    if not i_dev.dev_mDC.init_dev(i_dev.C_type):
                        print_log(f'TIME ROTATOR initiation failed on port {ps.device}')
                        del i_dev.dev_mDC
                        continue

                    devs.append(i_dev)
                else :
                    print_log (f'Undefined device on port {ps.device} with s/n {serN}')
                    continue
                
                print_log(f'Added FAULHABER device on port {ps.device} with s/n {serN}')
                

            finally:
                pass
            
        if ps.vid == ZABER_ID:
            if ps.description.split()[0] == 'HAMEG':
                try:
        
                    for ind, sn in enumerate(HMP):
               
                        if ps.description.split()[0] == sn:
                            print_log(f'Trying add HMP device on port {ps.device}, s/n = {sn}')
                            guiIND = HMP.index(sn)+1
                            i_dev = CDev(DevType.HMP, ps.device, ps.description, \
                                            ps.serial_number, guiIND)
                            i_dev.dev_hmp = HMP_PS(sn=sn, port=ps.device, parms=params_table)
                            devs.append(i_dev)
                            print_log(f'HAMEG device with SN = {sn} succesfully added')
                            
                        else:
                            print_err(f'HAMEG device with SN = {sn} can not be detected')

    
                except Exception as ex:
                    print_log(f"Fail to add HAMEG device. Exception: {ex}")
                    exptTrace(ex)
            elif not ((sn := DH_ROB_RGI_modbus.find_server(ps.device)) == None):

                print_log(f'Trying add DH Robotics device on port {ps.device}, s/n = {sn}')
                if not sn in DH:
                    print_log(f'Found device with sn = {sn} is not configured in the system')
                else:
                    guiIND = DH.index(sn)+1
                    i_dev = CDev(DevType.DH, ps.device, None, \
                                    sn, guiIND)
                    i_dev.dev_mDC = DH_ROB_RGI_modbus(sn=sn, port=ps.device, d_name = f'D{DH.index(sn)+1}', parms=params_table)
                    if not i_dev.dev_mDC.init_dev(i_dev.C_type):
                        print_err(f'Can not initiate DH Robotics devivce on port {ps.device}')
                    else:
                        devs.append(i_dev)
                        print_log(f'DH Robotics device with SN = {sn} succesfully added on port {ps.device}')

            else:               # ZABER
                try:
                    
                    if not Zaber_Motor.init_devs(ps.device):
                        print_log(f'Error initiation ZABER.')
                        continue

                    z_len = len(Zaber_Motor.device_port_list[ps.device]["device_list"])
                    print_log (f'{z_len} ZABER devices will be added')

                    for ind in range (z_len):
                            
                        axes, device_id, identity, d_name, d_serial_number = Zaber_Motor.GetInfo(ind, ps.device)


                        print_log (f'Device has {axes} axes')
                        print_log (f'DeviceID: {device_id}')
                        print_log (f'Device identity: {identity}')
                        print_log (f'Device name: {d_name}')
                        serN = int(d_serial_number)
                        print_log (f'Device serialNumber: {d_serial_number}/{serN}')
                        
                        if serN in ZABER:
                            i_dev = CDev(DevType.ZABER, ps.device, device_id, serN, \
                                                ZABER.index(d_serial_number)+1)
                            i_dev.dev_zaber = Zaber_Motor(ind, ps.device, params_table, f'Z{ZABER.index(serN)+1}')
                            devs.append(i_dev)
                            print_log(f'Added ZABER device on port {ps.device} with s/n {serN}')
                        else :
                            print_log (f'Undefined device on port {ps.device} with s/n {serN}')

                
                
                except MotionLibException as ex:
                    print_log(f'There no ZABER devices on port {ps.device}')
                    print_log(f"Exception: {ex}")
                    continue

                # except NoDeviceFoundException as ex:
                #     print_log(f'There no ZABER devices on port {ps.device}')
                #     print_log(f"Exception: {ex}")
                #     continue

                # except ConnectionFailedException as ex:
                #     print_log(f'Connection lossed on port {ps.device}')
                #     print_log(f"Exception: {ex} of type: {type(ex)}")
                #     continue

                except Exception as ex:
                    print_log(f"Unexpected Exception: {ex}")
                    continue
                else:
                    pass

                finally:
                    pass
                
                # Zaber_Motor.stop()
                    
      

    print_log('Scanning FHv3 comunnication (USB) ports')
    
    try:
        FHv3Devs = FH_Motor_v3.init_devices()
        print_log(f'FHv3 devs - {FHv3Devs}')

        if FHv3Devs == None or len(FHv3Devs) == 0:
            print_log(f'No Faulhaber v3 devices were found')
        else:
            FHv3_len = len(FHv3Devs)

            print_log (f'{FHv3_len} Faulhaber v3 devices could be added')

            for ind in range(len(FHv3Devs)):
                devFHv3 = FHv3Devs[ind]
                print_log (f'{ind} -> {devFHv3.devinfo}, s/n = {devFHv3.serialN}')

                
                if devFHv3.serialN in ROTATOR:
                    devStr = DevType.DIST_ROTATOR
                    guiIND = ROTATOR.index(devFHv3.serialN)+1
                    devName = f'R{guiIND}'
                    
                elif devFHv3.serialN in SPINNER:
                    devStr = DevType.TIME_ROTATOR
                    guiIND = SPINNER.index(devFHv3.serialN)+1
                    devName = f'S{guiIND}'

                elif devFHv3.serialN in GRIPPER:
                    devStr = DevType.GRIPPERv3
                    guiIND = GRIPPER.index(devFHv3.serialN)+1
                    devName = f'G{guiIND}'

                elif devFHv3.serialN in TROLLEY:
                    devStr = DevType.TROLLEY
                    guiIND = TROLLEY.index(devFHv3.serialN)+1
                    devName = f'T{guiIND}'
                
                else :
                    print_log (f'Undefined FHv3 device: {devFHv3}')
                    continue

                i_dev = CDev(devStr, devFHv3.port, devFHv3.devinfo, \
                                 devFHv3.serialN, guiIND)
                i_dev.dev_mDC = FH_Motor_v3(int(devFHv3.port), int(devFHv3.channel), params_table, devName)
                devs.append(i_dev)
                print_log(f'Added FHv3 device: {devFHv3}')


                # if not i_dev.dev_mDCv3.init_dev(i_dev.C_type):
                if not i_dev.dev_mDC.init_dev(i_dev.C_type):
                    print_log(f'FHv3 initiation failed for device: {i_dev}')
                    del i_dev
                    continue



    except Exception as ex:
        print_log(f"Fail to add FHv3 device. Unexpected Exception: {ex}")
        exptTrace(ex)
        


    print_log('Scanning MAXON ports')

    try:
        mxnDev = bytes(params_table['DEAFULT']['MAXON_DEV'], 'utf-8')
        mxnIntfc = bytes(params_table['DEAFULT']['MAXON_INTERFACE'], 'utf-8')
        MAXONDevs = MAXON_Motor.init_devices(mxnDev, mxnIntfc)
        print_log(f'MAXON devs - {MAXONDevs}')

        if MAXONDevs == None or len(MAXONDevs) == 0:
            print_log(f'No MAXON devices were found')
        else:
            MAXON_len = len(MAXONDevs)

            print_log (f'{MAXON_len} MAXON devices could be added')

            for ind in range(len(MAXONDevs)):
                devMAXON = MAXONDevs[ind]
                print_log (f'{ind} -> {devMAXON}, s/n = {devMAXON.sn}')

                
                if devMAXON.sn in ROTATOR:
                    devStr = DevType.DIST_ROTATOR
                    guiIND = ROTATOR.index(devMAXON.sn)+1
                    devName = f'R{guiIND}'
                    
                elif devMAXON.sn in SPINNER:
                    devStr = DevType.TIME_ROTATOR
                    guiIND = SPINNER.index(devMAXON.sn)+1
                    devName = f'S{guiIND}'

                elif devMAXON.sn in GRIPPER:
                    devStr = DevType.GRIPPERv3
                    guiIND = GRIPPER.index(devMAXON.sn)+1
                    devName = f'G{guiIND}'

                elif devMAXON.sn in TROLLEY:
                    devStr = DevType.TROLLEY
                    guiIND = TROLLEY.index(devMAXON.sn)+1
                    devName = f'T{guiIND}'
                
                else :
                    print_log (f'Undefined MAXON device: {devMAXON}')
                    continue

                i_dev = CDev(devStr, devMAXON.port, devMAXON, \
                                 devMAXON.sn, guiIND)
                i_dev.dev_mDC = MAXON_Motor(devMAXON, params_table, devName)
                devs.append(i_dev)
                print_log(f'Added MAXON device: {devMAXON}')


                # if not i_dev.dev_mDCv3.init_dev(i_dev.C_type):
                if not i_dev.dev_mDC.init_dev(i_dev.C_type):         
                    print_log(f'MAXON initiation failed for device: {i_dev}')
                    del i_dev
                    continue



    except Exception as ex:
        print_log(f"Fail to add MAXON device. Unexpected Exception: {ex}")
        exptTrace(ex)

    print_log('Looking for NI devices')
    
    try:
        
        for ind, sn in enumerate(NI):
            found_dev = NI6002.find_devs(sn)
            if found_dev:
                # guiIND = ind + 1
                guiIND = NI.index(sn)+1
                i_dev = CDev(DevType.DAQ-NI, found_dev.device, found_dev.model, \
                                 found_dev.sn, guiIND)
                i_dev.dev_ni = NI6002( f'DAQ{guiIND}', sn, params_table)
                devs.append(i_dev)
                print_log(f'NI device with SN = {hex(sn)} ({sn}) succesfully added')

##########################                

                print_log(f'Adding Interlock for NI DAQ{guiIND}')
                for interIndex, interDev in enumerate(INTERLOCK):
                    if interDev == f'DAQ{guiIND}':
                        intlck_dev = CDev(DevType.INTERLOCK, None, None, \
                                 None, interIndex+1)
                        intlck_dev.dev_interlock = InterLock(f'LCK{interIndex+1}', interDev, params_table)
                        devs.append(intlck_dev)
                        print_log(f'Interlock device {intlck_dev} assosiated with NI {i_dev} was added')

##########################                
            else:
                print_err(f'NI device with SN = {hex(sn)} ({sn}) can not be detected')

    
    except Exception as ex:
        print_log(f"Fail to add NI device. Exception: {ex}")
        exptTrace(ex)

    print_log(f'Looking for FESTO in stepper group: \n{ZABER}  ')
    try:
        print_log(f'Looking for  FESTO devs in neighbour nodes ')
        Festo_Motor.enum_devices()
        for ind, ip in enumerate(ZABER):
            # _f_rex = re.compile(r'(?:\d{1,3}\.){3}\d{1,3}')
            _f_rex = re.compile(r'(?:\d{1,3}\.){3}\d{1,3}')
            if _f_rex.match(str(ip)):
                _sn = Festo_Motor.find_dev(ip)
                if _sn is None:
                    print_err(f'No FESTO device found at IP = {ip}')
                    continue

                i_dev = CDev(DevType.ZABER, ip, '502', _sn, \
                                                ZABER.index(ip)+1)
                i_dev.dev_zaber = Festo_Motor(_sn, ip, f'Z{ZABER.index(serN)+1}', params_table)
                devs.append(i_dev)
                print_log(f'Added FESTO device on ip {ip} with s/n {_sn}, name = Z{ZABER.index(serN)+1}')


    except Exception as ex:
        exptTrace(ex)
        print_log(f"Fail while adding FESTO devs. Exception: {ex}")

    print_log(f'Looking for IO devs: \n{io_dev_dict} ')
    
    try:
        chk_lst = list()
        
        for dev_name, _io_ctl in io_dev_dict.items():
            io_dev_cnf = re.compile(r'^\s*\w+\s*,\s*port\d.line\d\s*(,\s*(NO|NC)\s*)?$')
            if  not io_dev_cnf.match(_io_ctl):
                raise Exception(f'Wrong format configuring IO control: {_io_ctl}')
            
            _pars = _io_ctl.split(',')
            __io_provider = _pars[0].strip()
            port_line = _pars[1].strip()
            _pl = port_line.split('.')
            __port = int(_pl[0][4])
            __line = int(_pl[1][4])

            _NO = True
            _nonc = None
            if len(_pars) == 2:
                _NO = True
            elif len(_pars) == 3:
                _nonc = _pars[2].strip()
                if  _nonc.upper() == 'NC':
                # if  _nonc == 'NC':
                    _NO = False
                else:
                    _NO = True
            i_dev = CDev(DevType.IOCONTROL, C_port=_pl, c_id=__io_provider, c_serialN=0, c_gui=None)
            i_dev.dev_iocontrol = IOcontrol(dev_name, __io_provider, __line, __port, params_table, _NO)
            devs.append(i_dev)

            print_log(f'IO control ({_pars}/{__io_provider}, {__port}.{__line}, {_NO} [{_nonc}] len = {len(_pars)}) was added for name= {dev_name} dev = {__io_provider}, port={__port}, line = {__line}, NO/^NC = {_NO}')
    
    except Exception as ex:
        exptTrace(ex)
        print_log(f"Fail to add IO CONTROL. Exception: {ex}")

    print_log(f'Looking for Phidgets: \n{PHG}  \n{phg_dev_dict} ')
    
    try:
        chk_lst = list()
        for ind, connection in enumerate(PHG):

            dev_chan = re.compile(r'^\d+/\d\s*,\s*(TRIGGER|ONOFF|TOGGLE)\s*(,\s*(TRUE|FALSE))?\s*$')
            if  dev_chan.match(connection):
                conf_con = connection.split(',')
                _sw_type = conf_con[1].strip() 
                sCon = conf_con[0].split('/')
                sn = sCon[0]
                chan = sCon[1]
                print_log(f'Looking dev. sn = {sn}, channel = {chan}')
                if connection in chk_lst:
                    print_err(f'-ERROR -- SN:Channel {connection} appearce more than once for Phidget device. ')
                    break
                else:
                    chk_lst.append(connection)

                found_dev = PhidgetRELAY.find_devs(sn, chan)
                if found_dev: 
                    _dName = None
                    guiIND = PHG.index(connection)+1
                    for _itm in phg_dev_dict.keys():
                        _dName = _itm
                        if phg_dev_dict[_itm] == connection:
                            _dType = None
                            match _sw_type:
                                case 'TRIGGER':
                                    _dType = PhidgetRELAY.devType.trigger
                                    break
                                case 'TOGGLE':
                                    _dType = PhidgetRELAY.devType.toggle
                                    break
                                case 'ONOFF':
                                    _dType = PhidgetRELAY.devType.onoff
                                    break
                                case _:
                                    print_err(f'ERROR -- unknown type: *{_sw_type}*. Device will not be added')
                                    break




                            # match _itm:
                            #     case 'DISP':
                            #         _dType = PhidgetRELAY.devType.dispenser
                            #         break
                            #     case 'UV':
                            #         _dType = PhidgetRELAY.devType.uv
                            #         break
                            #     case 'FRONTLIGHT':
                            #         _dType = PhidgetRELAY.devType.front_light
                            #         break

                            #     case 'BACKLIGHT':
                            #         _dType = PhidgetRELAY.devType.back_light
                            #         break

                            #     case 'HOLELIGHT':
                            #         _dType = PhidgetRELAY.devType.hole_light
                            #         break

                            #     case 'WELDER':
                            #         _dType = PhidgetRELAY.devType.welder
                            #         break
                                
                            #     case 'JTSE_HOTAIR':
                            #         _dType = PhidgetRELAY.devType.jtse_hotair
                            #         break

                            #     case _:
                            #         print_err(f'ERROR -- unknown type: {_itm}. Device will not be added')
                            #         break
                    if _dType:
                        i_dev = CDev(DevType.PHG, found_dev.channelName, found_dev.channelClassName, \
                                        found_dev.devSN, guiIND)
                        i_dev.dev_phg = PhidgetRELAY(sn, chan, _dType, _dName, params_table)
                        devs.append(i_dev)
                        print_log(f'PHG device with SN/channel = {connection}, name = {_dName}, type = {_dType}/{_itm} succesfully added')
                    else:
                        print_err(f'Error adding device:  SN/channel = {connection}, name = {_dName}, type = {_dType}/{_itm}')
                else:
                    print_err(f'PHG device with SN/channel = {connection}, SN = {sn}, channel = {chan} was not found')
            else:
                print_log(f'Wrong format declaring Phidget: {connection}')
    
    except Exception as ex:
        print_log(f"Fail to add PNG device. Exception: {ex}")
        exptTrace(ex)




    print_log('Looking for ModBus servers / Camera')
    
    try:
        for ind, connection in enumerate(CAM):
            ip_port = re.compile(r'(?:\d{1,3}\.){3}\d{1,3}:\d{1,5}')
            if  ip_port.match(connection):
                sCon = connection.split(':')
                ip = sCon[0]
                port = sCon[1]
                found_dev = Cam_modbus.find_server(ip, port)
                if found_dev:
                    # guiIND = ind + 1
                    guiIND = CAM.index(connection)+1
                    i_dev = CDev(DevType.CAM, connection, None, \
                                    None, guiIND)
                    i_dev.dev_cam = Cam_modbus(ip, port, params_table, f'CAM{guiIND}')
                    devs.append(i_dev)
                    print_log(f'ModBus device with ip = {ip}, port = {port} succesfully added')
                    
                else:
                    print_err(f'ModBus device at {connection} can not be detected')
            else:
                print_err(f'Wrong configuration format to connect ModBus server / Camera = {connection}')
    
    except Exception as ex:
        print_log(f"Fail to add ModBus device. Exception: {ex}")
        exptTrace(ex)


    print_log('Looking for Mecademic / Asyril')
    
    try:
        for ind, _ipAddr in enumerate(MCDMC):
            _ip_format = re.compile(r'(?:\d{1,3}\.){3}\d{1,3}')
            # parm_re = re.compile(r'\b(?:\d{1,3}\.){3}\d{1,3}:\d{1,5}\b')
            parm_re = re.compile(r'\b(?:\d{1,3}\.){3}\d{1,3}:\d{1,5}/\d{1,5}\b')
            # _asyrilIPStr = get_parm(f'MCDMC{ind+1}', params_table, 'ASYRIL')

            _asyrilIPStr = assign_parm(f'MCDMC{ind+1}', params_table, 'ASYRIL',None)
            __asyrilIPort = None if _asyrilIPStr is None else _asyrilIPStr.split('/')[0]
            _robot_type = assign_parm(f'MCDMC{ind+1}', params_table, 'TYPE', 'MECA500')

            if _asyrilIPStr is not None and _robot_type == 'MCS500':
                                                                    # Asyril is defined and SCARA robot (MCS500)
                print_log(F'Checking  for Mecademic / MCS500 ({_ipAddr}) / Asyril {_asyrilIPStr} together')
                if  _ip_format.match(_ipAddr) and parm_re.match(_asyrilIPStr):
                    found_dev = robotMecademic.find_avilable(_ipAddr, __asyrilIPort)
                    if found_dev:
                        guiIND = MCDMC.index(_ipAddr)+1
                        i_dev = CDev(DevType.MCDMC, _ipAddr, None, \
                                        None, guiIND)
                        i_dev.dev_mcdmc = robotMecademic(f'MCDMC{guiIND}', _ipAddr,  params_table)
                        devs.append(i_dev)
                        print_log(f'Mecademic robot at ip = {_ipAddr}, assosiated with Asyril at {_asyrilIPStr}  were succesfully added')
                        
                    else:
                        print_err(f'Mecademic robot at ip = {_ipAddr}  or assosiated with Asyril at {_asyrilIPStr} can not be detected')
                else:
                    print_err(f'Wrong configuration format: Mecademic robot = {_ipAddr} or Asyril = {_asyrilIPStr}')
            elif _robot_type == 'MECA500':
                print_log(F'Checking  for Mecademic / MECA500 ({_ipAddr}) robot availability')
                if  _ip_format.match(_ipAddr):
                    found_dev = robotMeca500.find_avilable(_ipAddr)
                    if found_dev:
                        guiIND = MCDMC.index(_ipAddr)+1
                        i_dev = CDev(DevType.MCDMC, _ipAddr, None, \
                                        None, guiIND)
                        i_dev.dev_mcdmc = robotMeca500(f'MCDMC{guiIND}', _ipAddr,  params_table)
                        devs.append(i_dev)
                        print_log(f'Mecademic robot / MECA500 at ip = {_ipAddr} was succesfully added')
                        
                    else:
                        print_err(f'Mecademic robot at ip = {_ipAddr} can not be detected')
                else:
                    print_err(f'Wrong configuration format: Mecademic robot = {_ipAddr} ')                
    
    except Exception as ex:
        print_log(f"Fail to add Mecademic device. Exception: {ex}")
        exptTrace(ex)

    
    print_log('Looking for ModBus Marco')
    
    try:
        for ind, connection in enumerate(MARCO):
            ip_port = re.compile(r'(?:\d{1,3}\.){3}\d{1,3}:\d{1,5}')
            if  ip_port.match(connection):
                sCon = connection.split(':')
                ip = sCon[0]
                port = sCon[1]
                found_dev = Marco_modbus.find_server(ip, port)
                if found_dev:

                    guiIND = MARCO.index(connection)+1
                    i_dev = CDev(DevType.MARCO, connection, None, \
                                    None, guiIND)
                    i_dev.dev_marco = Marco_modbus(ip, port, params_table, f'DISP{guiIND}' )
                    devs.append(i_dev)
                    print_log(f'Marco ModBus device with ip = {ip}, port = {port}, i_dev={i_dev}, guiIND={guiIND}, C_port = {connection}  succesfully added')
                    
                else:
                    print_err(f'Marco ModBus device at {connection} can not be detected')
            else:
                print_err(f'Wrong configuration format to connect ModBus server / Marco dispenser = {connection}')
    
    except Exception as ex:
        print_log(f"Fail to add ModBus device. Exception: {ex}")
        exptTrace(ex)

    
    if len(JTSE) > 0:
        print_log(f'Looking for {JTSE[0]} blower')
        try:
            _com = JTSEcontrol.findDev(JTSE[0])
            if _com is None or _com == '':
                print_log('No JTSE found')
            else:
                i_dev = CDev(DevType.JTSE, _com, None, \
                                    None, None)
                i_dev.dev_jtse = JTSEcontrol(JTSE[0], _com)
                devs.append(i_dev)
                print_log(f'JBC/JTSE dev ({JTSE[0]}) at port {_com} succesfully added')

        except Exception as ex:
            print_log(f"Fail to adding JTSE. Exception: {ex}")
            exptTrace(ex)
    else:
        print_log(f'No JTSE blower configured')


    print_log('Adding DB')
    try:
        _db = assign_parm('DB', params_table, 'DB_NAME', 'production.db')
        _tbl = assign_parm('DB', params_table, 'TBL_NAME', 'statistic')
        i_dev = CDev(DevType.DB, C_port=None, c_id=None, c_serialN=None, c_gui=None)
        i_dev.dev_DB = StatisticSQLite('DB', _db, _tbl)
        if not i_dev.dev_DB.StartDB():
            raise Exception(f'Error initiation DB operation')
        
        devs.append(i_dev)
        print_log(f'Added DB = {_db}, Table - {_tbl}')

    except Exception as ex:
        print_log(f"Fail to adding DB. Exception: {ex}")
        exptTrace(ex)

    return devs

def read_params(params_file = 'params.yml') -> dict:
    print_inf(f'Read params from file {params_file}')
    params_table = dict()
    
    try:
        with open(params_file) as p_file:
            doc = yaml.safe_load(p_file)
            for dev, dev_parms in doc.items():       # devivce parm is a dictionary 
                params_table[dev] = dict()
                # print_log(f'dev={dev} -> dev_parms = {dev_parms}')
                for i, parm in enumerate(dev_parms):
                    # print_log(f'{i} -> {parm} = {dev_parms[parm]}')
                    params_table[dev][parm] = dev_parms[parm]

            print_log(f'params_table={params_table}')

                 

    except Exception as ex:
        print_err(f'Error reading parameters file, exception = {ex}')
        exptTrace(ex)
        pass
    
    return params_table

def print_dev_parms(parms_table, dev_name):
    if dev_name in parms_table.keys():
        val = parms_table[dev_name]
        t_len = len(parms_table[dev_name])
        print_log(f'{dev_name} -> {val},  t_len = {t_len}')
        for enum_par in parms_table[dev_name]:
            print_log(f"{enum_par} = {parms_table[dev_name][{enum_par}]}")
    else:
        print_log(f'No parameters for dervive {dev_name}')

    pass
    

#------------------------- U N I T E S T ----------------------------
if __name__ == "__main__":
    def str2tuple(_str:str) -> tuple:
        if _str is None:
            return 0, 0, 0
        
        _str = re.sub(r"\s+", "", _str)
        return tuple(map(float, _str.split(',')))
    
    def str2ip(_str:str) -> tuple:
        if _str is None:
            return 0, 0
        
        _str = re.sub(r"\s+", "", _str)
        return tuple(map(str, _str.split(':')))

    print_log ('Starting')

    # load_dev_config('serials.yml')
    # sys.exit()

    a = systemDevices()
    sys.exit()
    
    devs = port_scan()
    print_log(f'Ports scanning done. Found {len(devs)} devices')
    for op_dev in devs:
        print_log (f'Deleting {op_dev}')
        del op_dev
    print_log(f'EOP')

    parms = read_params('params.yml')
   

    print(f"WORK_ZONE_LIMIT_XYZ_MIN = {str2tuple(get_parm('MCDMC1', parms, 'WORK_ZONE_LIMIT_XYZ_MIN'))}")
    print(f"WORK_ZONE_LIMIT_XYZ_MAX = {str2tuple(get_parm('MCDMC1', parms, 'WORK_ZONE_LIMIT_XYZ_MAX'))}")
    print(f"RELIEF_POSITION = {str2tuple(get_parm('MCDMC1', parms, 'RELIEF_POSITION'))}")
    print(f"MID_JOINTS_POSITION = {str2tuple(get_parm('MCDMC1', parms, 'MID_JOINTS_POSITION'))}")
    print(f"PICK_ANGLE_RANGE = {str2tuple(get_parm('MCDMC1', parms, 'PICK_ANGLE_RANGE'))}")
    print(f"START_POSITION = {str2tuple(get_parm('MCDMC1', parms, 'START_POSITION'))}")
    print(f"ASYRIL = {str2ip(get_parm('MCDMC1', parms, 'ASYRIL'))}")



#----- Loading dev anumated gif -----
gif103 = b'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'
                