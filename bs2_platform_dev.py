from __future__ import annotations


__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "5.1.0"
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
from typing import TypeVar, Generic, TypeAlias, Any
import weakref


from bs1_ads import commADS, symbolsADS
from bs1_plc_dev import PLCNode
from bs1_sysdev import sysDevice
from bs1_marco_modbus import Marco_modbus
from bs1_base_motor import BaseMotor, BaseDev
from bs1_DH_RB_modbus import DH_ROB_RGI_modbus

from bs1_base_motor import BaseDev
from bs1_faulhaber import FH_Motor, FH_cmd, fh_baudrate, fh_timeout, fh_break_duration
from bs1_ni6002 import NI6002
from bs1_HAMEG import HMP_PS
from bs1_interlock import InterLock
from bs1_io_control import IOcontrol
from bs1_festo_modbus import Festo_Motor
from bs1_sqlite import StatisticSQLite 
from bs1_jtse_serial import JTSEcontrol

from bs2_DSL_cmd import DevType, confDevCmd
from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, void_f, \
                        assign_parm, get_parm  



class abstractNode(ABC):
    _devRegistry = weakref.WeakValueDictionary()

    def __init__(self, __conf_file:str=None, _params_file:str=None):

        self.__device: BaseDev = None

    
   # bracket notation for getting/setting devices by logical name (example: sysDevs['T1'] / sysDevs['G1'] = CDev(...) ) 
   # where 'T1', 'G1' are device logical names defined in the configuration file
   # and CDev is the container class for the physical device
    def __getitem__(self, _devType):
        return None if _devType not in list(self.platformDevs.keys()) else self.platformDevs[_devType]
    
    def __setitem__(self, _devType, _dev):
        raise Exception(f'Unsupported bracket notation ([]) operation for {self} object')

    
    def getDevs(self):
        return self.platformDevs

    # factory method to load physical devices from configuration for the specific provider (PC) 
    # or specific controllet (PLC platform)    
    @classmethod
    @abstractmethod
    def load_devices(cls, physicalDevsinConfig: devConfigType) -> dict[str, abstractNode]:
        raise NotImplementedError('load_devices method must be implemented in the derived class')
    
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


class plcPlatformNode(abstractNode):
    _max_num_of_devs:int = None

    def __init__(self, _config_file:str = None, _params_file:str = None, _ads_netid:str = None, _remote_ip:str = None):

        super().__init__()
        self.ADS_NETID = _ads_netid
        self.REMOTE_IP = _remote_ip
        self._ads:commADS = None
        self._plcDevs:dict = dict()   # the way it presented in PLC (JSON)
        self.number_of_devs:int = 0

        self.__config_file = _config_file
        try:
         
            if self.ADS_NETID is None or self.REMOTE_IP is None:
                raise Exception(f'Error: ADS_NETID or REMOTE_IP are not defined ')
                
            _ams_re = r'\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)(?:\.1\.1)\b'
            _remip_re  = r'\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\b'
            _ams_re_compiled = re.compile(_ams_re)
            _remip_re_compiled = re.compile(_remip_re)
            if not _ams_re_compiled.match(self.ADS_NETID) or not _remip_re_compiled.match(self.REMOTE_IP):
                raise Exception(f'Error: Wrong ADS_NETID ({self.ADS_NETID}) or REMOTE_IP ({self.REMOTE_IP}) format in the configuration file {_config_file}')
            
            self._ads = commADS(self.ADS_NETID, self.REMOTE_IP)

            # plcPlatformNode._max_num_of_devs = self._ads.readVar(symbol_name=symbolsADS._max_num_of_devs, var_type=int)
            
            self.read_configuration()


        except Exception as ex:
            print_err(f'Error reading configuration file, exception = {ex}')
            exptTrace(ex)
            raise ex
        

    def createPlatformDev(self, _sysDevs:systemDevices, dev_name:str, plc_dev_name:str)->BaseDev:
        # create CDev objects for each PLC device and add them to the systemDevices object
        try:
            _dev:BaseDev = PLCNode(dev_name=dev_name, plc_dev_name=plc_dev_name, _comADS=self._ads)

        except Exception as ex:
            print_err(f'Error adding PLC platform device, exception = {ex}')
            exptTrace(ex)   
            _dev = None
        
        return _dev

    # read configuration from PLC and create devices
    # the system may be expanded to multyple PLC platforms in the future
    # therefore, read_configuration is a instance method rather than static method  
    def read_configuration(self):
        try: 
            plcPlatformNode._max_num_of_devs = self._ads.readVar(symbol_name=symbolsADS._max_num_of_devs, var_type=int)
            print_log(f'PLC platform max number of devices = {plcPlatformNode._max_num_of_devs}')

            print_log(f'PLC based devices operation. Read configuration from PLC at {self.REMOTE_IP}, AMS NET ID = {self.ADS_NETID}')
            
            _dev_idx = 0

            _devLst = PLCNode.enum_nodes(_comADS=self._ads)
            self.number_of_devs = len(_devLst)
            print_log(f'Found {_devLst} ({self.number_of_devs}) devices in PLC configuration')

            for _dev_idx, _plc_dev_name in enumerate(_devLst):
                _plc_dev_name = _plc_dev_name
                print_log(f'Reading PLC device index {_dev_idx}, PLC device name = {_plc_dev_name}')
                if _plc_dev_name is  None:
                    print_err(f'Unexpexted end of devices list. Error reading device name for device index {_dev_idx}')
                    break
                                # device name got from PLC configuration
                                # _plc_dev_name is the name defined in the PLC configuration


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
PLC:
    PLC_NAME:
        REMOTE_IP: '192.168.10.153'
        ADS_NETID: '192.168.137.1.1.1'

        
# All devices defined by their serial numbers / IP addresses or PLC NAME/Device Name

# Grippers
GR: (Gn: s/n)
    G1: 512000099                          
# Trolleys (Tn: s/n )
TR:
    T1: 162400865                     
# Zaber -- stepper actuator(Zn: s/n)
ZB:
    Z1: 123883
    Z2: PLC_NAME/Coining Axis            # Operated via PLC
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


# IO control lines (Name: Device Name, port.line, NO/NC  or PLC_NAME, PLC_IO_NAME)
IOCONTROL:
    GPI:                            # input  lines
        DOOR: DAQ1, port0.line0
        EMERG: DAQ1, port0.line1, NC
        ON_OFF_BUTTON: DAQ1, port0.line2
        D1_D2_GRIPERS_TOGGLE:  DAQ1, port0.line3
        PISTON_HOME_SENSOR: PLC, PISTON_HOME
    GPO:                           # output lines
        START_BUTTON_LED: DAQ1, port1.line0
        PNEUMOATIC_VALVE_1: PLC, PNEUMATIC_1
        PNEUMOATIC_VALVE_2: PLC, PNEUMATIC_2
    IO:                     # input/output devices (manufacturere/Serial Number  or PLC_NAAME))
        DAQ1: NI, 020DC90D
        PLC: PLC_NAME    

# JTSE blower
JTSE:
    BL: JTSE

'''   

class pcPlatformNode(abstractNode):
    devTypesTbl:dict = {'TR':'T', 'GR':'G', 'ZB':'Z', 'RT':'R', 'SP':'S', 'NI':'DAQ', 'CAM':'CAM', 'HMP':'H', \
                   'DH':'D', 'PHG':'*', 'MCDMC':'MCDMC', 'MARCO':'DISP', 'INTERLOCK':'LCK', 'IOCONTROL_GPI':'*', \
                    'IOCONTROL_GPO':'*','JTSE':'*'}
    freeStyleDevs:set = set()
    def __init__(self, _config_file:str = 'serials.yml', _params_file:str = 'params.yml'):
        
        super().__init__(_config_file, _params_file)
        self.__config_file = _config_file
        self.__params_file = _params_file
        self._serialPorts:dict = dict()   # {port: {'vid':, 'description':, 'hwid':, 'serial_number':, 'location':, 'manufacturer':, 'product':, 'interface':} }
        

        print_log(f'PC based devices operation. Load configuration from {self.__config_file} configuration file')
        self.platformDevs = pcPlatformNode.read_configuration(self.__config_file)
        self.params_table:dict =  pcPlatformNode.read_params(_params_file)  
                                                        # {devName: {parm1: value1, parm2: value2, ...}, ...}


    def loadConf(self, _sysDevs:systemDevices):
        
        print_log(f'Scanning installed devices...')
        self.__addSerialDevs(_sysDevs)
        self.__addFHv3Devs(_sysDevs)
        self.__addMaxonDevs(_sysDevs)
        self.__addFESTODevs(_sysDevs)
        self.__addDAQDevs(_sysDevs)
        self.__addIODevs(_sysDevs)
        self.__addPHGDevs(_sysDevs)
        self.__addCamDevs(_sysDevs)
        self.__addMecademicDevs(_sysDevs)
        self.__addMarcoDevs(_sysDevs)
        self.__addPLCDevs(_sysDevs)
        self.__addDBDev(_sysDevs)
        

    # static method to read configuration file and validate device names/formats
    # it's classmethod because the only one PC platform is available in the system
    @classmethod
    def read_configuration(cls,configuration_file:str) -> dict:
        _dev_class = None
        _devices = None
        _allDevs = dict()
        
        try:
            with open(configuration_file) as conf_file:

                _allDevs = yaml.safe_load(conf_file)
                _iteration_devs = _allDevs.copy()    # copy of the device dictionary
                                                     #  to avoid 'dictionary changed size during iteration' error
                print_log(f'conf. file = {_allDevs}')
                for _dev_class, _devices in _iteration_devs.items():       # devs is a dictionary 
                    if _dev_class == 'PLC':
                        print_log(f'Skipping PLC section {_iteration_devs[_dev_class]} in PC platform configuration file, ')
                        _allDevs.pop(_dev_class)
                        # PLC section is handled by plcPlatformNode class
                        continue
                    elif _dev_class not in cls.devTypesTbl.keys():
                        print_err(f'Error: Unknown device class {_dev_class}={_iteration_devs[_dev_class]}, removed')
                        _allDevs.pop(_dev_class)
                        continue
                        
                    print_log(f'_dev_class = {_dev_class}, _devices = {_devices}')
                    if _devices == None:
                        print_err(f'Error: Empty device list for dev type {_dev_class},  removed')
                        _allDevs.pop(_dev_class)
                        
                    elif not isinstance(_devices, dict) or len(_devices) == 0:
                        print_err(f'Warning: Wrong device list format (or empty) for dev type {_dev_class},  removed')
                        _allDevs.pop(_dev_class)
                        
                    elif cls.devTypesTbl[_dev_class] == '*':
                        # for IO control and Phidgets relay we don't know the device ID format, 
                        # so we just verify name uniqueness
                        _iter_devs = _devices.copy()   # to avoid 'dictionary changed size during iteration' error
                        for _dev in _iter_devs.items():
                            if _dev in cls.freeStyleDevs:   
                                print_err(f'Error: The device {_dev} of dev class {_dev_class} is already defined, removed')
                                _devices.pop(_dev)
                            else:       
                                cls.freeStyleDevs.add(_dev_class)
                        if len(_devices) == 0:
                            print_err(f'No valid devices for dev class {_devices}. Removed')
                            _allDevs.pop(_devices)
                    else:
                        # _dev_re += fr'{cls.devTypesTbl[_dev_class]}\d*'
                        _dev_re = fr'{cls.devTypesTbl[_dev_class]}\d*'  # e.g. T1 for Trolleys
                        _dev_re_compiled = re.compile(_dev_re)
                        _iter_devs = _devices.copy()   # to avoid 'dictionary changed size during iteration' error
                        for _dev, _id in _iter_devs.items():
                            if not _dev_re_compiled.match(_dev):
                                print_err(f'Error. Wrong format dev name {_dev} in dev class {_dev_class}={_devices}, id= {_id}, removed')
                                _devices.pop(_dev)
                        if len(_devices) == 0:
                            print_err(f'No valid devices for dev class {_devices}. Removed')
                            _allDevs.pop(_devices)

                print_log(f'All valid devices in configuration: {_allDevs}')
                print_log(f'FreeStyle devices: {cls.freeStyleDevs}')
                        
            
        except Exception as ex:
            print_log(f'Exception -> {ex} of type {type(ex)}')
            print_log(f'Wrong configuration format in class: {_dev_class} [list: {_devices}]')
            exptTrace(ex)
            
        else:
            print_log(f'')

        return _allDevs



    '''
    Serial devices:
    VID = 2134 (FAULHABER) 
    VID = 1027 (ZABER, HMP, DH)
    JTSE
    '''
    def __addSerialDevs(self, _sysDevs:systemDevices):
        from bs1_jtse_serial import  JTSEcontrol
        from bs1_zaber import Zaber_Motor

        try:

            # Seial port devices: ZB, FAULHABER (TR, GR, SP), HMP, DH, JTSE
            serialDevsClasses = set(['ZB', 'TR', 'GR', 'SP', 'HMP', 'DH', 'JTSE'])
            faulhaberTypes = set([ 'TR', 'GR', 'SP'])
            configuredDevs = set(list(self.platformDevs.keys()))      # set of configured device types

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

                            if self['TR'] is not None and _fhSN in self.platformDevs['TR'].values():
                                _devName = getDevbySN(self.platformDevs['TR'], _fhSN)
                                if _devName is None:
                                    print_err(f'ERROR: Something went wrong while retriving Faulhaber TROLLEY dev name with S/N = {serN} ')
                                    continue


                                dev_trolley = FH_Motor(ps.device, fh_baudrate, fh_timeout, self.params_table, _devName)
                                # i_dev = CDev(DevType.TROLLEY, ps.device, ps.vid, _fhSN, dev_trolley, list(self.platformDevs['TR'].keys()).index(_devName)+1)
                                i_dev = CDev(DevType.TROLLEY, dev_trolley, list(self.platformDevs['TR'].keys()).index(_devName)+1)

                                if not dev_trolley.init_dev(i_dev.C_type):
                                    print_log(f'Trolley initiation failed on port {ps.device}')
                                    del dev_trolley
                                    continue

                                _sysDevs[_devName] = i_dev
                                # devs.append(i_dev)

                            elif self['GR'] is not None and _fhSN in self.platformDevs['GR'].values():
                                _devName = getDevbySN(self.platformDevs['GR'], _fhSN)
                                if _devName is None:
                                    print_err(f'ERROR: Something went wrong while retriving Faulhaber GRIPPER dev name with S/N = {serN} ')
                                    continue


                                dev_gripper = FH_Motor(ps.device, fh_baudrate, fh_timeout, self.params_table, _devName)
                                # i_dev = CDev(DevType.GRIPPER, ps.device, ps.vid, _fhSN, dev_gripper, list(self.platformDevs['GR'].keys()).index(_devName)+1)
                                i_dev = CDev(DevType.GRIPPER, dev_gripper, list(self.platformDevs['GR'].keys()).index(_devName)+1)

                                if not dev_gripper.init_dev(i_dev.C_type):
                                    print_log(f'Gripper initiation failed on port {ps.device}')
                                    del dev_gripper
                                    continue

                                _sysDevs[_devName] = i_dev
                                # devs.append(i_dev)

                            elif self['SP'] is not None and _fhSN in self.platformDevs['SP'].values():
                                _devName = getDevbySN(self.platformDevs['SP'], _fhSN)
                                if _devName is None:
                                    print_err(f'ERROR: Something went wrong while retriving Faulhaber SPINNER dev name with S/N = {serN} ')
                                    continue


                                dev_spinner = FH_Motor(ps.device, fh_baudrate, fh_timeout, self.params_table, _devName)
                                # i_dev = CDev(DevType.TIME_ROTATOR, ps.device, ps.vid, _fhSN, dev_gripper, dev_spinner, list(self.platformDevs['SP'].keys()).index(_devName)+1)
                                i_dev = CDev(DevType.TIME_ROTATOR, dev_spinner, list(self.platformDevs['SP'].keys()).index(_devName)+1)

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
                        if  self['HMP'] is not None and len(self.platformDevs['HMP'].values()) > 0:
                            try:
                                for sn in enumerate(self.platformDevs['HMP'].values()):
                                    # if ps.serial_number == sn:
                                    if ps.description.split()[0] == sn:
                                        _devName = getDevbySN(self.platformDevs['HMP'], sn)
                                        guiIND = list(self.platformDevs['HMP'].keys()).index(_devName)+1
                                        hmp_dev = HMP_PS(sn=sn, port=ps.device, parms=self.params_table)
                                        # i_dev = CDev(DevType.HMP, ps.device, ps.description, ps.serial_number, hmp_dev, guiIND)
                                        i_dev = CDev(DevType.HMP, hmp_dev, guiIND)

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

                    elif self['DH'] is not None and (len(self.platformDevs['DH'].values()) > 0) and (sn := DH_ROB_RGI_modbus.find_server(ps.device)) is not None:

                        print_log(f'Trying add DH Robotics device on port {ps.device}, s/n = {sn}')
                        if not sn in self.platformDevs['DH'].values():
                            print_log(f'Found device with sn = {sn} is not configured in the system')
                        else:
                            _devName = getDevbySN(self.platformDevs['DH'], sn)
                            guiIND = list(self.platformDevs['DH'].keys()).index(_devName)+1

                            dh_dev = DH_ROB_RGI_modbus(sn=sn, port=ps.device, d_name = _devName, parms=self.params_table)
                            # i_dev = CDev(DevType.DH, ps.device, None, sn, dh_dev, guiIND)
                            i_dev = CDev(DevType.DH, dh_dev, guiIND)
                            if not dh_dev.init_dev(i_dev.C_type):
                                print_err(f'Can not initiate DH Robotics devivce on port {ps.device}')
                            else:
                                _sysDevs[_devName] = i_dev
                                # devs.append(i_dev)
                                print_log(f'DH Robotics device with SN = {sn} succesfully added on port {ps.device}')

                    else:               # ZABER
                        try:
                            if self['ZB'] is not None and len(self.platformDevs['ZB'].values()) > 0 and Zaber_Motor.init_devs(ps.device):        
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
                                    
                                    devName = getDevbySN(self.platformDevs['ZB'], serN)
                                    if devName is None:
                                        print_err(f'ERROR: ZABER dev name with S/N = {serN} is not configured in the system')
                                        continue
                                    
                                    guiIND = list(self.platformDevs['ZB'].keys()).index(_devName)+1
                                    dev_zaber = Zaber_Motor(ind, ps.device, self.params_table, devName)
                                    # i_dev = CDev(DevType.ZABER, ps.device, device_id, serN, dev_zaber, guiIND)
                                    i_dev = CDev(DevType.ZABER, dev_zaber, guiIND)
                                    
                                    _sysDevs[_devName] = i_dev
                                    # devs.append(i_dev)
                                    print_log(f'Added ZABER device on port {ps.device} with s/n {serN}')
                                
                        
                        
                        except MotionLibException as ex:
                            print_log(f'There was error adding ZABER devices on port {ps.device}')
                            print_log(f"Exception: {ex}")
                            
                if self['JTSE'] is not None and len(self.platformDevs['JTSE'].values()) > 0 and isinstance(self.platformDevs['JTSE'], dict):
                    devName = list(self.platformDevs['JTSE'].keys())[0]
                    _the_first_device = list(self.platformDevs['JTSE'].values())[0]

                    print_log(f'Looking for devName /{_the_first_device}/ blower on port {ps.device}')
                    try:
                        _com = JTSEcontrol.findDev(_the_first_device, ps.device)
                        if _com is None or _com == '':
                            print_log('No JTSE found')
                        else:
                            dev_jtse = JTSEcontrol(devName, _com)
                            # i_dev = CDev(DevType.JTSE, _com, None, None, dev_jtse, None)
                            i_dev = CDev(DevType.JTSE,  dev_jtse, None)
                           
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
            exptTrace(ex)
        else:
            pass

        finally:
            pass
                


    def __addFHv3Devs(self, _sysDevs:systemDevices):
        from bs1_FHv3 import FH_Motor_v3

        FHv3Types = set([ 'TR', 'GR', 'SP', 'RT'])
        configuredDevs = set(list(self.platformDevs.keys()))

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

                
                if self['RT'] is not None and devFHv3.serialN in self.platformDevs['RT'].values():
                    devType = DevType.DIST_ROTATOR
                    devName = getDevbySN(self.platformDevs['RT'], devFHv3.serialN)
                    guiIND = list(self.platformDevs['RT'].keys()).index(devName)+1
                    
                elif self['SP'] is not None and devFHv3.serialN in self.platformDevs['SP'].values():
                    devType = DevType.TIME_ROTATOR
                    devName = getDevbySN(self.platformDevs['SP'], devFHv3.serialN)
                    guiIND = list(self.platformDevs['SP'].keys()).index(devName)+1

                elif self['GR'] is not None and devFHv3.serialN in self.platformDevs['GR'].values():
                    devType = DevType.GRIPPERv3
                    devName = getDevbySN(self.platformDevs['GR'], devFHv3.serialN)
                    guiIND = list(self.platformDevs['GR'].keys()).index(devName)+1

                elif self['TR'] is not None and devFHv3.serialN in self.platformDevs['TR'].values():
                    devType = DevType.TROLLEY
                    devName = getDevbySN(self.platformDevs['TR'], devFHv3.serialN)
                    guiIND = list(self.platformDevs['TR'].keys()).index(devName)+1

                else :
                    print_log (f'Undefined FHv3 device: {devFHv3}')
                    continue

                dev_mDC = FH_Motor_v3(int(devFHv3.port), int(devFHv3.channel), self.params_table, devName)
                # i_dev = CDev(devType, devFHv3.port, devFHv3.devinfo, devFHv3.serialN, dev_mDC, guiIND)
                i_dev = CDev(devType,  dev_mDC, guiIND)
                
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
        from bs1_maxon import MAXON_Motor

        maxonTypes = set(['GR', 'RT'])
        configuredDevs = set(list(self.platformDevs.keys()))

        if len(maxonTypes & configuredDevs) == 0:
            print_log(f'No MAXON compitable devices defined in the configuration')
            return

        print_log('Scanning MAXON devs')

        try:
            # mxnDev = bytes(self.params_table['DEAFULT']['MAXON_DEV'], 'utf-8')
            # mxnIntfc = bytes(self.params_table['DEAFULT']['MAXON_INTERFACE'], 'utf-8')

            mxnDev =  assign_parm('DEFAULT', self.params_table,  'MAXON_DEV')
            mxnIntfc =  assign_parm('DEFAULT', self.params_table,  'MAXON_INTERFACE')
            print_log(f'MAXON parameters: DEV={mxnDev}, INTF={mxnIntfc}')
            
            if mxnDev is None or mxnIntfc is None:
                print_log(f'MAXON device type or interface is not correctly defined in the parameters table')
                return
            
            MAXONDevs = MAXON_Motor.init_devices(bytes(mxnDev, 'utf-8'), bytes(mxnIntfc, 'utf-8'))
            print_log(f'MAXON devs - {MAXONDevs}')

            if MAXONDevs == None or len(MAXONDevs) == 0:
                print_log(f'No MAXON devices were found')
                return
            MAXON_len = len(MAXONDevs)

            print_log (f'{MAXON_len} MAXON devices could be added')

            for ind in range(len(MAXONDevs)):
                devMAXON = MAXONDevs[ind]
                print_log (f'{ind} -> {devMAXON}, s/n = {devMAXON.sn}')

                if self['RT'] is not None and devMAXON.sn in self.platformDevs['RT'].values():
                    devType = DevType.DIST_ROTATOR
                    devName = getDevbySN(self.platformDevs['RT'], devMAXON.sn)
                    guiIND = list(self.platformDevs['RT'].keys()).index(devName)+1

                elif self['GR'] is not None and devMAXON.sn in self.platformDevs['GR'].values():
                    devType = DevType.GRIPPER
                    devName = getDevbySN(self.platformDevs['GR'], devMAXON.sn)
                    guiIND = list(self.platformDevs['GR'].keys()).index(devName)+1

                else :
                    print_log (f'Undefined MAXON device: {devMAXON}')
                    continue

                dev_mDC = MAXON_Motor(devMAXON, self.params_table, devName)
                # i_dev = CDev(devType, devMAXON.port, devMAXON, devMAXON.sn, dev_mDC, guiIND)
                i_dev = CDev(devType, dev_mDC, guiIND)
                # 
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

        if self['NI'] is None or 'NI' not in self.platformDevs.keys():
            print_log(f'No NI DAQ devices defined in the configuration')
            return
        
        NI = list(self.platformDevs['NI'].values())      # serial numbers list

        print_log('Looking for NI devices')
        
        try:
            
            for ind, sn in enumerate(NI):
                found_dev = NI6002.find_devs(sn)
                if found_dev:

                    devType = DevType.DAQ_NI
                    devName = getDevbySN(self.platformDevs['NI'], sn)
                    guiIND = list(self.platformDevs['NI'].keys()).index(devName)+1
                    
                    dev_ni = NI6002( devName, sn, self.params_table)
                    # i_dev = CDev(devType, found_dev.device, found_dev.model, found_dev.sn, dev_ni, guiIND)
                    i_dev = CDev(devType, dev_ni, guiIND)
                    
                    # devs.append(i_dev)
                    _sysDevs[devName] = i_dev

                    print_log(f'NI device with SN = {hex(sn)} ({sn}) succesfully added')


    ##########################                
                    if self['INTERLOCK'] is not None and f'DAQ{guiIND}' in list(self.platformDevs['INTERLOCK'].values()):
                        print_log(f'Adding Interlock for NI DAQ{guiIND}')
                        INTERLOCK = list(self.platformDevs['INTERLOCK'].values()) 
                        for interIndex, interDev in enumerate(INTERLOCK):
                            if interDev == f'DAQ{guiIND}':
                                iLockName = getDevbySN(self.platformDevs['INTERLOCK'], interDev)
                                dev_interlock = InterLock(iLockName, interDev, self.params_table)
                                # i_dev = CDev(DevType.INTERLOCK, None, None, None, dev_interlock, interIndex+1)
                                i_dev = CDev(DevType.INTERLOCK, dev_interlock, interIndex+1)
                                
                                _sysDevs[iLockName] = i_dev
                                # devs.append(intlck_dev)
                                print_log(f'Interlock device {dev_interlock} assosiated with NI {i_dev} was added')
                    else:
                        print_log(f'No Interlock associated with NI DAQ{guiIND} is defined in the configuration')

    ##########################                
                else:
                    print_err(f'NI device with SN = {hex(sn)} ({sn}) can not be detected')

        
        except Exception as ex:
            print_log(f"Fail to add NI device. Exception: {ex}")
            exptTrace(ex)


    def __addFESTODevs(self, _sysDevs:systemDevices):

        if self['ZABER'] is None :              # FESTO devices are in the stepper group 
                                                  # (use ZABER key since they replace ZABER stepper actuators)
            print_log(f'No stepper group including FESTO devices defined in the configuration')
            return

        print_log(f'Looking for FESTO in stepper group: \n{self.platformDevs["ZABER"]}  ')
        try:
            print_log(f'Looking for  FESTO devs in neighbour nodes ')
            Festo_Motor.enum_devices()

            ZABER = list(self.platformDevs['ZABER'].values())      # SN/IP list
            for ind, ip in enumerate(ZABER):

                _f_rex = re.compile(r'(?:\d{1,3}\.){3}\d{1,3}')
                if _f_rex.match(str(ip)):
                    _sn = Festo_Motor.find_dev(ip)
                    if _sn is None:
                        print_err(f'No FESTO device found at IP = {ip}')
                        continue

                    devType = DevType.ZABER
                    devName = getDevbySN(self.platformDevs['ZABER'], ip)
                    guiIND = list(self.platformDevs['ZABER'].keys()).index(devName)+1

                    dev_mDC = Festo_Motor(_sn, ip, devName, self.params_table)
                    # i_dev = CDev(devType, ip, '502', _sn, dev_mDC, guiIND)
                    i_dev = CDev(devType, dev_mDC, guiIND)
                    _sysDevs[devName] = i_dev
                    # devs.append(i_dev)
                    print_log(f'Added FESTO device on ip {ip} with s/n {_sn}, name = {devName}')


        except Exception as ex:
            exptTrace(ex)
            print_log(f"Fail while adding FESTO devs. Exception: {ex}")

# BUGBUG 

    def __addIODevs(self, _sysDevs:systemDevices):
        if self['IOCONTROL'] is None:
            print_log(f'No IO control (IOCONTROL) devices defined in the configuration')
            return
        
        io_dev_dict = self.platformDevs['IOCONTROL']      # {dev_name: 'provider, port.line, NO/NC', ...}
    
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
                
                dev_iocontrol = IOcontrol(dev_name, __io_provider, __line, __port, self.params_table, _NO)
                # i_dev = CDev(C_type=DevType.IOCONTROL, C_port=_pl, c_id=__io_provider, c_serialN=0, c_dev = dev_iocontrol, c_gui=None)
                i_dev = CDev(C_type=DevType.IOCONTROL,  c_dev = dev_iocontrol, c_gui=None)
                
                # devs.append(i_dev)

                print_log(f'IO control ({_pars}/{__io_provider}, {__port}.{__line}, {_NO} [{_nonc}] len = {len(_pars)}) was added for name= {dev_name} dev = {__io_provider}, port={__port}, line = {__line}, NO/^NC = {_NO}')
        
        except Exception as ex:
            exptTrace(ex)
            print_log(f"Fail to add IO CONTROL. Exception: {ex}")

    
    
    def __addPHGDevs(self, _sysDevs:systemDevices):
        from bs1_phidget import PhidgetRELAY
    
        if self['PHG'] is None:
            print_log(f'No Phidgets (PHG) devices defined in the configuration')
            return
        

        phg_dev_dict = self.platformDevs['PHG']      # {dev_name: 'sn/channel, type, TRUE/FALSE', ...}
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
                            dev_phg = PhidgetRELAY(sn, chan, _dType, _dName, self.params_table)
                            i_dev = CDev(DevType.PHG, dev_phg, guiIND)
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
        from bs1_cam_modbus import Cam_modbus

        if self['CAM'] is None:
            print_log(f'No ModBus Camera devices defined in the configuration')
            return
        
        CAM = self.platformDevs['CAM']     # IP:port dictionary

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
                        dev_cam = Cam_modbus(ip, port, self.params_table, _devName)
                        # i_dev = CDev(DevType.CAM, connection, None, None,dev_cam, guiIND)
                        i_dev = CDev(DevType.CAM, dev_cam, guiIND)

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
        from bs1_mecademic import robotMecademic
        from bs1_meca500 import robotMeca500

        if self['MCDMC'] is None:
            print_log(f'No Mecademic / Asyril  devices defined in the configuration')
            return

        print_log('Looking for Mecademic / Asyril')
        MCDMC = self.platformDevs['MCDMC']                 # Mecademic devices dictionary  {dev_name: ip_address, ...}

        try:
            for _devName, _ipAddr in MCDMC.items():
                _ip_format = re.compile(r'(?:\d{1,3}\.){3}\d{1,3}')
                # parm_re = re.compile(r'\b(?:\d{1,3}\.){3}\d{1,3}:\d{1,5}\b')
                parm_re = re.compile(r'\b(?:\d{1,3}\.){3}\d{1,3}:\d{1,5}/\d{1,5}\b')
                # _asyrilIPStr = get_parm(_devName, params_table, 'ASYRIL')

                _asyrilIPStr = assign_parm(_devName, self.params_table, 'ASYRIL',None)
                __asyrilIPort = None if _asyrilIPStr is None else _asyrilIPStr.split('/')[0]
                _robot_type = assign_parm(_devName, self.params_table, 'TYPE', 'MECA500')

                if _asyrilIPStr is not None and _robot_type == 'MCS500':
                                                                        # Asyril is defined and SCARA robot (MCS500)
                    print_log(F'Checking  for Mecademic / MCS500 ({_ipAddr}) / Asyril {_asyrilIPStr} together')
                    if  _ip_format.match(_ipAddr) and parm_re.match(_asyrilIPStr):
                        found_dev = robotMecademic.find_avilable(_ipAddr, __asyrilIPort)
                        if found_dev:
                            guiIND = list(MCDMC.keys()).index(_devName)+1
                            # guiIND = MCDMC.index(_ipAddr)+1
                            dev_mcdmc = robotMecademic(_devName, _ipAddr,  self.params_table)
                            # i_dev = CDev(DevType.MCDMC, _ipAddr, None, None, dev_mcdmc, guiIND)
                            i_dev = CDev(DevType.MCDMC, dev_mcdmc, guiIND)
                            
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
                            dev_mcdmc = robotMeca500(_devName, _ipAddr,  self.params_table)
                            # i_dev = CDev(DevType.MCDMC, _ipAddr, None,  None, dev_mcdmc, guiIND)
                            i_dev = CDev(DevType.MCDMC, dev_mcdmc, guiIND)
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
        if 'MARCO' not in list(self.platformDevs.keys()):
            print_log(f'No ModBus Marco devices defined in the configuration')
            return
        MARCO = self.platformDevs['MARCO']     # {devName: 'IP:port'} dictionary  
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
                        dev_marco = Marco_modbus(ip, port, self.params_table, _devName )
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


    def __addPLCDevs(self, _sysDevs:systemDevices):
        try:
            with open(self.__config_file) as conf_file:
                _plc_devs = None
                _allDevs = yaml.safe_load(conf_file)
                                                     #  to avoid 'dictionary changed size during iteration' error
                if 'PLC' in _allDevs.keys():
                    _plc_devs = _allDevs['PLC'].copy()
                    print_log(f'PLC devices from config: {_plc_devs}')
                else:
                    print_log(f'No PLC devices defined in the configuration')
                    return
                     
        except Exception as ex:
            print_log(f"Fail to read configuration file {self.__config_file}. Exception: {ex}")
            exptTrace(ex)
            return
        

        print_log(f'Adding PLC devices from ({_plc_devs})')
        
        try:
            for _devclass, _devgroup in self.platformDevs.items():  #
                guiIND:int = 0
                if not isinstance(_devgroup, dict):
                    print_log(f'WARNING: Skipping non dict device group: {_devclass}. group = {_devgroup}')
                    continue
                print_log(f'Looking for PLC in {_devclass} devices: {_devgroup} ')
                for _devName, _devSN in _devgroup.items():
                    guiIND += 1
                    if _plc_devs is not None and str(_devSN).split('/')[0]  in _plc_devs.keys():
                        print_log(f'Found PLC device {_devName} of class {_devclass} with name = { _devSN.split("/")[1] } on PLC {_devSN.split("/")[0]}')
                        _new_plc = _sysDevs.PLCdevs.createPlatformDev(_sysDevs, dev_name=_devName, plc_dev_name=_devSN.split("/")[1])
                        if _new_plc is not None:
                            # i_dev = CDev(DevType.PLCDEV, {_devSN.split("/")[0]}, None, None, _new_plc, guiIND)
                            i_dev = CDev(DevType.PLCDEV, _new_plc, guiIND)
                            _sysDevs[_devName] = i_dev
                            print_log(f'PLC device {_devName} of class {_devclass} with SN = {_devSN} succesfully added')
                        else:
                            print_log(f'Error creating instance of PLC device {_devName} of class {_devclass} with SN = {_devSN}')

                    else:
                        print_log(f'No PLC device found for {_devName} of class {_devclass} with SN = {_devSN}')

            pass
        
        except Exception as ex:
            print_log(f"Fail to add PLC device. Exception: {ex}")
            exptTrace(ex)


    def __addDBDev(self, _sysDevs:systemDevices):

        print_log('Adding DB')
        try:
            _db = assign_parm('DB', self.params_table, 'DB_NAME', 'production.db')
            _tbl = assign_parm('DB', self.params_table, 'TBL_NAME', 'statistic')
            dev_DB = StatisticSQLite('DB', _db, _tbl)
            # i_dev = CDev(DevType.DB, C_port=None, c_id=None, c_serialN=None, dev = dev_DB, c_gui=None)
            i_dev = CDev(DevType.DB, dev = dev_DB, c_gui=None)
            
            if not dev_DB.StartDB():
                raise Exception(f'Error initiation DB operation')
            
            _sysDevs['DB'] = i_dev
            # devs.append(i_dev)
            print_log(f'Added DB = {_db}, Table - {_tbl}')

        except Exception as ex:
            print_log(f"Fail to adding DB. Exception: {ex}")
            exptTrace(ex)


