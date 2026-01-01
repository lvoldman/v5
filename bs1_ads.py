from __future__ import annotations

__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "1.0.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Tool"


import ctypes
import sys, time
from enum import Enum
import json

from collections import namedtuple

from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, real_num_validator, \
    int_num_validator, real_validator, int_validator, globalEventQ, smartLocker, clearQ, globalEventQ, event2GUI

import pyads
from _ctypes import Structure
from dataclasses import dataclass

class STATUS(Enum):
    READY = 0
    BUSY = 1
    DONE = 800
    ERROR = 900

PLC_TYPE_MAP = {
    bool: pyads.PLCTYPE_BOOL,
    int: pyads.PLCTYPE_INT,
    float: pyads.PLCTYPE_REAL,
    str: pyads.PLCTYPE_STRING

}

@dataclass
class symbolsADS:           # ADS symbols used in PLC configuration w/default values
    _max_num_of_devs:str = 'G_Constant.MaxNumOfDrivers'
    # _dev_array_str:str = 'G_System.fbExternalAPI.arDeviceInfo'
    _num_of_devices:str = 'G_System.fbExternalAPI.stDriverPool.NumberOfDriversInPool'
    _max_number_of_runners:str = 'G_System.fbExternalAPI.const_MaxNumOfExecutor'
    _runner_array_str:str = 'G_System.fbExternalAPI.fbExternalRunner'
    _device_access:str = 'G_System.fbExternalAPI.stDriverPool.DriverPool'  
                                    # API: G_System.fbExternalAPI.stDriverPool.DriverPool[<index>]._API
    #  _deviceResult   BUGBUG 

pick_method = Enum("pick_method", ["random", "up_end", "low_end"])

class Pool:
    def __init__(self, size:int=1024, method:pick_method = pick_method.low_end):
        self._size: int = size                                          # maximum number of IDs in the pool
        self._pool: set[int] = set(range(1, self._size + 1))            # available IDs in the pool
        self._method: pick_method = method                             # method to pick IDs from the pool

    # Allocate an ID from the pool
    def alloc(self) -> int :
        try:
            ret_val = None
            if len(self._pool) == 0:                                # no IDs available
                raise MemoryError(f'ADS POOL ERROR: The pool is empty. Cannot allocate new ID')
            if self._method == pick_method.random:              # pick a random ID
                ret_val = self._pool.pop() 
            elif self._method == pick_method.low_end:            # pick the lowest ID
                ret_val = min(self._pool)
                self._pool.remove(ret_val)
            elif self._method == pick_method.up_end:            # pick the highest ID
                ret_val = max(self._pool)
                self._pool.remove(ret_val)
            return ret_val
        except Exception as ex:
            exptTrace(ex)
            raise ex


    def release(self, id: int) -> None:
        try:
            if id < 1 or id > self._size:
                raise ValueError(f'ADS POOL ERROR: ID {id} is out of range [1..{self._size}]')
            if id in self._pool:
                raise ValueError(f'ADS POOL ERROR: ID {id} is already released')
            self._pool.add(id)
        except Exception as ex:
            exptTrace(ex)
            raise ex

class EN_DeviceCoreState(Enum):
    START = 0
    IDLE = 1
    INIT = 2
    READY = 3
    RUN = 4
    DONE = 5
    STOPPING = 6
    PAUSING = 7
    PAUSED = 8
    RESUMING = 9
    RESUME = 10
    ERROR = 11
    RESET = 12
    
class apiPLC:
    def __init__(self, adsCom:commADS):
        pass

class commADS:
    def __init__(self, ams_net_id:str, remote_ip_address:str, ams_net_port:int=pyads.PORT_TC3PLC1):
        self.__ams_net_id = ams_net_id
        self.__remote_ip_address = remote_ip_address
        self.__ams_net_port = ams_net_port
        self.__plc = None
        self.__plc_name = None
        self.__plc_version = None
        try:
            print_log(f'ADS INFO: Connecting to PLC with AMS NET ID={self.__ams_net_id} at IP={self.__remote_ip_address} on port={self.__ams_net_port}...')
            self.__plc = pyads.Connection(ams_net_id=self.__ams_net_id, \
                                         ams_net_port=self.__ams_net_port, ip_address = self.__remote_ip_address)
            if self.__plc is None:
                raise Exception(f'ADS ERROR: Cannot create PLC connection object')
            
            self.__plc.open()
            self.__plc_name, self.__plc_version = self.__plc.read_device_info()
            print_log(f'ADS INFO: Connected to PLC NAME = {str(self.__plc_name)} VER= {str(self.__plc_version)}, State={self.__plc.read_state()}')

        except Exception as ex: 
            exptTrace(ex)
            raise ex
        

    def __del__(self):
        try:
            if self.__plc is not None:
                self.__plc.close()
                print_log(f'ADS INFO: Disconnected from PLC NAME = {str(self.__plc_name)} VER= {str(self.__plc_version)}')
        except Exception as ex:
            exptTrace(ex)

    # def readVar(self, symbol_name:str, variable:object = None, size:int = None) -> str | int | bool | float | list | tuple | None:
    def readVar(self, symbol_name:str, var_type:type = None, size:int = None) -> str | int | bool | float | list | dict | None:
        ret_val = None
        try:
            if self.__plc is None:
                raise Exception(f'ADS ERROR: PLC connection is not established')
            
            if size is None:
                # if variable is None:
                if var_type is None:
                    ret_val = self.__plc.read_by_name(symbol_name, pyads.PLCTYPE_INT)
                else:
                    # ret_val = self.__plc.read_by_name(symbol_name, plc_datatype=PLC_TYPE_MAP[type(variable)])
                    ret_val = self.__plc.read_by_name(symbol_name, plc_datatype=PLC_TYPE_MAP[var_type])
            elif size <= 1024:
                ret_val = self.__plc.read_by_name(symbol_name, pyads.PLCTYPE_STRING)
            else:
                _data = self.__plc.read_by_name(symbol_name, pyads.PLCTYPE_BYTE * size)
                _cut_data = _data[:_data.index(0)]               # cut zero bytes
                ret_val =   "".join(map(chr, _cut_data)) 

            return ret_val
        
        except Exception as ex:
            print_err(f'ADS ERROR: Exception occurred while reading variable {symbol_name}. Exception: {ex}')
            exptTrace(ex)
            raise ex    

    def writeVar(self, symbol_name:str, dataToSend:object = None) -> bool:
        try:
            var_type = type(dataToSend)
            if var_type == str:
                size = len(dataToSend)
            else:
                size = None 

            if self.__plc is None:
                raise Exception(f'ADS ERROR: PLC connection is not established')
            
            if size is None:
                if var_type is None:
                    raise Exception(f'ADS ERROR: Cannot write variable without type info')
                else:
                    self.__plc.write_by_name(symbol_name, dataToSend, plc_datatype=PLC_TYPE_MAP[var_type])
            elif size <= 1024:
                self.__plc.write_by_name(symbol_name, dataToSend, pyads.PLCTYPE_STRING)
            else:
                raise Exception(f'ADS ERROR: Writing large data blocks is not supported yet')

            return True
        
        except Exception as ex:
            print_err(f'ADS ERROR: Exception occurred while writing variable {symbol_name}. Exception: {ex}')
            exptTrace(ex)
            raise ex


############################################ UNITEST SECTION #######################################

if __name__ == '__main__':

    import threading



# refer to https://pyads.readthedocs.io/en/latest/documentation


    # create some constants for connection
    # CLIENT_NETID = "119.158.229.117.1.1"
    # CLIENT_IP = "119.158.229.117"
    # TARGET_IP = "192.168.10.171"
    # TARGET_USERNAME = "Administrator"
    # TARGET_PASSWORD = "BMachine!"
    # ROUTE_NAME = "RPC_route"

    '''
    **Basic**
    b : BOOL
    I : INT - signed 16 bit
    ui : UINT - unsigned 16
    si : SINT - signed 8 bit
    di : DINT - signed 32 bit
    ud : UDINT - unsigned 32 bit
    rr : REAL -  signed 32 bit
    en : EN_Test(INT Enum) (currently have 0,1 properties inside)
    s80: STRING 80 chars
    s200: STRING     200 chars
    
    **Struct**
    st1 : ST_Test (contain all basic)

    **Global**
    G_Dudu.__ - all basic+struct name
    G_Dudu.arb -Array[1..10] of BOOL
    G_Dudu.ari -Array[1..10] of INT
    G_Dudu.arr -Array[1..10] of REAL
    G_Dudu.arst -Array[1..10] of ST_Test
    G_Dudu.aen -Array[1..10] of EN_Test

    '''
    SYMBOL_BASIC_NAME = "Main"
    SYMBOL_STRUCT_NAME = "st1"    
    SYMBOL_GLOBAL_NAME = "G_Dudu"

    SYMBOL_NAME_BOOL = "b"    
    SYMBOL_NAME_INT = "I"    
    SYMBOL_NAME_UINT = "ui"    
    SYMBOL_NAME_SINT = "si"    
    SYMBOL_NAME_DINT = "di"    
    SYMBOL_NAME_UDINT = "ud"    
    SYMBOL_NAME_REAL = "rr"   
    SYMBOL_NAME_ENUM = "en"  
    SYMBOL_NAME_STRING80 = "s80" 
    SYMBOL_NAME_STRING200 = "s200" 




    # AMS_NETID = '192.168.230.2.1.1'
    sender_ams = '192.168.10.153.1.1'
    sender_ip = '192.168.10.153'
    remote_ip = '192.168.10.92'
    remote_ads = '192.168.10.136.1.1'
    # AMS_NETID = '192.168.1.10.1.1'
    AMS_NETID = '192.168.10.92.1.1'

    # test with coining PLC
    #   
    remote_ip = '192.168.10.96'
    AMS_NETID = '192.168.10.96.1.1'
    

    def _watch_dog_thread(_adsCom:commADS, execution_id:int, _index:int):
        print_log(f'Watch dog thread is alive...')
        if _adsCom is None:
            raise Exception(f'ADS ERROR: PLC connection is not established in watch dog thread')
        while True:
            try:
                sym_exStatus = f'G_System.fbExternalAPI.ExecutionStatus[{_index}].eExecutionStatus'
                exStatus:int = _adsCom.readVar(symbol_name=sym_exStatus, var_type=int)
                print(f'ExecutionStatus = {STATUS(exStatus)} ({exStatus}) for ExecutionID={execution_id} ')
                if exStatus == STATUS.DONE.value:   # Completed
                    print_log(f'ExecutionID={execution_id} completed successfully')
                    break
                time.sleep(0.5)
                
            except Exception as ex:
                exptTrace(ex)
                break




    try:
        _adsCom = commADS(ams_net_id=AMS_NETID, remote_ip_address=remote_ip)
        _num_of_devs:int = _adsCom.readVar('G_Constant.MaxNumOfDrivers')
        print(f'Number of devices = {_num_of_devs}')

        # print info about all devices
        for i, _ in enumerate(range(_num_of_devs)):
            pass



        sys.exit()

        sym_exData = 'G_System.fbExternalAPI.ExecutionInfo'
        send_exData:dict = \
        {
            "ExecutionID": 4,
            "Devices": [
                        {
                            "InstanceName": "Stages_AxisX",
                            "TaskName": "Move Absolute",
                            "TaskParams": "{\"Pos\":0}",
                            "IsBreak": False
                        },
                        {
                            "InstanceName": "FrontStage_AxisY",
                            "TaskName": "Move Absolute",
                            "TaskParams": "{\"Pos\":0}",
                            "IsBreak": False
                        }
                ]
        }
        print(f'Writing ExecutionInfo = {send_exData}')
        _adsCom.writeVar(symbol_name=sym_exData, dataToSend = json.dumps(send_exData))

        sym_load = 'G_System.fbExternalAPI.DoLoadInfo'
        _adsCom.writeVar(symbol_name=sym_load, dataToSend = True)

        _index:int = _adsCom.readVar('G_System.fbExternalAPI.LastExecutorLoaded', var_type=int)
        print(f'Loaded ExecutionInfo on index = {_index}')

        _adsCom.writeVar(symbol_name='G_System.fbExternalAPI.RunExecutionID', dataToSend = send_exData["ExecutionID"] )

        _adsCom.writeVar(symbol_name='G_System.fbExternalAPI.DoRun', dataToSend = True )

        wd = threading.Thread(target = _watch_dog_thread, args=(_adsCom, send_exData["ExecutionID"], _index  ))
        wd.start()
        wd.join()
        print (f'Watch dog thread for devices {send_exData["Devices"]} on ExecutionID = {send_exData["ExecutionID"]}  ended')



        sys.exit()


        #  For Linux
        # # add a new route to the target plc
        # pyads.open_port()
        # pyads.set_local_address(sender_ams)

        # pyads.add_route_to_plc(
        #     sending_net_id=sender_ams, adding_host_name=sender_ip, ip_address = remote_ip, route_name='ROS', username = '', password = '',
        # )

        # pyads.ads.add_route_to_plc(
        #     sending_net_id=sender_ams, adding_host_name=sender_ip, ip_address = remote_ip, route_name='ROS', username = '', password = '',
        # )

        # pyads.pyads_ex.adsAddRouteToPLC(sending_net_id= sender_ams, adding_host_name=sender_ip, 
        #                                  ip_address = remote_ip, username = '', password = '', rroute_name='ROS')

        # connect to plc and open connection using TwinCAT3.
        # route is added automatically to client on Linux, on Windows use the TwinCAT router

        plc = pyads.Connection(ams_net_id=AMS_NETID, ams_net_port=pyads.PORT_TC3PLC1, ip_address = remote_ip)
        # plc = pyads.Connection(ams_net_id=AMS_NETID, ams_net_port=851, ip_address = remote_ip)
        if plc is None:
                raise Exception(f'ADS ERROR: Cannot create PLC connection object for ADS = {AMS_NETID} at {remote_ip}')
        plc.open()
        device_name, version = plc.read_device_info()
        print(f'INFO: NAME = {str(device_name)} VER= {str(version)}')
        print(f'State={ plc.read_state()}')

    except Exception as ex:
        exptTrace(ex)
        raise ex
    print(f'Connected to {AMS_NETID}')
    
    _state = None
    _val = None
    # _name = SYMBOL_GLOBAL_NAME+'.'+SYMBOL_NAME_INT
    # _name = SYMBOL_GLOBAL_NAME+'.ari'
    # _name = SYMBOL_GLOBAL_NAME+'.arst'
    _name = f'{symbolsADS._device_access}[1]._instanceName'   # 

    _str_def = (
        (SYMBOL_NAME_BOOL, pyads.PLCTYPE_BOOL, 1),
        (SYMBOL_NAME_INT, pyads.PLCTYPE_INT, 1),
        (SYMBOL_NAME_UINT, pyads.PLCTYPE_UINT, 1),
        (SYMBOL_NAME_SINT, pyads.PLCTYPE_SINT, 1),     
        (SYMBOL_NAME_DINT, pyads.PLCTYPE_DINT, 1),    
        (SYMBOL_NAME_UDINT, pyads.PLCTYPE_UDINT, 1),     
        (SYMBOL_NAME_REAL, pyads.PLCTYPE_REAL, 1),  
        (SYMBOL_NAME_ENUM, pyads.PLCTYPE_INT, 1),
        (SYMBOL_NAME_STRING80, pyads.PLCTYPE_STRING, 1, 80), 
        (SYMBOL_NAME_STRING200, pyads.PLCTYPE_STRING, 1, 200)
    ) 
    # print(f'Type of _str_def = {type(_str_def)}')
    # print(f'Type of (SYMBOL_NAME_BOOL, pyads.PLCTYPE_BOOL, 1) = {type((SYMBOL_NAME_BOOL, pyads.PLCTYPE_BOOL, 1))}')
    # sys.exit()

    # class _struct_def (pyads.StructureDef):
    #     _str_array = [
    #         (SYMBOL_NAME_BOOL, pyads.PLCTYPE_BOOL, 1),
    #         (SYMBOL_NAME_INT, pyads.PLCTYPE_INT, 1),
    #         (SYMBOL_NAME_UINT, pyads.PLCTYPE_UINT, 1),
    #         (SYMBOL_NAME_SINT, pyads.PLCTYPE_SINT, 1),     
    #         (SYMBOL_NAME_DINT, pyads.PLCTYPE_DINT, 1),    
    #         (SYMBOL_NAME_UDINT, pyads.PLCTYPE_UDINT, 1),     
    #         (SYMBOL_NAME_REAL, pyads.PLCTYPE_REAL, 1),  
    #         (SYMBOL_NAME_ENUM, pyads.PLCTYPE_INT, 1),
    #         (SYMBOL_NAME_STRING80, pyads.PLCTYPE_STRING, 1, 80), 
    #         (SYMBOL_NAME_STRING200, pyads.PLCTYPE_STRING, 1, 200)
    #     ]

    # _str_alloc = [_struct_def] * 10
    _num_of_devs = plc.read_by_name('G_Constant.MaxNumOfDrivers')
    print(f'Number of devices = {_num_of_devs}')
    # loop to read value
    # _sym_test = 'G_System.fbExternalAPI.sTest'
    # _test_val = plc.read_by_name( _sym_test,  pyads.PLCTYPE_BYTE * 4000 )
    # print(f'Test value({_sym_test}, size = {len(_test_val)} bytes)= { _test_val} ')
    # # print(f'Test value({_sym_test}, size = {len(_test_val)} bytes)= {"".join(map(chr, _test_val))} ')
    # sys.exit()

    for i, _ in enumerate(range(_num_of_devs)):
        _symb_dev_name = f'{symbolsADS._device_access}[{i+1}]._instanceName'
        _dev_name = plc.read_by_name(_symb_dev_name)
        if _dev_name.strip() == '':
            break
        _sym_dev = f'{symbolsADS._device_access}[{i+1}]._API'
        # _dev_API = plc.read_by_name( {f'G_System.fbExternalAPI.arDeviceInfo[{i+1}].API'})

        _dev_API = plc.read_by_name( _sym_dev,  pyads.PLCTYPE_BYTE * 4000 )
        cut_API = _dev_API[:_dev_API.index(0)]  # cut zero bytes


        _dev_info = f'{symbolsADS._device_access}[{i+1}]._instanceInfo'
        _dev_INFO = plc.read_by_name( _dev_info,  pyads.PLCTYPE_STRING * 1024 )

        _sym_state = f'{symbolsADS._device_access}[{i+1}].eState'
        _dev_STATE = plc.read_by_name( _sym_state,  pyads.PLCTYPE_INT )

        # print(f'Device[{i+1}]({len(_dev_API)} bytes) API={"".join(map(chr, _dev_API)) }  ')
        # print(f'Device[{i+1}]({len(_dev_API)}bytes) API={"".join(map(chr, _dev_API)) }  \n {_dev_API}')
        print(f'\n>>>>>>Device[{i+1}] Name={_dev_name}<<<<<<<')
        print(f'Device[{i+1}]({len(cut_API)}bytes) API={"".join(map(chr, cut_API)) }  ')
        print(f'Device[{i+1}] INFO={_dev_INFO}  ')
        print(f'Device[{i+1}] STATE={_dev_STATE}  ')


    sys.exit()

    while True:
        try:
            _new_state = plc.read_state()
            # _new_val = plc.read_by_name(_name)
            _new_val = plc.read_structure_by_name(_name, _str_def)

            
            if _state is None or _state != _new_state:
                _state = _new_state
                print(f'State = {_state}')
            if _val is None or _val != _new_val or True:
                _val = _new_val
                print(f'{_name} [{type(_new_val)}] = {_new_val}')

            time.sleep(0.5)    
            _write_val = None  
            if isinstance(_new_val, int) or isinstance(_new_val, float):
                _write_val = _new_val + 1    
            elif isinstance(_new_val, str):
                _write_val:str = ''.zfill(len(_new_val))
                for _index, char in enumerate(_new_val):
                    _write_val[_index] = char + 1
            elif isinstance(_new_val, list):
                _write_val:str = [0] * len(_new_val) 
                for _index, _el in enumerate(_new_val):
                    _write_val[_index] = _el + 1
            # elif isinstance(_new_val, tuple):
            #     _write_val:tuple = tuple()
            #     for _index, _el in enumerate(_new_val):
            #         if isinstance(_new_val[_index], int) or isinstance(_new_val[_index], float):
            #             _write_val = _new_val + 1    
            #         elif isinstance(_new_val, str):
            #             _write_val:str = ''.zfill(len(_new_val))
            #             for _index, char in enumerate(_new_val):
            #                 _write_val[_index] = char + 1
            #         elif isinstance(_new_val, list):
            #             _write_val:str = [0] * len(_new_val) 
            #             for _index, _el in enumerate(_new_val):
            #                 _write_val[_index] = _el + 1
                    
            # plc.write_by_name(_name, _write_val)
            time.sleep(0.5)                  

        except KeyboardInterrupt as ex:
            print(f'Exiting by ^C \n{ex}')
            sys.exit()
            break

    # check the connection state

    

    # read int value by name
    i = plc.read_by_name(SYMBOL_BASIC_NAME+'.'+SYMBOL_NAME_INT)

    # write int value by name
    plc.write_by_name(SYMBOL_BASIC_NAME+'.'+SYMBOL_NAME_REAL, 42.0)

    # create a symbol that automatically updates to the plc value
    real_val = plc.get_symbol(SYMBOL_BASIC_NAME+'.'+SYMBOL_NAME_REAL, auto_update=True)
    print(real_val.value)
    real_val.value = 5.0
    print(plc.read_by_name(SYMBOL_BASIC_NAME+'.'+SYMBOL_NAME_REAL))
    

    # close connection
    plc.close()