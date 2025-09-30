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

from collections import namedtuple
from typing import List

from bs1_utils import exptTrace
import pyads
from _ctypes import Structure



if __name__ == '__main__':
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
    remote_ip = '192.168.10.136'
    remote_ads = '192.168.10.136.1.1'
    AMS_NETID = '192.168.10.136.1.1'

    try:
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
    _name = SYMBOL_GLOBAL_NAME+'.arst'
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