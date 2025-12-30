
__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["Leonid Voldman"]
__license__ = "SLA"
__version__ = "2.0.1"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Prototype"

import asyncio, time
from collections import namedtuple
from pyModbusTCP.client import ModbusClient
import serial as serial
import random
import time, sys
from inputimeout  import inputimeout , TimeoutOccurred
import threading
from threading import Lock
from queue import Queue 
from collections import namedtuple
from bs1_base_motor import BaseDev

from numpy import uint32

'''
For messages formats and adresses see the reference:
file:///C:/Program%20Files/Cognex/In-Sight/In-Sight%20Vision%20Suite%20Editors/In-Sight/In-Sight%2024.3.0/Docs/Help_InSight/Content/Topics/IndustrialCommunications/ModbusTCP_5x.htm?tocpath=Industrial%20Communications%7CSetting%20Up%20the%20Industrial%20Communications%7CModbus%20TCP%20Communications%7C_____2
https://support.cognex.com/docs/is_613/web/EN/ise/Content/Communications_Reference/TCPIPModbusCommunications.htm
https://support.cognex.com/docs/is_613/web/EN/ise/Content/Communications_Reference/ModbusTCP_5x.htm


COGNEX -> COMPUTER translation 
16 bit:
AB CD >>> CD AB


'''




COGNEX_TIMEOUT = 5

MAX_MODBUS = 125
WAIT_TRIGGER_DATA = 0

VISION_CONTROL = 0              # 1-2
VISION_STATUS = 100             # 1-2
INPUT = 2000                    # 1-2001
OUTPUT = 7000                   # 1 - 2005
NUMBER_OF_PARTS = 7006
PARTS_IN_BUFFER = 7008
PARTS_POS = 7010
PART_DATA_SIZE = 3
TRIGER_TIMEOUT = 4
BUSY_WAIT_STEP = 0.5

TRIGGER_ENABLE = 0x00010000
TRIGGER_SHOT = 0x00020000
BUFFER_RESULT_ENABLE_FLAG = 0x00040000
INSPECTION_RESULT_ACK = 0x00080000
EXCECUTE_CMD = 0x00100000
EXTERNAL_EVENT = 0x000100

TRIGGER_READY_STATUS = 0x00010000
TRIGGER_ACK_STATUS = 0x00020000
ONLINE_STATUS = 0x00800000

RESULT_VALID_STATUS =  0x08000000 
INSPECTION_COMPLETED_STATUS =  0x02000000 
COMMAND_COMPLETED_STATUS = 0x20000000 
COMMAND_FAILED_STATUS = 0x40000000

MODBUS_TIMEOUT = 0.5
MODBUS_POLING_STEP = 0.1


JOBID_TO_BE_LOAD_ADDRESS = INPUT + 0

STATUS_RESOLUTION = 2           # in sec

ErrorCode = { 
    0x0000:	'No Error', # No error occurred.
    0x0100:	'Trigger set while disabled', # Occurs when the Trigger bit is set while the Trigger Enable bit is cleared and the Trigger parameter of the AcquireImage function is set to External, Network or Industrial Ethernet.
    0x0101:	'Trigger set while Offline', # Occurs when the Trigger bit is set while the vision system is Offline.
    0x0400:	'Already Executing Error', # Occurs when the Execute Command bit is set and the Executing bit is still High.
    0x0401:	'Job load requested while Online', # Occurs when a Job Load command is issued while the vision system is Online.
    0x0402:	'Job load requested does not exist on the camera' # Occurs when a JobID sent in the Command field does not exist as a prefix for any job in the file system. For example, if you send Command = 123 and no job on the camera contains 123xxxxx.job.

}


CommandResultCode = {
    0x0000: 'Command succeeded / unknown failure',
    0x0002: 'Bad command',
    0x208D: 'Request is incompatible with system state'
}

from enum import Enum

from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, set_parm, get_parm, unsigned_16, CDAB_converter 

'''
FLAGS:
SIDE_FLAG               - True(1) - right side /  False(0) - wrong side
DIRECTION_FLAG          - True(1) - 0 degree rotation  /  False(0) - 180 degree rotation required
'''


'''
All MASK: TRUE -ok, FALSE - not valid
'''
SIDE_FLAG_MASK = 0x00000001 
DIRECTION_FLAG_MASK = 0x00000002
OUT_OF_RANGE_FLAG_MASK = 0x00000004
PRODUCT_FLAG_MASK = 0x00000008


_partPosFields = ["x", "y", "flags"]
_partPos = namedtuple("_partPos",  _partPosFields, defaults=[None,] * len(_partPosFields))
_partPos.__annotations__={'x':float,  'y':float, 'flags':int} 

_statFields = ['wrong_products', 'up_side_down', 'out_of_range', 'valid_products']
statisticData = namedtuple("statisticData",  _statFields, defaults=[0,] * len(_statFields))
statisticData.__annotations__={'wrong_products':int, 'up_side_down':int, 'out_of_range':int, 'valid_products':int} 

class CognexCAM(BaseDev):
    @staticmethod
    def find_device(t_ip, t_port, _unit_id = 1) -> bool:
        try:
            print_log(f'Looking COGNEX ModBus server at IP:port = {t_ip}:{t_port}')
            test_client = ModbusClient(host=t_ip,                            # Initiating modbus client connection
                    port = int(t_port), unit_id = _unit_id, debug=False, timeout = COGNEX_TIMEOUT, auto_open=True)
            print_log (f'COGNEX test_client = {test_client}')
             
            
            if test_client is None:
                print_err(f'COGNEX CAM ModBus server with IP:port = {t_ip}:{t_port} not found')
                return False

            _res = test_client.open()
            print_log(f'Connecting to COGNEX  Modbus server at IP:port = {t_ip}:{t_port} has {"SUCCEDED" if _res else "FAILED"}')

            if not test_client.is_open:
                print_log(f'COGNEX ModBus client can not connect to server >> IP:port = {t_ip}:{t_port} ')
                test_client.close()
                del test_client
                return False
            
            test_client.close()
            del test_client
            return True
        
        except Exception as ex:
            print_err(f'Error detecting COGNEX ModBus server with IP:port = {t_ip}:{t_port}')
            exptTrace(ex)
            return False
        
    def __init__(self, _ip_addr:str, _port:int, _unit_id = 1) -> None:
        self.ip_addr = _ip_addr
        self.port = _port
        self.m_client = None
        self.wd:threading.Thread = None
        self.__last_status_update_time = 0
        self.__stored_online_status = False
        self.available_parts:list[_partPos] = list()
        self.unavailable_parts:list[_partPos] = list()
        self.wrong_parts:list[_partPos] = list()

        self.__wrong_products:int = 0
        self.__up_side_down:int = 0
        self.__out_of_range:int = 0
        self.__valid_products:int = 0

        



        try:
            random.seed()
            print_log(f'Connecting COGNEX ModBus server at IP:port = {_ip_addr}:{_port}')
            self.m_client = ModbusClient(host=_ip_addr,            # Initiating modbus client connection
                    port = int(_port),
                    unit_id = _unit_id,
                    debug=False, timeout = COGNEX_TIMEOUT, auto_open=True)
            
            self.m_client.timeout = 5
        
            self.__controlWord:list = [0]*2
            self.__statusWord:list = [0]*2
            self.__controlWord = self.__get_setVisionControl()
            self.__statusWord = self.__getVisionStatus()

        except Exception as ex:
            self.m_client = None
            print_log(f'COGNEX ModBus initialization failed. Exception = {ex}')
            exptTrace(ex)
        else:
            print_log(f'COGNEX ModBus connected at server={_ip_addr}, port={_port}')

    def __del__(self):
        self.m_client.close()
        del self.m_client


    def __getVisionStatus(self)->list:
        if not self.m_client:
            print_err(f'ERROR: ModBus client is not initiazed')
            return None
        
        try:
            val:list[int] = [0]*2
            _start_time = time.time()
            while abs(time.time() - _start_time) < MODBUS_TIMEOUT:
                _tmp_val:list[int] = [0]*2
                _tmp_val = self.m_client.read_holding_registers(VISION_STATUS, 2)
                if _tmp_val is None:
                    time.sleep(MODBUS_POLING_STEP)
                else:
                    val = _tmp_val
                    break

            if val is None:
                print_err(f'Error reading MODBUS value at {VISION_STATUS}. Please verify MODBUS is set active, trying time = {time.time() - _start_time}')
                return None

        except Exception as ex:
            print_log(f'Error reading vision status at address {VISION_STATUS}')
            exptTrace(ex)
            return None
        self.__statusWord = val
        return val
    
    def __get_setVisionControl(self, _vc:list = None)->list:
        if not self.m_client:
            print_err(f'ERROR: ModBus client is not initiazed')
            return False

        if _vc is not None:
            _new_vc = [0] * 2
            _new_vc[0] = _vc[0]
            _new_vc[1] = _vc[1]
            try:
                _start_time = time.time()
                while abs(time.time() - _start_time) < MODBUS_TIMEOUT:
                    res = self.m_client.write_multiple_registers(int(VISION_CONTROL), _new_vc)
                    if not res:
                        time.sleep(MODBUS_POLING_STEP)
                    else:
                        break

                if not res:
                    print_log(f'Error setting vision control word {_new_vc}([{_new_vc[0]:#06x}][{_new_vc[1]:#06x}])  at address {VISION_CONTROL}')


            except Exception as ex:
                print_log(f'Error seting vision control word {_new_vc} ([{_new_vc[0]:#06x}][{_new_vc[1]:#06x}])   at address {VISION_CONTROL}')
                exptTrace(ex)
                

        try:
            val:list[int] = [0] * 2
            _start_time = time.time()
            while abs(time.time() - _start_time) < MODBUS_TIMEOUT:
                _tmp_val:list[int] = [0]*2
                _tmp_val = self.m_client.read_holding_registers(VISION_CONTROL, 2)
                if _tmp_val is None:
                    time.sleep(MODBUS_POLING_STEP)
                else:
                    val = _tmp_val
                    break

            if val is None:
                print_err(f'Error reading MODBUS value at {VISION_CONTROL}. Please verify MODBUS is set active')
                return None

        except Exception as ex:
            print_log(f'Error reading vision status at address {VISION_CONTROL}')
            exptTrace(ex)
            return None
        
        self.__controlWord = val
        return val


    def __loadId(self, __id:int, devNotificationQ:Queue):
        # Set INPUT + 0 -> JOB ID
        # EXECUTE
        # wait until EXECUTION completed or failed
        # sent result to devNotificationQ
        pass

    def LoadJob(self, jobid:int, devNotificationQ:Queue)->bool:
        __id = jobid
        try:
            if self.wd is not None and not self.wd.is_alive():
                self.wd = threading.Thread(target=self.__loadId, args=(__id, devNotificationQ,))
                self.wd.start()
            else:
                raise Exception(f'The LOAD oparation is already running')
            
        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'Error loading COGNEX Job')
            return False
        
        return True

    def operateDevice(self, command, **kwards):
                                        # pass, nothing to do
        return super().operateDevice(command, **kwards) 
    
    
    def Trigger(self)->bool:
        print_log(f'Trigger: Setting BUFFER_RESULT_ENABLE_FLAG = {BUFFER_RESULT_ENABLE_FLAG:#010x}')
        self.__setVisionControlBit(_flag = BUFFER_RESULT_ENABLE_FLAG)
        _status = [0] * 2
        _status = self.__getVisionStatus()

        if _status is None:
            print_err(f'ERROR: Can not get Vision Status ')
            return False

        if  not ((ONLINE_STATUS >> 16) & _status[0]):
            print_err(f'COGNEX is disconnected or OFFLINE. Move it to ONLINE mode')
            return False

       
        _control = [0] * 2
        _control = self.__get_setVisionControl()

        if _control is None:
            print_err(f'ERROR: Can not get Vision Control word ')
            return False
        
        print_log(f'VisionControl = {_control} ([{_control[0]:#06x}][{_control[1]:#06x}]) Setting TRIGGER_ENABLE and clearing TRIGGER_SHOT')
        # _control[0] = (_control[0] | TRIGGER_ENABLE >> 16) & ~TRIGGER_SHOT >> 16           # set enable and zero shot
        # _control = self.__get_setVisionControl(_control)

        self.__setVisionControlBit(TRIGGER_SHOT, False)
        self.__setVisionControlBit(TRIGGER_ENABLE)
        _control = self.__get_setVisionControl()

        if _control is None:
            print_err(f'ERROR: Can not set Vision Control word ')
            return False

        print_log(f'VisionControl = {_control} ([{_control[0]:#06x}][{_control[1]:#06x}]) ')

        print_log(f'Waiting for TRIGGER_READY_STATUS ({TRIGGER_READY_STATUS:#010x})')

        _triggerReady = time.time()
        while not ((TRIGGER_READY_STATUS >> 16) & _status[0]):
            if time.time() - _triggerReady > TRIGER_TIMEOUT:
                print_err(f'ERROR: Trigger was not get into READY state, start time = {_triggerReady}, current time = {time.time()} timeout = {TRIGER_TIMEOUT}')
                return False
            
            time.sleep(BUSY_WAIT_STEP)
            _status = self.__getVisionStatus()

        
        print_log(f'Setting TRIGGER_ENABLE = {TRIGGER_ENABLE:#010x} and TRIGGER_SHOT = {TRIGGER_SHOT:#010x}')
        # _control[0] = (_control[0] | TRIGGER_ENABLE >> 16) | TRIGGER_SHOT  >> 16          # set enable and arm shot
        # _control = self.__get_setVisionControl(_control)

        self.__setVisionControlBit( TRIGGER_ENABLE  | TRIGGER_SHOT )
        _control = self.__get_setVisionControl()

        if _control is None:
            print_err(f'ERROR: Can not get Vision Control word ')
            return False

        print_log(f'VisionControl = {_control} ([{_control[0]:#06x}][{_control[1]:#06x}]) / Trigger done')

        return True
        
    @property
    def statisticINFO(self):
        return  statisticData(wrong_products = self.__wrong_products, up_side_down = self.__up_side_down, \
                              out_of_range = self.__out_of_range, valid_products = self.__valid_products)

    @property
    def onlineStatus(self)->bool:
        _current_time = time.time()     
        if _current_time - self.__last_status_update_time < STATUS_RESOLUTION:
            return self.__stored_online_status
        

        _status = [0] * 2
        _status = self.__getVisionStatus()

        if _status is None:
            print_err(f'ERROR: Can not get Vision Status ')
            return False

        self.__last_status_update_time = _current_time

        if  not ((ONLINE_STATUS >> 16) & _status[0]):
            if self.__stored_online_status:                 # print only when changed
                print_err(f'COGNEX is disconnected or OFFLINE. Move it to ONLINE mode')
            self.__stored_online_status = False
            return False
        self.__stored_online_status = True 
        return True
    
    def __waitStatusFlag(self, _flag:int, _timeout:float, _set:bool = True)->bool:
        try:
            _triggerTime = time.time()
            _status = self.__getVisionStatus()
            if _status is None:
                print_err(f'ERROR: Can not get Vision Status ')
                return False
            _high = _status[0] if _set else ~_status[0]
            _low = _status[1] if _set else ~_status[1]
            while ((_flag >> 16) & _high) | ((_flag ) & _low) == 0:
                if time.time() - _triggerTime > _timeout:
                    print_err(f'ERROR: Trigger ack not set during timeout, flag = {_flag:#010x}  start time = {_triggerTime}, current time = {time.time()} timeout = {_timeout}')
                    print_log(f'Status: ({_status[0]:#06x}, {_status[1]:#06x})')
                    return False
                time.sleep(BUSY_WAIT_STEP)
                _status = self.__getVisionStatus()
                if _status is None:
                    print_err(f'ERROR: Can not get Vision Status ')
                    return False
                _high = _status[0] if _set else ~_status[0]
                _low = _status[1] if _set else ~_status[1]
            
            print_log(f'Status: ({_status[0]:#06x}, {_status[1]:#06x})')
        
        except Exception as ex:
            exptTrace(ex)
            raise Exception(f'Error waiting trigger done')  

        
        
        return True
    
    def __clearExtEvents(self):
        for _it in range(8):
            self.__setVisionControlBit(EXTERNAL_EVENT << _it, False)

    def __setVisionControlBit(self, _flag:int, _set=True)->int:
        try:
            _control_orig = [0] * 2
            _control = [0] * 2
            _control_orig = self.__get_setVisionControl()
            if _control_orig is None:
                raise Exception('Error reading  Vision Control data')
           
            if _set:
                _control[0] = (_control_orig[0] | _flag >> 16)     # set hgh
                _control[1] = (_control_orig[1] | (_flag & 0x0000FFFF) )     # set low
            else:
                _control[0] = (_control_orig[0] & ~(_flag >> 16))     # clear  high
                _control[1] = (_control_orig[1] & ~(_flag & 0x0000FFFF))     # clear low


            print_log(f' {"Setting" if _set else "Clearing"}  Orig = ({_control_orig[0]:#06x}, {_control_orig[1]:#06x}) Flag = {_flag:#010x}, Vision Control = ({_control[0]:#06x}, {_control[1]:#06x})' )
            _control = self.__get_setVisionControl(_control)
            if _control is None:
                print_log(f'Error setting/getting VisionControl' )
            else:
                print_log(f'Vision Control = ({_control[0]:#06x}, {_control[1]:#06x}) -> [{((_control[0] << 16) | _control[1]):#010X}]' )


        except Exception as ex:
            exptTrace(ex)
            print_log(f'Error setting Vision Control bit')
            raise ex
        
        return _control


    def __readBufMODBUS(self, _start_buffer_addr:int, _size:int)->list:
        _data_list = list()
        try:
            __div = _size // MAX_MODBUS
            __rem =  _size % MAX_MODBUS
            for _it in range (__div):
                _pdata = [0] * MAX_MODBUS
                _pdata = self.m_client.read_holding_registers(_start_buffer_addr + MAX_MODBUS * (_it), MAX_MODBUS)
                if _pdata is None:
                    raise Exception(f'Error reading Modbus data at adress {_start_buffer_addr + MAX_MODBUS * (_it)}, size = {MAX_MODBUS}')
                _data_list.extend(_pdata)
            
            _remdata = [0] * __rem
            _remdata = self.m_client.read_holding_registers(_start_buffer_addr + MAX_MODBUS*__div,  __rem)
            if _remdata is None:
                    raise Exception(f'Error reading Modbus data at adress {_start_buffer_addr + MAX_MODBUS*__div}, size = {__rem}')
            _data_list.extend(_remdata)

        except Exception as ex:
            exptTrace(ex)
            print_log(f'Error reading MODBUS buffer, size = {_size}')
            raise ex
        
        return _data_list
    


    def updatePartsPos(self)->bool:
        expect_more_data:bool = True
        _control_parts:int = 0
        _data = list()
                                            # reset info
        self.__wrong_products = 0
        self.__up_side_down = 0
        self.__out_of_range = 0
        self.__valid_products = 0


        try:
            _InspectionCompleted:bool = (((self.__statusWord[0]<<16) & (self.__statusWord[0])) & INSPECTION_COMPLETED_STATUS) > 0
            print_log(f'INSPECTION_COMPLETED_STATUS = {_InspectionCompleted}, status =  ({self.__statusWord[0]:#06x}, {self.__statusWord[1]:#06x})')

            print_log(f'Setting INSPECTION_RESULT_ACK = {INSPECTION_RESULT_ACK:#010x} ')
            self.__setVisionControlBit(INSPECTION_RESULT_ACK)           # set INSPECTION_RESULT_ACK to clear RESULT_VALID_STATUS
                                                                        # setting the Inspection Results Ack bit to True. After the Results
                                                                        # Valid bit is cleared, the Automation Controller sets the Inspection 
                                                                        # Results Ack bit back to False to allow the queued results to be 
                                                                        # placed into the Output Block

            print_log(f'Clearing external events')
            self.__clearExtEvents()
            print_log(f'Setting BUFFER_RESULT_ENABLE_FLAG = {BUFFER_RESULT_ENABLE_FLAG:#010x} ')
            self.__setVisionControlBit(BUFFER_RESULT_ENABLE_FLAG)
            self.Trigger()
            status = self.__waitStatusFlag(_flag = TRIGGER_ACK_STATUS, _timeout=TRIGER_TIMEOUT)
            if not status:
                print_log(f'WARNING: TRIGGER_ACK_STATUS ({TRIGGER_ACK_STATUS:#010x}) was no set during timeout')
            time.sleep(WAIT_TRIGGER_DATA)

            # print_log(f'Waiting for INSPECTION_COMPLETED_STATUS = {INSPECTION_COMPLETED_STATUS:#010x} ')
            # _res = self.__waitStatusFlag(_flag = INSPECTION_COMPLETED_STATUS, _timeout = TRIGER_TIMEOUT)
            # if not _res:
            #     raise Exception(f'ERROR: Trigger ack TIMEOUT')

            
            _InspectionCompleted:bool = (((self.__statusWord[0]<<16) & (self.__statusWord[0])) & INSPECTION_COMPLETED_STATUS) > 0
            print_log(f'INSPECTION_COMPLETED_STATUS = {_InspectionCompleted}, status =  ({self.__statusWord[0]:#06x}, {self.__statusWord[1]:#06x})')

            print_log(f'Waiting for INSPECTION_COMPLETED_STATUS = {INSPECTION_COMPLETED_STATUS:#010x} be toggled. Currently = {_InspectionCompleted}')
            _res = self.__waitStatusFlag(_flag = INSPECTION_COMPLETED_STATUS, _timeout = TRIGER_TIMEOUT, _set = not _InspectionCompleted)
            if not _res:
                raise Exception(f'ERROR: Trigger ack TIMEOUT')


            _it = 0
            _read_parts = 0
            while expect_more_data:

                print_log(f'Setting EXTERNAL_EVENT # {_it} = {(EXTERNAL_EVENT << _it):#010x} ')
                self.__setVisionControlBit( EXTERNAL_EVENT << _it)


                print_log(f'Clearing INSPECTION_RESULT_ACK = {INSPECTION_RESULT_ACK:#010x} ')
                self.__setVisionControlBit(INSPECTION_RESULT_ACK, False)        # Clearing the Inspection Results Ack bit causes the 
                                                                                # vision system to set the Results Valid bit if the buffer is not empty.
                print_log(f'Waiting for clearing  RESULT_VALID_STATUS = {RESULT_VALID_STATUS:#010x} ')
                _res = self.__waitStatusFlag(_flag = RESULT_VALID_STATUS, _timeout = TRIGER_TIMEOUT)
                if not _res:
                    raise Exception(f'ERROR: RESULT_VALID_STATUS TIMEOUT')


                # print_log(f'Setting INSPECTION_RESULT_ACK = {INSPECTION_RESULT_ACK:#010x} ')
                # self.__setVisionControlBit(INSPECTION_RESULT_ACK)       # setting the Inspection Results Ack bit to True. After the Results
                #                                                         # Valid bit is cleared, the Automation Controller sets the Inspection 
                #                                                         # Results Ack bit back to False to allow the queued results to be 
                #                                                         # placed into the Output Block

                # print_log(f'Waiting for RESULT_VALID_STATUS = {RESULT_VALID_STATUS:#010x} ')
                # _res = self.__waitStatusFlag(_flag = RESULT_VALID_STATUS, _timeout = TRIGER_TIMEOUT)
                # if not _res:
                #     raise Exception(f'ERROR: RESULT_VALID_STATUS ack TIMEOUT')

                _valid_parts = [0] * 2
                _parts_in_buf_lst = [0] * 2
                _valid_parts = self.__readBufMODBUS(NUMBER_OF_PARTS, 2)
                _parts_num_CDAB = _valid_parts[0] << 16 | _valid_parts[1]
                _parts:int = int(round(CDAB_converter(_parts_num_CDAB)))

                if _control_parts != 0 and _parts != _control_parts:
                    print_err(f'WARNING TOTAL number of parts not the same shown in buffers {_it} and {_it -1}')

                _control_parts = _parts

                _parts_in_buf_lst = self.__readBufMODBUS(PARTS_IN_BUFFER, 2)
                _parts_in_buff_CDAB = _parts_in_buf_lst[0] << 16 | _parts_in_buf_lst[1]
                _parts_in_buffer:int = int(round(CDAB_converter(_parts_in_buff_CDAB)))

                print_log(f'Found {_parts} parts, in the current buffer # {_it} are {_parts_in_buffer} parts')
                if _parts_in_buffer > 0:
                    _parts_buf = [0] * _parts_in_buffer
                    _parts_buf = self.__readBufMODBUS(PARTS_POS, _parts_in_buffer * PART_DATA_SIZE * 2)
                    _data.extend(_parts_buf)
                    print_log(f'Data received from COGNEX ({_parts_in_buffer} parts) in the current buffer # {_it}: {_parts_buf}')
                    _read_parts += _parts_in_buffer
                    print_log(f'Read {_read_parts} of {_parts}')
                else:
                    print_err(f'WARNING: {_parts_in_buffer} parts are avaiable in the current buffer # {_it}. Up to now read {_read_parts} of {_parts}')
                    expect_more_data = False

                # self.__setVisionControlBit(INSPECTION_RESULT_ACK)
                
                if _read_parts >= _parts:
                    expect_more_data = False
                    print_log(f'Got {_read_parts} of {_parts}. No more parts to get')

                print_log(f'Setting INSPECTION_RESULT_ACK = {INSPECTION_RESULT_ACK:#010x} ')
                self.__setVisionControlBit(INSPECTION_RESULT_ACK)       # set INSPECTION_RESULT_ACK to clear RESULT_VALID_STATUS
                                                                        # setting the Inspection Results Ack bit to True. After the Results
                                                                        # Valid bit is cleared, the Automation Controller sets the Inspection 
                                                                        # Results Ack bit back to False to allow the queued results to be 
                                                                        # placed into the Output Block
                                                                        

                print_log(f'Clearing  EXTERNAL_EVENT # {_it} = {(EXTERNAL_EVENT << _it):#010x} ')
                self.__setVisionControlBit( EXTERNAL_EVENT << _it, False)
                _it += 1


            
            
            self.available_parts.clear()
            for it in range(_read_parts):
                _part_coord_X = CDAB_converter(_data[it * PART_DATA_SIZE * 2] << 16 |  _data[it * PART_DATA_SIZE * 2 + 1])
                _part_coord_Y = CDAB_converter(_data[it * PART_DATA_SIZE * 2 + 2] << 16 |  _data[it * PART_DATA_SIZE * 2 + 3])
                _flags = int(CDAB_converter(_data[it * PART_DATA_SIZE*2 + 4] << 16 |  _data[it * PART_DATA_SIZE * 2 + 5]))
                
                if not (_flags & PRODUCT_FLAG_MASK):
                    self.__wrong_products += 1
                if not (_flags & SIDE_FLAG_MASK):
                    self.__up_side_down += 1
                if not (_flags & OUT_OF_RANGE_FLAG_MASK):
                    self.__out_of_range += 1
        

                _availability:bool = (_flags & SIDE_FLAG_MASK) and  (_flags & OUT_OF_RANGE_FLAG_MASK) and (_flags & PRODUCT_FLAG_MASK)
                print_log(f'COGNEX found #{it}: X={_part_coord_X}  Y={_part_coord_Y}  flag = 0x{_flags:#010x} availability={_availability}')
                if _part_coord_X == 0 or  _part_coord_Y == 0:
                    print_err(f'ERROR: null cordinates received: x={_part_coord_X}, y={_part_coord_Y} ')

                if  _availability:
                    self.available_parts.append(_partPos(x=_part_coord_X, y=_part_coord_Y , flags= _flags)) 
                    self.__valid_products += 1
                else:
                    self.wrong_parts.append(_partPos(x=_part_coord_X, y=_part_coord_Y , flags= _flags)) 

            

        except Exception as ex:
            exptTrace(ex)
            print_log(f'Error reading vision parts details')
            return False
    
        return True


    def __updatePartsPosOBSOLETE(self)->bool:

        try:
            self.Trigger()
            # _res = self.__waitTriggerDone()
            _res = self.__waitStatusFlag(_flag = INSPECTION_COMPLETED_STATUS, _timeout = TRIGER_TIMEOUT)
            if not _res:
                raise Exception(f'ERROR: Trigger ack TIMEOUT')
                
            
            _parts_num_list = [0] * 2  
            _parts_num_list = self.m_client.read_holding_registers(NUMBER_OF_PARTS, 2)
            if _parts_num_list is None:
                raise Exception(f'Error reading parts number at address {NUMBER_OF_PARTS}')
            _parts_num_CDAB = _parts_num_list[0] << 16 | _parts_num_list[1]
            _parts:int = int(round(CDAB_converter(_parts_num_CDAB)))
            print_log(f'Found {_parts} parts')
            if _parts < 1:
                print_log(f'No parts founds. ')
                return False
            # _parts_data_list = [0] * _parts * PART_DATA_SIZE * 2         # each coordinate - 2 regs + 2 regs of flags
            _parts_data_list = list()

            __div = (_parts * PART_DATA_SIZE * 2) // MAX_MODBUS
            __rem =  (_parts * PART_DATA_SIZE * 2) % MAX_MODBUS

            print_log(f'parts = {_parts}, size = {_parts * PART_DATA_SIZE * 2} regs, div = {__div}, rem = {__rem}')

            for _it in range (__div):
                _pdata = [0] * MAX_MODBUS
                _pdata = self.m_client.read_holding_registers(PARTS_POS + MAX_MODBUS * (_it), MAX_MODBUS)
                _parts_data_list.extend(_pdata)
                # print_log(f'{_it+1} piece of data  [{MAX_MODBUS} regs] (addr= {PARTS_POS + MAX_MODBUS * (_it)}) = {_pdata}')
                # print_log(f'{_it+1} step/ BIG list = {_parts_data_list}')
            
            _remdata = [0] * __rem
            _remdata = self.m_client.read_holding_registers(PARTS_POS + MAX_MODBUS*__div,  __rem)
            _parts_data_list.extend(_remdata)
            # print_log(f'last piece of data [{__rem} regs](addr= {PARTS_POS + MAX_MODBUS*__div}) = {_remdata}')


            print_log(f'Data received from COGNEX: {_parts_data_list}')
            
            
            # _parts_data_list =  self.m_client.read_holding_registers(PARTS_POS, _parts * PART_DATA_SIZE *2)
                                                                                   
            if _parts_data_list is None:
                raise Exception(f'Error reading {_parts} parts positions at address {PARTS_POS} of size: {_parts * PART_DATA_SIZE}')
            
            self.available_parts.clear()
            for it in range(_parts):
                _part_coord_X = CDAB_converter(_parts_data_list[it * PART_DATA_SIZE * 2] << 16 |  _parts_data_list[it * PART_DATA_SIZE * 2 + 1])
                _part_coord_Y = CDAB_converter(_parts_data_list[it * PART_DATA_SIZE * 2 + 2] << 16 |  _parts_data_list[it * PART_DATA_SIZE * 2 + 3])
                _flags = _parts_data_list[it * PART_DATA_SIZE*2 + 4] << 16 |  _parts_data_list[it * PART_DATA_SIZE * 2 + 5]
                print_log(f'COGNEX found: X={_part_coord_X}  Y={_part_coord_Y}  flag = 0x{_flags:8x} availability={_flags & SIDE_FLAG_MASK}')
                print_log(f'flag1 = 0x{_parts_data_list[it * PART_DATA_SIZE*2 + 4]:4x}, flag2 = 0x{_parts_data_list[it * PART_DATA_SIZE*2 + 5]:4x}')
                if  _flags & SIDE_FLAG_MASK:
                    self.available_parts.append(_partPos(x=_part_coord_X, y=_part_coord_Y , flags= _flags)) 
                else:
                    self.wrong_parts.append(_partPos(x=_part_coord_X, y=_part_coord_Y , flags= _flags)) 

            

        except Exception as ex:
            exptTrace(ex)
            print_log(f'Error reading vision parts details')
            return False
    
        return True

    def get_pos (self, _try:int = 0) -> tuple:
        _offset_sign_x:int = 1
        _offset_sign_y:int = 1
        _angle = 0
        try:
            if _try == 0:
                if not self.updatePartsPos():
                    raise Exception (f'Error updating parts by COGNEX')    
                

            if len(self.available_parts) == 0:
                raise Exception (f'No more PCB found valid')
            
            _ind = random.randint(0, len(self.available_parts)-1)

            _part = self.available_parts[_ind]
            self.available_parts.remove(_part)
            self.unavailable_parts.append(_part)
            if not ((_part.flags & DIRECTION_FLAG_MASK) > 0):
                pass
                _angle = 180
            else:
                _offset_sign_x = -1   
                _offset_sign_y = -1      

            

            print_log(f'pick part [{_ind}] -> x= {_part.x}, y= {_part.y}, flags={_part.flags:#010x}, angle = {_angle}, offset direction = {_offset_sign_x}/{_offset_sign_y}')
        
        
        except Exception as ex:
            exptTrace(ex)
            raise(ex)

        return _part.x, _part.y, _angle, _offset_sign_x, _offset_sign_y


if __name__ == "__main__":
    pass