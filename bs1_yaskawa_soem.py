
from typing import List

import ctypes, struct
import sys, time
import pysoem, threading
from bs1_utils import exptTrace

from enum import Enum

'''
BOOL BOOL 8-bit without sign 0 … 255
USINT UINT8 8-bit without sign 0 … 255
SINT SINT8 8-bit with sign –128 … 127
UINT UINT16 16-bit without sign 0 … 65535
INT SINT16 16-bit with sign –32768 … 32767
UDINT UINT32 32-bit without sign 0 … (232 –1)
DINT SINT32 32-bit with sign –231 … (231 –1)
ULINT UINT64 64-bit without sign 0 … (264 –1)
LINT SINT64 64-bit with sign –263 … (263 –1)
REAL FLOAT32 32-bit with floating decimal 1.17 * 10–38 … 3.4 * 1038
LREAL FLOAT64 64-bit with floating decimal 2.2 * 10–308 … 1.8 * 10308
STRING(X) CHAR X * 8-bit without sign X * 0 … 255
'''


# Configuration parameters
SLAVE_POS = 1    # Slave position in the EtherCAT network
TIMEOUT = 5000   # Timeout in microseconds for SDO operations
# CYCLE_TIME = 0.0025  # 1 ms cycle time
CYCLE_TIME = 0.01  # 1 ms cycle time

TOLERANCE_POSITION = 100 # Tolerance for position comparison in increments

# # CiA 402 Object Dictionary in full format

controlWord = (0x6040, 0, 2)
statusWord = (0x6041, 0, 2)
modesOfOperation = (0x6060, 0, 1)
positionActualValue = (0x6064, 0, 4)
targetPosition = (0x607A, 0, 4)
profileVelocity = (0x6081, 0, 4)
profileAcceleration = (0x6083, 0, 4)
profileDeceleration = (0x6084, 0, 4)
velocityOffset = (0x60B1, 0, 4)
torqueOffset = (0x60b2, 0, 2)
touchProbeFunction = (0x60b8, 0, 2)
targetVelocity = (0x60FF, 0, 4)
interpolationTimePeriodValue = (0x60C2, 1, 1)
interpolationTimeIndex = (0x60C2, 2, 1)
hommingMethod = (0x6098, 0, 1)
userParameterConfiguration = (0x2300, 0, 4)
        # 1. Change the SERVOPACK to the Switch ON Disabled state.
        # 2. Set the new parameter settings.
        # 3. Set user parameter configuration (2300 hex) to 1. The parameter settings will be enabled.
        # After execution, object 2300 hex will automatically be reset to 0.


positionUserUnitNumerator = (0x2301, 1, 4)
positionUserUnitDenominator = (0x2301, 2, 4)
        #  user-defined position reference unit 
        # 1 [Pos. unit] = 2301:01 hex/2301:02 hex [inc]

velocityUserUnitNumerator = (0x2302, 1, 4)
velocityUserUnitDenominator = (0x2302, 2, 4)
        # user-defined velocity reference unit 
        # 1 [Vel. unit] = 2302:01 hex/2302:02 hex [inc/s]

accelerationUserUnitNumerator = (0x2303, 1, 4)
accelerationUserUnitDenominator = (0x2303, 2, 4)
        # user-defined acceleration reference unit
        # 1 [Acc. unit] = 2303:01 hex/2303:02 hex × 104 [inc/s2]


# CiA 402 Object Dictionary indices
CONTROL_WORD = 0x6040
STATUS_WORD = 0x6041
MODES_OF_OPERATION = 0x6060
TARGET_POSITION = 0x607A
PROFILE_VELOCITY = 0x6081
PROFILE_ACCELERATION = 0x6083
PROFILE_DECELERATION = 0x6084
ACTUAL_POSITION = 0x6064
TARGET_VELOCITY = 0x60FF

# """
EC_STATE_NONE           = 0x00
EC_STATE_INIT           = 0x01
EC_STATE_PRE_OP         = pysoem.PREOP_STATE           # 0x02
EC_STATE_BOOT           = pysoem.BOOT_STATE           # 0x03
EC_STATE_SAFE_OP        = pysoem.SAFEOP_STATE           # 0x04 
EC_STATE_OPERATIONAL    = pysoem.OP_STATE           # 0x08
EC_STATE_ACK            = 0x10
EC_STATE_ERROR          = 0x10

FAULT_BIT = 0x08
FAULT_CLEARANCE_BIT = 0x80

OpMode = Enum("OpMode", ["position_mode", "velocity_mode", "homming_mode",  "velocity_test", \
                         "position_test", "velocity_PDO_test", \
                        "position_PDO_test", "cyclic_synchronous_position_Mode_PDO_test"])


GLOBAL_SLAVE_0 = None
_pd_thread_stop_event = None
rxpdo_mapping = None 
txpdo_mapping = None
sync:threading.Lock = threading.Lock()

def _initOp(_interface:str):
    global GLOBAL_SLAVE_0
    global _pd_thread_stop_event 
    _pd_thread_stop_event = threading.Event()
    _pd_thread_stop_event.set()  # Initialize the stop event to allow the thread to run
    """
    Initialize the EtherCAT master and configure the PDOs for the Yaskawa BS1 servo drive.
    """
    try:
        # Initialize EtherCAT master
        master = pysoem.Master()
        master.in_op = False
        master.open(_interface)

    except Exception as ex:
        print(f"Failed to open interface {_interface}: {ex}")
        exptTrace(ex)
        raise ex

    try:
        # Discover slaves
        if (_ci := master.config_init()) <= 0:
                raise Exception(f'No slaves found on interface {_interface}')
        print(f'Master {master} opened on interface {_interface}, slaves by config init = {_ci}')

        
        # Configure slave (assumes one slave at position 1)
        slave = master.slaves[SLAVE_POS - 1]
        print(f'Working with slave: {slave}')
        GLOBAL_SLAVE_0 = slave

        slave.config_func = slave_conf_func
        slave.setup_func = slave_setup_func


        slave.dc_sync(act=False, sync0_cycle_time=1000000, sync0_shift_time=0, sync1_cycle_time=None)
                                        # Sync to Distributed Clock (DC) synchronization to 25 ms period


        '''
        #   FESTO stupid feature: Attempt to read to a write only object
        sync_manager = slave.sdo_read(0x1C00, 0)
        print(f"Sync Manager Entries: {sync_manager.hex()}")
        for i in range(1, 5):
            sm_type = slave.sdo_read(0x1C00, i)
            print(f"Sync Manager {i}: {sm_type.hex()}")
        '''

    except Exception as ex:
        print(f"Failed to config interface {_interface}: {ex}")
        exptTrace(ex)
        raise ex    
    
    return master, slave


def _setCheckState(master, slave,  _newState:int, __timeout:int = TIMEOUT):
        _state = master.state_check(_newState, timeout=__timeout)
        print(f'[OP_STATE =  {_state}/{getState(_state)}] <<<', end='')
        if _state == _newState:
            print(f' [already in {_newState}/{getState(_newState)}]')
            return
        
        master.state =_newState
        master.write_state()
        _state = master.state_check(_newState, timeout=__timeout)
        
        if _state != _newState:
            master.read_state()
            print(f'!!![OP_STATE =  {_state}({slave.state})/{getState(_state)}] rather than {_newState}/{getState(_newState)}')
            print(f'al status code {hex(slave.al_status)} ({ pysoem.al_status_code_to_string(slave.al_status)})')
        else:
            print(f'>>>[OP_STATE =  {_state}/{getState(_state)}]')                  


def _configPDO(master, slave)->bool:    # Configure PDO mappings
    global rxpdo_mapping, txpdo_mapping
    # RxPDO: Control Word (16-bit), Target Position (32-bit), Profile Velocity (32-bit)
    # rxpdo = [
    #     0x60400010,  # Control Word (16-bit)
    #     0x607A0020,  # Target Position (32-bit)
    #     0x60810020   # Profile Velocity (32-bit)
    # ]
    # # TxPDO: Status Word (16-bit), Actual Position (32-bit)
    # txpdo = [
    #     0x60410010,  # Status Word (16-bit)
    #     0x60640020   # Actual Position (32-bit)
    # ]


    # RxPDO: Control Word (16-bit), Target Position (32-bit), Profile Velocity (32-bit)
    rxpdo = [
        0x1600,  
        # 0x1610,  
        # 0x1620,  
        # 0x1630
    ]
    # TxPDO: Status Word (16-bit), Actual Position (32-bit)
    txpdo = [
        0x1a00,  
        # 0x1a10,  
        # 0x1a20,  
        # 0x1a30
    ]

    rxpdo_mapping = [  
        0x60400010,  # Control Word (32-bit)         0 - 2
        0x607A0020,  # Target Position (32-bit)      2 - 6
                    0x60810020,   # PROFILE VELOCITY             6 - 10
                    0x60FF0020,     # TARGET_VELOCITY            10 - 14
                    # 0x60600008,    # Mode of operation           14 - 15
                    # 0x00000008,    # padding           15 - 16
        0x60b10020,   # Velocity Offset (32-bit)   14 - 18
        0x60b20010,     # Torque Offset (16)       18 - 20
        0x60b80010,     # Touch probe function (16)  20 - 22
    ]

    txpdo_mapping = [
        0x60410010,     # Status Word (16-bit)  0 - 2
        0x60640020,     # Actual Position (32-bit) 2 - 6
        # 0x00000008,     # PADDING for Mode of operation (8? 16? bit) 6 - 7
        # 0x60610008,     # Mode of operation (8? 16? bit) 6 - 7
        0x60770010,     # Torque actual value (32)
        0x60b90010,     # Touch probe status (32  bit)
        0x60ba0020      # Touch probe 1 position value   (32  bit)
    ]
        
    try:

        _setCheckState(master, slave,  EC_STATE_PRE_OP)


        # # Disable PDO mappings
        # slave.sdo_write(0x1C12, 0, struct.pack('<B', 0))  # Clear RxPDO
        # slave.sdo_write(0x1C13, 0, struct.pack('<B', 0))  # Clear TxPDO

        # # Write RxPDO mappings
        # slave.sdo_write(0x1C12, 1, struct.pack('<L', rxpdo[0]))
        # slave.sdo_write(0x1C12, 2, struct.pack('<L', rxpdo[1]))
        # slave.sdo_write(0x1C12, 3, struct.pack('<L', rxpdo[2]))
        # slave.sdo_write(0x1C12, 0, struct.pack('<B', 3))  # Enable 3 RxPDOs


        # # Write TxPDO mappings
        # slave.sdo_write(0x1C13, 1, struct.pack('<L', txpdo[0]))
        # slave.sdo_write(0x1C13, 2, struct.pack('<L', txpdo[1]))
        # slave.sdo_write(0x1C13, 0, struct.pack('<B', 2))  # Enable 2 TxPDOs



########################

        # Disable PDO mappings

        # slave.sdo_write(0x1C12, 0,  bytes(ctypes.c_char (0)))  # Clear RxPDO
        # slave.sdo_write(0x1C13, 0, bytes(ctypes.c_char (0)))  # Clear TxPDO
        slave.sdo_write(0x1C12, 0,  struct.pack('<B', 0))  # Clear RxPDO
        slave.sdo_write(0x1C13, 0, struct.pack('<B', 0))  # Clear TxPDO
        # Write RxPDO mappings
        for i, _rxpdo in enumerate(rxpdo):
            print(f'RxPDO 0x1C12 {i + 1} = 0x1C12:{i+1} <- {_rxpdo:#06x}')
            # slave.sdo_write(0x1C12, i + 1, bytes(ctypes.c_uint16 (_rxpdo)))
            slave.sdo_write(0x1C12, i + 1, struct.pack('<H', _rxpdo))
        print(f'RxPDO 0x1C12 {0} = 0x1C12:{0} <- {len(rxpdo):#04x}')  
        # slave.sdo_write(0x1C12, 0, bytes(ctypes.c_char (len(rxpdo))))  # Enable RxPDOs
        slave.sdo_write(0x1C12, 0, struct.pack('<B', len(rxpdo)))  # Enable RxPDOs

        # Write TxPDO mappings
        for i, _txpdo in enumerate(txpdo):
            print(f'TxPDO 0x1C13 {i + 1} = 0x1C13:{i+1} <- {_txpdo:#06x}')
            # slave.sdo_write(0x1C13, i + 1, bytes(ctypes.c_uint16 (_txpdo)))  
            slave.sdo_write(0x1C13, i + 1, struct.pack('<H', _txpdo))
        print(f'TxPDO 0x1C13 {0} = 0x1C13:{0} <- {len(txpdo):#04x}')  
        # slave.sdo_write(0x1C13, 0, bytes(ctypes.c_char (len(txpdo))))  # Enable TxPDOs
        slave.sdo_write(0x1C13, 0, struct.pack('<B', len(txpdo)))  # Enable TxPDOs

        # slave.sdo_write(rxpdo[0], 0, bytes(ctypes.c_char (0)))  # Clear RxPDOs
        # slave.sdo_write(txpdo[0], 0, bytes(ctypes.c_char (0)))  # Clear RxPDOs
        slave.sdo_write(rxpdo[0], 0, struct.pack('<B', 0))  # Clear RxPDOs
        slave.sdo_write(txpdo[0], 0, struct.pack('<B', 0))  # Clear RxPDOs

        for i, _rxpdo_map in enumerate(rxpdo_mapping):
            print(f'RxPDO mapping {i + 1} = {rxpdo[0]:#06x}:{i+1} <- {_rxpdo_map:#010x}')
            # slave.sdo_write(rxpdo[0], i + 1, bytes(ctypes.c_uint32 (_rxpdo_map)))
            slave.sdo_write(rxpdo[0], i + 1, struct.pack('<L', _rxpdo_map))
        print(f'RxPDO mapping {0} = {rxpdo[0]:#04x}:{0} <- {len(rxpdo_mapping):#04x}')
        # slave.sdo_write(rxpdo[0], 0, bytes(ctypes.c_char (len(rxpdo_mapping))))  # Enable RxPDOs
        slave.sdo_write(rxpdo[0], 0, struct.pack('<B', len(rxpdo_mapping)))  # Enable RxPDOs

        for i, _txpdo_map in enumerate(txpdo_mapping):
            print(f'TxPDO mapping {i + 1} = {txpdo[0]:#06x}:{i+1} <- {_txpdo_map:#010x}')
            # slave.sdo_write(txpdo[0], i + 1, bytes(ctypes.c_uint32 (_txpdo_map)))
            slave.sdo_write(txpdo[0], i + 1, struct.pack('<L', _txpdo_map))
        print(f'TxPDO mapping {0} = {txpdo[0]:#04x}:{0} <- {len(txpdo_mapping):#04x}')       
        # slave.sdo_write(txpdo[0], 0, bytes(ctypes.c_char (len(txpdo_mapping))))  # Enable TxPDOs
        slave.sdo_write(txpdo[0], 0, struct.pack('<B', len(txpdo_mapping)))  # Enable TxPDOs

        # Interoperation times setting
        # slave.sdo_write(0x60C2, 1, bytes(ctypes.c_char (2)))  #  Interoperation time period
        # slave.sdo_write(0x60C2, 2, bytes(ctypes.c_int8 (-2)))  #  Interoperation time index
        # slave.sdo_write(MODES_OF_OPERATION, 0, bytes(ctypes.c_char (8)))  #  OP mode

        # slave.sdo_write(0x60C2, 1, struct.pack('<b', 0x2))  #  Interoperation time period
        # slave.sdo_write(0x60C2, 2, struct.pack('<b', -0x6))  #  Interoperation time index

        # slave.sdo_write(MODES_OF_OPERATION, 0, struct.pack('<B', 0))  #  OP mode

        master.config_map()
        # master.config_overlap_map()

        master.send_processdata()
        master.receive_processdata(1000)



#######################

        # Map PDOs to process data
        master.config_map()
        # Configure Distributed Clocks (DC)
        master.config_dc()

    except Exception as ex:
        
        exptTrace(ex)
        master.close()
        raise Exception(f"Failed to configure PDOs: {ex}")


    return True

def _initOpState(master, slave):
# Transition to Operational state
    try:
        # master.state_check(pysoem.EC_STATE_PRE_OP, TIMEOUT)
        # master.write_state(0, pysoem.EC_STATE_PRE_OP)
        # master.state_check(pysoem.EC_STATE_SAFE_OP, TIMEOUT)
        # master.write_state(0, pysoem.EC_STATE_SAFE_OP)
        # master.state_check(pysoem.EC_STATE_OPERATIONAL, TIMEOUT)
        # master.write_state(0, pysoem.EC_STATE_OPERATIONAL)
        
        # slave.sdo_write(MODES_OF_OPERATION, 0, struct.pack('<B', 0x0))  #  OP mode = 8?

        _setCheckState(master, slave,  EC_STATE_PRE_OP)
        _setCheckState(master, slave,  EC_STATE_SAFE_OP)
        # _setCheckState(master, slave,  EC_STATE_OPERATIONAL, TIMEOUT*3)
        # _setCheckState(master, slave,  pysoem.OP_STATE, TIMEOUT*3)
        
        # BUGBUG
        # master.send_processdata()
        # master.receive_processdata(1000)
        time.sleep(CYCLE_TIME)


    except Exception as ex:
        print(f"Failed to transition to OP state: {ex}")
        master.close()
        return

def write_control_word_PDO(master, slave, value):
    # try:
    #     outdata = bytearray(slave.output)
    #     outdata[0:2] = struct.pack('<H', value)  # Write control word to output PDO
    #     slave.output = bytes(outdata)  # Update the output buffer

    #     master.send_processdata()
    #     master.receive_processdata(1000)
    #     time.sleep(0.1)
    
    # except Exception as ex:
    #     exptTrace(ex)
    write_RxPDO(master, slave, value, offset=0, len=2)  # Write control word to RxPDO

def read_write_RxPDO_SDO(master, slave, index:int, subindex:int, size:int, value:int = None, unsigned:bool = True)->int:
    global rxpdo_mapping, txpdo_mapping
    
    if rxpdo_mapping is None or not isinstance(rxpdo_mapping, list):
        raise Exception(f'ERROR rxpdo_mapping is {rxpdo_mapping} of type [{type(rxpdo_mapping)}]')
    
    _old_value =  None
    try:
        _offset:int = 0
        _len:int = 0
        size *= 8               # to bits conversion
        for _ref in rxpdo_mapping: 
            if (index == (_ref & 0xFFFF0000) >> 16) and (subindex == (_ref & 0x0000FF00) >> 8):
                _len = (_ref & 0x000000FF)
                if size != _len:
                    print(f'WARNING. Register {index:#04x}.{subindex} size ({size}) is different of RxPDo size {_len}')
                break
            _offset += (_ref & 0x000000FF)

        _form = None
        if _len > 0:
            size = _len

        if size == 0x08:
            _form = '<B' if unsigned else '<b'  # 1 bytes for 16-bit value (unsigned/signed)
        elif  size == 0x10:
            _form = '<H' if unsigned else '<h' # 2 bytes for 16-bit value (unsigned/signed)
        elif  size == 0x20:
            _form = '<L' if unsigned else '<l' # 4 bytes for 32-bit value (unsigned/signed)
        else:
            raise ValueError(f"Unsupported length for RxPDO write: ({_len}). register = {_ref}. value = {value}")
        
        if _len > 0:                # register is mapped in RxPDO

            sync.acquire(timeout=CYCLE_TIME)
            outdata = bytearray(slave.output)
            _old_value = struct.unpack(_form, bytes(outdata[int(_offset/8):int((_offset+_len)/8)]))[0]
            if value is not None:
                outdata[int(_offset/8):int((_offset+_len)/8)] = struct.pack(_form, value)  # Write control word to output PDO
                slave.output = bytes(outdata)  # Update the output buffer
                print(f'Writing PDO <- {index:#04x}.{subindex} / RxPDO [{int(_offset/8)}:{int((_offset+_len)/8)}] value = {value}({value:#04x})// Old value = {_old_value}({_old_value:#04x}) ')
            else:
                print(f'Read RxPDO: {index:#04x}.{subindex} / RxPDO [{int(_offset/8)}:{int((_offset+_len)/8)}]  value = {_old_value}({_old_value:#04x}) ')

            sync.release()
        else:                   
            _old_value =  struct.unpack(_form, slave.sdo_read(index, subindex))[0]
                                            # register is not mapped into RxPDO
            if value is not None:
                slave.sdo_write(index, subindex, struct.pack(_form, value))
                print(f'Writing SDO <- {index:#04x}.{subindex}  value = {value}({value:#04x}) len = {size}, Old value = {_old_value}({_old_value:#04x} ')
            else:
                print(f'Reading SDO: {index:#04x}.{subindex}  value =  {_old_value}({_old_value:#04x} ')

        time.sleep(CYCLE_TIME)

    except Exception as ex:
        print(f' Exception while writing {index:#04x}.{subindex} <- {value} of {size} size, format = {_form}, signed = {not unsigned}')
        exptTrace(ex)
    
    return _old_value

def read_TxPDO_SDO(master, slave, index:int, subindex:int, size:int, unsigned:bool = True)->int:
    global rxpdo_mapping, txpdo_mapping
    _read_value = None
    if rxpdo_mapping is None or not isinstance(rxpdo_mapping, list):
        raise Exception(f'ERROR rxpdo_mapping is {rxpdo_mapping} of type [{type(rxpdo_mapping)}]')
    
    try:
        _offset:int = 0
        _len:int = 0
        size *= 8               # to bits conversion
        # print(f'Reading data: from {index:#04x}.{subindex} of size {size} ', end ='')
        for _ref in txpdo_mapping: 
            if (index == (_ref & 0xFFFF0000) >> 16) and (subindex == (_ref & 0x0000FF00) >> 8):
                _len = (_ref & 0x000000FF)
                if size != _len:
                    print(f'WARNING. Register {index:#04x}.{subindex} size ({size}) is different of RxPDo size {_len}')
                break
            _offset += (_ref & 0x000000FF)

        _form = None
        if _len > 0:
            size = _len

        if size == 0x08:
            _form = '<B' if unsigned else '<b'  # 1 bytes for 16-bit value (unsigned/signed)
        elif  size == 0x10:
            _form = '<H' if unsigned else '<h' # 2 bytes for 16-bit value (unsigned/signed)
        elif  size == 0x20:
            _form = '<L' if unsigned else '<l' # 4 bytes for 32-bit value (unsigned/signed)
        else:
            raise ValueError(f"Unsupported length for RxPDO write: ({_len}). register = {_ref}. value = {value}")
        
        if _len > 0:                # register is mapped in RxPDO

            _read_value = struct.unpack(_form, bytes(slave.input[int(_offset/8):int((_offset+_len)/8)]))[0]
            # print(f'PDO read {index:#04x}.{subindex} / RxPDO [{int(_offset/8)}:{int((_offset+_len)/8)}] value = {_read_value}({_read_value:#04x}) ')
        else:                       
                                            # register is not mapped into RxPDO
            
            _read_value = slave.sdo_read(index, subindex)
            print(f'SDO read {index:#04x}.{subindex}  value = {_read_value}({_read_value:#04x}) len = {size} ')


    except Exception as ex:
        print(f' Exception while reading {index:#04x}.{subindex} of {size} size')
        exptTrace(ex)
        return None
    
    return _read_value



def write_RxPDO(master, slave, value, offset=0, len=2):
    try:
        if abs(len) == 1:
            _form = '<B' if len > 0 else '<b'  # 1 bytes for 16-bit value (unsigned/signed)
        elif abs(len) == 2:
            _form = '<H' if len > 0 else '<h' # 2 bytes for 16-bit value (unsigned/signed)
        elif abs(len) == 4:
            _form = '<L' if len > 0 else '<l' # 4 bytes for 32-bit value (unsigned/signed)
        else:
            raise ValueError(f"Unsupported length for RxPDO write: ({len}). Use +- 1, 2 or 4 bytes.")
        
        outdata = bytearray(slave.output)
        outdata[offset:offset+abs(len)] = struct.pack(_form, value)  # Write control word to output PDO
        slave.output = bytes(outdata)  # Update the output buffer

        # BUGBUG
        # master.send_processdata()
        # master.receive_processdata(1000)
        time.sleep(CYCLE_TIME)
    
    except Exception as ex:
        exptTrace(ex)

# def preStartCyclicPDO(master, slave):
#     """
#     Prepare the slave for cyclic PDO communication.
#     This function is called before starting the main cycle.
#     """
#     try:
#         for _ in range(10):
#             master.send_processdata()
#             master.receive_processdata(1000)
#             time.sleep(CYCLE_TIME)

#     except Exception as ex:
#         print(f"Failed to prepare cyclic PDO: {ex}")
#         exptTrace(ex)
#         master.close()

def _PDO_faultReset(master, slave):
    try:
        _status_before = struct.unpack('<H', slave.input[0:2])[0]
        time.sleep(CYCLE_TIME)
        # Reset the slave by writing a specific control word
        if _status_before & FAULT_BIT:
            _controlWord = read_write_RxPDO_SDO(master, slave, *controlWord)
            if _controlWord & FAULT_CLEARANCE_BIT:
                _controlWord =  _controlWord & ~FAULT_CLEARANCE_BIT                # Clear FAULT CLEARENCE bit
                time.sleep(CYCLE_TIME)
                read_write_RxPDO_SDO(master, slave, *controlWord,  _controlWord)
                
            _controlWord = _controlWord | FAULT_CLEARANCE_BIT                   # Set FAULT CLEARENCE bit
            time.sleep(CYCLE_TIME)
            read_write_RxPDO_SDO(master, slave, *controlWord,  _controlWord)

            time.sleep(CYCLE_TIME)
            print(f"Sent 0x80 = Fault Reset, PDO STATUS changed: {hex(_status_before)} -->> {hex(struct.unpack('<H', slave.input[0:2])[0])}")
        else:
            print(f'STATUS = {_status_before}, NO FAULT condition no need to clear/acknowledge')
    except Exception as ex:
        print(f"Failed to reset slave: {ex}")
        exptTrace(ex)


def _faultReset(master, slave):             # clear/acknowledge  errors 
    # _PDO_faultReset(master, slave)
    # return
    """
    Reset the slave to clear any faults.
    This function is called before starting the main cycle.
    """
    try:
        _status_before = struct.unpack('<H', slave.input[0:2])[0]
        time.sleep(CYCLE_TIME)
        # Reset the slave by writing a specific control word
        if _status_before & FAULT_BIT:
            _controlWord = struct.unpack('<H',slave.sdo_read(CONTROL_WORD, 0))[0]
            _controlWord =  _controlWord & ~FAULT_CLEARANCE_BIT                # Clear FAULT CLEARENCE bit
            slave.sdo_write(CONTROL_WORD, 0, struct.pack('<H', _controlWord))
            _controlWord = _controlWord | FAULT_CLEARANCE_BIT                   # Set FAULT CLEARENCE bit
            slave.sdo_write(CONTROL_WORD, 0, struct.pack('<H', _controlWord))
            # read_write_RxPDO_SDO(master, slave, *controlWord, 0x80)
            # write_control_word_PDO(master, slave, 0x80)  # 0x80 = Fault Reset
            # time.sleep(0.1)
            # master.send_processdata()
            # master.receive_processdata(1000)
            # time.sleep(0.01)

            # print(f"Sent 0x80 = Fault Reset, STATUS = {hex(struct.unpack('<H',slave.sdo_read(STATUS_WORD, 0))[0])}<>{hex(struct.unpack('<H', slave.input[0:2])[0])}")
            print(f"Sent 0x80 = Fault Reset, SDO STATUS changed: {hex(_status_before)} -->> {hex(struct.unpack('<H', slave.input[0:2])[0])}")
        else:
            print(f'STATUS = {_status_before}, NO FAULT condition no need to clear/acknowledge')
    except Exception as ex:
        print(f"Failed to reset slave: {ex}")
        exptTrace(ex)

def _intOpEnabled(master, slave):
    # CiA 402: Transition to Operation Enabled
    try:

        _PDO_faultReset(master, slave)

        # Shutdown (Ready to Switch On)
        # slave.sdo_write(0x6040, 0, b'\x06\x00') # 0x06 = Shutdown

        # slave.sdo_write(CONTROL_WORD, 0, struct.pack('<H', 0x06)) # 0x06 = Shutdown
        # slave.sdo_write(*SHUTDOWN_CMD) # 0x06 = Shutdown
        write_control_word_PDO(master, slave, 0x06) # 0x06 = Shutdown

        # BUGBUG
        # time.sleep(0.01)
        # master.send_processdata()
        # master.receive_processdata(1000)
        time.sleep(CYCLE_TIME)

        # print(f" Sent 0x06 = Shutdown, STATUS = {hex(struct.unpack('<H',slave.sdo_read(STATUS_WORD, 0))[0])}<>{hex(struct.unpack('<H', slave.input[0:2])[0])}")
        print(f" Sent 0x06 = Shutdown, PDO readed STATUS = {hex(struct.unpack('<H', slave.input[0:2])[0])}")
        

        _PDO_faultReset(master, slave)
        
        # Switch On
        # slave.sdo_write(0x6040, 0, b'\x07\x00') # 0x07 = Switch On
        # slave.sdo_write(CONTROL_WORD, 0, struct.pack('<H', 0x07))     # 0x07 = Switch On
        # slave.sdo_write(*SWITCHON_CMD)     # 0x07 = Switch On
        write_control_word_PDO(master, slave, 0x07) # 0x07 = Switch On


        # BUGBUG
        # time.sleep(0.01)
        # master.send_processdata()
        # master.receive_processdata(1000)
        time.sleep(CYCLE_TIME)

        # print(f" Sent 0x07 = Switch On, STATUS = {hex(struct.unpack('<H',slave.sdo_read(STATUS_WORD, 0))[0])}<>{hex(struct.unpack('<H', slave.input[0:2])[0])}")
        print(f" Sent 0x07 = Switch On, PDO readed STATUS = {hex(struct.unpack('<H', slave.input[0:2])[0])}")

        _PDO_faultReset(master, slave)

        # Enable Operation
        # slave.sdo_write(0x6040, 0, b'\x0F\x00')     # 0x0F = Enable Operation
        # slave.sdo_write(CONTROL_WORD, 0, struct.pack('<H', 0x0F)) # 0x0F = Enable Operation
        # slave.sdo_write(*ENABLE_CMD) # 0x0F = Enable Operation
        write_control_word_PDO(master, slave, 0x0F) # 0x0F = Enable Operation

        # BUGBUG
        # time.sleep(0.01)
        # master.send_processdata()
        # master.receive_processdata(1000)
        time.sleep(CYCLE_TIME)

        # print(f" Sent 0x0F = Enable Operation, STATUS = {hex(struct.unpack('<H',slave.sdo_read(STATUS_WORD, 0))[0])}<>{hex(struct.unpack('<H', slave.input[0:2])[0])}")
        print(f" Sent 0x0F = Enable Operation, PDO readed STATUS = {hex(struct.unpack('<H', slave.input[0:2])[0])}")

        # ch_status_word = struct.unpack('<H',slave.sdo_read(STATUS_WORD, 0))[0]
        # print(f"Status WORD: {hex(ch_status_word)} <> {hex(struct.unpack('<H', slave.input[0:2])[0])}")
        print(f"PDO readed Status WORD = {hex(struct.unpack('<H', slave.input[0:2])[0])}")


        ############### Test Print ###############
        """
        print([hex(int.from_bytes(slave.sdo_read(0x1C12, i), 'little')) for i in range(0, 3)])
        
        n = slave.sdo_read(0x1600, 0)[0]
        print(f"0x1600 has {n} mapped entries")
        for i in range(1, n+1):
            raw = slave.sdo_read(0x1600, i)
            mapping = int.from_bytes(raw, 'little')
            index = mapping >> 16
            subindex = (mapping >> 8) & 0xFF
            bitlength = mapping & 0xFF
            print(f"  Entry {i}: 0x{index:04X}:{subindex} ({bitlength} bits)")
        """

        master.config_dc()
        # print(f"DC Sync0 Cycle Time: {slave.dc_sync0_cycle_time} ns, Shift Time: {slave.dc_sync0_shift_time} ns")
        # print(f"DC Sync1 Cycle Time: {slave.dc_sync1_cycle_time} ns, Shift Time: {slave.dc_sync1_shift_time} ns")
        # print(f"DC Sync0 Enabled: {slave.dc_sync0_enabled}, Sync1 Enabled: {slave.dc_sync1_enabled}")
        # print(f"DC Sync0 State: {slave.dc_sync0_state}, Sync1 State: {slave.dc_sync1_state}")
        # print(f"DC Sync0 Time: {slave.dc_sync0_time}, Sync1 Time: {slave.dc_sync1_time}")

### --------------
        # _faultReset(master, slave)
        cw = 0x0F             # Control Word
        target_pos = 100000   # Target Position
        b1 = 0                # 0x60B1
        b2 = 0                # 0x60B2
        b8 = 0                # 0x60B8

        # Pack into 14-byte buffer
        outdata = struct.pack('<H i i H H', cw, target_pos, b1, b2, b8)

        # Write to slave output
        # slave.output = outdata
### --------------


        # print(f"Status WORD: {hex(struct.unpack('<H',slave.sdo_read(STATUS_WORD, 0))[0])} <> {hex(struct.unpack('<H', slave.input[0:2])[0])}")
        print(f"Status WORD: {hex(struct.unpack('<H', slave.input[0:2])[0])} // AL Status: {hex(slave.al_status)} ({pysoem.al_status_code_to_string(slave.al_status)})")

        # master.send_processdata()
        # master.receive_processdata(1000)

        # print(f"Status WORD: {hex(struct.unpack('<H',slave.sdo_read(STATUS_WORD, 0))[0])} <> {hex(struct.unpack('<H', slave.input[0:2])[0])}")
        print(f"Status WORD: {hex(struct.unpack('<H', slave.input[0:2])[0])} // AL Status: {hex(slave.al_status)} ({pysoem.al_status_code_to_string(slave.al_status)})")

        ########### clear init for testing ################
        outdata = bytearray(slave.output)
        outdata[0:2] = struct.pack('<H', 0x06)      # Control word: Shutdown
        outdata[2:6] = struct.pack('<i', 0)         # Target position
        outdata[6:] = b'\x00' * (len(outdata) - 6)  # Fill rest with zeros
        # slave.output = bytes(outdata)

        ###################################################

        # preStartCyclicPDO(master, slave)

        _setCheckState(master, slave,  EC_STATE_OPERATIONAL, TIMEOUT*10)

        # ch_status_word = struct.unpack('<H',slave.sdo_read(STATUS_WORD, 0))[0]
        # print(f"Status WORD: {hex(ch_status_word)} <> {hex(struct.unpack('<H', slave.input[0:2])[0])} ")
        print(f"Status WORD: {hex(struct.unpack('<H', slave.input[0:2])[0])} // AL Status: {hex(slave.al_status)} ({pysoem.al_status_code_to_string(slave.al_status)})")

        # print(f"Transitioning to OP state from {getState(master.state)}")

        # while master.state != pysoem.OP_STATE:
        #     master.send_processdata()
        #     master.receive_processdata(TIMEOUT)
        #     status_word = struct.unpack('<H', slave.output[0:2])[0]
        #     actual_pos = struct.unpack('<l', slave.output[2:6])[0]

        #     if slave.al_status != 0:
        #         print(f"AL Status Error: {hex(slave.al_status)} ({pysoem.al_status_code_to_string(slave.al_status)})", end ='')
        #         print(f' [slave state = {slave.state} = {getState(slave.state)}]', end ='' )
        #         print(f' [Status Word: {hex(status_word)}, Actual Position: {actual_pos}]')
        #     time.sleep(0.1)
        
        # print(f"Transitioned to OP state: {getState(master.state)}")

    except Exception as ex:
        exptTrace(ex)
        print(f"Failed to transition CiA 402 states: {ex}")
        master.close()
        return
    

def _setUserUnits(master, slave):
    try:
        read_write_RxPDO_SDO(master, slave, *controlWord, 0xD)

        read_write_RxPDO_SDO(master, slave, *positionUserUnitNumerator, 131072)
        read_write_RxPDO_SDO(master, slave, *positionUserUnitDenominator, 1000)

        read_write_RxPDO_SDO(master, slave, *velocityUserUnitNumerator, 131072)
        read_write_RxPDO_SDO(master, slave, *velocityUserUnitDenominator, 1)

        read_write_RxPDO_SDO(master, slave, *accelerationUserUnitNumerator, 131072)
        read_write_RxPDO_SDO(master, slave, *accelerationUserUnitDenominator, 10000)

        read_write_RxPDO_SDO(master, slave, *userParameterConfiguration, 1)


        read_write_RxPDO_SDO(master, slave, *controlWord, 0xF)
    except KeyboardInterrupt:
        _pd_thread_stop_event.set()  # Stop cycle
        raise Exception("Test interrupted by user.")


def statusString(master, slave)->str:
    __status = 'ERROR GETTING STATUS'
    try:
        __status = f"Status Word: {hex(struct.unpack('<H', slave.input[0:2])[0])} "
        # __status += f" Mode of OP: {hex(struct.unpack('<B', slave.sdo_read(0x6061, 0))[0])} "
        __status += f' AL Status: {hex(slave.al_status)} ({pysoem.al_status_code_to_string(slave.al_status)})'
    except Exception as ex:
        exptTrace(ex)

    return __status


def _setMotionParms(master, slave, _mode:OpMode):
        # Set motion parameters
    print(f"Setting motion parameters for mode: {_mode}")
    print(f"Status Word: {hex(struct.unpack('<H', slave.input[0:2])[0])} ", end ='')
    print(f' AL Status: {hex(slave.al_status)} ({pysoem.al_status_code_to_string(slave.al_status)})')

    # _faultReset(master, slave)

    if _mode == OpMode.position_mode:       # Profile position mode
        try:
            slave.sdo_write(PROFILE_VELOCITY, 0, struct.pack('<L', 100000))  # 100 rpm
            slave.sdo_write(PROFILE_ACCELERATION, 0, struct.pack('<L', 50))  # 50 rpm/s
            slave.sdo_write(PROFILE_DECELERATION, 0, struct.pack('<L', 50))  # 50 rpm/s

            slave.sdo_write(MODES_OF_OPERATION, 0, struct.pack('<B', 0x1))  #  OP mode (1= Position Mode)

            slave.sdo_write(TARGET_POSITION, 0, struct.pack('<l', 5000000))
            # write_RxPDO(master, slave, 5000000, offset=2, len=4)        # TARGET_POSITION
            # time.sleep(CYCLE_TIME)
            # master.send_processdata()
            # master.receive_processdata(1000)
            # time.sleep(CYCLE_TIME)

            slave.sdo_write(CONTROL_WORD, 0, struct.pack('<H', 0x0F))
            # write_control_word_PDO(master, slave, 0x0F) # 0x0F = Enable Operation
            # time.sleep(CYCLE_TIME)
            # master.send_processdata()
            # master.receive_processdata(1000)
            # time.sleep(CYCLE_TIME)
            print(f"Status Word: {hex(struct.unpack('<H', slave.input[0:2])[0])} ", end ='')
            print(f' AL Status: {hex(slave.al_status)} ({pysoem.al_status_code_to_string(slave.al_status)})')


            slave.sdo_write(CONTROL_WORD, 0, struct.pack('<H', 0x5F)) # 5F - relative move, 1F - absolute move
            # write_control_word_PDO(master, slave, 0x5F) # 0x1F = Start abs Operation, 0x5F = Start relative Operation
            # time.sleep(CYCLE_TIME)velocityUserUnitDenominator
            # master.send_processdata()
            # master.receive_processdata(1000)
            # time.sleep(CYCLE_TIME)
            print(f"Status Word: {hex(struct.unpack('<H', slave.input[0:2])[0])} ", end ='')
            print(f' AL Status: {hex(slave.al_status)} ({pysoem.al_status_code_to_string(slave.al_status)})')

            # _PDO_faultReset(master, slave)

        except Exception as ex:
            print(f"Failed to set motion parameters: {ex}")
            exptTrace(ex)
            master.close()
            return
        
    elif _mode == OpMode.velocity_mode:         # Profile velocity mode
        try:
            # Set velocity mode parameters
            
            slave.sdo_write(PROFILE_VELOCITY, 0, struct.pack('<L', 100000))  # 100 rpm
            slave.sdo_write(PROFILE_ACCELERATION, 0, struct.pack('<L', 50))  # 50 rpm/s
            slave.sdo_write(PROFILE_DECELERATION, 0, struct.pack('<L', 50))  # 50 rpm/s

            slave.sdo_write(TARGET_VELOCITY, 0, struct.pack('<l', 100000))  #  Target Velocity

            # Trigger motion (Enable Operation + New Setpoint)
            slave.sdo_write(MODES_OF_OPERATION, 0, struct.pack('<B', 0x3))  #  OP mode (3= Velocity Mode)

            slave.sdo_write(CONTROL_WORD, 0, struct.pack('<H', 0x0F))
            write_control_word_PDO(master, slave, 0x0F) # 0x0F = enable operation

        except Exception as ex:
            print(f"Failed to set velocity mode parameters: {ex}")
            exptTrace(ex)
            master.close()
            return
    elif _mode == OpMode.velocity_test:         # Profile velocity mode  
        try:
            slave.sdo_write(*DISABLE_CMD)   ##### RELEASE previous stall mode
            slave.sdo_write(MODES_OF_OPERATION, 0, struct.pack('<B', 0x3))  #  OP mode (3= Velocity Mode)
            slave.sdo_write(PROFILE_VELOCITY, 0, struct.pack('<L', 100000))  # 100 rpm
            slave.sdo_write(PROFILE_ACCELERATION, 0, struct.pack('<L', 50))  # 50 rpm/s
            slave.sdo_write(PROFILE_DECELERATION, 0, struct.pack('<L', 50))  # 50 rpm/s
            slave.sdo_write(*SHUTDOWN_CMD)
            slave.sdo_write(*ENABLE_CMD)
            slave.sdo_write(TARGET_VELOCITY, 0, struct.pack('<l', 100000))  #  Target Velocity

        except Exception as ex:
            print(f"Failed to set velocity mode parameters: {ex}")
            exptTrace(ex)
            master.close()
            return
        
    elif _mode == OpMode.velocity_PDO_test:         # Profile velocity mode  
        try:
            """
            write_RxPDO(master, slave, 0x0D)   ##### RELEASE previous stall mode

            # write_RxPDO(master, slave, 130000, offset=2, len= 4)        # 0x607A - TARGET_POSITION

            # write_RxPDO(master, slave, 5300000, offset=6, len= 4)  # 0x6081 PROFILE VELOCITY 500 rpm

            slave.sdo_write(MODES_OF_OPERATION, 0, struct.pack('<B', 0x3))  #  0x6060 - OP mode (3= Velocity Mode)
            # write_RxPDO(master, slave, 0x3, offset=14, len=1)  #  0x6060 OP mode (3= Velocity Mode)

            # slave.sdo_write(PROFILE_ACCELERATION, 0, struct.pack('<L', 50))  # 50 rpm/s
            # slave.sdo_write(PROFILE_DECELERATION, 0, struct.pack('<L', 50))  # 50 rpm/s
            write_RxPDO(master, slave, 0x06)    # Shutdown
            write_RxPDO(master, slave, 0x0F)    # Enable

            write_RxPDO(master, slave, 1000000, 10, 4)    # 0x60FF TARGET_VELOCITY

            """
            print(f'{statusString(master, slave)} ## ', end = '')
            read_write_RxPDO_SDO(master, slave, *controlWord, 0x0D)   ##### RELEASE previous stall mode

            # read_write_RxPDO_SDO(master, slave, *targetPosition, 130000)    # 0x607A - TARGET_POSITION

            print(f'{statusString(master, slave)} ## ', end = '')
            read_write_RxPDO_SDO(master, slave, *modesOfOperation, 0x3)    # 0x6060 - OP mode (3= Velocity Mode)

            # read_write_RxPDO_SDO(master, slave, *profileAcceleration, 50)  # 50 rpm/s2
            # read_write_RxPDO_SDO(master, slave, *profileDeceleration, 50)  # 50 rpm/s

            print(f'{statusString(master, slave)} ## ', end = '')
            read_write_RxPDO_SDO(master, slave, *controlWord, 0x06)    # Shutdown

            print(f'{statusString(master, slave)} ## ', end = '')
            read_write_RxPDO_SDO(master, slave, *controlWord, 0x0F)    # Enable

            print(f'{statusString(master, slave)} ## ', end = '')
            # read_write_RxPDO_SDO(master, slave, *targetVelocity, -10, False) # 0x60FF TARGET_VELOCITY
            read_write_RxPDO_SDO(master, slave, *targetVelocity, -10000, False) # 0x60FF TARGET_VELOCITY
            

            print(f'{statusString(master, slave)}')

        except Exception as ex:
            print(f"Failed to set velocity mode parameters: {ex}")
            exptTrace(ex)
            master.close()
            return
        
    elif _mode == OpMode.position_test:           # Profile torque mode
        try:
            slave.sdo_write(*DISABLE_CMD)   ##### RELEASE previous stall mode

            slave.sdo_write(MODES_OF_OPERATION, 0, struct.pack('<B', 0x1))  #  OP mode (1= Position Mode)

            slave.sdo_write(PROFILE_VELOCITY, 0, struct.pack('<L', 500000))  # 100 rpm
            
            
            slave.sdo_write(PROFILE_ACCELERATION, 0, struct.pack('<L', 50))  # 50 rpm/s
            slave.sdo_write(PROFILE_DECELERATION, 0, struct.pack('<L', 50))  # 50 rpm/s
            
            slave.sdo_write(*SHUTDOWN_CMD)

            # slave.sdo_write(*ENABLE_CMD)

            # write_RxPDO(master, slave, 100000, offset=2, len=-4)        # TARGET_POSITION

            slave.sdo_write(CONTROL_WORD, 0, struct.pack('<H', 0x2F)) 
            time.sleep(CYCLE_TIME)
            slave.sdo_write(CONTROL_WORD, 0, struct.pack('<H', 0x3F)) 
                                        # Bit 4: 0 → 1  -- New setpoint.
                                        # Load the set-point immediately or after the end of a current movement task:
                                        # Bit 5 = 1: Movement towards the position starts immediately.
                                        # Bit 5 = 0: Movement towards the new position does not start until the preceding positioning task has been completed.
                                        # Bit 6 = 0: Positioning task is absolute.
                                        # Bit 6 = 1: Positioning task is relative.
        except Exception as ex:
            print(f"Failed to set velocity mode parameters: {ex}")
            exptTrace(ex)
            master.close()
            return

    elif _mode == OpMode.position_PDO_test:           #
        try:
            """
            slave.sdo_write(PROFILE_ACCELERATION, 0, struct.pack('<L', 500))  # 50 rpm/s
            slave.sdo_write(PROFILE_DECELERATION, 0, struct.pack('<L', 500))  # 50 rpm/s
            
            write_RxPDO(master, slave, 0x0D)        # Disable

            slave.sdo_write(MODES_OF_OPERATION, 0, struct.pack('<B', 0x1))  #  0x6060 - OP mode (3= Velocity Mode, 1 - Profile Position Mode)

            write_RxPDO(master, slave, 50, offset=6, len= 4)       # 0x6081 - PROFILE_VELOCITY

            # write_RxPDO(master, slave, 99000, 10, 4)    # 0x60FF -TARGET_VELOCITY

            # write_RxPDO(master, slave, 0x01, offset=14, len=1)       # OP mode (1= Position Mode)

            write_RxPDO(master, slave, 0x06)    # shutdown

            write_RxPDO(master, slave, -500000, offset=2, len= 4)        # 0x607A - TARGET_POSITION

            write_RxPDO(master, slave, 0x3F)            
                                        # Load the set-point immediately or after the end of a current movement task:
                                        # Bit 5 = 1: Movement towards the position starts immediately.
                                        # Bit 5 = 0: Movement towards the new position does not start until the preceding positioning task has been completed.

            print(f"Status Word: {hex(struct.unpack('<H', slave.input[0:2])[0])} ", end ='')
            print(f" Mode of OP: {hex(struct.unpack('<H', slave.input[6:8])[0])} ", end ='')
            print(f' AL Status: {hex(slave.al_status)} ({pysoem.al_status_code_to_string(slave.al_status)})')


            # slave.sdo_write(*ENABLE_CMD)

            # time.sleep(0.05)

            # write_RxPDO(master, slave, 0x0F)        # TARGET_POSITION
            # time.sleep(10*CYCLE_TIME)
            # write_RxPDO(master, slave, 0x7F)        # TARGET_POSITION

                                        # Bit 4: 0 → 1  -- New setpoint.
                                        # Load the set-point immediately or after the end of a current movement task:
                                        # Bit 5 = 1: Movement towards the position starts immediately.
                                        # Bit 5 = 0: Movement towards the new position does not start until the preceding positioning task has been completed.
                                        # Bit 6 = 0: Positioning task is absolute.
                                        # Bit 6 = 1: Positioning task is relative.
            """
            _PDO_faultReset(master, slave)

            # read_write_RxPDO_SDO(master, slave, *controlWord, 0x0d)           # Disable
            # read_write_RxPDO_SDO(master, slave, *controlWord, 0x07)    # switch on
            # read_write_RxPDO_SDO(master, slave, *controlWord, 0x0f)    # enable

            # read_write_RxPDO_SDO(master, slave, *profileAcceleration, 5000)  # 50 rpm/s2  rps/s2
            # read_write_RxPDO_SDO(master, slave, *profileDeceleration, 5000)  # 50 rpm/s
            
            print(f'{statusString(master, slave)} ## ', end = '')
            read_write_RxPDO_SDO(master, slave, *modesOfOperation, 0x1)  #  0x6060 - OP mode (3= Velocity Mode, 1 - Profile Position Mode)

            print(f'{statusString(master, slave)} ## ', end = '')
            read_write_RxPDO_SDO(master, slave, *profileVelocity, 1000)        # 0x6081 - PROFILE_VELOCITY
            # read_write_RxPDO_SDO(master, slave, *profileVelocity, 10000)        # 0x6081 - PROFILE_VELOCITY // maxon

            print(f'{statusString(master, slave)} ## ', end = '')
            # read_write_RxPDO_SDO(master, slave, *targetPosition, 1000000, False)     # 0x607A - TARGET_POSITION
            read_write_RxPDO_SDO(master, slave, *targetPosition, 0, False)     # 0x607A - TARGET_POSITION


            print(f'{statusString(master, slave)} ## ', end = '')
            read_write_RxPDO_SDO(master, slave, *controlWord, 0x06)    # shutdown

            print(f'{statusString(master, slave)} ## ', end = '')
            read_write_RxPDO_SDO(master, slave, *controlWord, 0x07)    # switch on

            print(f'{statusString(master, slave)} ## ', end = '')
            read_write_RxPDO_SDO(master, slave, *controlWord, 0x0f)    # enable

            time.sleep(0.1)
            print(f'{statusString(master, slave)} ## ', end = '')
            read_write_RxPDO_SDO(master, slave, *controlWord, 0x1F)  
                                        # Load the set-point immediately or after the end of a current movement task:
                                        
                                        # Bit 4: 0 → 1  -- New setpoint.
                                        # Load the set-point immediately or after the end of a current movement task:
                                        # Bit 5 = 1: Movement towards the position starts immediately.
                                        # Bit 5 = 0: Movement towards the new position does not start until the preceding positioning task has been completed.
                                        # Bit 6 = 0: Positioning task is absolute.
                                        # Bit 6 = 1: Positioning task is relative.

            print(f'{statusString(master, slave)}')


                                        #

        except Exception as ex:
            print(f"Failed to set velocity mode parameters: {ex}")
            exptTrace(ex)
            master.close()
            return

    elif _mode == OpMode.cyclic_synchronous_position_Mode_PDO_test:           #
        try:
            """
            write_RxPDO(master, slave, 0x0D)        # Disable

            # slave.sdo_write(PROFILE_ACCELERATION, 0, struct.pack('<L', 500))  # 50 rpm/s
            # slave.sdo_write(PROFILE_DECELERATION, 0, struct.pack('<L', 500))  # 50 rpm/s

            slave.sdo_write(0x60C2, 1, struct.pack('<B', 200))                #  Interpolation time period value
            slave.sdo_write(0x60C2, 2, struct.pack('<b', -5))                #  Interpolation time index

            slave.sdo_write(MODES_OF_OPERATION, 0, struct.pack('<B', 0x8))  #  OP mode (3= Velocity Mode, 
                                                                            # 1 - Profile Position Mode)
                                                                            # 8 -cyclic synchronous position Mode

            
            # slave.sdo_write(0x60B1, 0, struct.pack('<L', 0))                #  Velocity offset
            # slave.sdo_write(0x60B2, 0, struct.pack('<H', 0))                #  Torque offset

            write_RxPDO(master, slave, 0, offset=14, len= 4)         #  Velocity offset
            write_RxPDO(master, slave, 0, offset=18, len= 2)        #  Torque offset

            write_RxPDO(master, slave, 129686390, offset=2, len= 4)        # 0x607A - TARGET_POSITION


            write_RxPDO(master, slave, 0x06)    # shutdown

            write_RxPDO(master, slave, 0x1F)    # Enable
            """
            read_write_RxPDO_SDO(master, slave, *controlWord, 0x0D)           # Disable

            read_write_RxPDO_SDO(master, slave, *profileAcceleration, 500)  # 500 rpm/s2
            read_write_RxPDO_SDO(master, slave, *profileDeceleration, 500)  # 500 rpm/s

            read_write_RxPDO_SDO(master, slave, *interpolationTimePeriodValue, 200)  # 500 rpm/s
            read_write_RxPDO_SDO(master, slave, *interpolationTimeIndex, -5, False)  # 500 rpm/s

            read_write_RxPDO_SDO(master, slave, *modesOfOperation, 0x8)
                                                                            #  OP mode (3= Velocity Mode, 
                                                                            # 1 - Profile Position Mode)
                                                                            # 8 -cyclic synchronous position Mode

            

            read_write_RxPDO_SDO(master, slave, *velocityOffset, 1000, False)        #  Velocity offset
            read_write_RxPDO_SDO(master, slave, *torqueOffset, 10, False)        #  Torque offset

            read_write_RxPDO_SDO(master, slave, *targetPosition, 129686390, False)       # TARGET POSITION

            read_write_RxPDO_SDO(master, slave, *controlWord, 0x06)    # shutdown

            read_write_RxPDO_SDO(master, slave, *controlWord, 0x1F)    # Enable

        except Exception as ex:
            print(f"Failed to set velocity mode parameters: {ex}")
            exptTrace(ex)
            master.close()
            return



    elif  _mode == OpMode.homming_mode:
        try:
            _PDO_faultReset(master, slave)

            read_write_RxPDO_SDO(master, slave, *controlWord, 0x06)    # Shutdown
            read_write_RxPDO_SDO(master, slave, *controlWord, 0x0E)        # op
            read_write_RxPDO_SDO(master, slave, *controlWord, 0x0F)    # Enable


            read_write_RxPDO_SDO(master, slave, *modesOfOperation, 0x6)
            # slave.sdo_write(MODES_OF_OPERATION, 0, struct.pack('<H', 0x6))  
                                                                            #  OP mode (3= Velocity Mode, 
                                                                            # 1 - Profile Position Mode)
                                                                            # 6 - home
            # slave.sdo_write(0x6098, 0, struct.pack('<H', 35))               # homing method, ; 35 -Homing with the current position
            read_write_RxPDO_SDO(master, slave, *hommingMethod, 0x37, False)  # FESTO homing method, ; 37 -Homing with the current position


            read_write_RxPDO_SDO(master, slave, *controlWord, 0x0F)    # Enable
            read_write_RxPDO_SDO(master, slave, *controlWord, 0x1F)    # starting homming
            read_write_RxPDO_SDO(master, slave, *controlWord, 0x0D)    # Disable

            
            time.sleep(0.1)
        except Exception as ex:
            print(f"Failed to set velocity mode parameters: {ex}")
            exptTrace(ex)
            master.close()
            return
        print(f"Homing done. Current position - {int.from_bytes(slave.sdo_read(*POSITION_ACTUAL_VALUE_QUERY), byteorder='little', signed=False)}")

    else:
        print(f"Unsupported operation mode: {_mode}")
    

    


def _mainCycle(master, slave):
    # Cyclic PDO loop for monitoring
    global _pd_thread_stop_event
    
    try:
        actual_pos = None
        posInd = None
        statInd = None
        ch_status_word = 0

        while True:
            # master.send_processdata()
            # master.receive_processdata(TIMEOUT)

            # Read Status Word and Actual Position from TxPDO and SDO
            # ch_status_word = struct.unpack('<H',slave.sdo_read(STATUS_WORD, 0))[0]

            # status_word = struct.unpack('<H', slave.input[0:2])[0]
            status_word =  read_TxPDO_SDO(master, slave, *statusWord)
            if status_word & FAULT_BIT:
                _PDO_faultReset(master, slave)
            # actual_pos = struct.unpack('<l', slave.input[2:6])[0]
            actual_pos = read_TxPDO_SDO(master, slave, *positionActualValue, False)


            if posInd is None or (abs(posInd - actual_pos) > TOLERANCE_POSITION) or statInd != status_word:
                posInd = actual_pos
                statInd = status_word
                print(f"CYCLE RUN* Status Word: {hex(status_word)} /{hex(ch_status_word)}, Actual Position: {actual_pos}", end ='')
                print(f' AL Status: {hex(slave.al_status)} ({pysoem.al_status_code_to_string(slave.al_status)})')

                
            if status_word & (1 << 10):  # Target reached
                print("Target position reached")
                break


            # time.sleep(CYCLE_TIME)
            time.sleep(0.1)
        
    
    except KeyboardInterrupt:
        _pd_thread_stop_event.set()  # Stop cycle
        raise Exception("Test interrupted by user.")
        

    except Exception as ex:
        print(f"Error in PDO loop: {ex}")
        exptTrace(ex)

    _pd_thread_stop_event.set()  # Stop cycle

    print(f">>> Current Position = {actual_pos}")

def _clear_PDO_data (master, slave):
    _output_buf_lenght = len(slave.output)
    _fill_buf = bytearray([0 for _ in range(_output_buf_lenght)])
    slave.output = bytes(_fill_buf)

def _processdata_thread(master, slave):

    global _pd_thread_stop_event
    while not _pd_thread_stop_event.is_set():
        sync.acquire()
        master.send_processdata()
        _actual_wkc = master.receive_processdata(timeout=100_000)
        sync.release()
        _expected_wkc = master.expected_wkc
        if _actual_wkc != _expected_wkc:
            print(f"incorrect wkc, actual = {_actual_wkc}, expected_wkc = {_expected_wkc}")
        time.sleep(CYCLE_TIME)
    print(f'End of PDO process data cycle')

def testAbs(_interface:str):
    global     _pd_thread_stop_event

    master = None
    try:    
        master, slave = _initOp(_interface)
        if not master or not slave:
            print("Failed to initialize EtherCAT master or slave.")
            sys.exit()
        _configPDO(master, slave)

        _clear_PDO_data (master, slave)

        _pd_thread_stop_event.clear()  # Clear the stop event to allow the thread to run

        _pdo_thread = threading.Thread(target=_processdata_thread, args=(master, slave, ))
        _pdo_thread.start() 
        # send one valid process data to make outputs in slave happy
        master.send_processdata()
        master.receive_processdata(timeout=2000)

        _initOpState(master, slave)     

        _setMotionParms(master, slave, OpMode.homming_mode)
        # raise Exception("Homimg done")

        _intOpEnabled(master, slave)
        if master.state == pysoem.OP_STATE:
            master.in_op = True
        else:
            # print(f'ERROR: The slave is in non OP state. Current state = {master.state} / {getState(master.state)}')
            raise Exception(f'ERROR: The slave is in non OP state. Current state = {master.state} / {getState(master.state)}')

        # time.sleep(3)

        # _setMotionParms(master, slave, OpMode.homming_mode)
        

        # _setUserUnits(master, slave)


        # _setMotionParms(master, slave, OpMode.cyclic_synchronous_position_Mode_PDO_test)
        _setMotionParms(master, slave, OpMode.position_PDO_test)
        # _setMotionParms(master, slave, OpMode.velocity_PDO_test)
        # _setMotionParms(master, slave, OpMode.position_test)      
        # _setMotionParms(master, slave, OpMode.velocity_test)      
        # _setMotionParms(master, slave, OpMode.position_mode)      
        # _setMotionParms(master, slave, OpMode.velocity_mode)    

        # _pd_thread_stop_event.clear()  # Clear the stop event to allow the thread to run

        # _pdo_thread = threading.Thread(target=_processdata_thread, args=(master, slave, ))
        # _pdo_thread.start()          

        _mainCycle(master, slave)
        _pdo_thread.join()
        master.state = pysoem.INIT_STATE
        master.write_state()


    
    except Exception as ex:
        print(f"Error. ABS movement failed: {ex}")
        exptTrace(ex)

    if master is not None:
        master.close()

def getState(_state:int) -> str:
    
    if _state == 0x00:
        return 'EC_STATE_NONE'
    elif _state == 0x01:
        return 'EC_STATE_INIT'
    elif _state == 0x02:
        return 'EC_STATE_PRE_OP'
    elif _state == 0x03:
        return 'EC_STATE_BOOT'
    elif _state == 0x04:
        return 'EC_STATE_SAFE_OP'
    elif _state == 0x08:
        return 'EC_STATE_OPERATIONAL'
    elif _state == 0x10:
        return 'EC_STATE_ACK'
    else:
        return f'UNKNOWN STATE-{_state}'



def _print_SDO(device, ind:int, sub_ind:int = 0): 
    try:    
        print(f' [{ind}, {sub_ind}] ( [{hex(ind)}, {hex(sub_ind)}])', end = '')

        _data = None
        _obj = device.od[ind]
        _sdo_index = _obj.index

        print(f'=[{hex(_sdo_index)}, {hex(sub_ind)}], size=[{len(_obj.entries)}] ', end = '')


        if len(_obj.entries) > 0:
            _entry = _obj.entries[sub_ind]
        else:
            _entry = None

        _len = _entry.bit_length if _entry is not None else _obj.bit_length
        print(f' data len = [{_len}] ', end ='')

        if _len > 64:
            _data = device.sdo_read(_sdo_index, sub_ind).decode('utf-8')
        else:
            _data = int.from_bytes(device.sdo_read(_sdo_index, sub_ind), byteorder='little', signed=True)
    except pysoem.SdoInfoError as ex:
        print(f'_print_SDO: no SDO info for {device.name}, exception = {ex}')
    except Exception as ex:
        print(f'critical SDO exception = {ex}')
    else:
        # print(f' ** data = {_data:#06x}')
        if _len > 64:
            print(f' ** data = {_data}')
        else:
            print(f' ** data = {hex(_data)}')


def slave_conf_func(slave_pos):
    print(f'Callback [ Pre-Operational >>> Operationa], slave pos = {slave_pos}, state = {GLOBAL_SLAVE_0.state})/{getState(GLOBAL_SLAVE_0.state)}')

def slave_setup_func(slave_pos):
    print(f'Callback [ Pre-Operational >>> Safe-Operationa], slave pos = {slave_pos}, state = {GLOBAL_SLAVE_0.state})/{getState(GLOBAL_SLAVE_0.state)}')



def read_sdo_info(ifname):
    master = pysoem.Master()
    
    master.open(ifname)
    
    if master.config_init() > 0:
        print(f'Adapter {ifname} opened, slaves: {master.slaves}')
    
        for slave in master.slaves:
            try:
                od = slave.od
            except pysoem.SdoInfoError as ex:
                exptTrace(ex)
                print('read_sdo_info: no SDO info for {}'.format(slave.name))
            else:
                print(slave.name)

                for __ind, obj in enumerate(od):
                    # print(' Idx: {}; Code: {}; Type: {}; BitSize: {}; Access: {}; Name: "{}"'.format(
                    #     hex(obj.index),
                    #     obj.object_code,
                    #     obj.data_type,
                    #     obj.bit_length,
                    #     hex(obj.obj_access),
                    #     obj.name))
                    print(f' Idx: {hex(obj.index)} ({hex(__ind)}); Code: {obj.object_code}; Type: {obj.data_type}; ', end='')
                    print(f'BitSize: {obj.bit_length}; Access: {hex(obj.obj_access)}; Name: "{obj.name}" ', end = '')
                    if len(obj.entries) == 0:
                        _print_SDO(slave, __ind)
                        continue
                    else:
                        print(f'')
                    for i, entry in enumerate(obj.entries):
                        if entry.data_type > 0 and entry.bit_length > 0:
                            # print('  Subindex {}; Type: {}; BitSize: {}; Access: {} Name: "{}"'.format(
                            #     i,
                            #     entry.data_type,
                            #     entry.bit_length,
                            #     hex(entry.obj_access),
                            #     entry.name))
                            print(f'  Subindex {i} ({hex(i)}); Type: {entry.data_type}; BitSize: {entry.bit_length}; ', end ='')
                            print(f'(Access: {hex(entry.obj_access)} Name: "{entry.name}" ', end ='')
                            # _data = None
                            # if entry.bit_length > 64:
                            #     _data = slave.sdo_read(obj.index, i).decode('utf-8')
                            # else:
                            #     _data = int.from_bytes(slave.sdo_read(obj.index, i), byteorder='little', signed=True)
                            # print(f'data = {_data}')
                            _print_SDO(slave, __ind, i)

    else:
        print('no slave available')
        
    master.close()

DEVICE_TYPE_QUERY = (0x1000, 0x00)
DEVICE_NAME_QUERY = (0x1008, 0x00)
ACTUAL_POSITION_QUERY = (0x6064, 0x0)

VENDOR_ID_QUERY = (0x1018, 0x01)
PRODUCT_CODE_QUERY = (0x1018, 0x02)
SN_QUERY = (0x1018, 0x04)
POSITION_ACTUAL_VALUE_QUERY = (0x6064, 0x0)


SHUTDOWN_CMD = (0x6040, 0x0, struct.pack('H', 0x06))
SWITCHON_CMD = (0x6040, 0x0, struct.pack('H', 0x07))
ENABLE_CMD =   (0x6040, 0x0, struct.pack('H', 0x0F))
# ENABLE_CMD =  (0x6040, 0x0, (0xF).to_bytes(2, byteorder='little', signed=False))
DISABLE_CMD = (0x6040, 0x0, bytes(ctypes.c_uint16(0xD)))
# PROFILE_POSITION_MODE = (0x6060, 0x0, 0x1, 0x1)
PROFILE_VELOCITY_MODE = (0x6060, 0x0, 0x3, 0x1)





class InputPdo(ctypes.Structure):
    _pack_ = 1
    _fields_ = [('position_actual_value', ctypes.c_int32)]  # Adjust based on mapping


def PDO_test(inteface):
    try:
        master = pysoem.Master()

        # master.open(sys.argv[1])
        master.open(inteface)
        
        if master.config_init() > 0:
            master.config_map()
            master.config_overlap_map()
            while True:
                time.sleep(1)
                master.state = pysoem.OP_STATE
                master.write_state()
                _state = master.state_check(pysoem.OP_STATE)
                print(f'[OP_STATE =  {_state}]  ', end = '')
                for slave_pos, __dev in enumerate(master.slaves):
                    # master.read_state() 
                    _POSITION = int.from_bytes(__dev.sdo_read(*ACTUAL_POSITION_QUERY, release_gil=False), byteorder='little', signed=False)
                    print(f'[{_POSITION}]  ', end = '')

                    master.send_processdata()

                    master.state = pysoem.OP_STATE
                    master.write_state()
                    

                    wkc = master.receive_processdata(1000)
                    print(f'PDO: {wkc} / {__dev.input}')

                    input_data = InputPdo.from_buffer_copy(master.slaves[slave_pos].input)
                    print(f"Position from PDO: {input_data.position_actual_value}")
                print(f'')
        else:
            raise Exception(f'Config init failed error')

    except KeyboardInterrupt as ex:
        print(f'Exiting by ^C \n{ex}')
        
    except Exception as ex:
        print(f'Exiting by critical exeption \n{ex}')
        exptTrace(ex)

    master.close()

def emergency_callback(emergency_data:pysoem.Emergency):
        print(f'EMERGENCY CALLBACK: {emergency_data:}')
    # Handle emergency data here, e.g., log it or take action


def _confPrint():

    adapters = pysoem.find_adapters()
    master = pysoem.Master()

    for i, adapter in enumerate(adapters):
        master.open(adapter.name)
        if master.config_init() > 0:
            _pdo_size = master.config_map()
            _pdo_ovearll = 0
            # _pdo_ovearll = master.config_overlap_map()
            print(f'Adapter {i}, name = {adapter.name}, desc = {adapter.desc}')
            
            for device in master.slaves:
                _faultReset(master, device)
              
                try:
                    print(f'Found Device {device.name}, ', end = '')
                    _TYPE = int.from_bytes(device.sdo_read(*DEVICE_TYPE_QUERY), byteorder='little', signed=False)
                    _NAME = device.sdo_read(*DEVICE_NAME_QUERY).decode('utf-8')
                    _SN = int.from_bytes(device.sdo_read(*SN_QUERY), byteorder='little', signed=False)
                    _VENDOR = int.from_bytes(device.sdo_read(*VENDOR_ID_QUERY), byteorder='little', signed=False)
                    _PRODUCT = int.from_bytes(device.sdo_read(*PRODUCT_CODE_QUERY), byteorder='little', signed=False)
                    _POSITION = int.from_bytes(device.sdo_read(*POSITION_ACTUAL_VALUE_QUERY), byteorder='little', signed=False)
                    print(f'TYPE = {_TYPE}, NAME= {_NAME}, VENDOR = {_VENDOR}, PRODUCT = {_PRODUCT}, SN = {_SN},', end ='')
                    print(f' POSITION = {_POSITION}', end = '')
                    # print(f"  Input size: {device.iopl}", end = '')
                    # print(f"  Output size: {device.opl}", end = '')
                    print(f'')
                    print(f" Input size: {len(device.input)} bytes", end = '')
                    print(f" Output size: {len(device.output)} bytes", end = '')
                    print(f" Configured state: {device.state}", end = '')
                    print(f" AL status code: {hex(device.al_status)}", end = '')
                    print(f'')
                    n_entries = device.sdo_read(0x1C12, 0)[0]
                    for i in range(1, n_entries + 1):
                        mapped_index = struct.unpack("<H", device.sdo_read(0x1C12, i)[:2])[0]
                        print(f"RxPDO Mapping {i}: 0x{mapped_index:04X}")
                except Exception as ex:
                    print(f'Exception printing configuration readed by EtherCAT')
                    exptTrace(ex)

            print(f' MAP SIZE = {_pdo_size}, MAP OVERALL = {_pdo_ovearll}')
        # else:
        #     print('no device found')

        master.close()



def bytes2HEX(data:bytes) -> str:
    return ' '.join(f'{b:02x}' for b in data)


if __name__ == '__main__':
# refer to https://github.com/bnjmnp/pysoem/blob/master/src/pysoem/pysoem.pyx
# example -> https://github.com/bnjmnp/pysoem/blob/master/examples/read_sdo_info.py





    _startup_bytes = {
        0x1A00 : [0x05, 0x00, 0x10, 0x00, 0x41, 0x60, 0x20, 0x00, 0x64, 0x60, 0x10, 0x00, 0x77, 0x60, 0x10, 0x00, 0xB9, 0x60, 0x20, 0x00, 0xBA, 0x60],
        0x1600 : [0x05, 0x00, 0x10, 0x00, 0x40, 0x60, 0x20, 0x00, 0x7A, 0x60, 0x20, 0x00, 0xB1, 0x60, 0x10, 0x00, 0xB2, 0x60, 0x10, 0x00, 0xB8, 0x60],
        0x1A10 : [0x05, 0x00, 0x10, 0x00, 0x41, 0x68, 0x20, 0x00, 0x64, 0x68, 0x10, 0x00, 0x77, 0x68, 0x10, 0x00, 0x89, 0x68, 0x20, 0x00, 0xBA, 0x68],
        0x1610 : [0x05, 0x00, 0x10, 0x00, 0x40, 0x68, 0x20, 0x00, 0x7A, 0x68, 0x20, 0x00, 0xB1, 0x68, 0x10, 0x00, 0xB2, 0x68, 0x10, 0x00, 0x88, 0x68],
        0x1A20 : [0x05, 0x00, 0x10, 0x00, 0x41, 0x70, 0x20, 0x00, 0x64, 0x70, 0x10, 0x00, 0x77, 0x70, 0x10, 0x00, 0x89, 0x70, 0x20, 0x00, 0xBA, 0x70],
        0x1620 : [0x05, 0x00, 0x10, 0x00, 0x40, 0x70, 0x20, 0x00, 0x7A, 0x70, 0x20, 0x00, 0x81, 0x70, 0x10, 0x00, 0xB2, 0x70, 0x10, 0x00, 0x88, 0x70],
        0x1A30 : [0x05, 0x00, 0x10, 0x00, 0x41, 0x78, 0x20, 0x00, 0x64, 0x78, 0x10, 0x00, 0x77, 0x78, 0x10, 0x00, 0xB9, 0x78, 0x20, 0x00, 0xBA, 0x78],
        0x1630 : [0x05, 0x00, 0x10, 0x00, 0x40, 0x78, 0x20, 0x00, 0x7A, 0x78, 0x20, 0x00, 0xB1, 0x78, 0x10, 0x00, 0x82, 0x78, 0x10, 0x00, 0x88, 0x78],
        0x1C12 : [0x04, 0x00, 0x00, 0x16, 0x10, 0x16, 0x20, 0x16, 0x30, 0x16],
        0x1C13 : [0x04, 0x00, 0x00, 0x1A, 0x10, 0x1A, 0x20, 0x1A, 0x30, 0x1A],
        
    }
    

    _confPrint()

    print('UNITEST script started')

    if len(sys.argv) != 2:
        print('give ifname as script argument')
        sys.exit()

    # read_sdo_info(sys.argv[1])
    # sys.exit()

    # PDO_test(sys.argv[1])

    testAbs(sys.argv[1])
    # _confPrint()
    sys.exit()
    
    slave = None
    try:
        master = pysoem.Master()
        master.open(sys.argv[1])
        if (_ci := master.config_init()) <= 0:
            raise Exception(f'SDO master opening failed')
        print(f'Master  {master} opened, slaves by config init = {_ci}')

        print(f'Adapter {sys.argv[1]} opened, slaves: {master.slaves}')
        slave = master.slaves[0]
        print(f'Working with slave: {slave}, vendor id = {slave.man}, name = {slave.name}, rev = {slave.rev}')

        _configPDO(master, slave)


        _cd = master.config_dc()
        print(f'DC slaves by config_dc = {_cd}')
        master.config_map()
        master.config_overlap_map()

        slave.add_emergency_callback(emergency_callback)
        slave.dc_sync(act=True, sync0_cycle_time=1000, sync0_shift_time=0, sync1_cycle_time=None)
        print(f'State: {master.read_state()}, {getState(master.read_state())}')



    except Exception as ex:
        print(f'Exiting by critical exeption \n{ex}')
        exptTrace(ex)
    
    
    try:

        slave.sdo_write(*SHUTDOWN_CMD)
        master.write_state()
        print(f'State: {master.read_state()}, {getState(master.read_state())}')
        slave.sdo_write(*SWITCHON_CMD)
        master.write_state()
        print(f'State: {master.read_state()}, {getState(master.read_state())}')
        slave.sdo_write(*ENABLE_CMD)
        master.write_state()
        print(f'State: {master.read_state()}, {getState(master.read_state())}')


        while True:
            try:
                time.sleep(1)                   
            except KeyboardInterrupt as ex:
                print(f'Exiting by ^C \n{ex}')
                break
        
    except Exception as ex:
        print(f'Exiting by critical exeption \n{ex}')
        exptTrace(ex)

    slave.sdo_write(*DISABLE_CMD)
    slave.sdo_write(*SHUTDOWN_CMD)
    master.close()
    print('UNITEST script finished')

    sys.exit()

