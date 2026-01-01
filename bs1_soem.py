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
import pysoem
from collections import namedtuple
from typing import List

from bs1_utils import exptTrace


"""
EC_STATE_NONE           = 0x00
EC_STATE_INIT           = 0x01
EC_STATE_PRE_OP         = pysoem.PREOP_STATE           # 0x02
EC_STATE_BOOT           = pysoem.BOOT_STATE           # 0x03
EC_STATE_SAFE_OP        = pysoem.SAFEOP_STATE           # 0x04 
EC_STATE_OPERATIONAL    = pysoem.OP_STATE           # 0x08
EC_STATE_ACK            = 0x10
EC_STATE_ERROR          = 0x10
"""


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
        # 1 [Acc. unit] = 2303:01 hex/2303:02 hex x 104 [inc/s2]


syncMngr2PDO = 0x1C12
        # Sync manager 2 PDO assignment
syncMngr2PDO_Step = 0x10                      # (0x1600, 0x1610, 0x1620, etc.) MUST be configurable per device
syncMngr3PDO = 0x1C13
        # Sync manager 3 PDO assignment
syncMngr3PDO_Step = 0x10                      #  (0x1A00, 0x1A10, 0x1A20, etc.) MUST be configurable per device


###########

DEVICE_TYPE_QUERY = (0x1000, 0x00)
DEVICE_NAME_QUERY = (0x1008, 0x00)
ACTUAL_POSITION_QUERY = (0x6064, 0x0)

VENDOR_ID_QUERY = (0x1018, 0x01)
PRODUCT_CODE_QUERY = (0x1018, 0x02)
SN_QUERY = (0x1018, 0x04)
POSITION_ACTUAL_VALUE_QUERY = (0x6064, 0x0)
# POSITION_ACTUAL_VALUE_QUERY = (0x6000, 0x11)
# POSITION_ACTUAL_VALUE_QUERY = (0x1a00, 0x1)




#### TIMEOUTS  ######

## YASKAWA ########
PreopTimeout = 4000
SafeopOpTimeout = 4000
BackToInitTimeout = 4000 
BackToSafeopTimeout = 100

MB_RequestTimeout = 100
MB_ResponseTimeout = 4000

__soem_slave_data = ["global_name", "dev_type", "dev_name", "dev_sn", "dev_vendor", "dev_product", \
                            "master_device", "slave_device"]
slaveItem = namedtuple("slaveItem",  __soem_slave_data, defaults=[None,] * len(__soem_slave_data))
slaveItem.__annotations__={'global_name':str, 'dev_type':int, 'dev_name':str, 'dev_sn':int, 'dev_vendor':int, \
                           'dev_product':int, 'master_device':pysoem.Master, 'slave_device':pysoem.CdefSlave}

__soem_config_data = ["adapter_name", "master_device", "slaves"]
masterItem = namedtuple("masterItem",  __soem_config_data, defaults=[None,] * len(__soem_config_data))
masterItem.__annotations__={'adapter_name':str, 'master_device':pysoem.Master, 'slaves':List[slaveItem]}



class BaseSOEM:
    adapters:List[masterItem] = None

    @staticmethod
    def initSOEM() -> List[masterItem]:   
        """
        Initialize SOEM and return list of adapters with slaves
        :return: List of masterItem namedtuples
        """
        BaseSOEM.adapters = list()

        adapters = pysoem.find_adapters()
        print(f'Found {len(adapters)} adapters ')
        for adapter in adapters:
            _tmp_master_dev = None
            _tmp_master_dev = pysoem.Master()
            try:
                _tmp_master_dev.open(adapter.name)
                if _tmp_master_dev.config_init() > 0:
                    _pdo_size = _tmp_master_dev.config_map()
                    print(f'EtherCAT Adapter {adapter.name} opened, slaves: {_tmp_master_dev.slaves}, PDO size: {_pdo_size}')
                    slaves = list()
                    for _tmp_slave_device in _tmp_master_dev.slaves:
                        print(f'Found Slave Device {_tmp_slave_device.name}: {_tmp_slave_device}: id={_tmp_slave_device.id}. Vendor ID of the slave (man) = {_tmp_slave_device.man}, name = {_tmp_slave_device.name}, state= {_tmp_slave_device.state}, al_status = {_tmp_slave_device.al_status}')
                        _type = int.from_bytes(_tmp_slave_device.sdo_read(*DEVICE_TYPE_QUERY), byteorder='little', signed=False)
                        # _name = _tmp_master_dev.sdo_read(*DEVICE_NAME_QUERY).decode('utf-8')
                        _name = _tmp_slave_device.name
                        _sn = int.from_bytes(_tmp_slave_device.sdo_read(*SN_QUERY), byteorder='little', signed=False)
                        # _vendor = int.from_bytes(_tmp_slave_device.sdo_read(*VENDOR_ID_QUERY), byteorder='little', signed=False)
                        _vendor = _tmp_slave_device.man
                        # _product = int.from_bytes(_tmp_slave_device.sdo_read(*PRODUCT_CODE_QUERY), byteorder='little', signed=False)
                        _product = _tmp_slave_device.id
                        slaves.append(slaveItem(_name, _type, _name, _sn, _vendor, _product,\
                                                 _tmp_master_dev, _tmp_slave_device))
                    BaseSOEM.adapters.append(masterItem(adapter.name, _tmp_master_dev, slaves))
                    print(f'Adapter {BaseSOEM.adapters[-1].adapter_name} configured and opened, master: {BaseSOEM.adapters[-1].master_device} slaves: {BaseSOEM.adapters[-1].slaves}')
                else:
                    # print(f'No slaves found for adapter {adapter.name}')
                    _tmp_master_dev.close()
                    
            except Exception as ex:
                exptTrace(ex)

                # raise Exception(f'Error initializing adapter {adapter.name}: {ex}')
                print(f'Error initializing adapter {adapter.name}: {ex}')
            
            # finally:
            #     _tmp_master_dev.close()  
        
        return BaseSOEM.adapters
    

    def __init__(self, adapter_name:str, slave_index = 0, slave_id = None):
        """
        Initialize BaseSOEM with a specific adapter name
        :param adapter_name: Name of the adapter to initialize
        """
        if not isinstance(self.adapter_name, str):
            raise TypeError(f'Adapter name must be a string, got {type(self.__adapter_name)}')
        if len(self.adapter_name) == 0:
            raise ValueError('Adapter name cannot be an empty string')


        # if BaseSOEM.adapters is None or len(BaseSOEM.adapters) == 0:
        #     raise Exception('No adapters found, please run BaseSOEM.initSOEM() first')

        if BaseSOEM.adapters is None:
            BaseSOEM.initSOEM()
        
        self.__adapter_name = adapter_name
        self.__master:masterItem = None
        self.__slave:slaveItem = None
        self.__master_index = None
        self.__slave_index = slave_index
        self.__slave_id = slave_id

        self._slave_device = None 
        self._master_device = None


        print(f'Initializing BaseSOEM for adapter {self.__adapter_name}, slave device at index: {self.__slave_index}')

        for _ind, _master_if in enumerate(BaseSOEM.adapters):
            if _master_if.adapter_name == self.adapter_name and _master_if.master is not None and len(_master_if.slaves) >= self.__slave_index:
                self.__master_index = _ind
                self.__master = _master_if  
                self.__slave = self.__master.slaves[self.__slave_index]
                break

        if self.__master is None:
            raise Exception(f'ERROR: No or None master found for adapter {self.adapter_name} ({_master_if.master}) or slave number {self.__slave_index} is greater than the number of slaves ({len(BaseSOEM.adapters[self.__master_index].slaves)})')
        
        self._slave_device = self.__slave.slave_device
        self._master_device = self.__master.master_device


        print(f'BaseSOEM initialized for device {self.__slave.dev_name} at adapter {self.adapter_name}, master: {self.__master}')
        self._slave_device.config_func = self.slave_conf_func
        self._slave_device.setup_func = self.slave_setup_func
        self._slave_device.state = pysoem.PREOP_STATE
        self._slave_device.write_state()
        self._slave_device.state_check(pysoem.PREOP_STATE, timeout=PreopTimeout)
        print(f'Slave device {self.__slave.dev_name} state set to PREOPERATIONAL, state = {self._slave_device.state}/{BaseSOEM.getState(self._slave_device.state)}')
    
        try:


            self._slave_device.dc_sync(act=False, sync0_cycle_time=1000000, sync0_shift_time=0, sync1_cycle_time=None)
                                            # Sync to Distributed Clock (DC) synchronization to 25 ms period


            
            sync_manager = self._slave_device.sdo_read(0x1C00, 0)
            print(f"Sync Manager Entries: {sync_manager.hex()}")
            for i in range(1, 5):
                sm_type = self._slave_device.sdo_read(0x1C00, i)
                print(f"Sync Manager {i}: {sm_type.hex()}")

        except Exception as ex:
            print(f"Failed to config slave {self.__slave.name} interface {self.__adapter_name}: {ex}")
            exptTrace(ex)
            raise ex 


    def __del__(self):
        """
        Destructor to close all masters
        """
        if BaseSOEM.adapters is not None and len(BaseSOEM.adapters) > 0:
            print(f'Slave device object for {self.__master}:{self.__slave} destructing...')
            if self.__master is not None and self.__slave in self.__master.slaves:   
                self.__master.slaves.remove(self.__slave)
                print(f'Removed slave {self.__slave.dev_name} from master {self.__master.adapter_name}')
                if len(self.__master.slaves) == 0:
                    print(f'No slaves left for master {self.__master.adapter_name}, closing master device and deleting master from adapters list')
                    self.__master.master_device.close()
                    BaseSOEM.adapters.remove(self.__master)
                    
            else:
                print(f'WARNING: No slave {self.__slave.dev_name} found in master {self.__master.adapter_name}')
            
            BaseSOEM.adapters = None
            print(f'BaseSOEM for {self.__master}:{self.__slave} destructed')
        else:
            print(f'WARNING: No adapters found to close in BaseSOEM list {BaseSOEM.adapters}')

    def __str__(self):
        """
        String representation of the BaseSOEM instance
        :return: String representation of the BaseSOEM instance
        """
        return f'BaseSOEM(adapter_name={self.__adapter_name},  slave={self.__slave})'
    
    def __repr__(self):
        """
        String representation of the BaseSOEM instance for debugging
        :return: String representation of the BaseSOEM instance
        """
        return f'BaseSOEM(adapter_name={self.__adapter_name}, master={self.__master}, slave={self.__slave})'    
    
    def slave_conf_func(self, slave_pos):
        print(f'Callback [ Pre-Operational >>> Operationa], slave pos = {slave_pos}, state = {self._slave_device.state})/{BaseSOEM._state_str(self._slave_device.state)}')

    def slave_setup_func(self, slave_pos):
        print(f'Callback [ Pre-Operational >>> Safe-Operationa], slave pos = {slave_pos}, state = {self._slave_device.state})/{BaseSOEM._state_str(self._slave_device.state)}')

    def __initPDO_map(self):
        """
        Initialize PDO map for the slave device
        :return: None
        """
        if self.__master is None or self.__slave is None:
            raise Exception(f'BaseSOEM not initialized for adapter {self.__adapter_name} or slave index {self.__slave_index}')
        
        
    @staticmethod
    def getState(_state:int) -> str:
        if _state == 0x00:
            return 'EC_STATE_NONE'
        elif _state == 0x01:
            return 'EC_STATE_INIT'
        elif _state == 0x02:                    # pysoem.PREOP_STATE
            return 'EC_STATE_PRE_OP'
        elif _state == 0x03:                    # pysoem.BOOT_STATE
            return 'EC_STATE_BOOT'
        elif _state == 0x04:                    # pysoem.SAFEOP_STATE
            return 'EC_STATE_SAFE_OP'
        elif _state == 0x08:                    # pysoem.OP_STATE
            return 'EC_STATE_OPERATIONAL'
        elif _state == 0x10:
            return 'EC_STATE_ACK'
        else:
            return f'UNKNOWN STATE-{_state}'    

    @staticmethod
    def readSDO_Info(adapter:str) -> list:
        """
        Read SDO info for a given interface name
        :param adapter: Interface name to read SDO info from
        """
        __SDO_Info = list()

        master = pysoem.Master()
        master.open(adapter)
        
        if master.config_init() > 0:
            __SDO_Info.append(f'Adapter {adapter} opened, slaves: {master.slaves}')
            time.sleep(0.1)
            
        
            for slave in master.slaves:
                try:
                    od = slave.od
                except pysoem.SdoInfoError as ex:
                    print(f'SDO info read failed. Adapter = {adapter}, slave= {slave}')
                    exptTrace(ex)
                    __SDO_Info.append(f'ERROR: no SDO info for {slave.name}')
                else:
                    __SDO_Info.append(slave.name)

                    for __ind, obj in enumerate(od):

                        __SDO_Info.append(f' Idx: {hex(obj.index)} ({hex(__ind)}); Code: {obj.object_code}; Type: {obj.data_type}; ')
                        __SDO_Info.append(f' ... BitSize: {obj.bit_length}; Access: {hex(obj.obj_access)}; Name: "{obj.name}" ')
                        if len(obj.entries) == 0:
                            __SDO_Info.append(BaseSOEM.__get_SDO(slave, __ind))
                            continue
                        else:
                            print(f'')
                        for i, entry in enumerate(obj.entries):
                            if entry.data_type > 0 and entry.bit_length > 0:
                                __SDO_Info.append(f'  Subindex {i} ({hex(i)}); Type: {entry.data_type}; BitSize: {entry.bit_length}; ')
                                __SDO_Info.append(f'  ... (Access: {hex(entry.obj_access)} Name: "{entry.name}" ')
                                __SDO_Info.append(BaseSOEM.__get_SDO(slave, __ind, i))

        else:
            __SDO_Info.append('ERROR: no slave available')
            
        master.close()
        return __SDO_Info


    @staticmethod
    def __get_SDO(slave_device, ind:int, sub_ind:int = 0)->str: 
        __sdo = str()
        try:    
            __sdo = __sdo + f' [{ind}, {sub_ind}] ( [{hex(ind)}, {hex(sub_ind)}])'

            _data = None
            _obj = slave_device.od[ind]
            _sdo_index = _obj.index

            __sdo = __sdo + f'=[{hex(_sdo_index)}, {hex(sub_ind)}], size=[{len(_obj.entries)}] ' 


            if len(_obj.entries) > 0:
                _entry = _obj.entries[sub_ind]
            else:
                _entry = None

            _len = _entry.bit_length if _entry is not None else _obj.bit_length
            __sdo = __sdo +f' data len = [{_len}] ' 

            if _len > 64:
                _data = slave_device.sdo_read(_sdo_index, sub_ind).decode('utf-8')
            else:
                _data = int.from_bytes(slave_device.sdo_read(_sdo_index, sub_ind), byteorder='little', signed=True)
        except pysoem.SdoInfoError as ex:
            __sdo =  f'_print_SDO: no SDO info for {slave_device.name}, exception = {ex}'
        except Exception as ex:
            __sdo = f'ERROR: critical SDO exception = {ex}'
        else:
            __sdo = __sdo + f' ** data = {_data}'

        return __sdo
    

    def read_RxPDO_SDO(self, index:int, subindex:int, size:int, unsigned:bool = True)->int:
        master = self.__master
        slave = self.__slave

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


#########################################

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
        print(f' ** data = {_data}')




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
                print(f'read_sdo_info: no SDO info for {slave.name}')
            else:
                print(slave.name)

                for __ind, obj in enumerate(od):

                    print(f' Idx: {hex(obj.index)} ({hex(__ind)}); Code: {obj.object_code}; Type: {obj.data_type}; ', end='')
                    print(f'BitSize: {obj.bit_length}; Access: {hex(obj.obj_access)}; Name: "{obj.name}" ', end = '')
                    if len(obj.entries) == 0:
                        _print_SDO(slave, __ind)
                        continue
                    else:
                        print(f'')
                    for i, entry in enumerate(obj.entries):
                        if entry.data_type > 0 and entry.bit_length > 0:
                            print(f'  Subindex {i} ({hex(i)}); Type: {entry.data_type}; BitSize: {entry.bit_length}; ', end ='')
                            print(f'(Access: {hex(entry.obj_access)} Name: "{entry.name}" ', end ='')
                            _print_SDO(slave, __ind, i)

    else:
        print('no slave available')
        
    master.close()




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


if __name__ == '__main__':
# refer to https://github.com/bnjmnp/pysoem/blob/master/src/pysoem/pysoem.pyx
# example -> https://github.com/bnjmnp/pysoem/blob/master/examples/read_sdo_info.py

    # BaseSOEM.initSOEM()  # Initialize SOEM and get adapters

    if len(sys.argv) == 2:
        info = BaseSOEM.readSDO_Info(sys.argv[1])
        print(info)
        # read_sdo_info(sys.argv[1])
        # sys.exit()

    sys.exit()

    adapters = pysoem.find_adapters()

    master = pysoem.Master()

    for i, adapter in enumerate(adapters):
        

        master.open(adapter.name)

        if master.config_init() > 0:
            _pdo_size = master.config_map()
            _pdo_ovearll = master.config_overlap_map()
            print(f'Adapter {i}, name = {adapter.name}, desc = {adapter.desc}')
            for device in master.slaves:
                
                print(f'Found Device {device.name}, ', end = '')
                _TYPE = int.from_bytes(device.sdo_read(*DEVICE_TYPE_QUERY), byteorder='little', signed=False)
                _NAME = device.sdo_read(*DEVICE_NAME_QUERY).decode('utf-8')
                _SN = int.from_bytes(device.sdo_read(*SN_QUERY), byteorder='little', signed=False)
                _VENDOR = int.from_bytes(device.sdo_read(*VENDOR_ID_QUERY), byteorder='little', signed=False)
                _PRODUCT = int.from_bytes(device.sdo_read(*PRODUCT_CODE_QUERY), byteorder='little', signed=False)
                _POSITION = int.from_bytes(device.sdo_read(*POSITION_ACTUAL_VALUE_QUERY), byteorder='little', signed=False)
                print(f'TYPE = {_TYPE}, NAME= {_NAME}, VENDOR = {_VENDOR}, PRODUCT = {_PRODUCT}, SN = {_SN},', end ='')
                print(f' POSITION = {_POSITION}', end = '')
                print(f'')
        # else:
        #     print('no device found')
            print(f' MAP SIZE = {_pdo_size}, MAP OVERALL = {_pdo_ovearll}')

        master.close()
    # sys.exit()

    print('UNITEST script started')

    if len(sys.argv) != 2:
        print('give ifname as script argument')
        sys.exit()

    # read_sdo_info(sys.argv[1])
    # sys.exit()

    PDO_test(sys.argv[1])
    
    sys.exit()
