__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["Leonid Voldman","Sasha Schechtman"]
__license__ = "SLA"
__version__ = "2.0.1"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Prototype"


import time, time 
from pymodbus.client import ModbusSerialClient 
from pymodbus.pdu.register_message import ReadHoldingRegistersResponse
# from pymodbus.register_read_message import ReadHoldingRegistersResponse
import serial as serial
import time, sys
from inputimeout  import inputimeout , TimeoutOccurred
import threading
from threading import Lock
from queue import Queue 
from collections import namedtuple
from isHex import isHex, isHexUpper, isHexLower
from bs1_base_motor import BaseMotor
from bs2_DSL_cmd import Command, devCmdCnfg, vType, pType

from enum import Enum

from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, set_parm, get_parm, unsigned_16 


MAX_DH_ANGLE = 65536

# params.yml
#
# DHRB:
#     ROTATION_FORCE: 50              # 20 - 100%
#     GRIPPER_FORCE: 50               # 20 -100 %
#     DEFAULT_ROTATION_SPEED: 50
#     DEFAULT_GRIPPER_SPEED: 50


INITCMD = 0x0100
INIT = {
    'INIT01': {'cmd': INITCMD, 'parms' : 1},
    'INIT02': {'cmd': INITCMD, 'parms' : 2},    
    'INIT_BACK_TO_ZERO': {'cmd': INITCMD, 'parms' : 3},   
    'INIT_CALIBRATE': {'cmd': INITCMD, 'parms' : 4}
}

ID_REG = 0x0A03

SET_GRIPPER_FORCE = 0x0101              # 20-100, percentage (Default force value 100%)
SET_GRIPPER_POSITION = 0x0103           # Set gripper position. 0-1000, thousandths ratio
SET_GRIPPER_SPEED = 0x0104              # gripper setting speed. 1%-100%, percentage
SET_ROTATION_ANGLE_HIGH = 0x0106 
                                        # -160 -  +160
SET_ROTATION_ANGLE_LOW = 0x0105         # -32768 - +32767

                                # Run to absolute rotation angle
                                # The angle value is: low data + high data * 32768 
                                # Note: Do not write to this address without special circumstances, the unit rotates 32768°

SET_ROTATION_SPEED = 0x0107     # 1-100, percentage
SET_ROTATION_FORCE = 0x0108     # 20-100, percentage
SET_REL_ROTATION_ANGLE = 0x0109     # -32768 - +32767, Angle value



CLAMPING_STATUS_QUERY = 0x0201  # 0: in motion, 1: arriving at position; 2: clamping object; 3: object falling
GRIPPER_POSITION_QUERY = 0x0202
ERROR_QUERY = 0x0205            #0: no problem;04 overheating; 08 overload;11 overspeed * 01: undervoltage, 02: overvoltage, 03: overcurrent
ROTATION_ANGLE_QUERY_HIGH = 0x0209
ROTATION_ANGLE_QUERY_LOW = 0x0208
ROTATION_STATUS_QUERY = 0x020B  # 0: in motion, 1: reached position; 2: blocked rotation;

ROTATION_STOP = 0x0502          # Write 1: Spin Stop

RESET_ROTATION_ANGLE = 0x0506   # 01: Reset the rotation angle to within ±360°.
                                # A5: actual rotation to initialized position (0°)and reset rotation angle to 0°



REACH_TARGET_ATTEMPTS = 3

# ModBus RTU parameters for DH Robotics RGIC-35-12
BAUDRATE = 115200
DATABITS = 8
STOPBIT = 1
CHECKSUM = False

gripperOpType = Enum("gripperOpType", ["clamp", "rotate", "none"])

# camResFields = ["res", "dist", "repQ"]
# camRes = namedtuple("camRes",  camResFields, defaults=[None,] * len(camResFields))
# camRes.__annotations__={'res':bool,  'dist':float, 'repQ':Queue}         # specify type of elements


class DH_ROB_RGI_modbus (BaseMotor):
    @staticmethod
    def find_server(MB_port) -> int:
        sn:int = None
        try:
            print_log(f'Looking ModBus server/slave at port = {MB_port}')
            test_client = ModbusSerialClient    \
                    (   port = MB_port,
                        baudrate =BAUDRATE,            # Initiating modbus client connection
                    bytesize  = DATABITS , parity  = 'N', stopbits =STOPBIT)
            print_log (f'test_client = {test_client}')
            
            
            if test_client == None:
                print_log(f"Can't detect ModBus server/slave at port = {MB_port}")
                return None

            _data = test_client.read_holding_registers(address=ID_REG, count=1, slave=1)
            if isinstance(_data, ReadHoldingRegistersResponse):
                sn = _data.registers[0]
                print_log(f"ModBus server/slave for DH RB device was detected at port = {MB_port}, ID={sn}")
                test_client.close()
            else:
                print_log(f"Can't detect ModBus server/slave at port = {MB_port}")
                if test_client:
                    test_client.close()
                return None
        
        except Exception as ex:
            print_err(f'Error detecting ModBus server at port = {MB_port}')
            exptTrace(ex)
            if test_client:
                test_client.close()
            return None

        return sn


    def __init__ (self, sn, port, d_name, parms):
        super().__init__(port, d_name, parms)
        self.ROTATION_FORCE = 100
        self.GRIPPER_FORCE = 100
        self.ROTATION_SPEED = 100
        self.GRIPPER_SPEED = 100
        self.EX_LIMIT = 0
        self.__m_client = None
        self.__operation:gripperOpType = gripperOpType.none
        self._mDev_SN = str(sn)
        self._mDev_port = port
        self._devName = d_name
        self._mDev_pos:int = 0
        self.el_current_on_the_fly:int = 0                  # On-the-fly current  -- for compatability only      
        self.__target_position = 0            
        self.__target_velocity = None

        self.devNotificationQ:Queue = Queue()
        self._title = None
        try:
            self.DevOpSPEED = self.ROTATION_SPEED
            print_log(f'Connecting ModBus server/slave at port = {self._mDev_port}')
            self.__m_client =ModbusSerialClient    \
                    (   port = self._mDev_port,
                        baudrate =BAUDRATE,            # Initiating modbus client connection
                    bytesize  = DATABITS , parity  = 'N', stopbits =STOPBIT)
        
            self.set_parms(parms=parms)

            # self.ROTATION_FORCE = get_parm('DHRB', parms, 'ROTATION_FORCE')
            # self.GRIPPER_FORCE = get_parm('DHRB', parms, 'GRIPPER_FORCE')
            # self.ROTATION_SPEED = get_parm('DHRB', parms, 'DEFAULT_ROTATION_SPEED')
            # self.GRIPPER_SPEED = get_parm('DHRB', parms, 'DEFAULT_GRIPPER_SPEED')
            # self._title = get_parm('DHRB', parms, 'TITLE')



        except Exception as ex:
            if self.__m_client:
                self.__m_client.close()
            self.__m_client = None
            print_log(f'ModBus initialization failed. Exception = {ex}')
            exptTrace(ex)
        else:
            self._mDev_status = True
            self.devName  = self._devName
            print_log(f'ModBus successfuly connected with the server at port={self._mDev_port}')


    def __del__(self):
        print_log(f'Exiting DH Robotics gripper on port {self._mDev_port}')
        self._mDev_in_motion = False
        try:
            if self.__m_client:
                self.mDev_stop()
                self.__m_client.close()
        except  Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'({ self._devName}) DH Rob on port {self._mDev_port} could not be closed. Exception: {ex} of type: {type(ex)}.')


    def init_dev(self, dev_type) -> bool:
        self._mDev_type = dev_type
        if not self._mDev_status:                      # the device is not active
            return False
        print_log(f'Initiating ({ self._devName}) device of {self._mDev_type} type on port {self._mDev_port}')
        try:

            self.__m_client.write_register(address=int(INITCMD), value=int(1), slave=1)       # init device
            self.__m_client.write_register(address=int(SET_ROTATION_FORCE), value=int(self.ROTATION_FORCE), slave=1)       
            self.__m_client.write_register(address=int(SET_GRIPPER_FORCE), value=int(self.GRIPPER_FORCE), slave=1)     
            self.__m_client.write_register(address=int(SET_GRIPPER_SPEED), value=int(self.GRIPPER_SPEED), slave=1)     
            self.__m_client.write_register(address=int(SET_ROTATION_SPEED), value=int(self.ROTATION_SPEED), slave=1)  
              
            
        except  Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'({self._devName}) failed to initialize on port {self._mDev_port}. Exception: {ex} of type: {type(ex)}.')
            return False


        return True
        
    def operateDevice(self, command, **kwards) -> tuple[bool, bool]:
        try:
            _cmd = Command(command)

            if _cmd.device != self._devName:        # device name mismatch
                raise Exception(f'({self._devName}) operateDevice: command device {_cmd.device} mismatch with actual device name {self._devName}.')
            if _cmd.op == 'HO':
                self.mDev_reset_pos(), False        # home operation, reset position to zero
            elif _cmd.op == 'MA':
                return self.go2pos(new_position=_cmd.args.get('position', 0), velocity=_cmd.args.get('velocity', None)), True
                                                    # absolute move to position
            elif _cmd.op == 'MR':
                return self.go2pos(new_position=self._mDev_pos + _cmd.args.get('rel_position', 0), velocity=_cmd.args.get('velocity', None)), True
                                                    # relative move to position
            elif _cmd.op == 'OPEN':
                return self.gripper_on(), True        # open gripper
            elif _cmd.op == 'CLOSE':
                return self.gripper_off(), True       # close gripper
            elif _cmd.op == 'STOP':
                return self.mDev_stop(), False        # stop device
            else:
                raise Exception(f'({self._devName}) operateDevice: unknown command operation {_cmd.op} for device {self._devName}.')    

        except Exception as ex:
            exptTrace(ex)
            print_err(f'({self._devName}) failed to operate command {command} for device on port {self._mDev_port}. Exception: {ex} of type: {type(ex)}.')
            return False, False
        

    def  mDev_watch_dog_thread(self):
        
        print_log (f'>>> Watch dog started on DH ROB port = {self._mDev_port},  dev = {self.devName}, position = {self._mDev_pos}')
        self._mDev_in_motion = True
        _attempts = 0
        while (self._mDev_in_motion):
            time.sleep(0.1)             
            try:
                if self.__operation == gripperOpType.rotate:
                    _statusMsg = self.__m_client.read_holding_registers(address=ROTATION_STATUS_QUERY, count=1, slave=1)
                    if not isinstance(_statusMsg, ReadHoldingRegistersResponse):
                        print_err(f"WARNING - error reading status on port {self._mDev_port} -> {_statusMsg}")
                        continue
                    _status = _statusMsg.registers[0]
                    match _status:
                        case 0:
                            continue
                        case 1:
                            print_log(f'({self._devName}) reached position on port {self._mDev_port}')
                            _current_pos = self.mDev_get_cur_pos()
                            if abs(self.__target_position - _current_pos) > self.EX_LIMIT:
                                if _attempts >= REACH_TARGET_ATTEMPTS:
                                    print_err(f'ERROR: after {REACH_TARGET_ATTEMPTS} attempts the device failed to reach postinion within tolerance={self.EX_LIMIT}')
                                    self._success_flag = False
                                    break
                                else:
                                    _attempts += 1
                                    print_log(f'WARNING: attempt # {_attempts} to reach position {self.__target_position}. actual position = {_current_pos}, tolerance = {self.EX_LIMIT}')
                                    self.go2pos(self.__target_position, round(int(self.__target_velocity)/(2*_attempts)))
                                    continue
                            else:
                                self._success_flag = True
                                break
                        case 2:
                            print_err(f'({self._devName}) Abnormally blocked on port {self._mDev_port}')
                            self._success_flag = False
                            break
                    
                        case _:
                            print_err(f'({self._devName}): Unexpected status {_status} on operation: ({self.__operation})')
                            continue

                elif self.__operation == gripperOpType.clamp:
                    _statusMsg = self.__m_client.read_holding_registers(address=CLAMPING_STATUS_QUERY, count=1, slave=1)
                    if not isinstance(_statusMsg, ReadHoldingRegistersResponse):
                        print_err(f"WARNING - error reading status on port {self._mDev_port} -> {_statusMsg}")
                        continue
                    _status = _statusMsg.registers[0]
                    match _status:
                        case 0:
                            continue
                        case 1:
                            print_log(f'({self._devName}) reached position on port {self._mDev_port}, but a clamped object was not detected')
                            self._success_flag = True
                            break
                        case 2:
                            print_log(f'({self._devName}) reached position on port {self._mDev_port} and keeps clamped object')
                            self._success_flag = True
                            break

                        case 3:
                            print_err(f'({self._devName}) clamped object was abnormally fallen on port {self._mDev_port}')
                            self._success_flag = False
                            break
                    
                        case _:
                            print_err(f'({self._devName}): Unexpected status {_status} on operation: ({self.__operation})')
                            continue
                else:
                    print_err(f'({self._devName}) error operation type ({self.__operation})  on port {self._mDev_port}')
                    self._success_flag = False
                    break
                
            except  Exception as ex:
                e_type, e_filename, e_line_number, e_message = exptTrace(ex)
                print_err(f'({ self._devName}) failed to complete the operation on port {self._mDev_port}. Exception: {ex} of type: {type(ex)}.')
                self._success_flag = False
                break


            

        end_time = time.time()
        print_log (f'>>> Watch dog completed on DH ROB port = {self._mDev_port},  dev = {self.devName}, position = {self._mDev_pos}')
        print_log(f'Start time = {self._start_time}, end time ={end_time}, delta = {end_time - self._start_time}, reach target attempts ={_attempts}')

        if self._mDev_in_motion:
            self.mDev_stop()

        if self._dev_lock.locked():
            self._dev_lock.release()
        else:
            print_err(f'-WARNING unlocket mutual access mutex')

        self.mDev_get_cur_pos()
        print_log(f'Exiting watch dog thread. Device  = {self._devName}, position = {self.mDev_stored_pos()}, success = {self._success_flag}')
        self.devNotificationQ.put(self._success_flag)


    def mDev_stop(self)-> bool:
        print_log(f'Stopping ({self._devName}) device on port {self._mDev_port}')
        try:
            self.__m_client.write_register(address=int(ROTATION_STOP), value=int(1), slave=1)       # stop device
        except  Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'({self._devName}) failed to stop on port {self._mDev_port}. Exception: {ex} of type: {type(ex)}.')
            return False
        # self.__operation = None
        return True

    def gripper_on(self)-> bool:
        if not self.mutualControl():
            return False
        
        self.__operation = gripperOpType.clamp
        print_log(f'Gripper ({self._devName}) will ON  on port {self._mDev_port}')
        try:
            self.__m_client.write_register(address=int(SET_GRIPPER_SPEED), value=int(self.GRIPPER_SPEED), slave=1)      
            self.__m_client.write_register(address=int(SET_GRIPPER_POSITION), value=int(1000), slave=1)      
            if not self.err_validation():
                self.mDev_stop()
                return False
            
        except  Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'({self._devName}) failed to go ON on port {self._mDev_port}. Exception: {ex} of type: {type(ex)}.')
            if self._dev_lock.locked():
                    self._dev_lock.release()
            return False
        
        self.mDev_watch_dog()
        self._gripper_onoff = True
        return True

    def gripper_off(self)-> bool:
        if not self.mutualControl():
            return False
        
        self.__operation = gripperOpType.clamp
        print_log(f'Gripper ({self._devName}) will OFF  on port {self._mDev_port}')
        try:
            self.__m_client.write_register(address=int(SET_GRIPPER_SPEED), value=int(self.GRIPPER_SPEED), slave=1)      
            self.__m_client.write_register(address=int(SET_GRIPPER_POSITION), value=int(0), slave=1)      
            if not self.err_validation():
                self.mDev_stop()
                return False
        except  Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'({self._devName}) failed to go OFF on port {self._mDev_port}. Exception: {ex} of type: {type(ex)}.')
            if self._dev_lock.locked():
                    self._dev_lock.release()
            return False
        
        self.mDev_watch_dog()
        self._gripper_onoff = False
        return True

    def go2pos(self, new_position, velocity = None, stall = None)->bool:
        if not self.mutualControl():
            return False
        
        self.__operation = gripperOpType.rotate
        _rotation_speed = int(self.DevOpSPEED if velocity == None else velocity)
        print_log(f'Gripper ({self._devName}) will go to {new_position} by {_rotation_speed} velocity on port {self._mDev_port}')
        try:
            self.__m_client.write_register(address=int(SET_ROTATION_SPEED), value=_rotation_speed, slave=1)      
            # self.__m_client.write_register(address=int(SET_ROTATION_ANGLE_HIGH), value=int(0), slave=1)      
            # self.__m_client.write_register(address=int(SET_ROTATION_ANGLE_LOW), value=int(new_position), slave=1)      
            print_log(f'Moving to {new_position}/{int(new_position)}/{hex(int(new_position))}/0x{unsigned_16(int(new_position)):04x}/{hex(unsigned_16(int(new_position)))}')

            # if int(new_pos) > (MAX_DH_ANGLE/2):
            #     new_pos = new_pos - MAX_DH_ANGLE
            # print_log(f'Moving to {new_position}/{int(new_position)}/{hex(int(new_position))}/0x{unsigned_16(int(new_position)):04x}/{hex(unsigned_16(int(new_position)))}')

            self.__m_client.write_register(address=int(SET_ROTATION_ANGLE_LOW), value=unsigned_16(int(new_position)), slave=1)  
            if not self.err_validation():
                self.mDev_stop()
                return False    
        except  Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'({self._devName}) failed to go to {new_position} by {velocity} velocity on port {self._mDev_port}. Exception: {ex} of type: {type(ex)}.')
            if self._dev_lock.locked():
                    self._dev_lock.release()
            return False
        self.__target_velocity = velocity
        self.__target_position = int(new_position)
        self.mDev_watch_dog()
        return True
    

    def mDev_stored_pos(self)->int:
        return self._mDev_pos
        

    def  mDev_reset_pos(self)->bool:
        print_log(f'Gripper ({self._devName}) at HOME opeartion on port {self._mDev_port}')
        try:
            # self.__m_client.write_register(address=int(RESET_ROTATION_ANGLE), value=int(0xA5), slave=1)      
            self.__m_client.write_register(address=int(INITCMD), value=int(0x01), slave=1)      
            if not self.err_validation():
                return False
            
        except  Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'({self._devName}) failed to set HOME on port {self._mDev_port}. Exception: {ex} of type: {type(ex)}.')
            return False
        
        return True
    
        
    def mDev_get_cur_pos(self)->int:
        try:
            _data = self.__m_client.read_holding_registers(address=ROTATION_ANGLE_QUERY_LOW, count=1, slave=1)      
            if not isinstance(_data, ReadHoldingRegistersResponse):
                print_err(f"WARNING - error reading status on port {self._mDev_port} -> {_data}")
            else:
                self._mDev_pos = s16(_data.registers[0])
        except  Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'({self._devName}) failed to go to query current position on port {self._mDev_port}. Exception: {ex} of type: {type(ex)}.')
        
        return self._mDev_pos


    def set_parms(self, parms):
        self.ROTATION_FORCE = get_parm('DHRB', parms, 'ROTATION_FORCE')
        self.GRIPPER_FORCE = get_parm('DHRB', parms, 'GRIPPER_FORCE')
        self.ROTATION_SPEED = get_parm('DHRB', parms, 'DEFAULT_ROTATION_SPEED')
        self.GRIPPER_SPEED = get_parm('DHRB', parms, 'DEFAULT_GRIPPER_SPEED')
        self._title = get_parm('DHRB', parms, 'TITLE')
        self.EX_LIMIT = get_parm('DHRB', parms, 'TOLERANCE_LIMIT')
        # self.EX_LIMIT = int(get_parm(self.devName, parms, 'TOLERANCE_LIMIT')) 

        self.DevOpSPEED = self.ROTATION_SPEED


    def err_validation(self) -> bool:
        return True
    
        try:
            _data = self.__m_client.read_holding_registers(address=ERROR_QUERY, count=4, slave=1)      
            if not isinstance(_data, ReadHoldingRegistersResponse):
                raise Exception(f"Error reading ERROR status on port {self._mDev_port} -> {_data}")
            
            _err_code = _data.registers[0]      #   0: no problem; 04 overheating; 08 overload; 11 overspeed * 
                                                #   01: undervoltage, 02: overvoltage, 03: overcurrent

            match int(_err_code): 
                case 0: 
                    print_log(f'DH operation reported SUCCESS')
                    return True
                case 1:
                    print_err(f'({self._devName}) failed to perform oprtation. ERROR = undervoltage / port {self._mDev_port}')
                case 2:
                    print_err(f'({self._devName}) failed to perform oprtation. ERROR = overvoltage / port {self._mDev_port}')
                case 3:
                    print_err(f'({self._devName}) failed to perform oprtation. ERROR = overcurrent / port {self._mDev_port}')
                case 4:
                    print_err(f'({self._devName}) failed to perform oprtation. ERROR = overheating / port {self._mDev_port}')
                case 8:
                    print_err(f'({self._devName}) failed to perform oprtation. ERROR = overload / port {self._mDev_port}')
                case 11:
                    print_err(f'({self._devName}) failed to perform oprtation. ERROR = overspeed / port {self._mDev_port}')
                case _:
                    print_err(f'({self._devName}) failed to perform oprtation. UNDOCUMENTED ERROR / port {self._mDev_port}')
                
            return False


        except  Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'({self._devName}) failed to go to query error status on port {self._mDev_port}. Exception: {ex} of type: {type(ex)}.')
            return False
        


### Unit tets module ############

if __name__ == "__main__":
    import serial.tools.list_ports


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
        DH_ROB_RGI_modbus.find_server(ps.device)

    _cl = None
    try:

        print(f'{len(sys.argv)} -> {sys.argv}')
        if not len(sys.argv) == 2  or  not sys.argv[1].isdecimal():
            print (f"Usage: python {sys.argv[0]} port  (1,2,3..)")
            sys.exit()

        _port = f'COM{sys.argv[1]}'

        # test_client = DH_ROB_RGI_modbus.find_server('COM7')
        
        _cl = ModbusSerialClient    \
                    (   port = _port,
                        baudrate =BAUDRATE,            # Initiating modbus client connection
                    bytesize  = DATABITS , parity  = 'N', stopbits =STOPBIT)
        

        
        if _cl == None:
            print(f' Error initiation = {_cl}')
            sys.exit()
    
    except Exception as ex:
        print(f'Exception operation error on {ex}')   
        sys.exit()

    while(True):
        try:
            val = input("Enter cmd, addr and value  (cmd/adr/val1): ").split("/")


            if len(val) > 3:
                print (f'Invalid input: {val}')
                continue
            
            if not (val[0].upper() == 'R' or val[0].upper() == 'W'):
                print(f'Incorrect cmd in: {val}')
                continue
            
            if not (val[1][0:2] == '0x' and isHex(val[1][2:])):
                print(f'Incorrect adr {val[1]}={val[1][0:1]}/{val[1][2:]}  in: {val}')
                continue

            if (val[0].upper() == 'W') and (not val[2].isnumeric()):
                print(f'Incorrect value in: {val}')
                continue

            _adr = int(val[1],16)
            if val[0].upper() == 'W':
                _val = val[2]

            try:
                if val[0].upper() == 'W':
                    _data = _cl.write_register(address=int(_adr), value=int(_val), slave=1)
                    print(f'DATA = {_data}, bits[] = {_data.bits}, registers[] = {_data.registers}')
                else:
                    
                    _data = _cl.read_holding_registers(address=_adr, count=1, slave=1)
                    print(f'Device response: {_data}, bits[] = {_data.bits}, registers[] = {_data.registers}')

            except Exception as ex:
                print(f'Something goes wrong input. Exception = {ex}')
                continue

        except KeyboardInterrupt as ex:
            print(f'Exiting by ^C \n{ex}')
            _cl.close()
            sys.exit()
        except Exception as ex:
            print(f'Exception operation error on {ex}')   
           
