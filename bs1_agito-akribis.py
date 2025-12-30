__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["Leonid Voldman","Sasha Schechtman"]
__license__ = "SLA"
__version__ = "2.0.1"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Prototype"


import time, sys
from inputimeout  import inputimeout , TimeoutOccurred
import threading
from queue import Queue 
from collections import namedtuple
from isHex import isHex, isHexUpper, isHexLower
from bs1_base_motor import BaseMotor
from enum import Enum

import socket
import struct

from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, set_parm, get_parm, unsigned_16, str2ip


class AACommRaw:
    def __init__(self):
        self.sock = None
        self.lock = threading.Lock()

    @property
    def is_connected(self):
        return self.sock is not None

    def Connect(self, IP, port):
        with self.lock:
            if self.is_connected:
                return  # Do nothing if already connected

            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((IP, port))

            except Exception as e:
                raise Exception(f"Error connecting to {IP}:{port} - {e}")

    def SendReceive(self, message, timeoutMsec=2000):
        with self.lock:
            if not self.is_connected:
                raise Exception("Socket is not connected")

            try:
                # Prefix 'A', append '\0', and send message
                full_message = 'A' + message + '\0'
                print_DEBUG(f'Sending data: {message}/{full_message}')
                self.sock.sendall(full_message.encode())

                # Set timeout for the socket
                self.sock.settimeout(timeoutMsec / 1000.0)

                # Receive data until '>' is encountered or timeout occurs
                rx = b""
                start_time = time.time()
                print_DEBUG(f'receiving data=', end='')
                while not rx.endswith(b'>'):
                    try:
                        data = self.sock.recv(1024)
                        print_DEBUG(f'{data}', end = '')
                        if not data:
                            raise Exception("Connection closed unexpectedly")
                        rx += data

                    except socket.timeout:
                        if (time.time() - start_time) * 1000 > timeoutMsec:
                            raise Exception("Timeout: No response within the specified time")

                print_DEBUG(f'rx={rx}')
                # Process the response
                return self.ConvertRxToAACommMsg(rx)

            except Exception as e:
                raise Exception(f"Error sending/receiving data - {e}")

    def Disconnect(self):
        with self.lock:
            if not self.is_connected:
                return  # Do nothing if not connected

            try:
                if self.sock:
                    self.sock.close()
                    self.sock = None

            except Exception as e:
                raise Exception(f"Error disconnecting - {e}")

    def ConvertRxToAACommMsg(self, rx):
        # Remove byte [0] that indicated message type (not used in this example)
        rx = rx[1:]
        rxLen = len(rx)

        if rx[rxLen - 1] != ord('>'):
            raise Exception("Rx message does not end with '>'")

        try:
            match rxLen:    # 'OK' reply
                case 1:
                    return "OK"
            
                case 3:     # ERR code
                    return f"ERR {struct.unpack('>h', rx[:2])[0]}"  # Big-endian 16 bit short

                case 5:     # 4 bytes data
                    return str(struct.unpack('>i', rx[:4])[0])      # Big-endian 32 bit int

                case 9:     # 8 bytes data
                    # return str(struct.unpack('>q', rx[:4])[0])      # Big-endian 64 bit long
                    return str(struct.unpack('>q', rx[:8])[0])      # Big-endian 64 bit long

                case _:
                    raise Exception(f"Unexpected length: {rxLen}")

        except Exception as e:
            raise Exception(f"Error processing Rx - {e}")
        

'''
Controller: AGM800
Default IP/port:  172.1.1.101:50000
Axis: A,B,C, ...
Start-up proc:
- phasing
- homing

'''

DEFAULT_SPEED = 10000
DEFAULT_TIMEOUT= 2000

class Agito_Akribis (BaseMotor):
    @staticmethod
    def find_dev(ip, port, id) -> bool:
        try:
            pass
        
        except Exception as ex:
            print_err(f'Error detecting  Agito Akribi device {ip}:{port}/{id}')
            exptTrace(ex)
            return False

        return True


    def __init__ (self, sn, port, d_name, axis, parms):
        super().__init__(port, d_name, parms)

        self.__axis = axis
        self.__m_client = None
        self._mDev_SN = str(sn)
        self._mDev_port = port
        self._devName = d_name
        self._mDev_pos:int = 0
        self.el_current_on_the_fly:int = 0                  # On-the-fly current  -- for compatability only      
        self.__target_position = 0            
        self.__DevOpSPEED = DEFAULT_SPEED
        self.devNotificationQ:Queue = Queue()
        self._title = None
        self.__timeout = DEFAULT_TIMEOUT
        try:
            
            print_log(f'Connecting AA controller at  = {self._mDev_port}')

            _ip_port = str2ip(self._mDev_port)
            self.__m_client = AACommRaw()
            self.__m_client.Connect(_ip_port[0], int(_ip_port[1]))
        
            self.set_parms(parms=parms)




        except Exception as ex:
            if self.__m_client:
                self.__m_client.close()
            self.__m_client = None
            print_log(f'Connection initialization failed. Exception = {ex}')
            exptTrace(ex)
        else:
            self.devName  = self._devName
            print_log(f'Connection to AA controller done at {self._mDev_port}')


    def __del__(self):
        print_log(f'Exiting connection at {self._mDev_port}')
        self._mDev_in_motion = False
        try:
            if self.__m_client:
                self.mDev_stop()
                self.__m_client.SendReceive(f"{self.__axis}MotorOn=0", self.__timeout)     
                self.__m_client.close()

        except  Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'({ self._devName})Connection to AA controller at {self._mDev_port} could not be closed. Exception: {ex} of type: {type(ex)}.')


    def init_dev(self, dev_type) -> bool:
        self._mDev_type = dev_type
        
        if not self.__m_client.is_connected:                      # connection lost
            return False
        
        print_log(f'Initiating ({ self._devName}) device of {self._mDev_type} type on port {self._mDev_port}')
        try:

            self.__m_client.SendReceive(f"{self.__axis}MotorOn=1", self.__timeout)     
            self.__m_client.SendReceive(f"{self.__axis}Speed={self.DevOpSPEED}", self.__timeout)       # init device
              
            
        except  Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'({self._devName}) failed to initialize on port {self._mDev_port}. Exception: {ex} of type: {type(ex)}.')
            return False


        return True
        

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
        print_log(f'Stopping ({self._devName}) device at {self._mDev_port}')
        try:
            self.__m_client.SendReceive(f"{self.__axis}Abort", self.__timeout)     
       # stop device
        except  Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'({self._devName}) failed at {self._mDev_port}. Exception: {ex} of type: {type(ex)}.')
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
        return True

    def go2pos(self, new_position, velocity = None)->bool:
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
        pass

    
        


### Unit tets module ############

if __name__ == "__main__":
    
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
           
