__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "1.0.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Tool"

from curses.ascii import isdigit
from weakref import finalize
import serial as serial
import sys, os, logging
import time
import threading
import ctypes
from threading import Lock

from inputimeout  import inputimeout , TimeoutOccurred
from queue import Queue 
from typing import List


from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, set_parm, get_parm 


# MEASUREMENT_DELAY:float = 0.25
# MINIMAL_OP_DURATION:float = 0.25

# GRIPPER_TIMEOUT:int = 5

# TROLLEY_PARKING_VELOCITY:int = 25                       #  velocity to set at parking stage

# DEFAULT_ROTATION_TIME:float = 5

# DEAFULT_TROLLEY_RPM:int = 840

# TROLLEY_DEFAULT_CURRENT_LIMIT:int = 1100

# # GRIPPER_DEFAULT_CURRENT_LIMIT:int = 120
# GRIPPER_DEFAULT_CURRENT_LIMIT:int = 120
# SPINNER_DEFAULT_CURRENT_LIMIT:int = 120

# # DEAFULT_VELOCITY_EV_VOLTAGE = 10000
# DEAFULT_VELOCITY_EV_VOLTAGE:int = 5000


# print_log = lambda x: print(x) if __name__ == "__main__" else None
# print_log = lambda x: print(x) if __name__ == "__main__" else print(x)

# format = "%(asctime)s: %(filename)s--%(funcName)s/%(lineno)d -- %(thread)d [%(threadName)s] %(message)s" 
# logging.basicConfig(format=format, level=logging.DEBUG, datefmt="%H:%M:%S")
# print_log = logging.debug

'''
# 1024 SR12 config:

RM 23700	
KN 1078	
ENCRES 4096	
LCC 180	
LPC 540	
KOEFFI 6581	
SP 840	
AC 32000	
DEC 16614	
SR 1	
POR 1	
I 20	
PP 64	
PD 1	
CI 255	

'''



TROLLEY_INIT_CMD = [
'RM 1630',
'KN 253',
'ENCRES 4096',
'LCC 2300',
'LPC 6900',
'KOEFFI 2386',
'SP 840',                           # init RPM = 840
# 'AC 8087',
'AC 20',
# 'DEC 2426',
'DEC 15',
'SR 1',
'POR 4',
'I 17',
'PP 64',
'PD 2',
'CI 18',
'CONTMOD'
# 'EN'
]


GRIPPER_INIT_CMD = [
'RM 23700',
'KN 1078',
'ENCRES 4096',
'LCC 180',
'LPC 540',
'KOEFFI 6581',  
'SP 840',                           # init RPM = 840
# 'AC 2000',
# 'DEC 16614',
'AC 20',
'DEC 15',
'SR 1',
'POR 1',
'I 20',
'PP 64',
'PD 1',
'CI 255',
'VOLTMOD', 
'EN'
]

# GRIPPER_INIT_CMD = [
MAXON535700 = [
'RM 23700',                   # motor resistance RM according to specification in data sheet (!)

'KN 1340',                      # speed constant Kn in accordance with information in the data

'ENCRES 4096',                # resolution of external encoder (4 times lines/mm) (!)

'LCC 113',                      # continuous current (mA) (!)
'LPC 166',                      # peak current (mA) (!)

'KOEFFI 6581',  

'SP 20000',                           # maximum speed. Setting applies to all modes (mm/s)

'AC 20',                               # acceleration value (mm/s²) (!)
'DEC 15',                               # deacceleration value (mm/s²) (!)

'SR 1',                                 # sampling rate of the velocity controller as a multiplier of 200 μs
'POR 1',                                # velocity controller amplification
'I 20',                                 # velocity controller integral term
'PP 64',                                # position controller amplification
'PD 1',                                 # position controller D-term
'CI 255',                               # integral term for current controller
'VOLTMOD', 
'EN'
]



# voltmod   /   contmod
# u 5000  /   v 600
# 100      /    400
# GRC -> 0  / 10

#======================================

fh_baudrate = 9600
fh_timeout = 2
fh_break_duration = 0.25


EX_LIMIT = 100

def FH_cmd(ser, i_cmd, lock = None):
    try:
        if lock:
            lock.acquire()
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        i_cmd += "\r\n"
        res = ser.write(i_cmd.encode('utf-8'))
        answ = ser.readline()
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        if lock:
            lock.release()
    except Exception as ex:
        e_type, e_filename, e_line_number, e_message = exptTrace(ex)
        print_log(f'FAULHABER command on port {ser.port} failed on cmd = {i_cmd}. Exception: {ex} of type: {type(ex)}.')
        raise ex
        # return -1, ''
    else:
        return res, answ


class FH_Motor: 
    def __init__(self, c_port, c_baudrate, c_timeout, parms, devName, cur_pos = 0, fh_break_duration = 0.25):
#############################         parameters per device
        self.MEASUREMENT_DELAY:float = 0.25
        self.MINIMAL_OP_DURATION:float = 0.25
        self.GRIPPER_TIMEOUT:int = 5
        self.TROLLEY_PARKING_VELOCITY:int = 25                       #  velocity to set at parking stage
        self.DEFAULT_ROTATION_TIME:float = 5
        self.DEAFULT_TROLLEY_RPM:int = 840
        self.TROLLEY_DEFAULT_CURRENT_LIMIT:int = 1100
        self.GRIPPER_DEFAULT_CURRENT_LIMIT:int = 120
        self.SPINNER_DEFAULT_CURRENT_LIMIT:int = 120
        self.DEFAULT_CURRENT_LIMIT:int = 300
        self.DEAFULT_VELOCITY_EV_VOLTAGE:int = 5000
        # self.DevMaxSPEED:int = 840
        # self.DevOpSPEED:int = 640
# for compitability
        self.DevMaxSPEED:int = 15000
        self.DevOpSPEED:int = 640
#################################
        self.fh_port = c_port                               # COM port
        self.mDev_baudrate = c_baudrate                       # baudrate
        self.mDev_timeout = c_timeout                         # COM timeout
        self.mDev_ser = None                                  # serial port identificator
        self.mDev_type = None                                 # devise type (trolley / gripper)
        self.mDev_pos = cur_pos                               # POS command - current position 
        self.el_current_limit = None                        # electrical current limit to stop 
        self.el_current_on_the_fly = None                  # On-the-fly current                    
        self.wd = None                                      # watch dog identificator
        self.mDev_SN = None                                   # GSER cmd - serial N
        self.mDev_status = False                              # device status (bool) / used for succesful initiation validation
        self.mDev_in_motion = False                           # is device in motion
        self.possition_control_mode = False                 # TRUE - control possition, FALSE - don't
        self.time_control_mode = False                      # TRUE - time control
        self.new_pos = 0
        # self.mDev_cur_pos = cur_pos
        self.mDev_pressed = False                             # is motion button pressed
        self.fh_lock = Lock()                               # COM port access mutex 
        self.gripper_onof = True                            # True means off )))
#------- Bad practice --------
        self.el_voltage = self.DEAFULT_VELOCITY_EV_VOLTAGE
        self.rpm = self.DEAFULT_TROLLEY_RPM 
#------- Bad practice --------
        self.start_time = None                              # Start thread time
        self.success_flag = True                            # end of op flag
        self.__stall_flag = False                           # stall after motion flag
        self.rotationTime:float = 0                               # rotation time
        self.devName = devName
        self.__title = None
        self.dev_lock = Lock()
        self.devNotificationQ = Queue()



        try:
            self.mDev_ser = serial.Serial(port = self.fh_port, baudrate = self.mDev_baudrate, timeout = self.mDev_timeout)
            self.mDev_name = self.mDev_ser.name
            print_log (f"name = {self.mDev_name}")
            self.mDev_ser.send_break(duration = fh_break_duration)                  
            answ = self.mDev_ser.readline()
            print_log(f'({ self.devName}) Serial port init returns - {answ}')
            if not answ.__str__().__contains__("FAULHABER"):
                print_err(f'FAULHABBER on port {self.fh_port} is NOT ACTIVE')
                return                                  # no valid FAULHABBER motor can be added

            res, answ = FH_cmd(self.mDev_ser, 'en', self.fh_lock)

            # res, answ = FH_cmd(self.mDev_ser, 'GRC', self.fh_lock)
            # current_A = int(answ)
            # print_log(f'Electric current read result = {answ}/{current_A}')
           
            res, answ = FH_cmd(self.mDev_ser, 'GSER', self.fh_lock)
            serN = int(answ)
            # self.mDev_ser.mDev_SN = serN
            self.mDev_SN = serN
            print_log(f'Serial number = {answ}/{serN}')

            res, answ = FH_cmd(self.mDev_ser, 'POS', self.fh_lock)

            self.mDev_pos = int(answ)
            print_log(f'Current actual position = {answ}/{self.mDev_pos}')

            res, answ = FH_cmd(self.mDev_ser, 'di', self.fh_lock)

#             self.MEASUREMENT_DELAY = parms[devName]['MEASUREMENT_DELAY'] if ((devName in parms.keys()) and ('MEASUREMENT_DELAY' in parms[devName].keys())) else parms['DEAFULT']['MEASUREMENT_DELAY']
#             self.MINIMAL_OP_DURATION = parms[devName]['MINIMAL_OP_DURATION'] if ((devName in parms.keys()) and ('MINIMAL_OP_DURATION' in parms[devName].keys())) else parms['DEAFULT']['MINIMAL_OP_DURATION']
#             self.GRIPPER_TIMEOUT  = parms[devName]['GRIPPER_TIMEOUT'] if ((devName in parms.keys()) and ('GRIPPER_TIMEOUT' in parms[devName].keys())) else parms['DEAFULT']['GRIPPER_TIMEOUT']
#             self.TROLLEY_PARKING_VELOCITY = parms[devName]['TROLLEY_PARKING_VELOCITY'] if ((devName in parms.keys()) and ('TROLLEY_PARKING_VELOCITY' in parms[devName].keys())) else parms['DEAFULT']['TROLLEY_PARKING_VELOCITY']
#                                                                                                 #  velocity to set at parking stage
#             self.DEFAULT_ROTATION_TIME = parms[devName]['DEFAULT_ROTATION_TIME'] if ((devName in parms.keys()) and ('DEFAULT_ROTATION_TIME' in parms[devName].keys())) else parms['DEAFULT']['DEFAULT_ROTATION_TIME']
#             self.DEAFULT_VELOCITY_EV_VOLTAGE = parms[devName]['DEAFULT_VELOCITY_EV_VOLTAGE'] if ((devName in parms.keys()) and ('DEAFULT_VELOCITY_EV_VOLTAGE' in parms[devName].keys())) else parms['DEAFULT']['DEAFULT_VELOCITY_EV_VOLTAGE']
#             self.DEAFULT_TROLLEY_RPM = parms[devName]['DEAFULT_TROLLEY_RPM'] if ((devName in parms.keys()) and ('DEAFULT_TROLLEY_RPM' in parms[devName].keys())) else parms['DEAFULT']['DEAFULT_TROLLEY_RPM']
#             self.TROLLEY_DEFAULT_CURRENT_LIMIT = parms[devName]['TROLLEY_DEFAULT_CURRENT_LIMIT'] if ((devName in parms.keys()) and ('TROLLEY_DEFAULT_CURRENT_LIMIT' in parms[devName].keys())) else parms['DEAFULT']['TROLLEY_DEFAULT_CURRENT_LIMIT']
#             self.GRIPPER_DEFAULT_CURRENT_LIMIT = parms[devName]['GRIPPER_DEFAULT_CURRENT_LIMIT'] if ((devName in parms.keys()) and ('GRIPPER_DEFAULT_CURRENT_LIMIT' in parms[devName].keys())) else parms['DEAFULT']['GRIPPER_DEFAULT_CURRENT_LIMIT']
#             self.SPINNER_DEFAULT_CURRENT_LIMIT = parms[devName]['SPINNER_DEFAULT_CURRENT_LIMIT'] if ((devName in parms.keys()) and ('SPINNER_DEFAULT_CURRENT_LIMIT' in parms[devName].keys())) else parms['DEAFULT']['SPINNER_DEFAULT_CURRENT_LIMIT']
# # for compitability
#             self.DevMaxSPEED = parms[devName]['DevMaxSPEED'] if ((devName in parms.keys()) and ('DevMaxSPEED' in parms[devName].keys())) else parms['DEAFULT']['DevMaxSPEED']
#             self.DevOpSPEED  = parms[devName]['DevOpSPEED'] if ((devName in parms.keys()) and ('DevOpSPEED' in parms[devName].keys())) else parms['DEAFULT']['DevOpSPEED']
            
#             self.__title = get_parm(devName, parms, 'TITLE')


#             self.el_voltage =  self.DEAFULT_VELOCITY_EV_VOLTAGE
#             self.DevOpSPEED = self.DEAFULT_TROLLEY_RPM
#             self.rpm = self.DEAFULT_TROLLEY_RPM 


            self.set_parms(parms=parms)
            
            # res, answ = FH_cmd(self.mDev_ser, 'GU', self.fh_lock)
            # print_log(f'PWM value = {answ}')

        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f"Connection to port {self.fh_port} was lost")
            print_err(f"Unexpected Exception: {ex}")
            return                                  # no valid FAULHABBER motor can be added
        
        else:
            self.mDev_status = True
            pass

        finally:
            pass

        

    def __del__(self):
        print_log(f'Exiting FAULHABER on port {self.fh_port}')
        self.mDev_in_motion = False
        try:
            if not self.mDev_ser == None:
                _cmd = 'di'                             # disable motor
                res, answ = FH_cmd(self.mDev_ser, _cmd, self.fh_lock)
                print_log(f'({ self.devName}) FAULHABER disabled on port {self.fh_port}.')
                # self.mDev_ser.__del__()
                # del self.mDev_ser
                self.mDev_ser.close()

        except  Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'({ self.devName}) FAULHABER on port {self.fh_port} could not be closed. Exception: {ex} of type: {type(ex)}.')
        finally:
            pass



    @staticmethod
    def recognizeDev(ps:serial.tools.list_ports.ListPortInfo, dev_name:str = None )->str:
        _res:str = None
        _ser = None
        serN = None
        try:
            print_log(f'Looking for FAULHABER {dev_name} at port {ps.device} ')
            _ser = serial.Serial(port=ps.device, baudrate = fh_baudrate, timeout = fh_timeout)
            name = _ser.name
            print_log (f"name = {name}")
            _ser.send_break(duration = fh_break_duration)                   
            answ = _ser.readline()
            print_log(f'Serial port init returns - {answ}')
            if not answ.__str__().__contains__("FAULHABER"):
                raise Exception(f'NO ACTIVE FAULHABBER found on port {ps.device}')
                                                    # no valid FAULHABBER motor can be added

            res, answ = FH_cmd(_ser, 'en')           # enabling motor
            res, answ = FH_cmd(_ser, 'GRC')         # reading current
            current_A = int(answ)
            res, answ = FH_cmd(_ser, 'GSER')     # reading serial N
            serN = int(answ)                        
            res, answ = FH_cmd(_ser, 'POS')      # reading position
            start_pos = int (answ)
            res, _gu = FH_cmd(_ser, 'GU')          # reading PWM value
            print_log(f'FAULHABER found on port {ps.device} with S/N = {serN} at position = {start_pos} with Electric current/GRC = {current_A} and PWM value = {_gu}')

            res, answ = FH_cmd(_ser, 'di')       # disabling motor

        except Exception as ex:
            print_log(f"Error recognizing device at port {ps.device}: {ex}")
            exptTrace(ex)


        if _ser is not None:
            _ser.close
            _ser.__del__()
            
        return str(serN)


    def init_dev(self, fh_type) -> bool:
        self.mDev_type = fh_type
        
        if not self.mDev_status:                      # the device is not active
            return False

        print_log(f'Initiating ({ self.devName}) devise of {self.mDev_type} type on port {self.fh_port}')
        try:
            if self.mDev_type == '--TROLLEY--':
                self.el_current_limit = self.TROLLEY_DEFAULT_CURRENT_LIMIT
                for _cmd in TROLLEY_INIT_CMD:
                    res, answ = FH_cmd(self.mDev_ser, _cmd, self.fh_lock)
                    print_log (f"Port= {self.fh_port},  init command =  {_cmd.encode('utf-8')} with res=  {res} and reply = {answ}" )
            elif self.mDev_type == '--GRIPPER--':
                self.el_current_limit = self.GRIPPER_DEFAULT_CURRENT_LIMIT
                for _cmd in GRIPPER_INIT_CMD:
                    res, answ = FH_cmd(self.mDev_ser, _cmd, self.fh_lock)
                    print_log (f"Port= {self.fh_port},  init command =  {_cmd.encode('utf-8')} with res=  {res} and reply = {answ}" )

            elif self.mDev_type == '--TIME_ROTATOR--':
                self.rotationTime = self.DEFAULT_ROTATION_TIME
                self.el_current_limit = self.SPINNER_DEFAULT_CURRENT_LIMIT
                for _cmd in GRIPPER_INIT_CMD:
                    res, answ = FH_cmd(self.mDev_ser, _cmd, self.fh_lock)
                    print_log (f"Port= {self.fh_port},  init command =  {_cmd.encode('utf-8')} with res=  {res} and reply = {answ}" )


        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_log(f'FAULHABER initiation on port {self.fh_port} failed on cmd = {_cmd}. Exception: {ex} of type: {type(ex)}.')
            return False

        return True

    def  mDev_watch_dog_thread(self):
        print_log (f'>>> Watch dog started on port = {self.mDev_ser.port}/{self.fh_port},  dev = {self.devName}, position = {self.mDev_pos}')
        time.sleep(self.MEASUREMENT_DELAY)    
        self.success_flag = True             # waif for a half of sec

        self.mDev_in_motion = True
        max_GRC = 0
        while (self.mDev_in_motion):
            try:
                fh_cmd = 'GRC'
                res, answ = FH_cmd(self.mDev_ser, fh_cmd, self.fh_lock)

                # print_log (f"Port= {self.fh_port},  GRC command =  {fh_cmd.encode('utf-8')} with res=  {res} and reply = {answ}" )
                _cur_answ = answ.decode('utf-8').strip()
                # if isinstance(_cur_answ, int):
                if _cur_answ.isdigit():
                    self.el_current_on_the_fly = int(_cur_answ)
                
                    max_GRC = self.el_current_on_the_fly if self.el_current_on_the_fly > max_GRC else max_GRC
                    if (self.el_current_on_the_fly > int(self.el_current_limit)):
                        print_log(f'GRC = {self.el_current_on_the_fly}, Limit = {self.el_current_limit}')
                        if (self.mDev_type == '--TROLLEY--' ) and self.possition_control_mode:
                                _pos = self.mDev_get_cur_pos()
                                if abs(_pos - self.new_pos) > EX_LIMIT:
                                    print_log(f'Desired position [{self.new_pos}] is not reached. Current position = {_pos}')
                                    self.success_flag = False
                        break
                else:
                    _pos = self.mDev_get_cur_pos()
                    print_err(f'FAULHABER ERROR - can not read current value //  {answ} // {_cur_answ} // pos = {_pos} **')
                    self.mDev_stop()
                    break
                

                if self.mDev_type == '--GRIPPER--':
                    end_time = time.time()
                    if end_time - self.start_time > self.GRIPPER_TIMEOUT:
                        print_log(f'GRIPPER operation canceled by timeout, port = {self.fh_port}, current GRC = {int(answ)}, Limit = {self.el_current_limit}, max GRC = {max_GRC} ')
                        self.mDev_ser.send_break(duration = 0.25)                  
                        answ = self.mDev_ser.readline()
                        print_log(f'Serial port init break returns - {answ}')
                        break

                if self.mDev_type == '--TIME_ROTATOR--':
                    end_time = time.time()
                    if end_time - self.start_time > self.rotationTime:
                        print_log(f' WatchDog FAULHABER: TIME ROTATOR operation completed, port = {self.fh_port}, current GRC = {int(answ)}, Limit = {self.el_current_limit}, max GRC = {max_GRC} ')
                        break


                if self.mDev_type == '--TROLLEY--' and self.possition_control_mode :
                    fh_cmd = 'OST'
                    res, answ = FH_cmd(self.mDev_ser, fh_cmd, self.fh_lock)
                    fh_op_status = int (answ)
                    # print_log(f'Operational status on FH  port {self.fh_port} = {bin(fh_op_status)}/{fh_op_status}')
                    if fh_op_status & 0b00010000000000000000:        # Position attained - 000 1000 0000 0000 00000 = 65536
                        print_log(f'Operational status on FH  port {self.fh_port} = {bin(fh_op_status)}/{fh_op_status}')
                        break

            except Exception as ex:
                e_type, e_filename, e_line_number, e_message = exptTrace(ex)
                print_log(f'WatchDog FAULHABER failed on port = {self.mDev_ser.port}/{self.fh_port}. Exception: {ex} of type: {type(ex)}.')
                self.success_flag = False
                break
            finally:
                pass

            time.sleep(0.1)             #  end of loop

            
        end_time = time.time()
        print_log (f'>>> Watch dog completed on port = {self.mDev_ser.port}/{self.fh_port},  dev = {self.devName}, position = {self.mDev_pos}')
        print_log(f'Start time = {self.start_time}, end time ={end_time}, delta = {end_time - self.start_time}')
        if end_time - self.start_time - self.MEASUREMENT_DELAY < self.MINIMAL_OP_DURATION:
            print_log(f'Abnormal FAULHABER termination on port = {self.mDev_ser.port}')
            self.success_flag = False

    
        if self.__stall_flag:
            self.mDev_stall()
        elif self.mDev_in_motion:
            self.mDev_stop()


        
        if self.dev_lock.locked():
            self.dev_lock.release()
        else:
            print_err(f'-WARNING unlocket mutual access mutex')

        self.mDev_get_cur_pos()
        self.devNotificationQ.put(self.success_flag)


        return
    

    def  mDev_watch_dog(self):
        self.start_time = time.time()
        self.wd = threading.Thread(target=self.mDev_watch_dog_thread)
        self.wd.start()
        return self.wd
    
    def mDev_stall(self):

        cmd_seq:List[str] = list()
        res = None
        answ = None
        try:

            if self.mDev_type == '--TROLLEY--':
                cmd_seq.append(f'EN')                          # re-enabling motor (only trolley gets disabled 
                cmd_seq.append(f'SP 0')                #  SP - Load Maximum Speed
                cmd_seq.append(f'v 0')
            elif self.mDev_type == '--GRIPPER--' or self.mDev_type == '--TIME_ROTATOR--':
                cmd_seq.append("u 0")
            else:
                raise Exception (f'FAULHABER ilegal type on stalling op')

            
            self.mDev_in_motion = False
        # try:
            for fh_cmd in cmd_seq:
                res, answ = FH_cmd(self.mDev_ser, fh_cmd, self.fh_lock)
                print_log (f"Port= {self.fh_port},  forward command =  {fh_cmd.encode('utf-8')} with res=  {res} and reply = {answ}" )
                
            res, answ = FH_cmd(self.mDev_ser, fh_cmd, self.fh_lock)

        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_log(f'FAULHABER stalling failed on port = {self.mDev_ser.port}/{self.fh_port}. Exception: {ex} of type: {type(ex)}.')
            self.__stall_flag = False
            return res, answ
        else:
            print_log (f"FAULHABER stalled on port = {self.mDev_ser.port}/{self.fh_port} with res=  {res} and reply = {answ}" )
        
        self.__stall_flag = False
        return res, answ

    def  mDev_stop(self) -> bool:

        if self.mDev_type == '--TROLLEY--':
            fh_cmd = "v 0"   
        elif self.mDev_type == '--GRIPPER--' or self.mDev_type == '--TIME_ROTATOR--':
            fh_cmd = "u 0"
        else:
            print_log(f'FAULHABER ilegal type on stoping')
            return False
        
        self.mDev_in_motion = False
        try:

            if self.mDev_type == '--TROLLEY--':

                if self.__stall_flag:
                    res, answ = self.mDev_stall()
                    print_log (f"FH motor STALLED on port= {self.fh_port}  with res=  {res} and reply = {answ}" )
                else:
                    en_cmd = f'DI'                          # disabling motor
                    res, answ = FH_cmd(self.mDev_ser, en_cmd, self.fh_lock)
                    print_log (f"FH motor on port= {self.fh_port},  disabled =  {en_cmd.encode('utf-8')} with res=  {res} and reply = {answ}" )

            else:
                res, answ = FH_cmd(self.mDev_ser, fh_cmd, self.fh_lock)
        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_log(f'FAULHABER stoping failed on port = {self.mDev_ser.port}/{self.fh_port}. Exception: {ex} of type: {type(ex)}.')
            return False
        else:
            print_log (f"FAULHABER stoped on port = {self.mDev_ser.port}/{self.fh_port} with res=  {res} and reply = {answ}" )

        self.mDev_get_cur_pos()
        return True

    def gripper_on(self)->bool:
        if not self.mutualControl():
            return False
        
        self.success_flag = True
        try:

          
            fh_cmd = f'u {self.el_voltage}'
            res, answ = FH_cmd(self.mDev_ser, fh_cmd, self.fh_lock)
            print_log (f"Port= {self.fh_port},  command =  {fh_cmd.encode('utf-8')} with res=  {res} and reply = {answ}" )
        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_log(f'FAULHABER gripper on failed on port = {self.fh_port}. Exception: {ex} of type: {type(ex)}.')
            self.success_flag = False
            if self.dev_lock.locked():
                self.dev_lock.release()
            return  False
            
        self.gripper_onof = True    
        self.mDev_watch_dog()
        return True

    def gripper_off(self)->bool:
        if not self.mutualControl():
            return False
        
        self.success_flag = True
        try:

         
            vlt = int(self.el_voltage) * (-1)
            fh_cmd = f'u {vlt}'
            # fh_cmd = f'u -{self.el_voltage}'
            res, answ = FH_cmd(self.mDev_ser, fh_cmd, self.fh_lock)
            print_log (f"Port= {self.fh_port},  command =  {fh_cmd.encode('utf-8')} with res=  {res} and reply = {answ}" )
        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_log(f'FAULHABER gripper on failed on port = {self.fh_port}. Exception: {ex} of type: {type(ex)}.')
            self.success_flag = False
            if self.dev_lock.locked():
                self.dev_lock.release()
            return False

        self.gripper_onof = False    
        self.mDev_watch_dog()
        return True

    def go2pos(self, new_position, velocity=None, stall=None)->bool:
        if not self.mutualControl():
            return False
        
        self.new_pos = new_position
        
        if velocity == None:
            velocity = self.rpm

        self.success_flag = True
        self.possition_control_mode = True
        self.time_control_mode = False 
        
        self.__stall_flag = False
        if stall is not None and stall:
            self.__stall_flag = True
            

        print_log(f'go2pos, pos={new_position}, velocity = {velocity}, stall = {stall}')

        try:


            fh_cmd = f'EN'                          # re-enabling motor
            res, answ = FH_cmd(self.mDev_ser, fh_cmd, self.fh_lock)
            print_log (f"FH Motor on port= {self.fh_port},  reenabled =  {fh_cmd.encode('utf-8')} with res=  {res} and reply = {answ}" )



            fh_cmd = f'SP {velocity}'
            res, answ = FH_cmd(self.mDev_ser, fh_cmd, self.fh_lock)
            print_log (f"Port= {self.fh_port},  load RPM command =  {fh_cmd.encode('utf-8')} with res=  {res} and reply = {answ}" )

            self.mDev_pos = new_position
            fh_cmd = f'LA {new_position}'
            res, answ = FH_cmd(self.mDev_ser, fh_cmd, self.fh_lock)
            print_log (f"Port= {self.fh_port},  load position command =  {fh_cmd.encode('utf-8')} with res=  {res} and reply = {answ}" )
            fh_cmd = f'M'
            res, answ = FH_cmd(self.mDev_ser, fh_cmd, self.fh_lock)
            print_log (f"Port= {self.fh_port},  motion command =  {fh_cmd.encode('utf-8')} with res=  {res} and reply = {answ}" )
        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_log(f'FAULHABER gripper on failed on port = {self.fh_port}. Exception: {ex} of type: {type(ex)}.')
            self.success_flag = False
            if self.dev_lock.locked():
                self.dev_lock.release()
            self.__stall_flag = False
            return False
            
        self.mDev_watch_dog()  
        return True

    
    def timeRotaterFw(self, time, velocity = None)->bool:
        if not self.mutualControl():
            return False
        
        if velocity == None:
            velocity = self.el_voltage

        self.success_flag = True
        self.possition_control_mode = False
        self.time_control_mode = True 
        self.rotationTime = time
        print_inf(f'TIME ROTATOR is going forward on port = {self.fh_port} for {time} seconds')

        
        try:

         
            vlt = int(velocity) 
            fh_cmd = f'u {vlt}'
            res, answ = FH_cmd(self.mDev_ser, fh_cmd, self.fh_lock)
            print_log (f"Port= {self.fh_port},  command =  {fh_cmd.encode('utf-8')} with res=  {res} and reply = {answ}" )
                                                                                          
        except Exception as ex:
                e_type, e_filename, e_line_number, e_message = exptTrace(ex)
                print_err(f'FAULHABER TIME ROTATOR  failed on port = {self.fh_port}. Exception: {ex} of type: {type(ex)}.')
                self.success_flag = False
                if self.dev_lock.locked():
                    self.dev_lock.release()
                return False
        else:
            print_inf (f"FAULHABER TIME ROTATOR FW started on port = {self.fh_port}" )
        
        self.mDev_watch_dog()
        return True
        

    def timeRotaterBw(self, time, velocity = None)-> bool:
        if not self.mutualControl():
            return False

        if velocity == None:
            velocity = self.el_voltage

        self.success_flag = True
        self.possition_control_mode = False
        self.time_control_mode = True 
        self.rotationTime = time
        print_inf(f'IME ROTATOR is going backward on port = {self.fh_port}')

      
        try:

         
            vlt = int(velocity) * (-1)
            fh_cmd = f'u {vlt}'
            res, answ = FH_cmd(self.mDev_ser, fh_cmd, self.fh_lock)
            print_log (f"Port= {self.fh_port},  command =  {fh_cmd.encode('utf-8')} with res=  {res} and reply = {answ}" )
                                                                                          
        except Exception as ex:
                e_type, e_filename, e_line_number, e_message = exptTrace(ex)
                print_err(f'FAULHABER TIME ROTATOR  failed on port = {self.fh_port}. Exception: {ex} of type: {type(ex)}.')
                self.success_flag = False
                if self.dev_lock.locked():
                    self.dev_lock.release()
                return False
        else:
            print_inf (f"FAULHABER TIME ROTATOR BW started on port = {self.fh_port}" )
 

        self.mDev_watch_dog()
        return True
       
    def  mDev_forward(self, velocity = None, timeout=None, polarity = None, stall = None)->bool:
        if not self.mutualControl():
            return False
        
        self.success_flag = True
        self.possition_control_mode = False
        self.time_control_mode = False 

        self.__stall_flag = False
        if stall is not None and stall:
            self.__stall_flag = True

        
        print_log(f'Going forward on port = {self.mDev_ser.port}/{self.fh_port}')

        cmd_seq = []
        
        try:


            if self.mDev_type == '--TROLLEY--':
                cmd_seq.append(f'EN')                          # re-enabling motor (only trolley gets disabled 
                rpm = int(self.rpm) * (-1)
                cmd_seq.append(f'SP {abs(rpm)}')                #  SP - Load Maximum Speed
                cmd_seq.append(f'v {rpm}')

            elif self.mDev_type == '--GRIPPER--' or self.mDev_type == '--TIME_ROTATOR--':
                cmd_seq.append(f'u {self.el_voltage}')
                fh_cmd = f'u {self.el_voltage}'

            else:
                print_log(f'FAULHABER ilegal type on forward')
                return
            for fh_cmd in cmd_seq:
                res, answ = FH_cmd(self.mDev_ser, fh_cmd, self.fh_lock)
                print_log (f"Port= {self.fh_port},  forward command =  {fh_cmd.encode('utf-8')} with res=  {res} and reply = {answ}" )
        except Exception as ex:
                print_log(f'FAULHABER forward failed on port = {self.mDev_ser.port}/{self.fh_port}. Exception: {ex} of type: {type(ex)}.')
                self.success_flag = False
                if self.dev_lock.locked():
                    self.dev_lock.release()
                return False
        else:
            print_log (f"FAULHABER forward on port = {self.mDev_ser.port}/{self.fh_port} with res=  {res} and reply = {answ}" )
        
        self.mDev_watch_dog()
        return True


    def  mDev_backwrd(self, velocity = None, timeout=None, polarity = None, stall = None)->bool:
        if not self.mutualControl():
            return False
        
        self.success_flag = True
        self.possition_control_mode = False
        self.time_control_mode = False

        self.__stall_flag = False
        if stall is not None and stall:
            self.__stall_flag = True




        print_log(f'Going forward on port = {self.mDev_ser.port}/{self.fh_port}')

        cmd_seq = []
        
        try:


            if self.mDev_type == '--TROLLEY--':
                cmd_seq.append(f'EN')                          # re-enabling motor (only trolley gets disabled 
                rpm = int(self.rpm) 
                cmd_seq.append(f'SP {abs(rpm)}')
                cmd_seq.append(f'v {rpm}')

            elif self.mDev_type == '--GRIPPER--' or  self.mDev_type == '--TIME_ROTATOR---':
                cmd_seq.append(f'u {self.el_voltage}')
                fh_cmd = f'u {self.el_voltage * (-1)}'

            else:
                print_log(f'FAULHABER ilegal type on forward')
                return
            for fh_cmd in cmd_seq:
                res, answ = FH_cmd(self.mDev_ser, fh_cmd, self.fh_lock)
                print_log (f"Port= {self.fh_port},  backward command =  {fh_cmd.encode('utf-8')} with res=  {res} and reply = {answ}" )
        except Exception as ex:
                print_log(f'FAULHABER backward failed on port = {self.mDev_ser.port}/{self.fh_port}. Exception: {ex} of type: {type(ex)}.')
                self.success_flag = False
                if self.dev_lock.locked():
                    self.dev_lock.release()
                return False
        else:
            print_log (f"FAULHABER backward on port = {self.mDev_ser.port}/{self.fh_port} with res=  {res} and reply = {answ}" )
 

        self.mDev_watch_dog()
        return True

   
    def mDev_stored_pos(self): 
        return self.mDev_pos
   
    def mDev_get_cur_pos(self) -> int:
        if self.mDev_name[0].upper() == 'S' or self.mDev_name[0].upper() == 'G':        
                                # spinner / gripper  -- no encoder
            print_log(f'Device {self.mDev_name} has no encoder setting pos = 0')
            return 0
        try:
            res, answ = FH_cmd(self.mDev_ser, 'POS', self.fh_lock)

            self.mDev_pos = int(answ)

        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_log(f"Connection to port {self.fh_port} was lost")
            print_log(f"Unexpected Exception: {ex}")
            return 0         
        else:
            return self.mDev_pos        

    def  mDev_reset_pos(self)->bool:
        self.mDev_stop()
        try:
            res, answ = FH_cmd(self.mDev_ser, 'HO', self.fh_lock)
            print_log (f"FAULHABER HOMED on port = {self.mDev_ser.port}/{self.fh_port} with res=  {res} and reply = {answ}" )

            self.mDev_pos = 0

        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_log(f"ULHABER HOMED fales on port {self.fh_port} was lost")
            print_log(f"Unexpected Exception: {ex}")
            return False         
        else:
            return True 
        
    def getTitle(self)->str:
        return self.__title

    def set_parms(self, parms):
        self.MEASUREMENT_DELAY = get_parm(self.devName, parms, 'MEASUREMENT_DELAY')
        self.MINIMAL_OP_DURATION = get_parm(self.devName, parms, 'MINIMAL_OP_DURATION')
        self.GRIPPER_TIMEOUT  = get_parm(self.devName, parms, 'GRIPPER_TIMEOUT')
        self.TROLLEY_PARKING_VELOCITY = get_parm(self.devName, parms, 'TROLLEY_PARKING_VELOCITY')
                                                                                            #  velocity to set at parking stage
        self.DEFAULT_ROTATION_TIME = get_parm(self.devName, parms, 'DEFAULT_ROTATION_TIME')
        self.DEAFULT_VELOCITY_EV_VOLTAGE = get_parm(self.devName, parms, 'DEAFULT_VELOCITY_EV_VOLTAGE')
        self.DEAFULT_TROLLEY_RPM = get_parm(self.devName, parms, 'DEAFULT_TROLLEY_RPM')
        self.TROLLEY_DEFAULT_CURRENT_LIMIT = get_parm(self.devName, parms, 'TROLLEY_DEFAULT_CURRENT_LIMIT')
        self.GRIPPER_DEFAULT_CURRENT_LIMIT = get_parm(self.devName, parms, 'GRIPPER_DEFAULT_CURRENT_LIMIT')
        self.SPINNER_DEFAULT_CURRENT_LIMIT = get_parm(self.devName, parms, 'SPINNER_DEFAULT_CURRENT_LIMIT')
# for compitability
        self.DevMaxSPEED = get_parm(self.devName, parms, 'DevMaxSPEED')
        self.DevOpSPEED  = get_parm(self.devName, parms, 'DevOpSPEED')
        
        self.__title = get_parm(self.devName, parms, 'TITLE')


        self.el_voltage =  self.DEAFULT_VELOCITY_EV_VOLTAGE
        self.DevOpSPEED = self.DEAFULT_TROLLEY_RPM
        self.rpm = self.DEAFULT_TROLLEY_RPM 

        if self.mDev_type == '--TROLLEY--':
            self.el_current_limit = self.TROLLEY_DEFAULT_CURRENT_LIMIT
            
        elif self.mDev_type == '--GRIPPER--':
            self.el_current_limit = self.GRIPPER_DEFAULT_CURRENT_LIMIT
            

        elif self.mDev_type == '--TIME_ROTATOR--':
            self.rotationTime = self.DEFAULT_ROTATION_TIME
            self.el_current_limit = self.SPINNER_DEFAULT_CURRENT_LIMIT
            

        print_inf (f'loading parms for {self.devName}')

    def mutualControl(self):
        if self.dev_lock.locked():
            print_err(f'ERROR- The device {self.devName} on port {self.fh_port} is active. Cant allow multiply activations')
            return False
        else:
            self.dev_lock.acquire()
            return True


#------------------------- U N I T E S T ----------------------------
    
if __name__ == "__main__":

    import PySimpleGUI as sg

    image_left = './Images/button_left_c.png'
    image_right = './Images/button_right_c.png'

# OFF button diagram
    toggle_btn_off = b'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'

# ON button diagram
    toggle_btn_on = b'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'


    trolley  = [[sg.Push(), sg.Text('TEST DEVICE (RPM)', font='Any 20'),  sg.Push()],
            [sg.T('Position'), sg.Text("_", size=(10, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                border_width = 2, key='-1-TROLLEY_POSSITION-'), sg.Button(button_text = "Reset", key='-1-TROLLEY_TARGET_RESET-') ],
            [sg.Text('Target'), sg.Input(size=(10, 1), enable_events=True, key='-TROLLEY_TARGET-', \
                font=('Arial Bold', 10), justification='left'), sg.Button(button_text = "Set & Go", key='-1-TROLLEY_POS_SET-')],
            [sg.Button( button_color=sg.TRANSPARENT_BUTTON, image_filename = image_left, image_size=(55, 60), \
                image_subsample=2, border_width=0, key='-1-TROLLEY_LEFT-'),
                sg.Frame('',[[sg.Text('Velocity (RPM)')], [sg.Input(size=(15, 1), enable_events=True, key='-1-TROLLEY_VELOCITY-', \
                    font=('Arial Bold', 10), justification='left')]], border_width=0),
            sg.Button( button_color=sg.TRANSPARENT_BUTTON, image_filename = image_right, image_size=(55, 60), \
                image_subsample=2, border_width=0, key='-1-TROLLEY_RIGHT-')], 
            [sg.Button(button_text = 'Stop',  key='-1-TROLLEY_STOP-')]]
    
    la_gripper = [[sg.Push(), sg.Text('TEST ON/OFF', font='Any 20'),  sg.Push()], [sg.Button('', image_data=toggle_btn_off, key='ONOFF', \
                button_color=(sg.theme_background_color(), sg.theme_background_color()), border_width=0)], 
                [sg.Button('Exit')]]
    
    i=1

    la_Rotator =  [[sg.Text(f'TEST mA DEVICE', font='Any 20'), sg.Text('On/Off'), \
              sg.Button(button_text ='On', size=(2, 1), button_color='white on green', key=f'-{i}-GRIPPER-ON-', 
                        disabled_button_color = 'light slate gray on navajo white'),
              sg.Button(button_text ='Off', size=(2, 1), button_color='white on red', key=f'-{i}-GRIPPER-OFF-', 
                        disabled_button_color = 'light slate gray on navajo white')],
            [ sg.Text(f'Velocity (mV/rpm) ->>', key = f'-{i}-VOLT-RPM-'), sg.Input(size=(4, 1), enable_events=True, key=f'-{i}-GRIPPER-RPM-')], 
            #  [sg.Text(f'Stop (mA)'), sg.Input(size=(3, 1), enable_events=True, key=f'-{i}-GRIPPER-CURR-')],
             [sg.Button(button_text = "Reset", key=f'-{i}-GRIPPER_TARGET_RESET-'), 
              sg.Text("_", size=(8, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', border_width = 2, key=f'-{i}-GRIPPER_POSSITION-'),
             sg.Input(size=(10, 1), enable_events=True, key=f'-{i}-GRIPPER_TARGET-', \
            font=('Arial Bold', 8), justification='left'), sg.Button(button_text = "Go", key=f'-{i}-GRIPPER_POS_SET-')],
            [sg.Text("_", size=(4, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
            border_width = 2, key=f'-{i}-GRIPPER_CUR_DISPLAY-'),
            sg.Text(f'Stop (mA) >\n< Curr. (mA)'), sg.Input(size=(4, 1), enable_events=True, key=f'-{i}-GRIPPER-CURR-'), sg.Button(button_text = 'Stop',  key=f'-{i}-GRIPPER_STOP-')],
             ]



    def main_loop():
        from bs1_faulhaber import FH_Motor 

    # Layout (COM / on / off)
        layout = [
            [sg.Frame('', la_gripper,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                    border_width=3, vertical_alignment='center', element_justification = "center")],
            [sg.Frame('', trolley,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                    border_width=3, vertical_alignment='center', element_justification = "center")],
            [sg.Frame('', la_Rotator,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                    border_width=3, vertical_alignment='center', element_justification = "center")]

        ]

        window = sg.Window('Unitest', layout, finalize = True)

        from bs1_config import port_scan
        print(f'Scanning ports')
        devs = port_scan()



        down = graphic_off = True
        
        # gripper = FH_Motor(m_port, m_baudrate, m_timeout)
       
        # if not gripper.init_dev('--GRIPPER--'):
        #     print_log(f'Gripper initiation failed on port {m_port}')
        #     del gripper
        #     sys.exit()
        dev_trolley = None
        dev_gripper =  None
        dev_rotator = None

        for fh_dev in devs:
            if fh_dev.C_type == '--TROLLEY--':
                dev_trolley = fh_dev.dev_mDC 
            elif fh_dev.C_type == '--GRIPPER--' or  fh_dev.C_type == '--TIME_ROTATOR--':
                dev_gripper = fh_dev.dev_mDC 
            elif fh_dev.C_type == '--TIME_ROTATOR--':
                dev_rotator = fh_dev.dev_mDC 

        if dev_trolley:
            dev_trolley.mDev_pressed = False

        if dev_gripper:
            dev_gripper.mDev_pressed = False

        if dev_rotator:
            dev_rotator.mDev_pressed = False
        


    #-----------------------------------------------------------   
        if dev_trolley:
            window['-TROLLEY_POSSITION-'].update(value = dev_trolley.mDev_pos)
            window['-TROLLEY_VELOCITY-'].update(value = dev_trolley.rpm)

        if dev_rotator:
            window['-1-GRIPPER-RPM-'].update(value = dev_trolley.rpm)

        while True:

            #get event
            event, values = window.read(timeout=100)

            # print(event, values)

            #When the window is closed or the Exit button is pressed
            if event in (sg.WIN_CLOSED, 'Exit'):
                print_log(f'Exiting')
                # sys.exit()
                break


            #When '-TOGGLE-GRAPHIC-' button is presseddisab
            elif event == 'ONOFF':
                graphic_off = not graphic_off
                window['ONOFF'].update(image_data=toggle_btn_off if graphic_off else toggle_btn_on)
                if dev_gripper: 
                    if graphic_off:
                        dev_gripper.gripper_on()
                        print_log(f'Switching on')
                    else:
                        dev_gripper.gripper_off()
                        print_log(f'Switching off')

                
                window['ONOFF'].update(disabled = True)
                
                if dev_gripper:
                    dev_gripper.mDev_pressed = True

            elif  event == '-TROLLEY_TARGET-':
                if  len(values['-TROLLEY_TARGET-']) > 0 and values['-TROLLEY_TARGET-'][-1] not in ('-0123456789'):
                    sg.popup("Only digits allowed")
                    window['-TROLLEY_TARGET-'].update(values['-TROLLEY_TARGET-'][:-1])
                    # window['-TROLLEY_TARGET-'].update(disabled = True)
            elif event == '-TROLLEY_POS_SET-':
                window['-TROLLEY_POSSITION-'].update(value = values['-TROLLEY_TARGET-'])


                if dev_trolley:
                    dev_trolley.go2pos(values['-TROLLEY_TARGET-'])
                    dev_trolley.mDev_pressed = True
                
                window['-TROLLEY_POS_SET-'].update(disabled = True)
                window['-TROLLEY_RIGHT-'].update(disabled = True)
                window['-TROLLEY_LEFT-'].update(disabled = True)

            elif event == '-TROLLEY_TARGET_RESET-':
                window['-TROLLEY_POSSITION-'].update(value = 0)
                window['-TROLLEY_TARGET-'].update(value = 0) 
                
            elif event == '-TROLLEY_RIGHT-':
                window['-TROLLEY_POS_SET-'].update(disabled = True)
                window['-TROLLEY_RIGHT-'].update(disabled = True)
                window['-TROLLEY_LEFT-'].update(disabled = True)
                if dev_trolley:
                    dev_trolley.mDev_forward()
                    dev_trolley.mDev_pressed = True

            elif event == '-TROLLEY_LEFT-':
                window['-TROLLEY_POS_SET-'].update(disabled = True)
                window['-TROLLEY_RIGHT-'].update(disabled = True)
                window['-TROLLEY_LEFT-'].update(disabled = True)
                if dev_trolley:
                    dev_trolley.mDev_backwrd()
                    dev_trolley.mDev_pressed = True
            
            elif event == '-TROLLEY_STOP-':
                if dev_trolley:
                    dev_trolley.mDev_stop()


                window['-TROLLEY_POS_SET-'].update(disabled = False)
                window['-TROLLEY_RIGHT-'].update(disabled = False)
                window['-TROLLEY_LEFT-'].update(disabled = False)
                

            if dev_gripper  and dev_gripper.mDev_pressed and not dev_gripper.wd.is_alive():
                print_log(f'Complete')
                window['ONOFF'].update(disabled = False)
                dev_gripper.mDev_pressed = False

            if dev_trolley and dev_trolley.mDev_pressed and not dev_trolley.wd.is_alive():
                print_log(f'Trolley Complete')
                window['-TROLLEY_POS_SET-'].update(disabled = False)
                window['-TROLLEY_RIGHT-'].update(disabled = False)
                window['-TROLLEY_LEFT-'].update(disabled = False)
                dev_trolley.mDev_pressed = False
            

            if  dev_trolley and dev_trolley.mDev_pressed and dev_trolley.wd.is_alive():
                new_pos = dev_trolley.mDev_get_cur_pos()
                # print_log(f'Updating possition: {new_pos}')
                window['-TROLLEY_POSSITION-'].update(value = new_pos)

        
        if dev_trolley:
            del  dev_trolley
        if dev_gripper:
            del dev_gripper    

        window.close()
        


    if (len(sys.argv) != 3) or \
        (sys.argv[1][0:3].upper() != "COM" ) or \
        (len(sys.argv[1]) != 4) or \
        (not sys.argv[1][3:4].isdecimal()) or \
        (not sys.argv[2].isdecimal()):
        print (f"Usage: python {sys.argv[0]} port max_current (where port is serial COM number i.e. COM3)")
        sys.exit()
    
    m_port = sys.argv[1]
    m_current = int(sys.argv[2])
    m_baudrate = 9600
    m_timeout = 2

    main_loop()