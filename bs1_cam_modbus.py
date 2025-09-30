__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "1.0.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Tool"


import time, re 
from pyModbusTCP.client import ModbusClient
import serial as serial
import time, sys
from inputimeout  import inputimeout , TimeoutOccurred
import threading
from threading import Lock
from queue import Queue 
from collections import namedtuple
from typing import List
import PySimpleGUI as sg





# TIMEOUT_ERROR = 10
# MAX_MOVEMENT = 400   
EMERGENCY_TIMEOUT = 30
DEFAULT_TIMEOUT_ERROR = 10


from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, set_parm, get_parm, \
    assign_parm , assign_type_parm


# camRes = namedtuple("camRes",  ["res", "dist", "repQ"])
# camRes.__annotations__={'res':bool,  'dist':float, 'repQ':Queue}         # specify type of elements

camResFields = ["res", "dist", "repQ", "abs"]
camRes = namedtuple("camRes",  camResFields, defaults=[None,] * len(camResFields))
camRes.__annotations__={'res':bool,  'dist':float, 'repQ':Queue, 'abs':bool}         # specify type of elements


# Holding registers
START_STOP_TRIGGER_REG = 40001      # 0 - stop  / 1 - start (trigger)

START_READ_BLOCK = 40002
BUSY_REG = 40002            # bussy = 1  / ready = 0  
OP_STATUS_PASS_REG = 40003       # operation fail = 0 / operation pass = 1 / operation error = 2
OP_STATUS_FAIL_REG = 40004       # operation fail = 0 / operation pass = 1 / operation error = 2  OBSOLETE!!!
OP_STATUS_CONTINUE_REG = 40005       # operation continue = 1 / finish = 0
                                
OP_RESULT_REG = 40100           # OP_RESULT_REG: 3 WORDS 
                                # 1 WORDS [0] -- sign //  1: - // 0: +
                                #  2 WORSD [1-2] (little-endian, first - the least significant word LSB,
                                # second - most significant word MSB  )

READ_BLOCK_SIZE = 120

RIGHT_MARGIN_SIGN_REG = 40044        # right margin sign //  1: - // 0: +
RIGHT_MARGIN_REG = 40045        # right margin offset
LEFT_MARGIN_SIGN_REG = 40046         # left margin segn //  1: - // 0: +
LEFT_MARGIN_REG = 40047         # left margin offset
NOZLLE_HEIGHT_REG = 40048       # The nozzle height determines the distance between the nozzle and the print bed
LOAD_PROFILE_REG = 40049        # Load profile (filter at cam)
PROFILE_NAME_REG  = 40050       # Profile (filter) namer -> string
CMD_TERMINATE_REG = 40009       # say to camera to terminate
POSITION_REPORT_REG = 40010     # position reporting >> 2  WORDS  [0,1] (little-endian, first - the least significant LSB,
                                # second - most significant  MSB 
                                # [2] sign // 1: -  0: + )


DEFAULT_NOZZLE_HEIGHT = 2000    # 2 mm in microns 

class Cam_modbus:
    @staticmethod
    def find_server(t_ip, t_port) -> bool:
        try:
            print_log(f'Looking ModBus server at IP:port = {t_ip}:{t_port}')
            test_client = ModbusClient(host=t_ip,            # Initiating modbus client connection
                    port = int(t_port), unit_id = 1, debug=False, auto_open=True)
            print_log (f'test_client = {test_client}')
             
            
            if test_client is None:
                print_err(f'CAM ModBus server with IP:port = {t_ip}:{t_port} not found')
                return False
            


            _res = test_client.open()
            print_log(f'Connecting to Modbus server at IP:port = {t_ip}:{t_port} has {"SUCCEDED" if _res else "FAILED"}')

            if not test_client.is_open:
                print_log(f'ModBus client can not connect to server >> IP:port = {t_ip}:{t_port} ')
                test_client.close()
                del test_client
                return False
            
            test_client.close()
            del test_client
            return True
        
        except Exception as ex:
            print_err(f'Error detecting ModBus server with IP:port = {t_ip}:{t_port}')
            exptTrace(ex)
            return False

    def __init__ (self, mb_ip, mb_port, parms, dName = 'CAM', mb_unit_id = 1):
        # self.TIMEOUT_ERROR = 10             
        # self.MAX_MOVEMENT = 400
        self.TIMEOUT_ERROR = 0
        self.MAX_MOVEMENT = 0
        self.MOTORS:tuple = tuple()
        self.start_time = 0
        self.motor = None
        self.window:sg.Window = None
        self.wThread = None
        self.__abs = False
        self.devName = dName
        self.devNotificationQ = Queue()
        self.__index = int(dName[-1])
        self.__dev_lock =  Lock()                 # device lock to avoid multiply access
        self.__term_signal = False
        self.__profiles:dict = None
        self.__nozzle_height = DEFAULT_NOZZLE_HEIGHT
        self.__left_margin:int = 0
        self.__right_margin:int = 0 
        try:
            print_log(f'Connecting ModBus server at IP:port = {mb_ip}:{mb_port}')
            self.m_client = ModbusClient(host=mb_ip,            # Initiating modbus client connection
                    port = int(mb_port),
                    unit_id = mb_unit_id,
                    debug=False, auto_open=True)
        
            self.set_parms(parms=parms)


        except Exception as ex:
            self.m_client = None
            print_log(f'ModBus initialization failed. Exception = {ex}')
            exptTrace(ex)
        else:
            print_log(f'ModBus connected at server={mb_ip}, port={mb_port}')


    def set_parms(self, parms)->bool:
        # self.TIMEOUT_ERROR = get_parm(self.devName, parms, 'TIMEOUT_ERROR')
        self.TIMEOUT_ERROR = assign_parm(self.devName, parms, 'TIMEOUT_ERROR', DEFAULT_TIMEOUT_ERROR)
        self.MAX_MOVEMENT = get_parm(self.devName, parms, 'MAX_MOVEMENT')
        _MOT = get_parm(self.devName, parms, 'MOTORS')
        if _MOT is not None:
            self.MOTORS = tuple(map(str, _MOT.split(', ')))
        print_log(f'Calibration motors = {self.MOTORS}')

        self.__nozzle_height = assign_parm(self.devName, parms, 'NOZZLE_HEIGHT', DEFAULT_NOZZLE_HEIGHT)
        self.__left_margin = assign_type_parm(self.devName, parms, 'LEFT_MARGIN', int, 0)
        self.__right_margin = assign_type_parm(self.devName, parms, 'RIGHT_MARGIN', int, 0)
        

        self.__profiles = assign_parm(self.devName, parms, 'PROFILES', None)
        if self.__profiles is not None:
            if not isinstance(self.__profiles, dict):
                print_err(f'Wrong CAM profile structure in the list: {self.__profiles}')
                return False
            
            print_log(f'CAM ({self.devName}) profiles: {self.__profiles} ')
            parm_re = re.compile(r'(\s*\d+\s*)$')

            for _p_name, _p_num in self.__profiles.items():
                if not parm_re.match(str(_p_num)):
                    print_err(f'ERROR: Wrong profile number for profile {_p_name}: {_p_num}')
                    self.__profiles.pop(_p_name)
                    continue
            print_log(f'Avaliable profiles: {self.__profiles}')




    def block_read(self, key, _size) -> List[bytes]:
        # val:List[bytes] = list(READ_BLOCK_SIZE)
        val:List[bytes] = [0]* READ_BLOCK_SIZE
        val = self.m_client.read_holding_registers(START_READ_BLOCK, READ_BLOCK_SIZE)
        _index = key - START_READ_BLOCK
        print_log(f'DEBUG: key = {key}, _index = {_index}, size = {_size}, read data = {val[_index:_index + _size]}')
        return val[_index:_index + _size]



    def is_ready(self):

        if not self.m_client:
            print_log(f'No Modbus connection found')
            return False
        
        self.__dev_lock.acquire()
        try:
            val = [0] 
            _val = self.m_client.read_holding_registers(BUSY_REG, 1)
            if _val is not None:
                val = _val[0]
            else:
                print_err(f'ERROR reading holding register at address {BUSY_REG} ')
                
            # val = self.block_read(BUSY_REG, 1)[0]
            print_log(f'Cam ready flag {BUSY_REG} = {val})')

            self.__dev_lock.release()
            return True if not val else False
        
        except Exception as ex:
            exptTrace(ex)
            print_log(f'Reading holding register at address {BUSY_REG} failed. Exception = {ex}')
            self.__dev_lock.release()
            return False
        
    def to_continue(self):
        if not self.m_client:
            print_log(f'No Modbus connection found')
            return False
        
        val = [0]
        try:
            val = self.m_client.read_holding_registers(OP_STATUS_CONTINUE_REG, 1)[0]
            # val = self.block_read(OP_STATUS_CONTINUE_REG, 1)[0]
            print_log(f'Cam continue flag {OP_STATUS_CONTINUE_REG} = {val})')

            return True if val else False
        
        except Exception as ex:
            exptTrace(ex)
            print_log(f'Reading holding register at address {OP_STATUS_CONTINUE_REG} failed. Exception = {ex}')
            return False


    def report_position(self, _position):
        print_log(f'Reporting position: {_position}, motor = {self.motor}')
        try:
            _in_microns= int(_position)
            _sign = 1 if _in_microns < 0 else 0
            _data_lst = [0] * 3
            if (self.motor[0] == 'Z'):
                _in_microns = int(_position*1000)
                
            _data_lst[2] = _sign
            _data_lst[1] = abs(_in_microns) >> 16
            _data_lst[0] = abs(_in_microns)  & 0x0000FFFF
            
            self.m_client.write_multiple_registers(POSITION_REPORT_REG, _data_lst)
            # self.m_client.write_single_register(BUSY_REG, 0)
            print_log(f'Sending position: Motor = {self.motor}  position = {_position}/{abs(_in_microns)}/{_data_lst}')
        except Exception as ex:
            print_log(f'Error reporting position Motor = {self.motor}  position =  {_position}/{abs(_in_microns)}/{_data_lst}. Exception = {ex}')
            exptTrace(ex)

        
    # def clearBusy(self):
    #     try:
    #         val = self.m_client.write_single_register(BUSY_REG, 1)
            
    #         return True if val else False
        
    #     except Exception as ex:
    #         print_log(f'Writing holding register at address {BUSY_REG} failed. Exception = {ex}')
    #         return False
        
    def is_success(self):

        if not self.m_client:
            print_log(f'No Modbus connection found')
            return False
        pass_val = [0]
        fail_val= [0]

        try:
            pass_val = self.m_client.read_holding_registers(OP_STATUS_PASS_REG, 1)[0]  
            fail_val = self.m_client.read_holding_registers(OP_STATUS_FAIL_REG, 1)[0]  

            # pass_val = self.block_read(OP_STATUS_PASS_REG, 1)[0]
            # fail_val = self.block_read(OP_STATUS_FAIL_REG, 1)[0]

            print_log(f'Cam pass = {pass_val}, fail = {fail_val})')
            if pass_val == fail_val:
                print_err(f'ERROR: PASS and FAIL registerts have a same value = {pass_val}')
            pass

        except Exception as ex:
            exptTrace(ex)
            return False

        return True if pass_val else False

    def cam_status(self):
        return True if self.m_client else False
    
    def start_cam_check_operation(self, profile, window:sg.Window)->bool:
        try:
            self.window = window
            self.motor = None
            self.start_time = time.time()
            self.wThread = threading.Thread(target=self.CheckComThread, args=(profile,))
            self.wThread.start()
        except Exception as ex:
            exptTrace(ex)
            return False

        return True

    def start_cam_tuning_operation(self, motor_n, profile, window:sg.Window, _abs=False) -> bool:
        self.__term_signal = False
        if not self.m_client:
            print_log(f'No Modbus connection found')
            return False
        self.motor = motor_n
        self.window = window
        if _abs:
            self.__abs =  _abs
        else:
            self.__abs = False
        self.start_time = time.time()
        self.wThread = threading.Thread(target=self.ModBusComThread, args=(profile,))
        self.wThread.start()
        return True


    def CheckComThread(self, profile_name):
        profile:str = None
        if self.__profiles is not None and profile_name in self.__profiles.keys():
            profile = str(self.__profiles[profile_name])
        else:
            print_log(f'Non existing profile {profile_name}. Profiles = {self.__profiles}, keys = {self.__profiles.keys()}')
            self.devNotificationQ.put(False)
            return
        
        print_log(f'Loading profile {profile_name}/{profile}')
        _loadProfileRes = self.load_profile(profile)
        if not _loadProfileRes:
            print_log(f'Failed loading profile {profile_name}/{profile}')
            self.devNotificationQ.put(False)
            return        
        resultV:camRes = self.calibrationStep()
        if resultV.res == False:
            self.devNotificationQ.put(False)
        else:
            print_log(f'Check result = {resultV.dist}')
            self.devNotificationQ.put(True)
            
        

    def ModBusComThread(self, profile = ''):
        _startTime = time.time()
        reportQ:Queue = Queue()

        _loadProfileRes = self.load_profile(profile)
        if not _loadProfileRes:
            self.devNotificationQ.put(False)
            self.window.write_event_value(f'-{self.__index}-CALIBRATE_DONE-', sys.maxsize)
            return
        
        _opStatus:bool = True
        while not self.__term_signal:
            resultV:camRes = self.calibrationStep()
            dist = 0
            
            if resultV.res == False:
                dist = sys.maxsize
                _opStatus = False
                # self.devNotificationQ.put(False)
                print_err(f'Error calibration or termination event detected ')
                break
            else:
                dist = resultV.dist
                self.window.write_event_value(f'-{self.__index}-CALIBRATE_CONTINUE-', camRes(res=True, dist=dist, repQ=reportQ, abs=self.__abs))
                try:
                    recvEvent = reportQ.get(block = True, timeout = EMERGENCY_TIMEOUT)
                except Exception as ex:
                    print_err(f'EMERGENCY TIMEOUT was triggered')
                    _opStatus = False
                    # self.devNotificationQ.put(False)
                    self.__term_signal =  True
                    break
                
                if time.time() - _startTime > EMERGENCY_TIMEOUT:
                        print_err(f'ERROR: Too long CALIBRATION period = {time.time() - _startTime}')
                        _opStatus = False
                        # self.devNotificationQ.put(False)
                        self.__term_signal = True
                        break
                
                if recvEvent.event == '-TASK_ERROR-':
                    _opStatus = False
                    # self.devNotificationQ.put(False)
                    print_log(f'Error motor calibration')
                    self.__term_signal = True
                    break

                if recvEvent.event == '-TASK_DONE-':                              # TASK_DONE/success
                    if not self.to_continue():
                        print_log(f'Stop calibration')
                        print_log(f'Task done received')
                        self.window.write_event_value(f'-{self.__index}-CALIBRATE_POSITION-', reportQ)
                        _position = 0
                        try:
                            _position = reportQ.get(block = True, timeout = EMERGENCY_TIMEOUT)
                            self.report_position(_position)
                        except Exception as ex:
                            print_err(f'EMERGENCY TIMEOUT was triggered while getting motor position')
                            self.__term_signal = True
                            
                        _opStatus = True
                        # self.devNotificationQ.put(True)
                        break
                    else:
                        print_log(f'Calibration continue')
                        continue


                
        if self.__term_signal:
            print_log(f'Termination CAM signal detected for {self.devName}')
            self.terminateCamera()

        self.devNotificationQ.put(_opStatus)
            
        self.window.write_event_value(f'-{self.__index}-CALIBRATE_DONE-', dist)

    def mDev_stop(self):                            # for compitability with CDev
        print_log(f'CAM modbus device stop. Not supported yet')
        pass

    def load_profile(self, profile:str='')->bool:
        try:
            # res = self.m_client.write_multiple_registers(PROFILE_NAME_REG, [char for char in profile])
            if profile == '' or profile == None:
                profile = 'base'
            
           
            print_log(f'Profile = {profile}: write_multiple_registers({PROFILE_NAME_REG})')
            res = self.m_client.write_multiple_registers(PROFILE_NAME_REG, [ord(char) for char in profile])
            if not res:
                print_log(f'Error writing value {profile} filter at address {PROFILE_NAME_REG}')
                return False
            print_log(f'next op = write_single_register({LOAD_PROFILE_REG})')
            res = self.m_client.write_single_register(NOZLLE_HEIGHT_REG, self.__nozzle_height)      # set nozzle height
            if not res:
                print_log(f'Error setting {self.__nozzle_height}  (nozzle height) at address {NOZLLE_HEIGHT_REG}')
                return False
            res = self.m_client.write_single_register(LEFT_MARGIN_SIGN_REG, 1 if self.__left_margin < 0 else 0)      
            if not res:
                print_log(f'Error setting {self.__left_margin}  (left margin sign) at address {LEFT_MARGIN_SIGN_REG}')
                return False
            res = self.m_client.write_single_register(LEFT_MARGIN_REG, abs(self.__left_margin))      
            if not res:
                print_log(f'Error setting {self.__left_margin}  (left margin) at address {LEFT_MARGIN_REG}')
                return False
            res = self.m_client.write_single_register(RIGHT_MARGIN_SIGN_REG, 1 if self.__right_margin < 0 else 0)
            if not res:
                print_log(f'Error setting {self.__right_margin}  (right margin sign) at address {RIGHT_MARGIN_SIGN_REG}')
                return False
            res = self.m_client.write_single_register(RIGHT_MARGIN_REG, abs(self.__right_margin))
            if not res:
                print_log(f'Error setting {self.__right_margin}  (right margin) at address {RIGHT_MARGIN_REG}')
                return False
            

            res = self.m_client.write_single_register(LOAD_PROFILE_REG, 1)      # set flag
            if not res:
                print_log(f'Error setting 1 at address {LOAD_PROFILE_REG}')
                return False
            
        except Exception as ex:
            print_log(f'Error communication ModBus/camera. Exception = {ex}')
            exptTrace(ex)
            return False
        
        return True

    def setTerminate(self)->bool:
        self.__term_signal = True
        return True

    def terminateCamera(self)->bool:
        self.__dev_lock.acquire()

        try:
            res = self.m_client.write_single_register(CMD_TERMINATE_REG, 1)
            if not res:
                raise Exception(f'Error terminating CAMERA, i.e. writing value 1 at address {CMD_TERMINATE_REG}')
                
            self.__dev_lock.release()
            return True
        
        except Exception as ex:
            exptTrace(ex) 
            self.__dev_lock.release()
            return False

    def calibrationStep(self)  -> camRes:
        try:
           
            while not self.is_ready():
                if (time.time() - self.start_time > self.TIMEOUT_ERROR) or self.__term_signal:
                    print_log(f'Waiting ready status timeout / terminate signal = {self.__term_signal}')
                    self.__term_signal =  True
                    # self.terminateCamera()
                    return camRes(res=False, dist=0)
                time.sleep(0.1)

            print_log(f'next op = write_single_register({START_STOP_TRIGGER_REG})')
            res = self.m_client.write_single_register(START_STOP_TRIGGER_REG, 1)
            if not res:
                print_log(f'Error writing value 1 at address {START_STOP_TRIGGER_REG}')
                return camRes(res=False, dist=0)
            
            while not self.is_ready():
                if (time.time() - self.start_time > self.TIMEOUT_ERROR) or self.__term_signal:
                    print_log(f'Waiting ready status timeout / terminate signal = {self.__term_signal}')
                    self.__term_signal =  True
                    # self.terminateCamera()
                    return camRes(res=False, dist=0)
                time.sleep(0.05)

            if not self.is_success():
                return camRes(res=False, dist=0)
            
            print_log(f'next op = read_holding_registers({OP_RESULT_REG}, 3)')
            valsign = [0] * 3
            valsign = self.m_client.read_holding_registers(OP_RESULT_REG, 3)
            
            # valsign = self.block_read(OP_RESULT_REG, 3)

            sign = valsign[0]       #  1: - // 0: +
            _unsign_value =  (valsign[2] << 16) | valsign[1]
            val = _unsign_value * (-1) if sign == 1 else _unsign_value
            print_log(f'move command: received data = {valsign}, sign = {"-" if sign == 1 else "+"} val = {_unsign_value}/{val} for motor {self.motor}')

            try:
                dig = int(val)
                _ret_dist = dig

                if self.motor is not None and abs(dig) > self.MAX_MOVEMENT:
                    print_log(f'Out of range data = {dig}. Max allowed value = {self.MAX_MOVEMENT}')
                    return  camRes(res=False, dist=0)

                if self.motor is not None and ((self.motor[0] == 'Z') or (self.motor[0] == 'R')):
                    _ret_dist =  dig / 1000                             # conver distance to mm  for Zaber only

                return  camRes(res=True, dist= _ret_dist)   # conver distance to mm  camRes(res=False, dist=0)
            
            except Exception as ex:
                exptTrace(ex)
                print_log(f'Wrong data = {val} read from {OP_RESULT_REG} holding register. Exception = {ex}')
                exptTrace(ex)
                return camRes(res=False, dist=0)

        except Exception as ex:
            print_log(f'Error communication ModBus/camera. Exception = {ex}')
            exptTrace(ex)
            return camRes(res=False, dist=0)
        

    # def CamOperation(self, profile='') -> camRes:
    #     try:
    #         _loadProfileRes = self.load_profile(profile)
    #         if not _loadProfileRes:
    #             return camRes(res=False, dist=0) 
            
    #         while True:
    #             _camRes = self.calibrationStep()
    #             if _camRes.res == False or not self.to_continue():
    #                 return _camRes
                
            

    #     except Exception as ex:
    #         print_log(f'Error communication ModBus/camera. Exception = {ex}')
    #         exptTrace(ex)
    #         return camRes(res=False, dist=0)
        


#####################  OLD SCHEME MPLEMENTATION ####################

#     def start_tuning (self, motor_n):
#         if not self.m_client:
#             print_log(f'No Modbus connection found')
#             return False
#         self.motor = motor_n
#         val = 1
#         try:
#             res = self.m_client.write_single_register(START_STOP_TRIGGER_REG, val)
#             if not res:
#                 print_log(f'Error writing value {val} at address {START_STOP_TRIGGER_REG}')
#                 return False

#             print_log(f'Calibration on {motor_n} started')
#         except Exception as ex:
#             self.m_client = None
#             print_log(f'Writing holding register at address {START_STOP_TRIGGER_REG} failed . Exception = {ex}')
#             return False

#         return True


#     def calibration_step (self):         
#                                     # operation fail = 0 //  operation pass = 1  // operation error = 2 //  not ready = 3 // 
#                                     # timeout = 4 // move = 5
#                                     # second return value - moving distance 
        
#         if  not self.is_ready():
#             return 3, 0 

#         if time.time() - self.start_time > self.TIMEOUT_ERROR:
#             print_log(f'Waiting ready status timeout')
#             self.clear_start_flag()
#             return 4, 0
#         try:
#             val = self.m_client.read_holding_registers(OP_STATUS_PASS_REG, 1)[0]
#             print_log(f'Read status = {val}')
#             if val == 1:                             # success, go read move register
#                 print_log(f'Calibration succeeded')
#                 self.clear_start_flag()             
#                 # return val, 0

# # BUGBUG   go to move               

#             elif val == 2:              # critical error
#                 print_log(f'Calibration critical error')
#                 self.clear_start_flag()
#                 return val, 0
#             elif val == 0:
#                 print_log(f"Calibration error // Can't happen")
#                 return val, 0
#             else: 
#                 return 6, 0

#         except Exception as ex:
#             # self.m_client = None
#             print_log(f'Reading holding register at address {OP_STATUS_PASS_REG} failed . Exception = {ex}')
#             self.clear_start_flag()
#             return 2, 0

#         try:
#             valsign = self.m_client.read_holding_registers(OP_RESULT_REG, 2)
#             sign = valsign[0]       #  1: - // 0: +
#             # val = valsign[1]
#             val = valsign[1] * (-1) if sign else valsign[1]
#             print_log(f'move command = {valsign}, val = {val} for motor {self.motor}')

#             try:
#                 dig = int(val)
#                 if abs(dig) > self.MAX_MOVEMENT:
#                     print_log(f'Out of range data = {dig}. Max allowed value = {self.MAX_MOVEMENT}')
#                     return 2, 0
                

#                 if self.motor == 'Z8':
#                     print_log(f'change direction for motor {self.motor}')
#                     dig *= -1           # Z8 has mirror calibration
                    
#                 return 5, dig / 1000   # conver distance to mm
#             except Exception as ex:
#                 print_log(f'Wrong data = {val} read from {OP_RESULT_REG} holding register. Exception = {ex}')
#                 self.clear_start_flag()
#                 return 2, 0
            
   
#         except Exception as ex:
#             print_log(f'Reading holding register at address {OP_RESULT_REG} failed . Exception = {ex}')
#             self.clear_start_flag()
#             return 2, 0
                
        

    # def stop_tuning (self):
    #     if not self.m_client:
    #         print_log(f'No Modbus connection found')
    #         return False
    #     self.start_time = 0
    #     self.clear_start_flag()

    #     return True
        
      
    # def clear_start_flag(self):

    #                  # Clearing start flag  
    #     val = 0
    #     try:
    #         res = self.m_client.write_single_register(START_STOP_TRIGGER_REG, val)
    #         if not res:
    #             print_log(f'Error writing value {val} at address {START_STOP_TRIGGER_REG}')
    #     except Exception as ex:
    #         print_log(f'Writing holding register at address {START_STOP_TRIGGER_REG} failed (value = {val}) . Exception = {ex}')       

        
   

    def __del__ (self):
        pass




if __name__ == "__main__":

    def ret2text(ret) -> str:
        if ret == 0:
            return('operation fail')
        elif ret == 1:
            return('operation pass')
        elif ret == 2:
            return('operation errorl')
        elif ret == 3:
            return('not ready')
        elif ret == 4:
            return('timeoutl')
        elif ret == 5:
            return('move')
        else:
            return('unknown')




    try:
        test_client = Cam_modbus.find_server('192.168.230.51', 1502)
        print_log(f'test_client = {test_client}')

        # calibration_cam = Cam_modbus('192.168.230.51', 1502)

        # calibration_in_process = calibration_cam.start_tuning('Z4')

        # if not calibration_in_process:
        #     print_log(f'Calibration on Z4 failed to start')
        #     sys.exit()
        # else:
        #     print_log(f'calibration will be operated on Z4')

        # while True:
        #     time.sleep(0.1)
        #     try:
        #         i_cmd = inputimeout(prompt='Press Enter to continue....', timeout=1000)
        #         # i_cmd = inputimeout(prompt='Press Enter to continue....', timeout=5)
        #         # print(">", end='')
        #         # i_cmd = inputimeout(timeout=5)


        #     except TimeoutOccurred:
        #         print_log(f'Input timeout ')
        #         continue
        #     else:
        #         pass

             
        #     ret, dist = calibration_cam.calibration_step()
        #                             # operation fail = 0 //  operation pass = 1  // operation error = 2 //  not ready = 3 // 
        #                             # timeout = 4 // move = 5
        #                             # second return value - moving distance 
        #     print_log(f'Calibration step returned ret={ret} ({ret2text(ret)}), dist={dist}')
        #     if ret == 5:
        #         print_log(f'MOVE, distance = {dist}')
        #         sys.exit()

        #     elif ret == 0:
        #         print_log(f'Calibration on {calibration_cam.motor} failed. Go to next iteration..')
        #         continue
                
        #     elif ret == 1:
        #         print_log(f'Calibration on {calibration_cam.motor} succeded.')
        #         calibration_cam.stop_tuning()
        #         calibration_in_process = False
        #         break
                
        #     elif ret == 2:
        #         print_log(f'Calibration on {calibration_cam.motor} critical error')
        #         calibration_cam.stop_tuning()
        #         calibration_in_process = False
        #         break
                
        #     elif ret == 3:
        #         pass

        #     elif ret == 4:
        #         print_log(f'Calibration on {calibration_cam.motor} timeout')
        #         calibration_cam.stop_tuning()
        #         calibration_in_process = False
        #         break

           
    except KeyboardInterrupt:
            print_log(f'Cntr-C pressed. Exit.')