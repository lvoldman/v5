__author__ = "Leonid Voldman"
__created_on__ = "2025-10-06"  
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "1.0.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Tool"

from threading import Lock
import threading

from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, \
                                        s32, num2binstr, set_parm, get_parm, void_f

import time, json
from bs1_base_motor import BaseMotor, BaseDev
from bs1_ads import commADS, STATUS, EN_DeviceCoreState



class PLCDev(BaseDev):
    global_lock:Lock = Lock()

    def __init__(self, dev_name:str='', _dev_idx:int=0, devAPI:str=None, devINFO:str=None, _comADS:commADS=None):
        try:
            super().__init__(devName=dev_name)
            self.__ads:commADS = _comADS 
            self.__devAPI:dict = json.loads(devAPI) if devAPI is not None else None
            self.__devINFO:dict = json.loads(devINFO) if devINFO is not None else None
            self._dev_lock:Lock = Lock()
            self._dev_idx:int = _dev_idx
            self._dev_info:dict = None
            self.__wd = None                 # watch dog thread for async operations
            
        except Exception as ex:
            exptTrace(ex)   
            print_err(f'[device {self._devName}] PLCDev initialization failed. Exception: {ex}')
    
    def __del__(self):
        try:
            if self.__ads:
                del self.__ads
        except Exception as ex:
            exptTrace(ex)

    def safeOp(self, op:list):
        ret_val = None
        try:
            with PLCDev.global_lock:
                ret_val = f_op(*args, **kwargs)
        except Exception as ex:
            exptTrace(ex)
        return ret_val

    # operateDevice command: str -> (blocked:bool, result:bool)
    # blocked - True if the device is in async mode and notification should be awaited
    # result - True if the command run was successful (in case of synchronous mode) or operation started successfully (in case of async mode)
    
    def operateDevice(self, command:str)-> tuple[bool, bool]:
        try:
            blocked:bool = True
            result:bool = False
            self.__wd = None
            
            send_exData = self.__parseCMD(command)
            if send_exData is None:
                print_err(f'[device {self._devName}] PLCDev operateDevice: command parsing failed for command="{command}"')
                return (blocked, result)

            print(f'[device {self._devName}] Writing ExecutionInfo = {send_exData}')

            self._dev_lock.acquire()    # mutex for load command sequence
            self.__ads.writeVar(symbol_name='G_System.fbExternalAPI.ExecutionInfo', dataToSend = json.dumps(send_exData))
            self.__ads.writeVar(symbol_name='G_System.fbExternalAPI.DoLoadInfo', dataToSend = True)
            _index:int = self.__ads.readVar('G_System.fbExternalAPI.LastExecutorLoaded', var_type=int)
            self._dev_lock.release()   # release mutex for load command sequence
            if _index == 0:
                raise Exception(f'[device {self._devName}] PLCDev operateDevice: Loading ExecutionInfo failed for command="{command}"')
            
            print_log(f'[device {self._devName}] Loaded ExecutionInfo on index = {_index}')

            self._dev_lock.acquire()    # mutex foor run command sequence
            self.__ads.writeVar(symbol_name='G_System.fbExternalAPI.RunExecutionID', dataToSend = send_exData["ExecutionID"] )
            self.__ads.writeVar(symbol_name='G_System.fbExternalAPI.DoRun', dataToSend = True )
            _errorMsg:str = self.__ads.readVar('G_System.fbExternalAPI.ErrorMessage', var_type=str, size=256)
            self._dev_lock.release()   # release mutex for run command sequence
            if _errorMsg != '':
                raise Exception(f'[device {self._devName}] PLCDev operateDevice: Starting ExecutionID={send_exData["ExecutionID"]} failed with error: {_errorMsg}')

            self.__wd = threading.Thread(target = self._watch_dog_thread, args=(send_exData["ExecutionID"], _index  ))
            self.__wd.start()

        except Exception as ex:
            print_log(f'[device {self._devName}] Error in operateDevice for command="{command}"')
            result = False
            if self.__wd is None or not self.__wd.is_alive():
                blocked = False
            exptTrace(ex)   

        return (blocked, result)
    
    def __parseCMD(self, cmd:str)-> dict | None:
        # BUGBUG: implement command parsing here
        return None
    
    
    def _watch_dog_thread(self, execution_id:int, _index:int):
        try:
            print_log(f'[device {self._devName}] Watch dog thread is alive... ExecutionID = {execution_id}, index = {_index }')
            
            if self.__ads is None:
                raise Exception(f'[device {self._devName}] ADS ERROR: PLC connection is not established in watch dog thread')
                
            
            while True:
                exStatus:int = self.__ads.readVar(symbol_name=f'G_System.fbExternalAPI.ExecutionStatus[{_index}].eExecutionStatus', \
                                                var_type=int)
                devState:int = self.__ads.readVar(symbol_name=f'G_System.fbExternalAPI.arDeviceInfo[{self._dev_idx}].State', \
                                                var_type=int)   
                __jsonINFO = self.__ads.readVar(symbol_name=f'G_System.fbExternalAPI.arDeviceInfo[{self._dev_idx}].DeviceInfo', \
                                                var_type=str, size=1024)   
                if self.__devINFO is not None:      # update device info
                    self.__devINFO |= (json.loads(__jsonINFO) if __jsonINFO is not None else dict())
                else:               # set device info   
                    self.__devINFO = (json.loads(__jsonINFO) if __jsonINFO is not None else None)
                
                print_log(f'[device {self._devName}] ExecutionStatus = {STATUS(exStatus)} ({exStatus}) for ExecutionID={execution_id} ')
                if exStatus == STATUS.DONE.value:   # Completed
                    print_log(f'[device {self._devName}] ExecutionID={execution_id} completed successfully')
                    break
                elif exStatus == STATUS.ERROR.value:   # Error
                    _errorMsg:str = self.__ads.readVar(symbol_name=f'G_System.fbExternalAPI.ExecutionStatus[{_index}].ErrorMessage', \
                                                        var_type=str, size=256)
                    print_err(f'[device {self._devName}] ExecutionID={execution_id} ended with ERROR: {_errorMsg}')
                    break
                elif devState == EN_DeviceCoreState.ERROR.value:   #    Device in error state
                    print_err(f'[device {self._devName}] Device entered ERROR state during ExecutionID={execution_id}')
                    break  
                
                time.sleep(0.5)
                    
           
        except Exception as ex:
            exptTrace(ex)   
        
        print (f'[device {self._devName}] Watch dog thread for devices on ExecutionID = {execution_id} / index = {_index} has ended')
