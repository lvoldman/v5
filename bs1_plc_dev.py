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

import time
from bs1_base_motor import BaseMotor, BaseDev
from bs1_ads import commADS, STATUS



class PLCDev(BaseDev):
    global_lock:Lock = Lock()

    def __init__(self, dev_name:str='', devAPI:dict=None, devINFO:dict=None, _comADS:commADS=None):
        try:
            super().__init__(devName=dev_name)
            self.__ads:commADS = _comADS 
            self.__devAPI: dict = devAPI
            self.__devINFO:dict = devINFO   
            self._dev_lock: Lock = Lock()
        except Exception as ex:
            exptTrace(ex)   
            print_err(f'PLCDev {dev_name} initialization failed. Exception: {ex}')
    
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
            blocked:bool = False
            result:bool = False
            
            send_exData = self.__parseCMD(command)
            if send_exData is None:
                print_err(f'PLCDev {self._devName} operateDevice: command parsing failed for command="{command}"')
                return (blocked, result)
            
            sym_exData = 'G_System.fbExternalAPI.ExecutionInfo'

            print(f'Writing ExecutionInfo = {send_exData}')
            self.__ads.writeVar(symbol_name=sym_exData, dataToSend = json.dumps(send_exData))

            sym_load = 'G_System.fbExternalAPI.DoLoadInfo'
            self.__ads.writeVar(symbol_name=sym_load, dataToSend = True)
            _index:int = self.__ads.readVar('G_System.fbExternalAPI.LastExecutorLoaded', var_type=int)
            print(f'Loaded ExecutionInfo on index = {_index}')

            self.__ads.writeVar(symbol_name='G_System.fbExternalAPI.RunExecutionID', dataToSend = send_exData["ExecutionID"] )
            self.__ads.writeVar(symbol_name='G_System.fbExternalAPI.DoRun', dataToSend = True )

            wd = threading.Thread(target = self._watch_dog_thread, args=(send_exData["ExecutionID"], _index  ))
            wd.start()
            wd.join()
            print (f'Watch dog thread for devices {send_exData["Devices"]} on ExecutionID = {send_exData["ExecutionID"]}  ended')

        except Exception as ex:
            exptTrace(ex)   

        return (blocked, result)
    
    def __parseCMD(self, cmd:str)-> dict:
        pass
    
    
    def _watch_dog_thread(self, execution_id:int, _index:int):
        try:
            print_log(f'Watch dog thread is alive...')
            
            if self.__ads is None:
                raise Exception(f'ADS ERROR: PLC connection is not established in watch dog thread')
                
            
            while True:
                sym_exStatus = f'G_System.fbExternalAPI.ExecutionStatus[{_index}].eExecutionStatus'
                exStatus:int = self.__ads.readVar(symbol_name=sym_exStatus, var_type=int)
                print_log(f'ExecutionStatus = {STATUS(exStatus)} ({exStatus}) for ExecutionID={execution_id} ')
                if exStatus == STATUS.DONE.value:   # Completed
                    print_log(f'ExecutionID={execution_id} completed successfully')
                    break
                time.sleep(0.5)
                    
           
        except Exception as ex:
            exptTrace(ex)   
        
