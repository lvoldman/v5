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
from bs1_ads import commADS, STATUS, EN_DeviceCoreState, symbolsADS, Pool

class runnerFactory:
    def __init__(self, num_runners:int):
        try:
            self._num_runners = num_runners
            self._runners_pool:Pool = Pool(size=num_runners)
            self.__runners_lst:list[list] = [ list() for _ in range(num_runners) ]   # list of lists to hold assigned devices for each runner
        except Exception as ex:
            exptTrace(ex)
            raise ex

    def attachDeviceToRunner(self, dev:BaseDev, indx:int = None)-> int:
        try:
            if indx is None:
                _run_indx = self._runners_pool.alloc()
                if _run_indx is None:
                    raise Exception(f'runnerFactory: No available runners to attach device {dev._devName}')

            else:
                _run_indx = indx

            self.__runners_lst[_run_indx - 1].append(dev)   # -1 because pool returns 1-based index

        except Exception as ex:
            exptTrace(ex)
            raise ex

        return _run_indx
    
    def detachDeviceFromRunner(self, dev:BaseDev, indx:int)-> bool:
        try:
            if dev in self.__runners_lst[indx - 1]:   # -1 because pool returns 1-based index
                self.__runners_lst[indx - 1].remove(dev)
                if len(self.__runners_lst[indx - 1]) == 0:
                    print_log(f'runnerFactory: No more devices attached to runner {indx}. Releasing runner.')
                    self._runners_pool.release(indx)
            else:
                raise Exception(f'runnerFactory: Device {dev._devName} is not attached to runner {indx}')

        except Exception as ex:
            exptTrace(ex)
            raise ex

        return True

class PLCDev(BaseDev):
    __global_lock:Lock = Lock()
    __number_of_runners:int = None
    __runner_factory:runnerFactory = None
    __ads:commADS = None
    __instances:int = 0

    def __init__(self, dev_name:str='', _dev_idx:int=0, devAPI:str=None, devINFO:str=None, _comADS:commADS=None):
        try:
            super().__init__(devName=dev_name)
            self.__devAPI:dict = json.loads(devAPI) if devAPI is not None else None
            self.__devINFO:dict = json.loads(devINFO) if devINFO is not None else None
            self._dev_lock:Lock = Lock()
            self._dev_idx:int = _dev_idx 
            self._dev_info:dict = None
            self.__wd = None                 # watch dog thread for async operations
            self.__runnerNum:int = None        # runner number assigned to the device
            self.__running:bool = False       # watch dog thread running flag

            if PLCDev.__ads is None:
                if _comADS is not None:
                    PLCDev.__ads = _comADS()
                else:
                    raise Exception(f'[device {self._devName}] PLCDev initialization failed. commADS instance is not provided')     
            if PLCDev.__number_of_runners is None:   # initialize number of runners
                PLCDev.__number_of_runners = PLCDev.__ads.readVar(symbol_name=symbolsADS._max___number_of_runners, var_type=int)
                print_log(f'[device {self._devName}] PLCDev initialized with __number_of_runners = {PLCDev.__number_of_runners}')
            if PLCDev.__runner_factory is None:          # initialize runner factory
                PLCDev.__runner_factory = runnerFactory(num_runners=PLCDev.__number_of_runners)
            PLCDev.__instances += 1
            print_log(f'[device {self._devName}] PLCDev instance created. Total instances = {PLCDev.__instances}')
            
        except Exception as ex:
            exptTrace(ex)   
            print_err(f'[device {self._devName}] PLCDev initialization failed. Exception: {ex}')
    
    def __del__(self):
        try:
            if self.__wd is not None and self.__wd.is_alive():
                print_log(f'[device {self._devName}] PLCDev __del__: Waiting for watch dog thread to end...')
                self.__wd.join(timeout=5)
                if self.__wd.is_alive():
                    print_err(f'[device {self._devName}] PLCDev __del__: Watch dog thread did not end within timeout')
                else:
                    print_log(f'[device {self._devName}] PLCDev __del__: Watch dog thread ended successfully')
            PLCDev.__instances -= 1
            print_log(f'[device {self._devName}] PLCDev instance deleted. Total remaining instances = {PLCDev.__instances}')
            if PLCDev.__instances == 0:
                del PLCDev.__ads
                del PLCDev.__runner_factory
                PLCDev.__ads = None
                PLCDev.__runner_factory = None  


        except Exception as ex:
            exptTrace(ex)


    
    # runDevicesOp runner:int -> bool  -- starts runner operation for device loaded to run for given runner number
    # than oiperates watch dog thread to monitor operation status for each of devices assigned to the runner
    # runner status is handled by by all devices assigned to the runner
    # returns (blocked:bool, result:bool)
    @staticmethod
    def runDevicesOp(runner:int)-> tuple[bool, bool]:
        try:
            if PLCDev.__runner_factory is None:
                raise Exception(f'PLCDev runDevicesOp: Runner factory is not initialized')
            if runner < 1 or runner > PLCDev.__number_of_runners:
                raise Exception(f'PLCDev runDevicesOp: Invalid runner number {runner}. Valid range is 1 to {PLCDev.__number_of_runners}')
            if len(PLCDev.__runner_factory.__runners_lst[runner-1]) == 0:
                raise Exception(f'PLCDev runDevicesOp: No devices assigned to runner number {runner}')

            PLCDev.__global_lock.acquire()    # global mutex for runner operation sequence
            PLCDev.__ads.writeVar(symbol_name=f'G_System.fbExternalAPI.fbExternalRunner[{runner}].DoRun', dataToSend = json.dumps(send_exData))
            PLCDev.__global_lock.release()   # release global mutex for runner operation sequence

            _errorMsg:str = PLCDev.__ads.readVar(f'G_System.fbExternalAPI.fbExternalRunner[{runner}].ErrorMessage', var_type=str, size=256)

            if _errorMsg != '':
                raise Exception(f'PLCDev runDevicesOp: Starting runner {runner} failed with error: {_errorMsg}')



            print_log(f'PLCDev runDevicesOp: Starting runner operation for runner number {runner} with {len(PLCDev.__runner_factory.__runners_lst[runner])} devices assigned')
            for dev in PLCDev.__runner_factory.__runners_lst[runner-1]:    # Operate watch dog thread for each device assigned to the runner  
                print_log(f'PLCDev runDevicesOp: Starting watch dog thread for device {dev._devName} on runner {runner}')
                dev.runWDThread()
            
        except Exception as ex:
            exptTrace(ex)
            print_log(f'PLCDev runDevicesOp: Error occurred while starting runner {runner}. Stopping all devices from the runner.')
            for dev in PLCDev.__runner_factory.__runners_lst[runner-1]:    # stop all devices assigned to the runner
                try:                    # stop watch dog thread if running
                    if dev.__wd is not None and dev.__wd.is_alive():
                        dev.stop()              # stop watch dog thread
                        print_log(f'[device {dev._devName}] PLCDev runDevicesOp: Waiting for watch dog thread to end...')
                        dev.__wd.join(timeout=0.5)   # wait for thread to end
                        if dev.__wd.is_alive():
                            print_err(f'[device {dev._devName}] PLCDev runDevicesOp: Watch dog thread did not end within timeout')
                        else:
                            print_log(f'[device {dev._devName}] PLCDev runDevicesOp: Watch dog thread ended successfully')
                except Exception as ex_detach:
                    exptTrace(ex_detach)

            return False, False
        
        return True, True

    # loadDeviceOp command: str -> (blocked:bool, result:bool, runner:int)
    # blocked - True if the device is in async mode and notification should be awaited
    # result - True if the command run was successful (in case of synchronous mode) or operation started successfully (in case of async mode)
    
    def loadDeviceOp(self, command:str, runnerNum = None)-> int:
        try:
            
            self.__wd = None
            
            send_exData = self.__parseCMD(command)
            if send_exData is None:
                print_err(f'[device {self._devName}] PLCDev loadDeviceOp: command parsing failed for command="{command}"')
                return runnerNum

            print_log(f'[device {self._devName}] Writing ExecutionInfo = {send_exData}')

            PLCDev.__ads.writeVar(symbol_name=f'G_System.fbExternalAPI.fbExternalRunner[{runnerNum}].ExecutionInfo', dataToSend = json.dumps(send_exData))
            PLCDev.__ads.writeVar(symbol_name=f'G_System.fbExternalAPI.fbExternalRunner[{runnerNum}].DoLoadInfo', dataToSend = True)
            
            print_log(f'[device {self._devName}] Loaded ExecutionInfo to PLC for command="{command}", runner={runnerNum}')


        except Exception as ex:
            print_log(f'[device {self._devName}] Error in loadDeviceOp for command="{command}"')
            exptTrace(ex)   
            
        PLCDev.__runner_factory.attachDeviceToRunner(self, indx=runnerNum)   # attach device to runner
        
        self.__runnerNum = runnerNum
        return runnerNum
    
    def __parseCMD(self, cmd:str)-> dict | None:
        # BUGBUG: implement command parsing here
        return None
    
    def runWDThread(self) -> bool:
        try:
            if  self.__wd.is_alive():
                raise Exception(f'[device {self._devName}] Watch dog thread is already running')
            self.__wd = threading.Thread(target = self._watch_dog_thread)
            self.__running = True
            self.__wd.start()
    
        except Exception as ex:
            exptTrace(ex)
            return False
    
        return True
    
        

    def _watch_dog_thread(self):
        try:
            print_log(f'[device {self._devName}] Watch dog thread is alive... runner = {self.__runnerNum}')
            
            if PLCDev.__ads is None:
                raise Exception(f'[device {self._devName}] ADS ERROR: PLC connection is not established in watch dog thread')
                
            
            while self.__running:
                exStatus:int = PLCDev.__ads.readVar(symbol_name=f'G_System.fbExternalAPI.ExecutionStatus[{self.__runnerNum}].eExecutionStatus', \
                                                var_type=int)
                devState:int = PLCDev.__ads.readVar(symbol_name=f'G_System.fbExternalAPI.arDeviceInfo[{self._dev_idx}].State', \
                                                var_type=int)   
                __jsonINFO = PLCDev.__ads.readVar(symbol_name=f'G_System.fbExternalAPI.arDeviceInfo[{self._dev_idx}].DeviceInfo', \
                                                var_type=str, size=1024)   
                if self.__devINFO is not None:      # update device info
                    self.__devINFO |= (json.loads(__jsonINFO) if __jsonINFO is not None else dict())
                else:               # set device info   
                    self.__devINFO = (json.loads(__jsonINFO) if __jsonINFO is not None else None)
                
                print_log(f'[device {self._devName}] ExecutionStatus = {STATUS(exStatus)} ({exStatus}) for runner = {self.__runnerNum}, DeviceState = {EN_DeviceCoreState(devState)} ({devState})')
                if exStatus == STATUS.DONE.value:   # Completed
                    print_log(f'[device {self._devName}] at runner={self.__runnerNum} completed successfully')
                    break
                elif exStatus == STATUS.ERROR.value:   # Error
                    _errorMsg:str = PLCDev.__ads.readVar(symbol_name=f'G_System.fbExternalAPI.ExecutionStatus[{self.__runnerNum}].ErrorMessage', \
                                                        var_type=str, size=256)
                    print_err(f'[device {self._devName}] runner={self.__runnerNum} ended with ERROR: {_errorMsg}')
                    break
                elif devState == EN_DeviceCoreState.ERROR.value:   #    Device in error state
                    print_err(f'[device {self._devName}] Device entered ERROR state during runner={self.__runnerNum}')
                    break  
                
                time.sleep(0.5)
                    
           
        except Exception as ex:
            exptTrace(ex)   
        
        if not self.__running:
            print_log (f'[device {self._devName}] Watch dog thread for devices on runner = {self.__runnerNum} is stopping...')
        else:
            self.__running = False

        PLCDev.__runner_factory.detachDeviceFromRunner(self, self.__runnerNum)

        print_log (f'[device {self._devName}] Watch dog thread for devices on runner = {self.__runnerNum} has ended')
        print_log (f'[device {self._devName}] Info = {self.__devINFO}')

        return
    
    def stop(self) -> bool:
        self.__running = False
        return True