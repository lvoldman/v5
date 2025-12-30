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
from queue import Queue 
import time, json

from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, \
                                        s32, num2binstr, set_parm, get_parm, void_f
from bs1_base_motor import BaseMotor, BaseDev
from bs1_ads import commADS, STATUS, EN_DeviceCoreState, symbolsADS, Pool

DEV_API_SIZE = 4000  # bytes
DEV_NAME_SIZE = 80  # chars
DEV_INFO_SIZE = 1024  # bytes

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
    
    def detachDeviceFromRunner(self, dev:BaseDev, indx:int)-> int:
        try:
            if dev in self.__runners_lst[indx - 1]:   # -1 because pool returns 1-based index
                self.__runners_lst[indx - 1].remove(dev)
                _runners_left = len(self.__runners_lst[indx - 1])
                print_log(f'runnerFactory: Device {dev._devName} detached from runner {indx}. Devices left on runner: {_runners_left}')
                if _runners_left == 0:
                    print_log(f'runnerFactory: No more devices attached to runner {indx}. Releasing runner.')
                    self._runners_pool.release(indx)
            else:
                raise Exception(f'runnerFactory: Device {dev._devName} is not attached to runner {indx}')

        except Exception as ex:
            exptTrace(ex)
            raise ex

        return _runners_left
    
    @property
    def runnersLst(self)-> list:
        return self.__runners_lst    

class PLCDev(BaseDev):
    devsList:list[str] = None
    __global_lock:Lock = Lock()
    __number_of_runners:int = None
    __runner_factory:runnerFactory = None
    __ads:commADS = None
    __instances:int = 0

    def __init__(self, dev_name:str='', _comADS:commADS=None):
        try:
            super().__init__(devName=dev_name)
            self.__devAPI:dict =   None
            self.__devINFO:dict =  None
            self._dev_info:dict = None
            self.__wd = None                 # watch dog thread for async operations
            self.__runnerNum:int = None        # runner number assigned to the device
            self.__wd_thread_stop_event = threading.Event()  # event to stop WD thread
            self.__lastCmd:dict = None          # last command executed
            self.success_flag = False          # flag to indicate successful operation


            self.devNotificationQ = Queue()  # notification queue for device events (uses for notification 
                                            # the caller about operation completion in async mode)

            self.__wd_thread_stop_event.set()    # initially stop the thread

            if PLCDev.devsList is None:
                PLCDev.enum_devs(_comADS=_comADS)

            self._dev_idx:int = PLCDev.lookUpDev(self._devName) + 1   # +1 because PLC array is 1-based index

            if PLCDev.__ads is None:
                if _comADS is not None:
                    PLCDev.__ads = _comADS
                    print_log(f'[device {self._devName}] PLCDev initialized with provided commADS instance')
                else:
                    raise Exception(f'[device {self._devName}] PLCDev initialization failed. commADS instance is not provided')     
                
            _tmpINFO = PLCDev.__ads.readVar(symbol_name=f'{symbolsADS._device_access}[{self._dev_idx}]._instanceInfo', var_type=str, size=DEV_INFO_SIZE)
            _tmpAPI = PLCDev.__ads.readVar(symbol_name=f'{symbolsADS._device_access}[{self._dev_idx}]._API', var_type=str, size=DEV_API_SIZE)

            self.__devINFO = (json.loads(_tmpINFO) if _tmpINFO is not None else None) 
            self.__devAPI = (json.loads(_tmpAPI) if _tmpAPI is not None else None)

            print_log(f'[device {self._devName}] PLCDev initialized. Device index = {self._dev_idx}, devAPI size = {len(self.__devAPI) if self.__devAPI is not None else 0}, devINFO size = {len(self.__devINFO) if self.__devINFO is not None else 0} / {self.__devINFO} ')

            if PLCDev.__number_of_runners is None:   # initialize number of runners
                PLCDev.__number_of_runners = PLCDev.__ads.readVar(symbol_name=symbolsADS._max_number_of_runners, var_type=int)
                print_log(f'[device {self._devName}] PLCDev initialized with __number_of_runners = {PLCDev.__number_of_runners}')
            if PLCDev.__runner_factory is None:          # initialize runner factory
                PLCDev.__runner_factory = runnerFactory(num_runners=PLCDev.__number_of_runners)
                print_log(f'[device {self._devName}] PLCDev runnerFactory instance created for {PLCDev.__number_of_runners} runners ')
            PLCDev.__instances += 1
            print_log(f'[device {self._devName}] PLCDev instance created. Total instances = {PLCDev.__instances}')
            

        except Exception as ex:
            exptTrace(ex)   
            print_err(f'[device {self._devName}] PLCDev initialization failed. Exception: {ex}')
            raise ex   
    
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

    @staticmethod
    def enum_devs(_comADS:commADS)-> list[str]:
        
        PLCDev.devsList:list[str] = list()
        try:
            num_of_devs:int = _comADS.readVar(symbol_name=symbolsADS._num_of_devices, var_type=int)
            print_log(f'PLCDev enum_devs: Number of configured devices in PLC = {num_of_devs}')

            for i in range(num_of_devs):
                dev_name:str = _comADS.readVar(symbol_name=f'{symbolsADS._device_access}[{i+1}]._instanceName', var_type=str, size=DEV_NAME_SIZE)
                PLCDev.devsList.append(dev_name)
                print_log(f'PLCDev enum_devs: Device index {i}, name = {dev_name}')

            print_log(f'PLCDev enum_devs: Total enumerated devices = {len(PLCDev.devsList)}: {PLCDev.devsList}')
        
        except Exception as ex:
            exptTrace(ex)
            print_err(f'PLCDev enum_devs: Exception occurred while enumerating devices from PLC. Exception: {ex}')
            raise ex

        return PLCDev.devsList

    @staticmethod
    def lookUpDev(dev_name:str) -> int:
        # Implementation to look up device index by name
        if PLCDev.devsList is None:
            raise Exception(f'PLCDev lookUpDev: Device list is not initialized. Call enum_devs first.')
        try:
            dev_index:int = PLCDev.devsList.index(dev_name)
            return dev_index
        except ValueError:
            raise Exception(f'PLCDev lookUpDev: Device name "{dev_name}" not found in device list.')

    # runDevicesOp runner:int -> bool  -- starts runner operation for device loaded to run for given runner number
    # than oiperates watch dog thread to monitor operation status for each of devices assigned to the runner
    # runner status is handled by by all devices assigned to the runner
    # returns (result:bool, blocked:bool )
    @staticmethod
    def runDevicesOp(runner:int)-> tuple[bool, bool]:
        cmdLst:list = list()
        try:
            if PLCDev.__runner_factory is None:
                raise Exception(f'PLCDev runDevicesOp: Runner factory is not initialized')
            if runner < 1 or runner > PLCDev.__number_of_runners:
                raise Exception(f'PLCDev runDevicesOp: Invalid runner number {runner}. Valid range is 1 to {PLCDev.__number_of_runners}')
            if len(PLCDev.__runner_factory.runnersLst[runner-1]) == 0:
                raise Exception(f'PLCDev runDevicesOp: No devices assigned to runner number {runner}')
            
            for dev in PLCDev.__runner_factory.runnersLst[runner-1]:    # Operate watch dog thread for each device assigned to the runner  
                command:str = dev.lastCmd
                if command is None:
                    raise Exception(f'PLCDev runDevicesOp: No command loaded to device {dev._devName} for runner {runner}')
                
                cmdLst.append(dev.lastCmd)
                

            formatedCmd:str = json.dumps( { "Devices": [ json.loads(cmd) for cmd in cmdLst ] } )

            print_log(f'PLCDev runDevicesOp: Formated command for runner {runner} = {formatedCmd}')

            PLCDev.__ads.writeVar(symbol_name=f'{symbolsADS._runner_array_str}[{runner}].ExecutionInfo', dataToSend = formatedCmd)

            PLCDev.__ads.writeVar(symbol_name=f'{symbolsADS._runner_array_str}[{runner}]._DoLoadInfo', dataToSend = True)
            
            print_log(f'[device {dev._devName}] Loaded ExecutionInfo to PLC for command="{cmdLst}"/"{formatedCmd}", runner={runner}')


            PLCDev.__global_lock.acquire()    # global mutex for runner operation sequence
            PLCDev.__ads.writeVar(symbol_name=f'{symbolsADS._runner_array_str}[{runner}]._DoRun', dataToSend = True)
            PLCDev.__global_lock.release()   # release global mutex for runner operation sequence

            _errorMsg:str = PLCDev.__ads.readVar(symbol_name=f'{symbolsADS._runner_array_str}[{runner}]._errorMessage', var_type=str, size=256)
            # BUGBUG // check if read will work right after write above
            if _errorMsg != '':
                raise Exception(f'PLCDev runDevicesOp: Starting runner {runner} failed with error: {_errorMsg}')



            print_log(f'PLCDev runDevicesOp: Starting runner operation for runner number {runner} with {len(PLCDev.__runner_factory.runnersLst[runner-1])} devices assigned')
            for dev in PLCDev.__runner_factory.runnersLst[runner-1]:    # Operate watch dog thread for each device assigned to the runner  
                print_log(f'PLCDev runDevicesOp: Starting watch dog thread for device {dev._devName} on runner {runner}')
                dev.runWDThread()
            
        except Exception as ex:
            exptTrace(ex)
            print_log(f'PLCDev runDevicesOp: Error occurred while starting runner {runner}. Stopping all devices from the runner.')
            for dev in PLCDev.__runner_factory.runnersLst[runner-1]:    # stop all devices assigned to the runner
                try:                    # stop watch dog thread if running
                    if dev.__wd is not None and dev.__wd.is_alive():  # if watch dog thread is running
                        dev.stop()              # stop watch dog thread
                        print_log(f'[device {dev._devName}] PLCDev runDevicesOp: Waiting for watch dog thread to end...')
                        dev.__wd.join(timeout=0.5)   # wait for thread to end
                        if dev.__wd.is_alive():
                            print_err(f'[device {dev._devName}] PLCDev runDevicesOp: Watch dog thread did not end within timeout')
                        else:
                            print_log(f'[device {dev._devName}] PLCDev runDevicesOp: Watch dog thread ended successfully')
                except Exception as ex_detach:
                    exptTrace(ex_detach)
                    print_err(f'[device {dev._devName}] PLCDev runDevicesOp: Exception occurred while stopping watch dog thread. Exception: {ex_detach}')

            return False, False
        
        return True, True

    # loadDeviceOp command: str -> (blocked:bool, result:bool, runner:int)
    # blocked - True if the device is in async mode and notification should be awaited
    # result - True if the command run was successful (in case of synchronous mode) or operation started successfully (in case of async mode)
    
    def loadDeviceOp(self, command:str, runnerNum = None)-> int:
        try:
            
            self.__wd = None
            # if runnerNum is None:
            #     runnerNum = PLCDev.__runner_factory._runners_pool.alloc()
            #     if runnerNum is None:
            #         raise Exception(f'[device {self._devName}] PLCDev loadDeviceOp: No available runners to load device command for command="{command}"')
            #     print_log(f'[device {self._devName}] PLCDev loadDeviceOp: Allocated runner number {runnerNum} for command="{command}"') 

            send_exData = self.__parseCMD(command)
            if send_exData is None:
                print_err(f'[device {self._devName}] PLCDev loadDeviceOp: command parsing failed for command="{command}"')
                return runnerNum

            self.__lastCmd = json.dumps(send_exData)
            print_log(f'[device {self._devName}] Adding ExecutionInfo = {self.__lastCmd} to PLC for command="{command}"  ')
                                                                        # load ExecutionInfo to PLC


        except Exception as ex:
            print_log(f'[device {self._devName}] Error in loadDeviceOp for command="{command}"')
            exptTrace(ex)   
            raise  ex   
            
        runnerNum = PLCDev.__runner_factory.attachDeviceToRunner(self, indx=runnerNum)   # attach device to runner
        
        self.__runnerNum = runnerNum
        return runnerNum
    
    def __parseCMD(self, cmd:str)-> dict | None:
        # BUGBUG: implement command parsing here
        if cmd is not None:
            try:
                cmdData:dict = json.loads(cmd)
                return cmdData
            except Exception as ex:
                exptTrace(ex)
                return None
            
        return None
 
    # operateDevice -- runs loadDeviceOp and runDevicesOp in sequence
    def operateDevice(self, command:str, **kwargs)-> tuple[bool, bool]:
        toBlock:bool = True 
        opResult:bool = True
        try:
            runnerNum:int = self.loadDeviceOp(command=command)
            print_log(f'[device {self._devName}] operateDevice: Device command loaded at runner = {runnerNum} for command="{command}"')
            if runnerNum is None:
                raise Exception(f'[device {self._devName}] operateDevice: loadDeviceOp failed for command="{command}"')
            
            opResult, toBlock = PLCDev.runDevicesOp(runner=runnerNum)
            print_log(f'[device {self._devName}] operateDevice: Device command loaded and run at runner = {runnerNum}')
    
        except Exception as ex:
            exptTrace(ex)
            return False, False
    
        return opResult, toBlock
    

    def runWDThread(self) -> bool:
        try:
            if  self.__wd is not None and self.__wd.is_alive():
                raise Exception(f'[device {self._devName}] Watch dog thread is already running')
            self.__wd = threading.Thread(target = self._watch_dog_thread)
            self.__wd_thread_stop_event.clear()  # allow thread to run
            self.success_flag = True   # assume success unless error occurs
            self.__wd.start()
    
        except Exception as ex:
            exptTrace(ex)
            raise ex
    
        return True
    
        

    def _watch_dog_thread(self):
        
        try:
            print_log(f'[device {self._devName}] Watch dog thread is alive... runner = {self.__runnerNum}')
            
            if PLCDev.__ads is None:
                raise Exception(f'[device {self._devName}] ADS ERROR: PLC connection is not established in watch dog thread')
                
            
            while not self.__wd_thread_stop_event.is_set():  # main watch dog loop
                exStatus:int = PLCDev.__ads.readVar(symbol_name=f'{symbolsADS._runner_array_str}[{self.__runnerNum}].eExecutionStatus', \
                                                var_type=int)
                devState:int = PLCDev.__ads.readVar(symbol_name=f'{symbolsADS._device_access}[{self._dev_idx}].eState', \
                                                var_type=int)   
                __jsonINFO = PLCDev.__ads.readVar(symbol_name=f'{symbolsADS._device_access}[{self._dev_idx}]._instanceInfo', \
                                                var_type=str, size=1024)   
                if self.__devINFO is not None:      # update device info
                    self.__devINFO |= (json.loads(__jsonINFO) if __jsonINFO is not None else dict())
                else:               # set device info   
                    self.__devINFO = (json.loads(__jsonINFO) if __jsonINFO is not None else None)
                
                # print_DEBUG(f'[device {self._devName}] ExecutionStatus = {STATUS(exStatus)} ({exStatus}) for runner = {self.__runnerNum}, DeviceState = {EN_DeviceCoreState(devState).name} ({devState})')
                print_DEBUG(f'[device {self._devName}]  S:{exStatus} r:{self.__runnerNum}dev state:{devState} INFO={self.__devINFO}')
                
                if exStatus == STATUS.DONE.value:   # Completed
                    print_DEBUG(f'[device {self._devName}] at runner={self.__runnerNum} completed successfully')
                    # break
                elif exStatus == STATUS.READY.value:   # Completed
                    print_DEBUG(f'[device {self._devName}] at runner={self.__runnerNum} was stoped by external request')
                    # break
                elif exStatus == STATUS.ERROR.value:   # Error
                    _errorMsg:str = PLCDev.__ads.readVar(symbol_name=f'{symbolsADS._runner_array_str}[{self.__runnerNum}]._errorMessage', \
                                                        var_type=str, size=256)
                    print_DEBUG(f'[device {self._devName}] runner={self.__runnerNum} ended with ERROR: {_errorMsg}')
                    
                    # break
                
                if devState == EN_DeviceCoreState.ERROR.value:   #    Device in error state
                    _errorMsg:str = PLCDev.__ads.readVar(symbol_name=f'{symbolsADS._device_access}[{self._dev_idx}]._errorMessage', \
                                                        var_type=str, size=256)

                    print_log(f'[device {self._devName}] ERROR: Device entered ERROR state during runner={self.__runnerNum}. Error meassage = {_errorMsg}')
                    self.success_flag = False
                    break  
                elif devState == EN_DeviceCoreState.DONE.value:   #    Device in error state
                    print_log(f'[device {self._devName}] Device entered DONE state during runner={self.__runnerNum}')
                    self.success_flag = True
                    PLCDev.__ads.writeVar(symbol_name=f'{symbolsADS._device_access}[{self._dev_idx}]._DoAck', dataToSend = True)
                    break  
                elif devState == EN_DeviceCoreState.READY.value:   #    Device in error state
                    print_log(f'[device {self._devName}] Device entered READY state during runner={self.__runnerNum}.')
                    self.success_flag = True
                    break 
                elif devState != EN_DeviceCoreState.RUN.value:   #    Device in error state
                    _errorMsg:str = PLCDev.__ads.readVar(symbol_name=f'{symbolsADS._device_access}[{self._dev_idx}]._errorMessage', \
                                                        var_type=str, size=256)
                    print_log(f'[device {self._devName}] ERROR: Device in non RUN state = ({EN_DeviceCoreState(devState).name}) runner={self.__runnerNum}. {"Error="+_errorMsg if _errorMsg != "" else ""}')
                    self.success_flag = False
                    break 
                
                time.sleep(0.5)
                    
           
        except Exception as ex:
            exptTrace(ex)   
            self.success_flag = False
        
        if self.__wd_thread_stop_event.is_set():        # the stop is applyed explicetly 
            print_log (f'[device {self._devName}] Watch dog thread for devices on runner = {self.__runnerNum} is stopping...')
        else:
            self.__wd_thread_stop_event.set()    # set the stop event if exiting normally

        _runners_left = PLCDev.__runner_factory.detachDeviceFromRunner(self, self.__runnerNum)
        if _runners_left == 0:          # last device on the runner
            print_log (f'[device {self._devName}] Last device on runner = {self.__runnerNum} has completed operation.') 
            PLCDev.__ads.writeVar(symbol_name=f'{symbolsADS._runner_array_str}[{self.__runnerNum}]._DoAck', dataToSend = True)
                                        # acknowledge runner operation completion in PLC

        print_log (f'[device {self._devName}] Watch dog thread for devices on runner = {self.__runnerNum} has ended')
        print_log (f'[device {self._devName}] Info = {self.__devINFO}')

        # BUGBUG: clean up if last device in runner
        self.devNotificationQ.put(self.success_flag)
        self.__lastCmd = None
        self.__wd = None

        return
    
    def stop(self) -> bool:
        try:
            PLCDev.__ads.writeVar(symbol_name=f'{symbolsADS._device_access}[{self._dev_idx}]._DoStop', dataToSend = True)
            # stop device operation in PLC
        except Exception as ex:
            exptTrace(ex)
            raise ex
        
        # self.__wd_thread_stop_event.set()   # signal the watch dog thread to stop
        return True

    def devQuery(self, query:str, timeout:float=0)-> str:
        return ''


    @property
    def devINFO(self)-> dict:
        return self.__devINFO

    @property
    def lastCmd(self)-> dict:
        return self.__lastCmd 
    
    # @property
    # def runnerNum(self)-> int:
    #     return self.__runnerNum
    

    

    ############  UNIT TEST  ##############
if __name__ == "__main__":
    # testData:dict = \
    #     {
    #         "Devices": [
    #                     {
    #                         "InstanceName": "Stages_AxisX",
    #                         "TaskName": "Move Absolute",
    #                         "TaskParams": "{\"Pos\":0}",
    #                         "IsBreak": False
    #                     },
    #                     {
    #                         "InstanceName": "FrontStage_AxisY",
    #                         "TaskName": "Move Absolute",
    #                         "TaskParams": "{\"Pos\":0}",
    #                         "IsBreak": False
    #                     }
    #             ]
    #     }

    '''
    REMOTE_IP = '192.168.10.92'
    AMS_NETID = '192.168.10.92.1.1'
    '''

    REMOTE_IP = '192.168.10.153'
    AMS_NETID = '192.168.137.1.1.1'


    _ads = commADS(AMS_NETID, REMOTE_IP )


    # testData:dict = {
    #     "InstanceName": "Stages_AxisX",
    #     "TaskName": "Move Absolute",
    #     "TaskParams": "{\"Pos\":0.12}",
    #     "IsBreak": False
    # }

    # testData2:dict = {
	# 		"InstanceName": "FrontStage_AxisY",
	# 		"TaskName": "Move Absolute",
	# 		"TaskParams": "{\"Pos\":8.34}",
	# 		"IsBreak": False
	# }



    testData:dict = {
        "InstanceName": "Single Axis",
        "TaskName": "Move Relative",
        "TaskParams": "{\"Dis\":100}",
        "IsBreak": False
    }
    testData2:dict =  {
        "InstanceName": "Single Axis 2",
        "TaskName": "Move Relative",
        "TaskParams": "{\"Dis\":200}",
        "IsBreak": False
    }

    testCMD:str = json.dumps(testData)
    testCMD2:str = json.dumps(testData2)
    print("[UNITEST]PLCDev module unit test")
    # testDev = PLCDev(dev_name='FronStage_AxisPCB',  _comADS=_ads)
    # testDev2 = PLCDev(dev_name='BackStage_AxisPCB',  _comADS=_ads)

    testDev = PLCDev(dev_name='Single Axis',  _comADS=_ads)
    testDev2 = PLCDev(dev_name='Single Axis 2',  _comADS=_ads)


    runnerNum = testDev.loadDeviceOp(command=testCMD)
    runnerNum = testDev2.loadDeviceOp(command=testCMD2, runnerNum=runnerNum)
    print(f'[UNITEST]Device command loaded at runner = {runnerNum} for command="{testCMD}"')
    toBlock, opResult = PLCDev.runDevicesOp(runner=runnerNum)
    print(f'[UNITEST]Runner started: toBlock={toBlock}, opResult={opResult}. Waiting for completion ...')    
    if toBlock:
        opResult = testDev.devNotificationQ.get()         # the ONLY block untill completed
        testDev.devNotificationQ.task_done()

    print(f'[UNITEST]Device operation completed. Device = {testDev._devName} + {testDev2._devName}. Result = {opResult}')

        