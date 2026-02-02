__author__ = "Leonid Voldman"
__created_on__ = "2025-10-06"  
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "1.0.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Tool"

import sys
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

    def attachNodeToRunner(self, dev:BaseDev, indx:int | None = None)-> int:
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
    
    def detachNodeFromRunner(self, dev:BaseDev, indx:int)-> int:
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
    def runnersLst(self)-> list[list]:
        return self.__runners_lst    

class PLCNode(BaseDev):
    nodesList:list[str] | None = None
    __global_lock:Lock = Lock()
    __number_of_runners:int | None = None
    __runner_factory:runnerFactory | None = None
    __ads:commADS | None = None
    __instances:int = 0

    def __init__(self, dev_name:str, plc_dev_name:str, _comADS:commADS | None = None):
        try:
            super().__init__(devName=dev_name, parms=None)
            self._plcNodeName:str = plc_dev_name
            self.__devAPI:dict | None =   None
            self.__devINFO:dict | None =  None
            self._dev_info:dict | None = None
            self.__runnerNum:int | None = None        # runner number assigned to the device
            self.__wd_thread_stop_event = threading.Event()  # event to stop WD thread
            self.__lastCmd:str | None = None          # last command executed
            self.success_flag = False          # flag to indicate successful operation

            self.__wd_thread_stop_event.set()    # initially stop the thread
            PLCNode.__instances += 1

            assert _comADS is not None, 'PLCNode initialization failed: commADS instance is required'

            if PLCNode.nodesList is None:
                PLCNode.enum_nodes(_comADS=_comADS)

            self._dev_idx:int = PLCNode.lookUpDev(self._plcNodeName) + 1   # +1 because PLC array is 1-based index

            if PLCNode.__ads is None:
                if _comADS is not None:
                    PLCNode.__ads = _comADS
                    print_log(f'[device {self._devName}] PLCNode {self._plcNodeName} initialized with provided commADS instance')
                else:
                    raise Exception(f'[device {self._devName}] PLCNode {self._plcNodeName} initialization failed. commADS instance is not provided')     
            else:
                print_log(f'[device {self._devName}] PLCNode {self._plcNodeName} initialized with existing commADS instance')

                
                                                        # read device info and API from PLC
            _tmpINFO = PLCNode.__ads.readVar(symbol_name=f'{symbolsADS._device_access}[{self._dev_idx}]._instanceInfo', var_type=str, size=DEV_INFO_SIZE)
            _tmpAPI  = PLCNode.__ads.readVar(symbol_name=f'{symbolsADS._device_access}[{self._dev_idx}]._API', var_type=str, size=DEV_API_SIZE)
            
            print_log(f'[device {self._devName}] PLCNode {self._plcNodeName} read _instanceInfo and _API from PLC for device index = {self._dev_idx}: ')
            
            # print_DEBUG(f'[device {self._devName}] _instanceInfo = {_tmpINFO}')
            # print_DEBUG(f'[device {self._devName}] _API = {_tmpAPI}') 

            self.__devINFO = (json.loads(str(_tmpINFO)) if (_tmpINFO is not None and len(str(_tmpINFO)) > 0) else None) 
            self.__devAPI = (json.loads(str(_tmpAPI)) if (_tmpAPI is not None and len(str(_tmpAPI)) > 0) else None)

            print_DEBUG(f'[device {self._devName}] {self._plcNodeName}: _instanceInfo = {json.dumps(self.__devINFO, indent=1)}')
            print_DEBUG(f'[device {self._devName}] {self._plcNodeName}: _API = {json.dumps(self.__devAPI, indent=1)}') 


            print_log(f'[device {self._devName}] PLCNode {self._plcNodeName} initialized. Device index = {self._dev_idx}, devAPI size = {len(self.__devAPI) if self.__devAPI is not None else 0} / {self.__devAPI}, devINFO size = {len(self.__devINFO) if self.__devINFO is not None else 0} / {self.__devINFO} ')

            if PLCNode.__number_of_runners is None:   # initialize number of runners
                no_runners = PLCNode.__ads.readVar(symbol_name=symbolsADS._max_number_of_runners, var_type=int)
                PLCNode.__number_of_runners = int(no_runners) if no_runners is not None else 0
                print_log(f'[device {self._devName}] PLCNode {self._plcNodeName} initialized with __number_of_runners = {PLCNode.__number_of_runners}')
            if PLCNode.__runner_factory is None:          # initialize runner factory
                PLCNode.__runner_factory = runnerFactory(num_runners=PLCNode.__number_of_runners)
                print_log(f'[device {self._devName}] PLCNode {self._plcNodeName} runnerFactory instance created for {PLCNode.__number_of_runners} runners ')
            print_log(f'[device {self._devName}] PLCNode {self._plcNodeName} instance created. Total instances = {PLCNode.__instances}')
            

        except Exception as ex:
            exptTrace(ex)   
            print_err(f'[device {self._devName}] PLCNode {self._plcNodeName} initialization failed. Exception: {ex}')
            raise ex   
    
    def __del__(self):
        try:
            PLCNode.__instances -= 1
            if self._wd is not None and self._wd.is_alive():
                print_log(f'[device {self._devName}] PLCNode {self._plcNodeName} __del__: Waiting for watch dog thread to end...')
                self._wd.join(timeout=5)
                if self._wd.is_alive():
                    print_err(f'[device {self._devName}] PLCNode {self._plcNodeName} __del__: Watch dog thread did not end within timeout')
                else:
                    print_log(f'[device {self._devName}] PLCNode {self._plcNodeName} __del__: Watch dog thread ended successfully')
            
            print_log(f'[device {self._devName}] PLCNode {self._plcNodeName} instance deleted. Total remaining instances = {PLCNode.__instances}')
            if PLCNode.__instances == 0:
                del PLCNode.__ads
                del PLCNode.__runner_factory
                PLCNode.__ads = None
                PLCNode.__runner_factory = None  


        except Exception as ex:
            exptTrace(ex)
    '''
    enum_nodes method enumerates available devices from PLC via commADS instance
    '''
    @classmethod
    def enum_nodes(cls, _comADS:commADS)-> list[str]:
        cls.nodesList = list()   # initialize device list 
        try:
            num_of_devs:int = _comADS.readVar(symbol_name=symbolsADS._num_of_devices, var_type=int)
                                            # actual number of devices configured in PLC
            print_log(f'PLCNode enum_nodes: Number of configured devices in PLC = {num_of_devs}')

            for i in range(num_of_devs):
                dev_name:str = _comADS.readVar(symbol_name=f'{symbolsADS._device_access}[{i+1}]._instanceName', var_type=str, size=DEV_NAME_SIZE)
                                                    # read device name from PLC (1-based index)
                cls.nodesList.append(dev_name)
                print_log(f'PLCNode enum_nodes: Device index {i}, name = {dev_name}')

            print_log(f'PLCNode enum_nodes: Total enumerated devices = {len(cls.nodesList)}: {cls.nodesList}')
        
        except Exception as ex:
            exptTrace(ex)
            print_err(f'PLCNode enum_nodes: Exception occurred while enumerating devices from PLC. Exception: {ex}')
            raise ex

        return cls.nodesList

    @classmethod
    def lookUpDev(cls, dev_name:str) -> int:
        # Implementation to look up device index by name
        if cls.nodesList is None:
            raise Exception(f'PLCNode lookUpDev device {dev_name}: Device list is not initialized. Call enum_nodes first.')
        try:
            dev_index:int = cls.nodesList.index(dev_name)
            print_log(f'PLCNode lookUpDev device {dev_name}: Found device at index {dev_index}')
            return dev_index
        except ValueError:
            raise Exception(f'PLCNode lookUpDev device {dev_name}: Device name not found in device list: {cls.nodesList}')

    # runNodesOp runner:int -> bool  -- starts runner operation for device loaded to run for given runner number
    # than oiperates watch dog thread to monitor operation status for each of devices assigned to the runner
    # runner status is handled by by all devices assigned to the runner
    # returns (result:bool, blocked:bool )
    @classmethod
    def runNodesOp(cls, runner:int)-> tuple[bool, bool]:
        cmdLst:list = list()
        try:
            if cls.__runner_factory is None:
                raise Exception(f'PLCNode runNodesOp: Runner factory is not initialized')
            if runner < 1 or runner > cls.__number_of_runners:
                raise Exception(f'PLCNode runNodesOp: Invalid runner number {runner}. Valid range is 1 to {cls.__number_of_runners}')
            if len(cls.__runner_factory.runnersLst[runner-1]) == 0:
                raise Exception(f'PLCNode runNodesOp: No devices assigned to runner number {runner}')
            
            for dev in cls.__runner_factory.runnersLst[runner-1]:    # Operate watch dog thread for each device assigned to the runner  
                command:str = dev.lastCmd
                if command is None:
                    raise Exception(f'PLCNode runNodesOp: No command loaded to device {dev._devName} for runner {runner}')
                
                cmdLst.append(dev.lastCmd)
                

            formatedCmd:str = json.dumps( { "Devices": [ json.loads(cmd) for cmd in cmdLst ] } )

            print_log(f'PLCNode runNodesOp: Formated command for runner {runner} = {formatedCmd}')

            cls.__ads.writeVar(symbol_name=f'{symbolsADS._runner_array_str}[{runner}].ExecutionInfo', dataToSend = formatedCmd)

            cls.__ads.writeVar(symbol_name=f'{symbolsADS._runner_array_str}[{runner}]._DoLoadInfo', dataToSend = True)
            
            print_log(f'[device {dev._devName}] Loaded ExecutionInfo to PLC for command="{cmdLst}"/"{formatedCmd}", runner={runner}')


            cls.__global_lock.acquire()    # global mutex for runner operation sequence
            cls.__ads.writeVar(symbol_name=f'{symbolsADS._runner_array_str}[{runner}]._DoRun', dataToSend = True)
            cls.__global_lock.release()   # release global mutex for runner operation sequence
            _errorMsg:str = cls.__ads.readVar(symbol_name=f'{symbolsADS._runner_array_str}[{runner}]._errorMessage', var_type=str, size=256)
            # BUGBUG // check if read will work right after write above
            if _errorMsg != '':
                raise Exception(f'PLCNode runNodesOp: Starting runner {runner} failed with error: {_errorMsg}')



            print_log(f'PLCNode runNodesOp: Starting runner operation for runner number {runner} with {len(cls.__runner_factory.runnersLst[runner-1])} devices assigned')
            for dev in cls.__runner_factory.runnersLst[runner-1]:    # Operate watch dog thread for each device assigned to the runner  
                print_log(f'PLCNode runNodesOp: Starting watch dog thread for device {dev._devName} on runner {runner}')
                dev.runWDThread()
            
        except Exception as ex:
            exptTrace(ex)
            print_log(f'PLCNode runNodesOp: Error occurred while starting runner {runner}. Stopping all devices from the runner.')
            for dev in cls.__runner_factory.runnersLst[runner-1]:    # stop all devices assigned to the runner
                try:                    # stop watch dog thread if running
                    if dev._wd is not None and dev._wd.is_alive():  # if watch dog thread is running
                        dev.stop()              # stop watch dog thread
                        print_log(f'[device {dev._devName}] PLCNode runNodesOp: Waiting for watch dog thread to end...')
                        dev._wd.join(timeout=0.5)   # wait for thread to end
                        if dev._wd.is_alive():
                            print_err(f'[device {dev._devName}] PLCNode runNodesOp: Watch dog thread did not end within timeout')
                        else:
                            print_log(f'[device {dev._devName}] PLCNode runNodesOp: Watch dog thread ended successfully')
                except Exception as ex_detach:
                    exptTrace(ex_detach)
                    print_err(f'[device {dev._devName}] PLCNode runNodesOp: Exception occurred while stopping watch dog thread. Exception: {ex_detach}')

            return False, False
        
        return True, True

    # loadNodeOp command: str -> (blocked:bool, result:bool, runner:int)
    # blocked - True if the device is in async mode and notification should be awaited
    # result - True if the command run was successful (in case of synchronous mode) or operation started successfully (in case of async mode)
    
    def loadNodeOp(self, command:str, runnerNum:int | None = None)-> int | None:
        try:
            
            self._wd = None
            # if runnerNum is None:
            #     runnerNum = PLCNode.__runner_factory._runners_pool.alloc()
            #     if runnerNum is None:
            #         raise Exception(f'[device {self._devName}] PLCNode loadNodeOp: No available runners to load device command for command="{command}"')
            #     print_log(f'[device {self._devName}] PLCNode loadNodeOp: Allocated runner number {runnerNum} for command="{command}"') 

            send_exData = self.__parseCMD(command)
            if send_exData is None:
                print_err(f'[device {self._devName}] PLCNode loadNodeOp: command parsing failed for command="{command}"')
                return runnerNum

            self.__lastCmd = json.dumps(send_exData)
            print_log(f'[device {self._devName}] Adding ExecutionInfo = {self.__lastCmd} to PLC for command="{command}"  ')
                                                                        # load ExecutionInfo to PLC


        except Exception as ex:
            print_log(f'[device {self._devName}] Error in loadNodeOp for command="{command}"')
            exptTrace(ex)   
            raise  ex   
            
        runnerNum = PLCNode.__runner_factory.attachNodeToRunner(self, indx=runnerNum) if PLCNode.__runner_factory  else 0   # attach device to runner
        
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
 
    # operateNode -- runs loadNodeOp and runNodesOp in sequence
    def operateNode(self, command:str, **kwargs)-> tuple[bool, bool]:
        toBlock:bool = True 
        opResult:bool = True
        try:
            runnerNum:int = self.loadNodeOp(command=command)
            print_log(f'[device {self._devName}] operateNode: Device command loaded at runner = {runnerNum} for command="{command}"')
            if runnerNum is None:
                raise Exception(f'[device {self._devName}] operateNode: loadNodeOp failed for command="{command}"')
            
            opResult, toBlock = PLCNode.runNodesOp(runner=runnerNum)
            print_log(f'[device {self._devName}] operateNode: Device command loaded and run at runner = {runnerNum}')
    
        except Exception as ex:
            exptTrace(ex)
            return False, False
    
        return opResult, toBlock
    

    def runWDThread(self) -> bool:
        try:
            if  self._wd is not None and self._wd.is_alive():
                raise Exception(f'[device {self._devName}] Watch dog thread is already running')
            self._wd = threading.Thread(target = self._watch_dog_thread)
            self.__wd_thread_stop_event.clear()  # allow thread to run
            self.success_flag = True   # assume success unless error occurs
            self._wd.start()
    
        except Exception as ex:
            exptTrace(ex)
            raise ex
    
        return True
    
        

    def _watch_dog_thread(self):
        
        try:
            print_log(f'[device {self._devName}] Watch dog thread is alive... runner = {self.__runnerNum}')
            
            if PLCNode.__ads is None:
                raise Exception(f'[device {self._devName}] ADS ERROR: PLC connection is not established in watch dog thread')
                
            
            while not self.__wd_thread_stop_event.is_set():  # main watch dog loop
                exStatus:int = PLCNode.__ads.readVar(symbol_name=f'{symbolsADS._runner_array_str}[{self.__runnerNum}].eExecutionStatus', \
                                                var_type=int)
                devState:int = PLCNode.__ads.readVar(symbol_name=f'{symbolsADS._device_access}[{self._dev_idx}].eState', \
                                                var_type=int)   
                __jsonINFO = PLCNode.__ads.readVar(symbol_name=f'{symbolsADS._device_access}[{self._dev_idx}]._instanceInfo', \
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
                    _errorMsg:str = PLCNode.__ads.readVar(symbol_name=f'{symbolsADS._runner_array_str}[{self.__runnerNum}]._errorMessage', \
                                                        var_type=str, size=256)
                    print_DEBUG(f'[device {self._devName}] runner={self.__runnerNum} ended with ERROR: {_errorMsg}')
                    
                    # break
                
                if devState == EN_DeviceCoreState.ERROR.value:   #    Device in error state
                    _errorMsg:str = PLCNode.__ads.readVar(symbol_name=f'{symbolsADS._device_access}[{self._dev_idx}]._errorMessage', \
                                                        var_type=str, size=256)

                    print_err(f'[device {self._devName}] ERROR: Device entered ERROR state during runner={self.__runnerNum}. Error meassage = {_errorMsg}')
                    self.success_flag = False
                    break  
                elif devState == EN_DeviceCoreState.DONE.value:   #    Device in error state
                    print_log(f'[device {self._devName}] Device entered DONE state during runner={self.__runnerNum}')
                    self.success_flag = True
                    PLCNode.__ads.writeVar(symbol_name=f'{symbolsADS._device_access}[{self._dev_idx}]._DoAck', dataToSend = True)
                    break  
                elif devState == EN_DeviceCoreState.READY.value:   #    Device in error state
                    print_log(f'[device {self._devName}] Device entered READY state during runner={self.__runnerNum}.')
                    self.success_flag = True
                    break 
                elif devState != EN_DeviceCoreState.RUN.value:   #    Device in error state
                    _errorMsg:str = PLCNode.__ads.readVar(symbol_name=f'{symbolsADS._device_access}[{self._dev_idx}]._errorMessage', \
                                                        var_type=str, size=256)
                    print_err(f'[device {self._devName}] ERROR: Device in non RUN state = ({EN_DeviceCoreState(devState).name}) runner={self.__runnerNum}. {"Error="+_errorMsg if _errorMsg != "" else ""}')
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

        _runners_left = PLCNode.__runner_factory.detachNodeFromRunner(self, self.__runnerNum)
        if _runners_left == 0:          # last device on the runner
            print_log (f'[device {self._devName}] Last device on runner = {self.__runnerNum} has completed operation.') 
            PLCNode.__ads.writeVar(symbol_name=f'{symbolsADS._runner_array_str}[{self.__runnerNum}]._DoAck', dataToSend = True)
                                        # acknowledge runner operation completion in PLC

        print_log (f'[device {self._devName}] Watch dog thread for devices on runner = {self.__runnerNum} has ended')
        print_log (f'[device {self._devName}] Info = {self.__devINFO}')

        # BUGBUG: clean up if last device in runner
        self.devNotificationQ.put(self.success_flag)
        self.__lastCmd = None
        self._wd = None

        return
    
    def stop(self) -> bool:
        try:
            PLCNode.__ads.writeVar(symbol_name=f'{symbolsADS._device_access}[{self._dev_idx}]._DoStop', dataToSend = True)
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

    
    # REMOTE_IP = '192.168.10.92'
    # AMS_NETID = '192.168.10.92.1.1'
    

    # REMOTE_IP = '192.168.10.153'
    # AMS_NETID = '192.168.137.1.1.1'

    # REMOTE_IP = '192.168.10.96'
    # AMS_NETID = '192.168.10.96.1.1'

    REMOTE_IP = '192.168.10.172'
    AMS_NETID = '192.168.10.153.1.1'


    _ads = commADS(AMS_NETID, REMOTE_IP )
    _devs:list[PLCNode] = list()

    devLst:list = PLCNode.enum_nodes(_comADS=_ads)
    for i, devName in enumerate(devLst):
        print(f'[UNITEST]Creating PLCNode instance for device {i}: {devName}')
        _tmpDev = PLCNode(dev_name=f'Device_{i}', plc_dev_name=devName,  _comADS=_ads)
        _devs.append(_tmpDev)   


    sys.exit(0)

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
    print("[UNITEST]PLCNode module unit test")
    # testDev = PLCNode(dev_name='FronStage_AxisPCB',  _comADS=_ads)
    # testDev2 = PLCNode(dev_name='BackStage_AxisPCB',  _comADS=_ads)

    testDev = PLCNode(dev_name='Single Axis',  _comADS=_ads)
    testDev2 = PLCNode(dev_name='Single Axis 2',  _comADS=_ads)


    runnerNum = testDev.loadNodeOp(command=testCMD)
    runnerNum = testDev2.loadNodeOp(command=testCMD2, runnerNum=runnerNum)
    print(f'[UNITEST]Device command loaded at runner = {runnerNum} for command="{testCMD}"')
    toBlock, opResult = PLCNode.runNodesOp(runner=runnerNum)
    print(f'[UNITEST]Runner started: toBlock={toBlock}, opResult={opResult}. Waiting for completion ...')    
    if toBlock:
        opResult = testDev.devNotificationQ.get()         # the ONLY block untill completed
        testDev.devNotificationQ.task_done()

    print(f'[UNITEST]Device operation completed. Device = {testDev._devName} + {testDev2._devName}. Result = {opResult}')

        