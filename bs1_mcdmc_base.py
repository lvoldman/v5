__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "1.0.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Base Tool"

import mecademicpy.robot as mdr
import mecademicpy.mx_robot_def as mdr_def
import mecademicpy.robot_initializer as initializer
import mecademicpy.tools as tools
import mecademicpy.robot_classes as robot_classes

from abc import ABC, abstractmethod


from enum import Enum
from collections.abc import Callable

import time, re
import os.path
from collections import namedtuple

import logging, sys, datetime, yaml
import threading
from threading import Lock
from queue import Queue 
from enum import Enum



from mecademic_error import mecademicErrorMsg

from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, set_parm, \
                      get_parm, unsigned_16, assign_parm

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from bs2_config import DevType

STATUS_RESOLUTION = 2.0            # seconds between status updates

robotOpType = Enum("robotOpType", ["moveabs", "movesafe", "moverel", "moveup",  "pickup"])

contexFuncParmsFields = ["point", "coord", "velocity"]
contexFuncParms = namedtuple("contexFuncParms", contexFuncParmsFields, defaults=[None,] * len(contexFuncParmsFields))
contexFuncParms__annotations__={'point':str,  'coord':namedtuple, "velocity":int} 

SYNC_MODE =  True                       # syncronious mode enabled
DISCONNECT_MODE = False                 # don't disconnect on exception

class baseRobot(ABC):
    def __init__(self, devName,  IP, parms):
        self.devName = devName
        self.IPadr = IP
        self.robot:mdr.Robot = mdr.Robot()
        self.model:str = None
        self.robot_model:mdr.MxRobotModel = None
        self._stored_online_status = False
        self._last_status_update_time = 0
        self._last_pos_update_time = 0
        self.dev_lock =  Lock()                 # device lock to avoid multiply access
        self.devNotificationQ = Queue()
        self.type = self.robot.GetRobotInfo().model
        self.__3Dcoord:dict = None
        self.__3Doffset:dict = None


    def __del__(self):
        self._stop_clear()
        if self.robot is not None:
            del self.robot
            self.robot = None

    def _init(self):
        self.callbacks = mdr.RobotCallbacks()
        self.callbacks.on_error = self._errorCallback
        self.callbacks.on_connected = self._on_connectedCallback
        self.callbacks.on_disconnected = self._on_disconnectedCallback
        self.callbacks.on_activated = self._on_activatedCallback
        self.callbacks.on_deactivated = self._on_deactivatedCallback
        self.callbacks.on_homed = self._on_homedCallback
        self.robot.RegisterCallbacks(callbacks=self.callbacks, run_callbacks_in_separate_thread=True)

    def _connect(self, IP_adr:str = '192.168.0.100') -> bool:
        try:
            self.robot.Connect(address=IP_adr, enable_synchronous_mode = SYNC_MODE, \
                        disconnect_on_exception=DISCONNECT_MODE)
            self.model = self.robot.GetRobotInfo().model
            self.robot_model = self.robot.GetRobotInfo().robot_model
            print_log(f'Robot Model = {self.model}/{self.robot_model}')
        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_log(f'Connection  failed: {ex}')
            return False
        
        return True

    def _activate(self) -> bool:
        try:
            self.robot.ActivateRobot()
            self.robot.Home()
            self.robot.SetConfTurn(0)
            self.robot.SetJointAcc(self.JOINT_ACC)
            self._getJointAcc()


        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_log(f'Activation  failed: {ex}')
            return False
        
        return True


    def _stop_clear(self):
        try:
            
            if not SYNC_MODE:
                print_log(f'Waiting to Idle')
                self.robot.WaitIdle()

            print_log(f'Deactivating')
            self.robot.DeactivateRobot()

            if not SYNC_MODE:
                print_log(f'Waiting for Deactivation')
                self.robot.WaitDeactivated()

            print_log(f'Disconnect now')
            self.robot.Disconnect()
            
        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_log(f'Stoping/clearing proc failed: {ex}')

    def _errorRecovery(self):                       # in general singularity is fixed right here
        pass

    def _errorCallback(self):
        robot_status = self.robot.GetStatusRobot(synchronous_update = True)
        # print_log(f'Error callback... error_status = {robot_status.error_status}, robot_status.error_code ={robot_status.error_code} ')
        print_log(f'Error callback... error_status = {robot_status.error_status} ')
        if robot_status.error_status:
                    # print_log(f'Error detected. Code = {robot_status.error_code} / {mecademicErrorMsg(int(robot_status.error_code))} ')
                    print_log(f'Error detected. Code = {robot_status.error_code}  ')
                    self.robot.ResetError()
                    if robot_status.pause_motion_status:
                        self.robot.ResumeMotion()
                    self._errorRecovery()

        else:
            print_log(f'Error???')

    def _on_connectedCallback(self):
        print_log(f'-> Connected...')

    def _on_disconnectedCallback(self):
        print_log(f'-> Disconnected...')

    def _on_activatedCallback(self):
        print_log(f'-> Activated...')

    def _on_deactivatedCallback(self):
        print_log(f'-> Deactivated...')

    def _on_homedCallback(self):
        print_log(f'-> Homed...')


    def setActivate(self)->bool:
        try:
            self._connect()
            self._activate()
        except Exception as ex:
            exptTrace(ex)
            print_log(f'Robot  activation failed: {ex}')
            return False
        
        self._stored_online_status = True
        return True


    def setUnActive(self)->bool:
        try:
            self._stop_clear()
        except Exception as ex:
            exptTrace(ex)
            print_log(f'Robot  de-activation failed: {ex}')
            return False

        self._stored_online_status =  False
        return True

    def _getCartAcc(self)->float:

        try:
            response_codes = [mdr_def.MxRobotStatusCode.MX_ST_GET_CART_ACC]
            response = self.robot.SendCustomCommand('GetCartAcc', expected_responses=response_codes, timeout=10)
            _stat_res = tuple(map(int, response.data.split(',')))
            print_log(f'Cart Acceleration response = {response.data} / {_stat_res}')
        except Exception as ex:
            exptTrace(ex)
            return None

        return response.data
    
    def _getJointAcc(self)->float:

        try:
            response_codes = [mdr_def.MxRobotStatusCode.MX_ST_GET_JOINT_ACC]
            response = self.robot.SendCustomCommand('GetJointAcc', expected_responses=response_codes, timeout=10)
            _stat_res = tuple(map(int, response.data.split(',')))
            print_log(f'Joint Acceleration response = {response.data} / {_stat_res}')
        except Exception as ex:
            exptTrace(ex)
            return None

        return response.data
    
    def _getProductType(self)->str:
        try:                                                                # [2084][Meca500] or [2084][MCS500]
            response_codes = [mdr_def.MxRobotStatusCode.MX_ST_GET_PRODUCT_TYPE]
            response = self.robot.SendCustomCommand('GetProductType', expected_responses=response_codes, timeout=10)
            print_log(f'Type (model) of the product = {response.data} ')
        except Exception as ex:
            exptTrace(ex)
            return None

        return response.data            


    '''
    asyncCompletionAwait() ->
    [2007][as, hs, sm, es, pm, eob, eom]
    - as: activation state (1 if robot is activated, 0 otherwise);
    - hs: homing state (1 if homing already performed, 0 otherwise);
    - sm: simulation mode (1 if simulation mode is enabled, 0 otherwise);
    - es: error status (1 for robot in error mode, 0 otherwise);
    - pm: pause motion status (1 if robot is in pause motion, 0 otherwise);
    - eob: end of block status (1 if robot is not moving and motion queue is empty, 0 otherwise);
    - eom: end of movement status (1 if robot is not moving, 0 if robot is moving).
    '''

    def asyncCompletionAwait(self):
        try:
             while True:
                print_log(f'Jonts = {self.robot.GetJoints()} Position = {self.robot.GetPose()}')
                time.sleep(0.5)
                response_codes = [mdr_def.MxRobotStatusCode.MX_ST_GET_STATUS_ROBOT]
                response = self.robot.SendCustomCommand('GetStatusRobot', expected_responses=response_codes, timeout=10)
                _stat_res = tuple(map(int, response.data.split(',')))
                print_log(f'Status response = {response.data} / {_stat_res}')
                
                if _stat_res[0] == 1 and _stat_res[1] == 1 and  _stat_res[5] == 1 and _stat_res[6] == 1  :
                    print_log(f'End working cycle. Status = {response.data}/{_stat_res}')
                    break


                # ev:robot_classes.InterruptableEvent = self.robot.SetCheckpoint(7)

        except Exception as ex:
            print_log(f'Completion Await failed: {ex}')
            raise RuntimeError('Completion Await failed: {ex}')
        
    '''
    GetRtCartPos
        [2211][t, x, y, z, alpha, beta, gamma] or [2211][t, x, y, z, gamma]
        - t: timestamp in microseconds;
        - x, y, z: the coordinates of the origin of the TRF with respect to the WRF, in mm;
        - alpha, beta, gamma: Euler angles representing the orientation of the TRF with respect to the WRF, in degrees.
    '''

    def getRobotPosition(self)->tuple:
        try:
            # response_codes = [mdr_def.MxRobotStatusCode.MX_ST_RT_TARGET_CART_POS]
            # time.sleep(1)
            # response = self.robot.SendCustomCommand('GetRtCartPos', expected_responses=response_codes, timeout=10)
            # _stat_res = tuple(map(float, response.data.split(',')[1,]))

            _stat_res = tuple(self.robot.GetPose())
            print_log(f'Robot Position =  {_stat_res}')
            
            return _stat_res

        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            raise RuntimeError('Get Robot Position: {ex}')


    def robotOperation(self, __cmd:robotOpType, __parm:contexFuncParms):
        if self.dev_lock.locked():
            print_log(f'WARNING: Trying operate Robot during active operation. Will wait untill previous op completed')
        self.dev_lock.acquire()
        
        try:
            self.wd = threading.Thread(target=self.robotOpThread, args=(__cmd, __parm,))
            self.wd.start()
        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'ERROR Operating Robot. cmd = {robotOpType}, parms = {contexFuncParms}')
            self.dev_lock.release()
            return False
        
        return True


    def robotOpThread(self, __cmd:robotOpType, __parm:contexFuncParms = None):
        _res = True
        print_log(f'Operating Robot: cmd = {__cmd}, par = {__parm}')
        try:
            match __cmd:
                case robotOpType.moveabs:
                    _res = False if __parm is None else self.moveAbs( __parm)
                case robotOpType.movesafe:
                    _res = False if __parm is None else self.moveSafe(__parm)
                case robotOpType.moverel:
                    _res = False if __parm is None else self.moveRel(__parm)
                case robotOpType.moveup:
                    _res = self.moveUp(self)
                case robotOpType.pickup:
                    # try: 
                    #     if callable(self.pickUpPart):
                    #         pass
                    #     else:
                    #         raise('Unsupported method') 
                    # except:
                    #     raise(f'pickup {__cmd} command is not supported for device {self.devName} of type {self.type}')

                    _res = False if __parm is None else self.pickUpPart(__parm)
                case _:
                    raise(f'Unexpected command: {__cmd}')

        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'ERROR Operating Robot. cmd = {robotOpType}, parms = {contexFuncParms}')
            _res = False

        self.dev_lock.release()
        self.devNotificationQ.put(_res)

    @abstractmethod
    def pickUpPart(self, __parm:contexFuncParms)->bool:
        pass

    @abstractmethod
    def moveAbs(self, __parm:contexFuncParms)->bool: 
        pass

    @abstractmethod
    def moveSafe(self, __parm:contexFuncParms)->bool: 
        pass


    @abstractmethod
    def moveRel(self, __parm:contexFuncParms)->bool: 
        pass

    @abstractmethod
    def moveUp(self)->bool:
        pass



    @property
    def isActive(self)->bool:
        _current_time = time.time()     
        if _current_time - self._last_status_update_time < STATUS_RESOLUTION:
            return self._stored_online_status
        
        try:

            robot_status = self.robot.GetStatusRobot(synchronous_update = True)
        except Exception as ex:
            
            if self._stored_online_status:                                     # only once per failure
                exptTrace(ex)
                print_log(f'Robot getting status failed: {ex}')
            self._stored_online_status = False
            return False
        
        self._last_status_update_time = _current_time

        if  not robot_status.activation_state:
            self._stored_online_status = False
            return False
        
        self._stored_online_status = True 
        return True

    @staticmethod
    def find_avilable_robot(_ipAddr:str) -> bool:
        _robot = mdr.Robot()
        try:
            _robot.Connect(address=_ipAddr, enable_synchronous_mode = SYNC_MODE, \
                        disconnect_on_exception=DISCONNECT_MODE)
            _robot.ActivateRobot()

        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_log(f'Connection to Mecademic robot can not be established / Ativation failed: {ex}')
            return False

        _robot.DeactivateRobot()
        _robot.Disconnect()

        return True
    
    @abstractmethod
    def set_parms(self, parms)->bool:
        pass

    @property
    @abstractmethod
    def getPos(self)-> namedtuple:
        pass
