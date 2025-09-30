__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "1.0.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Tool"

import mecademicpy.robot as mdr
import mecademicpy.mx_robot_def as mdr_def
import mecademicpy.robot_initializer as initializer
import mecademicpy.tools as tools
import mecademicpy.robot_classes as robot_classes

from enum import Enum
from typing import Callable
from bs1_asyril import AsyrilInterface

import time, re
import os.path
from collections import namedtuple

from typing import Optional

import logging, sys, datetime, yaml
import threading
from threading import Lock
from queue import Queue 
import webbrowser


from mecademic_error import mecademicErrorMsg


from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, set_parm, get_parm, unsigned_16, assign_parm
from bs1_cognex_modbus import CognexCAM, statisticData
from bs1_mcdmc_base import baseRobot, robotOpType, contexFuncParms

from enum import Enum

# pickupDev= Enum("pickupDev", ["asyril", "cognex"])
 

PCB_ANGLE_CORRECTION = -90



STATUS_RESOLUTION = 2
POS_UPDATE_RESOLUTION =1

DEFAULT_PICKUP_PRESSURE = -85

JOINT_ACC = 50

# -82.59525 - vacuum with no part
# -0.24776  - with no vacuum

OpType = Enum("OpType", ["cognex", "asyril"])


_mcs500_posFields = ['x', 'y', 'z', 'gamma']
_mcs500_pos = namedtuple("_mcs500_pos",  _mcs500_posFields, defaults=[None,] * len(_mcs500_posFields))
_mcs500_pos.__annotations__={'x':float,  'y':float,  'z':float, 'gamma':float}

partPosFields = ["x", "y", "p3D"]
partPos = namedtuple("partPos",  partPosFields, defaults=[None,] * len(partPosFields))
partPos.__annotations__={'x':float,  'y':float, 'p3D':str} 

# contexFuncParmsFields = ["point", "coord", "velocity"]
# contexFuncParms = namedtuple("contexFuncParms", contexFuncParmsFields, defaults=[None,] * len(contexFuncParmsFields))
# contexFuncParms__annotations__={'point':str,  'coord':_mcs500_pos, "velocity":int} 

# robotOpType = Enum("robotOpType", ["moveabs", "movesafe", "moverel", "moveup",  "pickup"])


class robotMecademic(baseRobot):

    def __init__(self, devName,  IP, parms):
        super().__init__(devName,  IP, parms)
        self.PUT_H_ASYRIL =  None                 # put part height 
        self.PICKUP_H_ASYRIL =  None             # pich part height (asyril) 
        self.WORKING_H =  None                     # movement height 
        self.VELOCITY =  5
        self.PICK_DELAY =  0.5
        self.PUT_DELAY =  0.5
        self.WORK_ZONE_LIMIT_XYZ_MIN =  (0, 0, 0)   #  SetWorkZoneLimits - xmin,ymin,zmin,
        self.WORK_ZONE_LIMIT_XYZ_MAX =  (0, 0, 0)    # xmax,ymax,zmax
        self.START_POSITION =  _mcs500_pos(0, 0, 0, 0)          # x, y, z, gamma (coordinates)
        self.RELIEF_POSITION =  _mcs500_pos(0, 0, 0, 0)         # q1,q2,q3,q4 for MoveJoints(q1,q2,q3,q4)
        self.MID_JOINTS_POSITION =  _mcs500_pos(-67.69506, -55.04702, -65.00323, -1548.44055)     
                                            # MoveJoints(q1,q2,q3,q4)
                                            # −140° ≤ q1 ≤ 140°, −145° ≤ q2 ≤ 145°, −102 mm ≤ q3 ≤ 0 mm, −3,600° ≤ q4 ≤ 3,600°.
        self.PICK_PRESSURE_EL =  -84.5
        self.REPEATS =  3
        self.PUT_ANGLE =  180                      
        self.VACUUM_PURGE_DURATION_EL =  0.5
        self.PICK_ANGLE_RANGE:tuple =  (0, 180)

        self.PICKUP_ANGLE = 0
        self.VACUUM_PURGE_DURATION_PCB = 0.5
        self.PICKUP_PRESSURE_PCB = -84.5
        self.PUT_H_COGNEX = None
        self.PICKUP_H_COGNEX = None
        self.PICKUP_OFFSET_PCB:partPos = partPos(0, 0)
        self.__3Dcoord:dict = None
        self.__3Doffset:dict = None
        self.DEFAULT_PICKUP_PRESSURE = DEFAULT_PICKUP_PRESSURE
        self.DOWN_VELOCITY = 5
        self.UP_VELOCITY = 5
        self.JOINT_ACC = JOINT_ACC


        # self.devName = devName
        # self.IPadr = IP
        # self.robot:mdr.Robot = mdr.Robot()
        # self.model:str = None
        # self.robot_model:mdr.MxRobotModel = None
        self.robot_pos:_mcs500_pos = _mcs500_pos(0,0,0,0)
        self.robot_put_pos:partPos = partPos(0,0)
        self.__workingPoint:_mcs500_pos = None
        self.__asyril:AsyrilInterface = None
        self.__cognexCam:CognexCAM = None
        self.wd = None
        # self.callbacks = None
        # self._stored_online_status = False
        # self._last_status_update_time = 0
        # self._last_pos_update_time = 0
        self.__stored_pos = _mcs500_pos(0, 0, 0, 0)
        # self.dev_lock =  Lock()                 # device lock to avoid multiply access
        # self.devNotificationQ = Queue()

        self.__proceded_counter:int = 0
        self.__elbow:int = 1

        self.set_parms(parms = parms)

        self._init()

        self._connect()

        self._activate()
        
        self.robot.SetConf(elbow = self.__elbow)

    def __del__(self):
        if self.__asyril is not None:
            del self.__asyril
        if self.__cognexCam is not None:
            del self.__cognexCam

    # def _init(self):
    #     self.callbacks = mdr.RobotCallbacks()
    #     self.callbacks.on_error = self._errorCallback
    #     self.callbacks.on_connected = self._on_connectedCallback
    #     self.callbacks.on_disconnected = self._on_disconnectedCallback
    #     self.callbacks.on_activated = self._on_activatedCallback
    #     self.callbacks.on_deactivated = self._on_deactivatedCallback
    #     self.callbacks.on_homed = self._on_homedCallback
    #     self.robot.RegisterCallbacks(callbacks=self.callbacks, run_callbacks_in_separate_thread=True)


    # def setActivate(self)->bool:
    #     try:
    #         self._connect()
    #         self._activate()
    #         # self.robot.ActivateRobot()
    #     except Exception as ex:
    #         exptTrace(ex)
    #         print_log(f'Robot  activation failed: {ex}')
    #         return False
        
    #     self._stored_online_status = True
    #     # self.robot.SetConf(-1)
    #     return True


    # def setUnActive(self)->bool:
    #     try:
    #         self._stop_clear()
    #         # self.robot.DeactivateRobot()
    #     except Exception as ex:
    #         exptTrace(ex)
    #         print_log(f'Robot  de-activation failed: {ex}')
    #         return False

    #     self._stored_online_status =  False
    #     return True
    #     # self.robot.SetAutoConf(True)

    @property
    def counter(self) -> int:
            return self.__proceded_counter

    # @property
    # def isActive(self)->bool:
    #     _current_time = time.time()     
    #     if _current_time - self._last_status_update_time < STATUS_RESOLUTION:
    #         return self._stored_online_status
        
    #     try:

    #         robot_status = self.robot.GetStatusRobot(synchronous_update = True)
    #     except Exception as ex:
            
    #         if self._stored_online_status:                                     # only once per failure
    #             exptTrace(ex)
    #             print_log(f'Robot getting status failed: {ex}')
    #         self._stored_online_status = False
    #         return False
        
    #     self._last_status_update_time = _current_time

    #     if  not robot_status.activation_state:
    #         self._stored_online_status = False
    #         return False
        
    #     self._stored_online_status = True 
    #     return True
    
    @property
    def isCognexOnline(self)->bool:
        if self.__cognexCam is not None:
            return self.__cognexCam.onlineStatus
        else:
            return False

    @property
    def statisticINFO(self)->statisticData:
        return self.__cognexCam.statisticINFO
    
    @property
    def valid_products(self):
        return self.__cognexCam.statisticINFO.valid_products

    @property
    def getPos(self)->_mcs500_pos:
        _pos:_mcs500_pos = _mcs500_pos(0, 0, 0, 0)
        _current_time = time.time()     
        if _current_time - self._last_pos_update_time < POS_UPDATE_RESOLUTION:
            return self.__stored_pos
        
        try:
            # _r_pos = self.robot.GetPose()
            _r_pos = self.robot.GetRtTargetCartPos()
            _pos = _mcs500_pos(*_r_pos)
        except Exception as ex:
            exptTrace(ex)
            print_log(f'Robot getting position failed: {ex}')
            return _mcs500_pos(0, 0, 0, 0)
        
        self._last_pos_update_time = _current_time

        self.__stored_pos = _pos 

        return _pos

    def validateCAM(self)->bool:
        return self.__cognexCam.updatePartsPos()


    def triggerCognex(self):
        if self.__cognexCam is not None:
            self.__cognexCam.Trigger()


    def openRobotURL(self):
        webbrowser.open(self.IPadr)


    # def _connect(self, IP_adr:str = '192.168.0.100') -> bool:
    #     try:
    #         self.robot.Connect(address=IP_adr, enable_synchronous_mode = SYNC_MODE, \
    #                     disconnect_on_exception=DISCONNECT_MODE)
    #         self.model = self.robot.GetRobotInfo().model
    #         self.robot_model = self.robot.GetRobotInfo().robot_model
    #         print_log(f'Robot Model = {self.model}/{self.robot_model}')
    #     except Exception as ex:
    #         e_type, e_filename, e_line_number, e_message = exptTrace(ex)
    #         print_log(f'Connection  failed: {ex}')
    #         return False
        
    #     return True

    # def _getCartAcc(self)->float:

    #     try:
    #         response_codes = [mdr_def.MxRobotStatusCode.MX_ST_GET_CART_ACC]
    #         response = self.robot.SendCustomCommand('GetCartAcc', expected_responses=response_codes, timeout=10)
    #         _stat_res = tuple(map(int, response.data.split(',')))
    #         print_log(f'Cart Acceleration response = {response.data} / {_stat_res}')
    #     except Exception as ex:
    #         exptTrace(ex)
    #         return None

    #     return response.data
    
    # def _getJointAcc(self)->float:

    #     try:
    #         response_codes = [mdr_def.MxRobotStatusCode.MX_ST_GET_JOINT_ACC]
    #         response = self.robot.SendCustomCommand('GetJointAcc', expected_responses=response_codes, timeout=10)
    #         _stat_res = tuple(map(int, response.data.split(',')))
    #         print_log(f'Joint Acceleration response = {response.data} / {_stat_res}')
    #     except Exception as ex:
    #         exptTrace(ex)
    #         return None

    #     return response.data
        
    # def _activate(self) -> bool:
    #     try:
    #         self.robot.ActivateRobot()
    #         self.robot.Home()
    #         self.robot.SetConfTurn(0)
    #         # self.robot.SetAutoConfTurn(0)
    #         # self.robot.SetCartAcc(1)
    #         self.robot.SetJointAcc(self.JOINT_ACC)
    #         # self._getCartAcc()
    #         self._getJointAcc()


    #     except Exception as ex:
    #         e_type, e_filename, e_line_number, e_message = exptTrace(ex)
    #         print_log(f'Activation  failed: {ex}')
    #         return False
        
    #     return True


    # def _stop_clear(self):
    #     try:
            
    #         if not SYNC_MODE:
    #             print_log(f'Waiting to Idle')
    #             self.robot.WaitIdle()

    #         print_log(f'Deactivating')
    #         self.robot.DeactivateRobot()

    #         if not SYNC_MODE:
    #             print_log(f'Waiting for Deactivation')
    #             self.robot.WaitDeactivated()

    #         print_log(f'Disconnect now')
    #         self.robot.Disconnect()
            
    #     except Exception as ex:
    #         e_type, e_filename, e_line_number, e_message = exptTrace(ex)
    #         print_log(f'Stoping/clearing proc failed: {ex}')

    # def _errorRecovery(self):                       # in general singularity is fixed right here
    #     pass

    # def _errorCallback(self):
    #     robot_status = self.robot.GetStatusRobot(synchronous_update = True)
    #     # print_log(f'Error callback... error_status = {robot_status.error_status}, robot_status.error_code ={robot_status.error_code} ')
    #     print_log(f'Error callback... error_status = {robot_status.error_status} ')
    #     if robot_status.error_status:
    #                 # print_log(f'Error detected. Code = {robot_status.error_code} / {mecademicErrorMsg(int(robot_status.error_code))} ')
    #                 print_log(f'Error detected. Code = {robot_status.error_code}  ')
    #                 self.robot.ResetError()
    #                 if robot_status.pause_motion_status:
    #                     self.robot.ResumeMotion()
    #                 self._errorRecovery()

    #     else:
    #         print_log(f'Error???')


    # def _on_connectedCallback(self):
    #     print_log(f'-> Connected...')

    # def _on_disconnectedCallback(self):
    #     print_log(f'-> Disconnected...')

    # def _on_activatedCallback(self):
    #     print_log(f'-> Activated...')

    # def _on_deactivatedCallback(self):
    #     print_log(f'-> Deactivated...')

    # def _on_homedCallback(self):
    #     print_log(f'-> Homed...')


    @property
    def presure(self)->float:
        return self._getPressure()

    def _getPressure(self)->float:
        response_codes = [mdr_def.MxRobotStatusCode.MX_ST_RT_VACUUM_PRESSURE]
        response = self.robot.SendCustomCommand('GetRtvacuumPressure', expected_responses=response_codes, timeout=10)
        _stat_res = tuple(map(float, response.data.split(',')))
        timestamp = _stat_res[0]
        pressure = _stat_res[1]
        print_log(f'PRESSURE: timestamp = {timestamp} / pressure = {pressure} / PICK_PRESSURE_EL = {self.PICK_PRESSURE_EL}')
        return float(pressure)

    
    # def asyncCompletionAwait(self):
    #     try:
    #          while True:
    #             print_log(f'Jonts = {self.robot.GetJoints()} Position = {self.robot.GetPose()}')
    #             time.sleep(0.5)
    #             response_codes = [mdr_def.MxRobotStatusCode.MX_ST_GET_STATUS_ROBOT]
    #             response = self.robot.SendCustomCommand('GetStatusRobot', expected_responses=response_codes, timeout=10)
    #             _stat_res = tuple(map(int, response.data.split(',')))
    #             print_log(f'Status response = {response.data} / {_stat_res}')
                
    #             if _stat_res[0] == 1 and _stat_res[1] == 1 and  _stat_res[5] == 1 and _stat_res[6] == 1  :
    #                 print_log(f'End working cycle. Status = {response.data}/{_stat_res}')
    #                 break

    #             # ev:robot_classes.InterruptableEvent = self.robot.SetCheckpoint(7)

    #     except Exception as ex:
    #         print_log(f'Completion Await failed: {ex}')
    #         raise RuntimeError('Completion Await failed: {ex}')
        
    # def getRobotPosition(self)->tuple:
    #     try:
    #         # response_codes = [mdr_def.MxRobotStatusCode.MX_ST_RT_TARGET_CART_POS]
    #         # time.sleep(1)
    #         # response = self.robot.SendCustomCommand('GetRtCartPos', expected_responses=response_codes, timeout=10)
    #         # _stat_res = tuple(map(int, response.data.split(',')))

    #         _stat_res = tuple(self.robot.GetPose())
    #         print_log(f'Robot Position =  {_stat_res}')
            
    #         return _stat_res

    #     except Exception as ex:
    #         e_type, e_filename, e_line_number, e_message = exptTrace(ex)
    #         raise RuntimeError('Get Robot Position: {ex}')
        
    def wrMovePose(self, x:float, y:float, z:float, gamma:float):
        if x > 0 and self.__elbow < 0:
            self.__elbow = 1
            self.robot.SetConf(elbow = self.__elbow)
        elif x < 0 and self.__elbow > 0:
            self.__elbow = -1
            self.robot.SetConf(elbow = self.__elbow)

        self.robot.MovePose(x, y, z, gamma)

# NEW ERA CODE ↓↓↓↓↓↓↓↓↓
#
#
#

    def __get3Dpos(self)->tuple:
        if self.__workingPoint is not None:
            return  self.__workingPoint.x, self.__workingPoint.y, self.__workingPoint.gamma
        else:
            print_err(f'ERROR: self.__workingPoint = {self.__workingPoint}')
            return None

    # def robotOperation(self, __cmd:robotOpType, __parm:contexFuncParms):
    #     if self.dev_lock.locked():
    #         print_log(f'WARNING: Trying operate Robot during active operation. Will wait untill previous op completed')
    #     self.dev_lock.acquire()
        
    #     try:
    #         self.wd = threading.Thread(target=self.robotOpThread, args=(__cmd, __parm,))
    #         self.wd.start()
    #     except Exception as ex:
    #         e_type, e_filename, e_line_number, e_message = exptTrace(ex)
    #         print_err(f'ERROR Operating Robot. cmd = {robotOpType}, parms = {contexFuncParms}')
    #         self.dev_lock.release()
    #         return False
        
    #     return True


    # def robotOpThread(self, __cmd:robotOpType, __parm:contexFuncParms = None):
    #     _res = True
    #     print_log(f'Operating Robot: cmd = {__cmd}, par = {__parm}')
    #     try:
    #         match __cmd:
    #             case robotOpType.moveabs:
    #                 _res = False if __parm is None else self.moveAbs( __parm)
    #             case robotOpType.movesafe:
    #                 _res = False if __parm is None else self.moveSafe(__parm)
    #             case robotOpType.moverel:
    #                 _res = False if __parm is None else self.moveRel(__parm)
    #             case robotOpType.moveup:
    #                 _res = self.moveUp(self)
    #             case robotOpType.pickup:
    #                 _res = False if __parm is None else self.pickUpPart(__parm)
    #             case _:
    #                 raise(f'Unexpected command: {__cmd}')

    #     except Exception as ex:
    #         e_type, e_filename, e_line_number, e_message = exptTrace(ex)
    #         print_err(f'ERROR Operating Robot. cmd = {robotOpType}, parms = {contexFuncParms}')
    #         _res = False

    #     self.dev_lock.release()
    #     self.devNotificationQ.put(_res)

    # def pickUpPart(self, __dev:str)->bool:
    def pickUpPart(self, __parm:contexFuncParms)->bool:

        if __parm.point is None:
            print_err(f'Incorrect parameters for pickup: {__parm}')
            return False

        __dev:str = __parm.point

        print_log(f'Pickup from {__dev}')
        try:
            
            self.__workingPoint = None
            
            _getPosFunc:Callable[[], tuple] = None
            _pH:float = None
            _pPr:float = None
            _pickOffset:partPos = None
            _placeAngleOffset = None

            match __dev.upper():
                case 'ASYRIL':
                    if self.__asyril is None:
                        print_err(f'Asysril is not initated')
                        return False
                    _getPosFunc = self.__asyril.get_pos
                    _pH = self.PICKUP_H_ASYRIL
                    _pPr = self.PICK_PRESSURE_EL
                
                case 'COGNEX':
                    if self.__cognexCam is None:
                        print_err(f'Cognex is not initated')
                        return False
                    _getPosFunc = self.__cognexCam.get_pos
                    _pH = self.PICKUP_H_COGNEX
                    _pPr = self.PICKUP_PRESSURE_PCB
                    _pickOffset:partPos = self.PICKUP_OFFSET_PCB
                    _placeAngleOffset = -90

                case _: 
                    if self.__3Dcoord is not None and not __dev in self.__3Dcoord.keys():
                        raise Exception(f'No coordinates defined for {__dev}')

                    if self.__3Dcoord is not None and __dev in self.__3Dcoord.keys():
                        self.__workingPoint = self.__3Dcoord[__dev]

                        _getPosFunc = self.__get3Dpos()
                    else:
                        raise Exception(f'Wrong coordinates defined for {__dev}: {self.__workingPoint}')
                    
                    if self.__workingPoint.x is None or self.__workingPoint.y is None or self.__workingPoint.gamma is None:
                        raise Exception(f'Wrong coordinates defined for {__dev}: {self.__workingPoint}')
                    
                    _pPr = self.DEFAULT_PICKUP_PRESSURE

                    _pH = self.__workingPoint.z
                    if _pH is None:
                        raise Exception(f'Wrong height or no height specifyed for {__dev}: {self.__workingPoint}')


            _res = self.__pickUp(_getPosFunc, _pH, _pPr, _pickOffset, _placeAngleOffset)

        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            # raise RuntimeError('Pickup failed. Error: {ex}')
            print_err('ERROR: Pickup failed. Error: {ex}')
            return False



    def __pickUp(self, __getPickUpPos:Callable[[], tuple], _pickUpHeght:float, \
               _pickPressure:float, _pickOffset:partPos = None, _placeAngleOffset = None) -> bool:
        try:
            _r_current_pos = self.robot.GetRtTargetCartPos()
            _cur_pos = _mcs500_pos(*_r_current_pos)


            self.robot.SetJointVel(self.UP_VELOCITY)
            self.wrMovePose(_cur_pos.x, _cur_pos.y, self.WORKING_H, gamma=_cur_pos.gamma)
            self.robot.SetJointVel(self.VELOCITY)

            for _rep in range (self.REPEATS):    
                _x, _y, _rz, _offset_sign_x, _offset_sign_y = __getPickUpPos(_rep)
                print_log(f'x= {_x}, y= {_y}, rz = {_rz}')
                if _pickOffset is not None:
                    _x += _pickOffset.x * _offset_sign_x
                    _y += _pickOffset.y * _offset_sign_y
                print_log(f'After offset correction x= {_x}, y= {_y}, rz = {_rz}, pickOffset = {_pickOffset}')


                __pickup_angle = int(self.PICKUP_ANGLE)
                if _placeAngleOffset is not None:
                    __place_angle = _rz + _placeAngleOffset
                else: 
                    __place_angle = int(self.PUT_ANGLE) -_rz
                    
                if __place_angle < int(self.PICK_ANGLE_RANGE[0]):
                    __place_angle += 180
                elif __place_angle > int(self.PICK_ANGLE_RANGE[1]):
                    __place_angle = __place_angle - 180

                print_log(f'x= {_x}, y= {_y}, rz = {_rz}, pickup_angle = {__pickup_angle}, place_angle= {__place_angle}, PICK_ANGLE_RANGE = {self.PICK_ANGLE_RANGE}[{self.PICK_ANGLE_RANGE[0]}:{self.PICK_ANGLE_RANGE[1]}]')

                print_log (f'Try # {_rep}')
                self.robot.VacuumRelease()
                self.wrMovePose(_x, _y, self.WORKING_H, gamma=__pickup_angle)
                self.robot.SetJointVel(self.DOWN_VELOCITY)
                # self.wrMovePose(_x, _y, _pickUpHeght - 5,  gamma=__pickup_angle)

                self.wrMovePose(_x, _y, self.WORKING_H + abs(_pickUpHeght - self.WORKING_H)/3,  gamma=__pickup_angle)
                self.robot.SetJointVel(self.DOWN_VELOCITY*2/3)
                self.wrMovePose(_x, _y, self.WORKING_H + abs(_pickUpHeght - self.WORKING_H)*2/3,  gamma=__pickup_angle)
                self.robot.SetJointVel(self.DOWN_VELOCITY/3)



                self.wrMovePose(_x, _y, _pickUpHeght,  gamma=__pickup_angle)
                self.getRobotPosition()
                
                self.robot.VacuumGrip()
                self.robot.Delay(self.PICK_DELAY)
                self.robot.SetJointVel(self.UP_VELOCITY)
                self.wrMovePose(_x, _y, self.WORKING_H, gamma=__pickup_angle)

                self.robot.Delay(0.1)


                if self._getPressure() < _pickPressure:
                    _rep += 1
                    break

            if _rep > self.REPEATS:
                print_err(f'{self.REPEATS} unsuccessful attempts to pick the part')
                self.robot.VacuumRelease()
                return False
                       

            self.__completeRobotOp()

        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            raise ex


    # def moveAbs(self, _pos:str, _velocity:int = None)->bool: 
    def moveAbs(self, __parm:contexFuncParms)->bool: 
        if __parm.point is None:
            print_err(f'Incorrect parameters for moving: {__parm}')
            return False
        
        _pos:str = __parm.point
        _velocity:int = __parm.velocity
        
        try:
            if not _pos in self.__3Dcoord.keys():
                raise Exception(f'No coordinates defined for {_pos}')

            __movePoint =  self.__3Dcoord[_pos]
           
            _r_current_pos = self.robot.GetRtTargetCartPos()
            _current_pos = _mcs500_pos(*_r_current_pos)
            print_log(f'Moving absolute to {_pos}, Current position = {_current_pos}')

            if __movePoint.x is None and __movePoint.y is None and __movePoint.z is None and __movePoint.gamma is None:
                print_log(f'Nothing to do for {_pos}: {__movePoint}')
                return True
                                                                        # if any exe is None use current pos
            _move_X = __movePoint.x if __movePoint.x is not None else _current_pos.x
            _move_Y = __movePoint.y if __movePoint.y is not None else _current_pos.y
            _move_Z = __movePoint.z if __movePoint.z is not None else _current_pos.z
            _move_GAMMA = __movePoint.gamma if __movePoint.gamma is not None else _current_pos.gamma
            
            _vel = _velocity if _velocity is not None else self.VELOCITY
            self.robot.SetJointVel(_vel)
            self.wrMovePose(_move_X, _move_Y, _move_Z, gamma = _move_GAMMA)

            self.__completeRobotOp()
                        
        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err('ERROR: Moving failed. Error: {ex}')
            return False
        
        return True

    # def moveSafe(self, _pos:str, _velocity:int = None)->bool:
    def moveSafe(self, __parm:contexFuncParms)->bool: 
        if __parm.point is None:
            print_err(f'Incorrect parameters for moving: {__parm}')
            return False
        
        _pos:str = __parm.point
        _velocity:int = __parm.velocity

        try:
            if not _pos in self.__3Dcoord.keys():
                raise Exception(f'No coordinates defined for {_pos}')

            __movePoint =  self.__3Dcoord[_pos]
           
            _r_current_pos = self.robot.GetRtTargetCartPos()
            _current_pos = _mcs500_pos(*_r_current_pos)
            print_log(f'Moving safe to {_pos},  Current position = {_current_pos}')


            if __movePoint.x is None and __movePoint.y is None and __movePoint.z is None and __movePoint.gamma is None:
                print_log(f'Nothing to do for {_pos}: {__movePoint}')
                return True
                                                                        # if any exe is None use current pos
            _move_X = __movePoint.x if __movePoint.x is not None else _current_pos.x
            _move_Y = __movePoint.y if __movePoint.y is not None else _current_pos.y
            _move_Z = __movePoint.z if __movePoint.z is not None else _current_pos.z
            _move_GAMMA = __movePoint.gamma if __movePoint.gamma is not None else _current_pos.gamma
            
            _vel = _velocity if _velocity is not None else self.VELOCITY
            self.robot.SetJointVel(_vel)

            self.wrMovePose(_current_pos.x, _current_pos.y, self.WORKING_H, gamma = _current_pos.gamma)
            self.wrMovePose(_move_X, _move_Y, self.WORKING_H, gamma = _move_GAMMA)
            self.wrMovePose(_move_X, _move_Y, _move_Z, gamma = _move_GAMMA)

            self.__completeRobotOp()


        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err('ERROR: Moving failed. Error: {ex}')
            return False
        
        return True

    # def moveRel(self, _rel:_mcs500_pos, _velocity:int = None)->bool:

    def moveRel(self, __parm:contexFuncParms)->bool: 
        try:

            if __parm.coord is not None:
                _rel:_mcs500_pos = __parm.coord
            elif  __parm.point is not None:
                _pos:str = __parm.point
                if not _pos in self.__3Doffset.keys():
                    raise Exception(f'No coordinates defined for {_pos}')
                _rel:_mcs500_pos =  self.__3Doffset[_pos]
            else:
                print_err(f'Incorrect parameters for moving: {__parm}')
                return False


            _velocity:int = __parm.velocity
        

        

            _r_current_pos = self.robot.GetRtTargetCartPos()
            _current_pos = _mcs500_pos(*_r_current_pos)

            print_log(f'Moving relative by {_rel}. Current position = {_current_pos}')

            if _rel.x == 0 and _rel.y == 0 and _rel.z == 0 and _rel.gamma == 0:
                print_log(f'Nothing to do for {_rel}')
                return True
                                                                        # if any exe is None use current pos
            _move_X =  _current_pos.x + _rel.x
            _move_Y = _current_pos.y + _rel.y
            _move_Z = _current_pos.z + _rel.z
            _move_GAMMA = _current_pos.gamma + _rel.gamma 
            
            _vel = _velocity if _velocity is not None else self.VELOCITY
            self.robot.SetJointVel(_vel)
            self.wrMovePose(_move_X, _move_Y, _move_Z, gamma = _move_GAMMA)

            self.__completeRobotOp()

        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err('ERROR: Moving failed. Error: {ex}')
            return False
        
        return True

    def moveUp(self)->bool:
        print_log(f'Moving Up')

        try:

            _r_current_pos = self.robot.GetRtTargetCartPos()
            _current_pos = _mcs500_pos(*_r_current_pos)

            print_log(f'Moving up to  {self.WORKING_H}. Current position = {_current_pos}')

            if _current_pos.z == self.WORKING_H:
                print_log(f'Nothing to do')
                return True
            
            self.wrMovePose(_current_pos.x, _current_pos.y, self.WORKING_H, gamma = _current_pos.gamma)
            self.__completeRobotOp()

        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err('ERROR: Moving failed. Error: {ex}')
            return False
        
        return True

    def Vacuum(self, onoff:bool = False, purge:float = None)->bool:
    # def Vacuum(self, __parm:contexFuncParms)->bool:


        try:
            if not onoff and purge is not None:
                self.robot.SetVacuumPurgeDuration(float(purge))
            if onoff:
                self.robot.VacuumGrip()
            else:
                self.robot.VacuumRelease()

            


        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err('ERROR: Vacuum operation failed. Error: {ex}')
            return False
        
        return True
    

# NEW ERA CODE  ↑↑↑↑↑↑↑↑↑↑↑↑
#
#
#

    def runMecademicCognexOp(self, putPosition:partPos = None) -> bool:
        if self.__cognexCam is None or not self.__cognexCam.onlineStatus:
            print_err(f'COGNEX is not active/online')
            return False
        
        if putPosition is None:
            print_err(f'Wrong insert position defined')
            return False
        else: 
            self.robot_put_pos = putPosition 


        # _coord = self.__cognexCam.get_pos()
        # if _coord is None:
        #     print_err(f'No valid PCB parts found')
        #     return False

        # print_log (f'INSERT: RELIEF_POSITION = {self.RELIEF_POSITION}, MID_JOINTS_POSITION = {self.MID_JOINTS_POSITION}')

        # _x, _y, _rz = _coord
        # print_log(f'PCB position (by Cognex) is x={_x}, y={_y}, z={_rz}')

        _res_op = True

        try:
            
            self.robot.SetVacuumPurgeDuration(self.VACUUM_PURGE_DURATION_PCB)
            _res_op = self.__pickUpandPlace(putPosition, self.__cognexCam.get_pos, self.PICKUP_H_COGNEX, \
                         self.PUT_H_COGNEX, self.PICKUP_PRESSURE_PCB, self.PICKUP_OFFSET_PCB, PCB_ANGLE_CORRECTION, _angleCorrection = False)



        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            return False


        return _res_op
        
    
    def __pickUpandPlace(self, putPosition:partPos, __getPickUpPos:Callable[[int], tuple], _pickUpHeght:float, \
                         _placeHeght:float, _pickPressure:float, _pickOffset:partPos, _placeAngleOffset = 0, _angleCorrection = False):
        try:
            # self._getCartAcc()

            self.robot.SetJointVel(self.VELOCITY)
            self.wrMovePose(self.RELIEF_POSITION.x, self.RELIEF_POSITION.y, self.RELIEF_POSITION.z, self.RELIEF_POSITION.gamma)

            _it = 0
            _success_pickup:bool = False
            for _rep in range (self.REPEATS):    
                _x, _y, _rz, _offset_sign_x, _offset_sign_y = __getPickUpPos(_rep)
                print_log(f'x= {_x}, y= {_y}, rz = {_rz}')

                _x += _pickOffset.x * _offset_sign_x
                _y += _pickOffset.y * _offset_sign_y
                print_log(f'After offset correction x= {_x}, y= {_y}, rz = {_rz}, pickOffset = {_pickOffset}')


                __pickup_angle = int(self.PICKUP_ANGLE)
                __place_angle = _rz + _placeAngleOffset

                if _angleCorrection:
                    __place_angle = int(self.PUT_ANGLE) -_rz
                    if __place_angle < int(self.PICK_ANGLE_RANGE[0]):
                        __place_angle += 180
                    elif __place_angle > int(self.PICK_ANGLE_RANGE[1]):
                        __place_angle = __place_angle - 180

                print_log(f'x= {_x}, y= {_y}, rz = {_rz}, pickup_angle = {__pickup_angle}, place_angle= {__place_angle}, PICK_ANGLE_RANGE = {self.PICK_ANGLE_RANGE}[{self.PICK_ANGLE_RANGE[0]}:{self.PICK_ANGLE_RANGE[1]}]')

                print_log (f'Try # {_rep}')
                self.robot.VacuumRelease()
                self.wrMovePose(_x, _y, self.WORKING_H, gamma=__pickup_angle)
                self.robot.SetJointVel(self.DOWN_VELOCITY)
                # self.wrMovePose(_x, _y, _pickUpHeght - 5,  gamma=__pickup_angle)
                self.wrMovePose(_x, _y, self.WORKING_H + abs(_pickUpHeght - self.WORKING_H)/3,  gamma=__pickup_angle)
                self.robot.SetJointVel(self.DOWN_VELOCITY*2/3)
                self.wrMovePose(_x, _y, self.WORKING_H + abs(_pickUpHeght - self.WORKING_H)*2/3,  gamma=__pickup_angle)
                self.robot.SetJointVel(self.DOWN_VELOCITY/3)
                self.wrMovePose(_x, _y, _pickUpHeght,  gamma=__pickup_angle)


                self.getRobotPosition()
                
                self.robot.VacuumGrip()
                self.robot.Delay(self.PICK_DELAY)
                self.robot.SetJointVel(self.UP_VELOCITY)
                self.wrMovePose(_x, _y, self.WORKING_H, gamma=__pickup_angle)
                self.robot.SetJointVel(self.VELOCITY)


                self.wrMovePose(self.MID_JOINTS_POSITION.x, self.MID_JOINTS_POSITION.y, self.MID_JOINTS_POSITION.z, self.MID_JOINTS_POSITION.gamma)
                self.robot.Delay(0.1)

                _current_pressure = self._getPressure()

                _it = _rep + 1
                if  _current_pressure < _pickPressure:
                    print_log(f'Succesful pickup done (rep = {_rep}, max repeats = {self.REPEATS})')
                    _success_pickup = True
                    break
            
                print_log(f'Part was not kept by vacuum propertly (rep = {_rep}, repeats = {self.REPEATS}). go to next iteration. current presure = {_current_pressure}, control presure = {_pickPressure}')


            if not _success_pickup:
                print_err(f'{self.REPEATS} unsuccessful attempts to pick the part. (rep = {_it}, max repeats = {self.REPEATS})')
                self.robot.VacuumRelease()
                return False
            else:
                print_log(f'Succesful pickup after {_it} tries)')
            
            self.wrMovePose(putPosition.x, putPosition.y, self.WORKING_H, gamma=__place_angle)

            # self.wrMovePose(putPosition.x, putPosition.y, _placeHeght - 10, gamma=__place_angle)
            # self.robot.SetJointVel(self.DOWN_VELOCITY)
            # self.wrMovePose(putPosition.x, putPosition.y, _placeHeght - 5, gamma=__place_angle)
            # self.wrMovePose(putPosition.x, putPosition.y, _placeHeght, gamma=__place_angle)

            self.robot.SetJointVel(self.DOWN_VELOCITY)
            self.wrMovePose(putPosition.x, putPosition.y, self.WORKING_H + abs(self.WORKING_H-_placeHeght)/3, gamma=__place_angle)
            self.robot.SetJointVel(self.DOWN_VELOCITY*2/3)
            self.wrMovePose(putPosition.x, putPosition.y, self.WORKING_H + abs(self.WORKING_H-_placeHeght)*2/3, gamma=__place_angle)
            self.robot.SetJointVel(self.DOWN_VELOCITY/3)
            self.wrMovePose(putPosition.x, putPosition.y, _placeHeght, gamma=__place_angle)


            

            self.__completeRobotOp()

        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            raise ex

        return True

    def __completeRobotOp(self):
        try:
            while True:
                print_log(f'Jonts = {self.robot.GetJoints()} Position = {self.robot.GetPose()}')
                time.sleep(0.5)
                response_codes = [mdr_def.MxRobotStatusCode.MX_ST_GET_STATUS_ROBOT]
                response = self.robot.SendCustomCommand('GetStatusRobot', expected_responses=response_codes, timeout=10)
                _stat_res = tuple(map(int, response.data.split(',')))
                print_log(f'Status response = {response.data} / {_stat_res}')
                
                if _stat_res[0] == 1 and _stat_res[1] == 1 and  _stat_res[5] == 1 and _stat_res[6] == 1  :
                    print_log(f'End robot working cycle. Status = {response.data}/{_stat_res}')
                    break
        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            raise ex
        
    def finalCorrection(self, _delta_x = 0, _delta_y = 0, _delta_z = 0)->bool:
        try:
            _r_pos = self.robot.GetRtTargetCartPos()
            _pos:_mcs500_pos = _mcs500_pos(*_r_pos)
            self.robot.SetJointVel(self.DOWN_VELOCITY)
            self.wrMovePose(_pos.x + float(_delta_x), _pos.y + float(_delta_y), _pos.z + float(_delta_z), gamma= _pos.gamma)
            self.robot.Delay(self.PUT_DELAY)
            self.robot.VacuumRelease()
            self.robot.Delay(self.PUT_DELAY)
            self.robot.SetJointVel(self.UP_VELOCITY)

            self.wrMovePose(_pos.x + float(_delta_x), _pos.y + float(_delta_y), self.WORKING_H, gamma=_pos.gamma)
            self.robot.SetJointVel(self.VELOCITY)

            self.wrMovePose(self.RELIEF_POSITION.x, self.RELIEF_POSITION.y, self.RELIEF_POSITION.z, self.RELIEF_POSITION.gamma)

        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            return False
        
        return True


    def runMecademicAsyrilOp(self, putPosition:partPos = None) -> bool:
        if self.__asyril is None:
            print_err(f'Asyril is not active')
            return False
        
        if putPosition == None:
            if not isinstance(self.robot_put_pos.x, float) or not isinstance(self.robot_put_pos.y, float):
                print_err(f'Incorrect put position defined using GUI: {self.robot_put_pos}')
                return False
            else:
                putPosition = self.robot_put_pos
        else: 
            self.robot_put_pos = putPosition 
            
        print_log (f'DEBUG: RELIEF_POSITION = {self.RELIEF_POSITION}, MID_JOINTS_POSITION = {self.MID_JOINTS_POSITION}, VACUUM_PURGE_DURATION={self.VACUUM_PURGE_DURATION_EL}')

        try:
            self.robot.SetJointVel(self.VELOCITY)
            self.robot.SetVacuumPurgeDuration(self.VACUUM_PURGE_DURATION_EL)
            self.wrMovePose(self.RELIEF_POSITION.x, self.RELIEF_POSITION.y, self.RELIEF_POSITION.z, self.RELIEF_POSITION.gamma)


            __pickup_angle:int =0
            __place_angle:int = 0
            _it = 0
            for _rep in range (self.REPEATS):    
                _x, _y, _rz, _dir_x, _dir_y = self.__asyril.get_pos()
                
                
                __pickup_angle = int(self.PUT_ANGLE)
                __place_angle = int(self.PUT_ANGLE) -_rz
                if __place_angle < int(self.PICK_ANGLE_RANGE[0]):
                    __place_angle += 180
                elif __place_angle > int(self.PICK_ANGLE_RANGE[1]):
                    __place_angle = __place_angle - 180

                print_log(f'x= {_x}, y= {_y}, rz = {_rz}, pickup_angle = {__pickup_angle}, place_angle= {__place_angle}, PICK_ANGLE_RANGE = {self.PICK_ANGLE_RANGE}[{self.PICK_ANGLE_RANGE[0]}:{self.PICK_ANGLE_RANGE[1]}]')

                print_log (f'Try # {_rep}')
                self.robot.VacuumRelease()
                self.wrMovePose(_x, _y, self.WORKING_H, gamma=__pickup_angle)
                self.robot.SetJointVel(self.DOWN_VELOCITY)
                # self.wrMovePose(_x, _y, self.PICKUP_H_ASYRIL - 5,  gamma=__pickup_angle)

                self.wrMovePose(_x, _y, self.WORKING_H + abs(self.WORKING_H-self.PICKUP_H_ASYRIL)/3, gamma=__pickup_angle)

                self.robot.SetJointVel(self.DOWN_VELOCITY*2/3)

                self.wrMovePose(_x, _y, self.WORKING_H + abs(self.WORKING_H-self.PICKUP_H_ASYRIL)*2/3, gamma=__pickup_angle)

                self.robot.SetJointVel(self.DOWN_VELOCITY/3)


                self.wrMovePose(_x, _y, self.PICKUP_H_ASYRIL,  gamma=__pickup_angle)

                self.getRobotPosition()
                
                self.robot.VacuumGrip()
                self.robot.Delay(self.PICK_DELAY)
                self.robot.SetJointVel(self.UP_VELOCITY) 
                self.wrMovePose(_x, _y, self.WORKING_H, gamma=__pickup_angle)
                self.robot.SetJointVel(self.VELOCITY)
                self.wrMovePose(self.MID_JOINTS_POSITION.x, self.MID_JOINTS_POSITION.y, self.MID_JOINTS_POSITION.z, self.MID_JOINTS_POSITION.gamma)
                self.robot.Delay(0.1)


                it = _rep
                if self._getPressure() < self.PICK_PRESSURE_EL:
                    print_log(f'Pickup Operation succeeded. Go next op.  rep = {_rep}, repeats = {self.REPEATS}')        
                    break

            print_err(f'Vacuum lost. Operation canceled. Go next step.  rep = {it}, repeats = {self.REPEATS}')
            if _rep >= self.REPEATS:
                print_err(f'{self.REPEATS} unsuccessful attempts to pich the part. rep = {it}, repeats = {self.REPEATS}')
                self.robot.VacuumRelease()
                return False
            
            self.wrMovePose(putPosition.x, putPosition.y, self.WORKING_H, gamma=__place_angle)
            # self.wrMovePose(putPosition.x, putPosition.y, self.PUT_H_ASYRIL - 10, gamma=__place_angle)
            # self.robot.SetJointVel(self.DOWN_VELOCITY)
            # self.wrMovePose(putPosition.x, putPosition.y, self.PUT_H_ASYRIL - 5, gamma=__place_angle)
            # self.wrMovePose(putPosition.x, putPosition.y, self.PUT_H_ASYRIL, gamma=__place_angle)
            self.robot.SetJointVel(self.DOWN_VELOCITY)
            self.wrMovePose(putPosition.x, putPosition.y, self.WORKING_H + abs(self.WORKING_H-self.PUT_H_ASYRIL)/3, gamma=__place_angle)
            self.robot.SetJointVel(self.DOWN_VELOCITY*2/3)
            self.wrMovePose(putPosition.x, putPosition.y, self.WORKING_H + abs(self.WORKING_H-self.PUT_H_ASYRIL)*2/3, gamma=__place_angle)
            self.robot.SetJointVel(self.DOWN_VELOCITY/3)
            self.wrMovePose(putPosition.x, putPosition.y, self.PUT_H_ASYRIL, gamma=__place_angle)

            self.getRobotPosition()
            self._getPressure()

            self.robot.VacuumRelease()
            self.robot.Delay(self.PUT_DELAY)
            self.robot.SetJointVel(self.VELOCITY)
            self.wrMovePose(putPosition.x, putPosition.y, self.WORKING_H, gamma=__place_angle)

            self.wrMovePose(self.RELIEF_POSITION.x, self.RELIEF_POSITION.y, self.RELIEF_POSITION.z, self.RELIEF_POSITION.gamma)

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
                    # return True
                

        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_log(f'Script run proc failed: {ex}')
            try:
                self.robot.VacuumRelease()
                _pos = self.getRobotPosition()

                print_log(f'Going up to: {_pos}')
                self.wrMovePose(_pos[0], _pos[1], self.RELIEF_POSITION.z, _pos[3])

                self.wrMovePose(self.RELIEF_POSITION.x, self.RELIEF_POSITION.y, self.RELIEF_POSITION.z, self.RELIEF_POSITION.gamma)
            
            except Exception as ex:
                e_type, e_filename, e_line_number, e_message = exptTrace(ex)

            return False
        
        return True

    def  mDev_stop(self)->bool:
        try:
            print_log(f'Stop Mecademic')
            self.robot.ClearMotion()
            self.robot.ResumeMotion()

        except Exception as ex:
                e_type, e_filename, e_line_number, e_message = exptTrace(ex)
                return False
        
        return True

        
    def startOP(self, _xy:partPos = None ) -> bool:
        print_log(f'Starting Robot ASYRIL OP to {_xy}')
        if self.dev_lock.locked():
            print_log(f'WARNING: Trying operate Robot during active operation. Will wait untill previous op completed')
        self.dev_lock.acquire()
        try:

            if _xy:
                if _xy.p3D is None:
                    _x:float = _xy.x
                    _y:float = _xy.y
                else:
                    __dev = _xy.p3D
                    if self.__3Dcoord is not None and not __dev in self.__3Dcoord.keys():
                        raise Exception(f'No coordinates defined for {__dev}')
                    
                    elif self.__3Dcoord is not None and __dev in self.__3Dcoord.keys() and self.__3Dcoord[__dev].x is not None and self.__3Dcoord[__dev].y is not None:
                        _x:float = self.__3Dcoord[__dev].x
                        _y:float = self.__3Dcoord[__dev].y

                    else:
                        raise Exception(f'Wrong coordinates defined for {__dev}: {self.__workingPoint}')

                self.robot_put_pos = _xy
            else:
                _x:float = self.robot_put_pos.x
                _y:float = self.robot_put_pos.y
        
            self.wd = threading.Thread(target=self.operationalThread, args=(_x, _y, OpType.asyril,))
            self.wd.start()
        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'ERROR Operating Robot.')
            self.dev_lock.release()
            return False
        
        return True
    
    def startCognexOP(self, _xy:partPos = None ) -> bool:
        print_log(f'Starting Robot COGNEX OP to {_xy}')
        if self.dev_lock.locked():
            print_log(f'WARNING: Trying operate Robot during active operation. Will wait untill previous op completed')

        self.dev_lock.acquire()
        try:
            if _xy:
                if _xy.p3D is None:
                    _x:float = _xy.x
                    _y:float = _xy.y
                else:
                    __dev = _xy.p3D
                    if self.__3Dcoord is not None and not __dev in self.__3Dcoord.keys():
                        raise Exception(f'No coordinates defined for {__dev}')
                    
                    elif self.__3Dcoord is not None and __dev in self.__3Dcoord.keys() and self.__3Dcoord[__dev].x is not None and self.__3Dcoord[__dev].y is not None:
                        _x:float = self.__3Dcoord[__dev].x
                        _y:float = self.__3Dcoord[__dev].y

                    else:
                        raise Exception(f'Wrong coordinates defined for {__dev}: {self.__workingPoint}')

                self.pcb_put_pos = _xy
            else:
                _x:float = self.pcb_put_pos.x
                _y:float = self.pcb_put_pos.y


            self.wd = threading.Thread(target=self.operationalThread, args=(_x, _y, OpType.cognex, ))
            self.wd.start()
        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'ERROR Operating Robot.')
            self.dev_lock.release()
            return False
        
        return True
    
    def operationalThread(self, _x, _y, _type:OpType):
        _res = True
        if _type == OpType.asyril:
            _res = self.runMecademicAsyrilOp(partPos(_x, _y))
            print_log(f'Mecademic operation on ASYRIL has completed with {_res} result. Exiting')
        elif _type == OpType.cognex:
            _res = self.runMecademicCognexOp(partPos(_x, _y))
            print_log(f'Mecademic operation on COGNEX has completed with {_res} result. Exiting')

        self.dev_lock.release()
        self.devNotificationQ.put(_res)


    def set_parms(self, parms)->bool:
        try:
            self.PUT_H_ASYRIL =  float(get_parm(self.devName, parms, 'PUT_H_ASYRIL'))
            self.PICKUP_H_ASYRIL = float(get_parm(self.devName, parms, 'PICKUP_H_ASYRIL'))
            self.WORKING_H = float(get_parm(self.devName, parms, 'WORKING_H'))
            self.VELOCITY = int(get_parm(self.devName, parms, 'VELOCITY'))
            self.PICK_DELAY = float(get_parm(self.devName, parms, 'PICK_DELAY'))
            self.PUT_DELAY = float(get_parm(self.devName, parms, 'PUT_DELAY'))
            self.WORK_ZONE_LIMIT_XYZ_MIN = get_parm(self.devName, parms, 'WORK_ZONE_LIMIT_XYZ_MIN')
            self.WORK_ZONE_LIMIT_XYZ_MAX = get_parm(self.devName, parms, 'WORK_ZONE_LIMIT_XYZ_MAX')
            self.START_POSITION = _mcs500_pos(*str2float_tuple(get_parm(self.devName, parms, 'START_POSITION')))
            self.RELIEF_POSITION = _mcs500_pos(*str2float_tuple(get_parm(self.devName, parms, 'RELIEF_POSITION')))
            self.MID_JOINTS_POSITION = _mcs500_pos(*str2float_tuple(get_parm(self.devName, parms, 'MID_JOINTS_POSITION')))
            self.PICK_PRESSURE_EL = float(get_parm(self.devName, parms, 'PICK_PRESSURE_EL'))
            self.REPEATS = int(get_parm(self.devName, parms, 'REPEATS'))
            self.PUT_ANGLE = int(get_parm(self.devName, parms, 'PUT_ANGLE'))
            self.VACUUM_PURGE_DURATION_EL = float(get_parm(self.devName, parms, 'VACUUM_PURGE_DURATION_EL'))
            self.PICK_ANGLE_RANGE = str2int_tuple(get_parm(self.devName, parms, 'PICK_ANGLE_RANGE'))
            self.JOINT_ACC =  int(assign_parm(self.devName, parms, 'JOINT_ACC', JOINT_ACC))

            self.PICKUP_ANGLE = int(get_parm(self.devName, parms, 'PICKUP_ANGLE'))
            self.VACUUM_PURGE_DURATION_PCB = float(get_parm(self.devName, parms, 'VACUUM_PURGE_DURATION_PCB'))
            self.PICKUP_PRESSURE_PCB = float(get_parm(self.devName, parms, 'PICKUP_PRESSURE_PCB'))
            self.PUT_H_COGNEX = float(get_parm(self.devName, parms, 'PUT_H_COGNEX'))
            self.PICKUP_H_COGNEX = float(get_parm(self.devName, parms, 'PICKUP_H_COGNEX'))
            self.PICKUP_OFFSET_PCB = partPos(*str2float_tuple(get_parm(self.devName, parms, 'PICKUP_OFFSET_PCB')))
            self.DEFAULT_PICKUP_PRESSURE = float(get_parm(self.devName, parms, 'DEFAULT_PICKUP_PRESSURE')) if get_parm(self.devName, parms, 'DEFAULT_PICKUP_PRESSURE') is not None else DEFAULT_PICKUP_PRESSURE

            print_log (f'DEBUG: PICKUP_PRESSURE_PCB = {self.PICKUP_PRESSURE_PCB}')
            print_log (f'DEBUG: PICKUP_OFFSET_PCB = {self.PICKUP_OFFSET_PCB} / {str2float_tuple(get_parm(self.devName, parms, "PICKUP_OFFSET_PCB"))}')
            print_log (f'DEBUG: RELIEF_POSITION = {self.RELIEF_POSITION} / {str2float_tuple(get_parm(self.devName, parms, "RELIEF_POSITION"))}, Velocity = {self.VELOCITY}')
            print_log (f'DEBUG: MID_JOINTS_POSITION = {self.MID_JOINTS_POSITION} / {str2float_tuple(get_parm(self.devName, parms, "MID_JOINTS_POSITION"))}')
            
            self.DOWN_VELOCITY = assign_parm(self.devName, parms, 'DOWN_VELOCITY', self.DOWN_VELOCITY)
            self.UP_VELOCITY = assign_parm(self.devName, parms, 'UP_VELOCITY', self.UP_VELOCITY)

            self.__title = get_parm(self.devName, parms, 'TITLE')
            
            _asyril_recipe= None
            _asyrilCfg = get_parm(self.devName, parms, 'ASYRIL')
            if _asyrilCfg is not None:
                _asyrilCfg = _asyrilCfg.split('/')
                
                if len(_asyrilCfg) > 0:
                    _asyrilIPStr = _asyrilCfg[0].strip()
                if len(_asyrilCfg) > 1:
                    _asyril_recipe_str = _asyrilCfg[1].strip()
                    _asyril_recipe = None if not _asyril_recipe_str.isdigit() else int(_asyril_recipe_str)
                    

                # _asyrilIPStr = get_parm(self.devName, parms, 'ASYRIL')


                parm_re = re.compile(r'\b(?:\d{1,3}\.){3}\d{1,3}:\d{1,5}\b')
                if not parm_re.match(_asyrilIPStr):
                    print_err(f'ERROR: Incorrect IP:pot data: {_asyrilIPStr}')
                    return False
                _asyrilIP = str2ip_port(_asyrilIPStr)

                if self.__asyril != None and (self.__asyril.ip_addr != _asyrilIP[0] or self.__asyril.port != int(_asyrilIP[1])):
                    del self.__asyril
                    self.__asyril = None


                if self.__asyril == None:
                    self.__asyril = AsyrilInterface(_asyrilIP[0], int(_asyrilIP[1]), _asyril_recipe)

            _COGNEX_conf =  get_parm(self.devName, parms, 'COGNEX')
            if _COGNEX_conf is not None:
                _COGNEX_conf_IP = _COGNEX_conf.strip()
                parm_re = re.compile(r'\b(?:\d{1,3}\.){3}\d{1,3}:\d{1,5}\b')
                if not parm_re.match(_COGNEX_conf_IP):
                    print_err(f'ERROR: Incorrect IP:pot data: {_COGNEX_conf_IP}')
                    return False
                _COGNEX_conf_IP_port = [0] * 2
                _COGNEX_conf_IP_port = str2ip_port(_COGNEX_conf_IP)

                if self.__cognexCam != None and (self.__cognexCam.ip_addr != _COGNEX_conf_IP_port[0] or self.__cognexCam.port != int(_COGNEX_conf_IP_port[1])):
                    del self.__cognexCam
                    self.__cognexCam = None

                _C = CognexCAM.find_device(_COGNEX_conf_IP_port[0], int(_COGNEX_conf_IP_port[1]))
                if _C:
                    if self.__cognexCam == None:
                        self.__cognexCam = CognexCAM(_COGNEX_conf_IP_port[0], int(_COGNEX_conf_IP_port[1]))
                else:
                    print_log(f'Cognex cant be initalized')
                    self.__cognexCam = None


 
            
            __3DPositions =  get_parm(self.devName, parms, '3DPOSITIONS')
            if __3DPositions is not None:
                if not isinstance(__3DPositions, dict):
                    print_err(f'Wrong Mecademic position structure in list: {__3DPositions}')
                    return False
                
                print_log(f'Mecademic pre-defined positions: {__3DPositions} ')
            
                

    
                # parm_re = re.compile(r'\s*(-?\d+(\.\d+)?\s+-?\d+(\.\d+)?(\s+-?\d+(\.\d+)?)?(\s+-?\d+(\.\d+)?)?)\s*$')
                # parm_re = re.compile(r'\s*(-?\d+(\.\d+)?)|(\.\.\.)\s*(,\s*(-?\d+(\.\d+)?)|(\.\.\.)\s*){3}$')
                parm_re = re.compile(r'(\s*((-?\d+(\.\d+)?)|(\.\.\.))\s*)(,\s*((-?\d+(\.\d+)?)|(\.\.\.))\s*){3}$')
                self.__3Dcoord = dict()
                for _pt, _coord in __3DPositions.items():
                    if not parm_re.match(_coord):
                        print_err(f'ERROR: Wrong coordinate format for point {_pt}: {_coord}')
                        # return False
                    else:
                        _coord_lst = _coord.split(',')
                        _4D = [0] * 4
                        for _ind, _axe in enumerate(_coord_lst):
                            if _axe.strip() == '...':
                                _4D[_ind] = None
                            else:
                                _4D[_ind] = float(_axe.strip())
                        self.__3Dcoord[_pt] = _mcs500_pos(_4D[0], _4D[1], _4D[2], _4D[3])
                        print_log(f'Added point {_pt} with coordinates: {self.__3Dcoord[_pt]}')
                print_log(f'Avaliable points: {self.__3Dcoord}')
            
            __3DPositions =  get_parm(self.devName, parms, '3DOFFSET')
            if __3DPositions is not None:
                if not isinstance(__3DPositions, dict):
                    print_err(f'Wrong Mecademic position structure in list: {__3DPositions}')
                    return False

                print_log(f'Mecademic pre-defined offsets: {__3DPositions} ')
            
                parm_re = re.compile(r'(\s*(-?\d+(\.\d+)?)\s*)(,\s*(-?\d+(\.\d+)?)\s*){3}$')
                self.__3Doffset = dict()
                for _pt, _coord in __3DPositions.items():
                    if not parm_re.match(_coord):
                        print_err(f'ERROR: Wrong coordinate format for point {_pt}: {_coord}')
                        # return False
                    else:
                        _coord_lst = _coord.split(',')
                        _4D = [0] * 4
                        for _ind, _axe in enumerate(_coord_lst):
                            _4D[_ind] = float(_axe.strip())
                        self.__3Doffset[_pt] = _mcs500_pos(_4D[0], _4D[1], _4D[2], _4D[3])
                        print_log(f'Added offsets {_pt} with coordinates: {self.__3Doffset[_pt]}')
                print_log(f'Avaliable offsets: {self.__3Doffset}')



        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'ERROR loading params.')
            return False


        print_inf (f'Loading parms for {self.devName}')
        return True

    @staticmethod
    def find_avilable(_ipAddr:str, _asyrilIPStr:str) -> bool:
        # _robot = mdr.Robot()
        # try:
        #     _robot.Connect(address=_ipAddr, enable_synchronous_mode = SYNC_MODE, \
        #                 disconnect_on_exception=DISCONNECT_MODE)
        # except Exception as ex:
        #     e_type, e_filename, e_line_number, e_message = exptTrace(ex)
        #     print_log(f'Connection to Mecademic robot can not be established: {ex}')
        #     return False
        # try:
        #     _robot.ActivateRobot()
        # except Exception as ex:
        #     e_type, e_filename, e_line_number, e_message = exptTrace(ex)
        #     print_log(f'Activation of Mecademic robot failed: {ex}')
        #     _robot.Disconnect()
        #     return False
        # _robot.DeactivateRobot()
        # _robot.Disconnect()
        status = baseRobot.find_avilable_robot(_ipAddr)
        if not status:
            return False
        
        _as_ip_port = str2ip_port(_asyrilIPStr)
        status = AsyrilInterface.is_avilable(_as_ip_port[0], int(_as_ip_port[1]))
        if status:
            print_log(f'Both Mecademic at {_ipAddr} and Asyril at {_asyrilIPStr} are available')

        return status


# SCRIPT framework for unitestings #################
#
#
#
#
#
#
#####################################################


    def test_automatic_script(self, scriptFile)->bool:
        try:
            with open(scriptFile) as script_file:

                subScript = yaml.safe_load(script_file)

                print_log(f'Script doc = {subScript}\n\n')

                if not self._validateScript(subScript):
                    print_log(f'Incorrect script format')
                    return False


        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_log(f'Script run proc failed: {ex}')
            return False
        
        return True


   


    def _runRobotCmd (self, r_cmd):
        print_log(f'Runnung: {r_cmd}')
        try:
            _splited_cmd = r_cmd.split("(")
            _cmd = _splited_cmd[0]
            
            parm:Optional[str] = None

            if len(_splited_cmd) > 1:
                parm = _splited_cmd[1].split(")")[0]

            
            match _cmd:
                case 'SetJointVel':
                    self.robot.SetJointVel(int(parm))
                    pass
                case 'MovePose':
                    # print_log(f'DEBUG parm = {parm}, cmd = {r_cmd}')
                    float_tuple_parm = tuple(map(float, parm.split(',')))
                    # (x, y, z, gamma) = float_tuple_parm
                    # print_log(f'DEBUG cmd = {r_cmd}, parm = {parm}, float_tuple_parm = {float_tuple_parm}, unpack = {x}, {y}, {z}, {gamma}')
                    # print_log(f'DEBUG cmd = {r_cmd}, parm = {parm}, float_tuple_parm = {float_tuple_parm}')
                    self.wrMovePose(*float_tuple_parm)
                    # self.robot.MovePose(x, y, z, gamma)

                case 'MoveJoints':
                    # print_log(f'DEBUG parm = {parm}, cmd = {r_cmd}')
                    float_tuple_parm = tuple(map(float, parm.split(',')))
                    # print_log(f'DEBUG parm = {parm}, float_tuple_parm = {float_tuple_parm}')
                    self.robot.MoveJoints(*float_tuple_parm)
                    pass
                case 'GripperOpen':
                    self.robot.GripperOpen()
                case 'GripperClose':
                    self.robot.GripperClose()
                
                case 'VacuumGrip':
                    self.robot.VacuumGrip()

                case 'VacuumRelease':
                    self.robot.VacuumRelease()

                case 'Delay':
                    float_tuple_parm = float(parm)
                    self.robot.Delay(float_tuple_parm)
                
                case 'SetVacuumPurgeDuration':
                    float_tuple_parm = float(parm)
                    self.robot.SetVacuumPurgeDuration(float_tuple_parm)


                case _:     
                    print_log(f'Unsupported command: {_cmd}')

            
            print_log(f'Jonts = {self.robot.GetJoints()} Position = {self.robot.GetPose()}')    

        
        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_log(f'Script robot cmd failed: {ex}')




    def runScript(self, scriptFile):
        try:
            with open(scriptFile) as script_file:

                subScript = yaml.safe_load(script_file)

                print_log(f'Script doc = {subScript}\n\n')

                if not self._validateScript(subScript):
                    print_log(f'Incorrect script format')
                    sys.exit()

                for  _key, _item  in subScript.items():
                    _splited_cmd = _item.split("(")
                    _cmd = _splited_cmd[0]
                    
                    parm:Optional[str] = None

                    if len(_splited_cmd) > 1:
                        parm = _splited_cmd[1].split(")")[0]

                
                    match _cmd:
                        case 'SetJointVel':
                            self.robot.SetJointVel(int(parm))
                            pass
                        case 'MovePose':
                            float_tuple_parm = tuple(map(float, parm.split(',')))
                            self.wrMovePose(*float_tuple_parm)
                        case 'MoveJoints':
                            float_tuple_parm = tuple(map(float, parm.split(',')))
                            self.robot.MoveJoints(*float_tuple_parm)
                            pass
                        case 'GripperOpen':
                            self.robot.GripperOpen()
                        case 'GripperClose':
                            self.robot.GripperClose()
                        
                        case 'VacuumGrip':
                            self.robot.VacuumGrip()

                        case 'VacuumRelease':
                            self.robot.VacuumRelease()

                        case 'Delay':
                            float_tuple_parm = float(parm)
                            self.robot.Delay(float_tuple_parm)
                        
                        case 'SetVacuumPurgeDuration':
                            float_tuple_parm = float(parm)
                            self.robot.SetVacuumPurgeDuration(float_tuple_parm)


                        case _:     
                            print_log(f'Unsupported command: {_cmd}')
                    
                    print_log(f'Jonts = {robot.GetJoints()} Position = {robot.GetPose()}')

            
            while True:
                print_log(f'Jonts = {robot.GetJoints()} Position = {robot.GetPose()}')
                time.sleep(0.5)
                response_codes = [mdr_def.MxRobotStatusCode.MX_ST_GET_STATUS_ROBOT]
                response = self.robot.SendCustomCommand('GetStatusRobot', expected_responses=response_codes, timeout=10)
                _stat_res = tuple(map(int, response.data.split(',')))
                print_log(f'Status response = {response.data} / {_stat_res}')
                
                if _stat_res[0] == 1 and _stat_res[1] == 1 and  _stat_res[5] == 1 and _stat_res[6] == 1  :
                    print_log(f'Exiting. Status = {response.data}/{_stat_res}')
                    break


        except KeyboardInterrupt as ex:
            print_log(f'Exiting by ^C \n{ex}')
            return
        

        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_log(f'Script run proc failed: {ex}')
            

    def runMecademicAsyrilScript(self, putPosition:partPos = None) -> bool:

        if putPosition == None:
            if not isinstance(self.robot_put_pos.x, float) or not isinstance(self.robot_put_pos.y, float):
                print_err(f'Incorrect put position defined using GUI: {self.robot_put_pos}')
                return False
            else:
                putPosition = self.robot_put_pos
        else: 
            self.robot_put_pos = putPosition 
            
        try:
            self._runRobotCmd(self.robot, f'SetJointVel({self.VELOCITY})')
            self._runRobotCmd(self.robot, f'SetVacuumPurgeDuration({self.VACUUM_PURGE_DURATION_EL})')
            # self._runRobotCmd(robot, f'MoveJoints{self.RELIEF_POSITION}')
            self._runRobotCmd(robot, f'MoveJoints({self.RELIEF_POSITION.x}, {self.RELIEF_POSITION.y}, {self.RELIEF_POSITION.z}, {self.RELIEF_POSITION.gamma})')



            for _rep in range (self.REPEATS):    
                _x, _y, _rz, _dir_x, _dir_y = self.__asyril.get_pos()
                print_log(f'x= {_x}, y= {_y}, rz = {_rz}')
                print_log (f'Try # {_rep}')
                self._runRobotCmd(robot, f'VacuumRelease()')
                self._runRobotCmd(robot, f'MovePose({_x}, {_y}, {self.WORKING_H}, {_rz})')
                self._runRobotCmd(self.robot, f'SetJointVel({self.DOWN_VELOCITY})')
                self._runRobotCmd(robot, f'MovePose({_x}, {_y}, {self.PICKUP_H_ASYRIL},  {_rz})')
                self.getRobotPosition()
                self._runRobotCmd(robot, 'VacuumGrip()')
                self._runRobotCmd(robot, f'Delay({self.PICK_DELAY})')
                self._runRobotCmd(self.robot, f'SetJointVel({self.VELOCITY})')

                self._runRobotCmd(robot, f'MovePose({_x}, {_y}, {self.WORKING_H}, {_rz})')

                # self._runRobotCmd(robot, f'MoveJoints{self.MID_JOINTS_POSITION}')
                self._runRobotCmd(robot, f'MoveJoints({self.MID_JOINTS_POSITION.x}, {self.MID_JOINTS_POSITION.y}, {self.MID_JOINTS_POSITION.z}, {self.MID_JOINTS_POSITION.gamma})')
                self._runRobotCmd(robot, f'Delay({self.PICK_DELAY})')

                if self._getPressure(robot) < self.PICK_PRESSURE_EL:
                    _rep += 1
                    break

            if _rep > self.REPEATS:
                print_err(f'{self.REPEATS} unsuccessful attempts to pich the part')
                self._runRobotCmd(robot, f'VacuumRelease()')
                return False
            
            self._runRobotCmd(robot, f'MovePose({putPosition.x}, {putPosition.y}, {self.WORKING_H}, {self.PUT_ANGLE})')
            self._runRobotCmd(robot, f'MovePose({putPosition.x}, {putPosition.y}, {self.PUT_H_ASYRIL-10}, {self.PUT_ANGLE})')
            self.asyncCompletionAwait(self)
            self._runRobotCmd(self.robot, f'SetJointVel({self.DOWN_VELOCITY})')
            self._runRobotCmd(robot, f'MovePose({putPosition.x}, {putPosition.y}, {self.PUT_H_ASYRIL}, {self.PUT_ANGLE})')
            self.getRobotPosition()
            self._runRobotCmd(robot, f'VacuumRelease()')
            self._runRobotCmd(robot, f'Delay({self.PUT_DELAY})')

            self._runRobotCmd(self.robot, f'SetJointVel({self.VELOCITY})')
            self._runRobotCmd(robot, f'MovePose({putPosition.x}, {putPosition.y}, {self.WORKING_H}, {self.PUT_ANGLE})')
            self._runRobotCmd(robot, f'MoveJoints({self.RELIEF_POSITION.x}, {self.RELIEF_POSITION.y}, {self.RELIEF_POSITION.z}, {self.RELIEF_POSITION.gamma})')

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

        except Exception as ex:
            print_log(f'Script run proc failed: {ex}')
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_log(f'Script run proc failed: {ex}')
            try:
                self.robot.VacuumRelease()
                _pos = self.getRobotPosition()

                print_log(f'Going up to: {_pos}')
                self.wrMovePose(_pos[1], _pos[2], self.RELIEF_POSITION.z, _pos[4])

                self.wrMovePose(self.RELIEF_POSITION.x, self.RELIEF_POSITION.y, self.RELIEF_POSITION.z, self.RELIEF_POSITION.gamma)
            
            except Exception as ex:
                e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            
            return False
        
        return True


    def _validateScript(self, _script:dict) -> bool:
        _cmdList = ['SetJointVel', 'MovePose', 'MoveJoints', 'GripperOpen', 'GripperClose', 'VacuumGrip', 'VacuumRelease', 'Delay', 'SetVacuumPurgeDuration' ]

        _model = self.robot.GetRobotInfo().robot_model
        print_log(f'Robot = {glRobot}, Model = {_model} / {"meca500" if tools.robot_model_is_meca500(_model) else "mcs500" if tools.robot_model_is_mcs500(_model) else "unknown"}   ')
        
        print_log(f'Script:')
        for  _key, _item  in _script.items():
            print_log(f'{_key}-> {_item}')
            _splited_cmd = _item.split("(")
            _cmd = _splited_cmd[0]
            if not _cmd in _cmdList: 
                print_log(f'Unrecognized cmd: {_item}')
                return False
            

            match  _cmd:

                case 'MovePose' | 'MoveJoints':
                    if not len(_splited_cmd) == 2:
                        print_log(f'Error parsing cmd: {_item} / {_splited_cmd}')
                        return False
                    if tools.robot_model_is_meca500(_model):     
                        cmd_re = re.compile(r'\s*-?\d+(\.\d+)?\s*,\s*-?\d+(\.\d+)?\s*,\s*-?\d+(\.\d+)?\s*,\s*-?\d+(\.\d+)?\s*,\s*-?\d+(\.\d+)?\s*,\s*-?\d+(\.\d+)?\s*\)\s*')
                    
                    
                    # elif tools.robot_model_is_mcs500(_model):
                    else:
                        cmd_re = re.compile(r'\s*-?\d+(\.\d+)?\s*,\s*-?\d+(\.\d+)?\s*,\s*-?\d+(\.\d+)?\s*,\s*-?\d+(\.\d+)?\s*\)\s*')
                        
                    # else:
                    #     print_log(f'Unsupported robot model: {_model}')
                    #     return False

                    if not cmd_re.match(_splited_cmd[1]):
                        print_log(f'Error parsing cmd: {_splited_cmd[1]}')
                        return False
                    
                    if not cmd_re.match(_splited_cmd[1]):
                        print_log(f'Error parsing cmd: {_splited_cmd[1]}')
                        return False


                case 'SetJointVel':
                    if not len(_splited_cmd) == 2:
                            print_log(f'Error parsing cmd: {_item} / {_splited_cmd}')
                            return False
                    cmd_re = re.compile(r'\s*\b([1-9]|[1-9][0-9]|100)\b\s*\)\s*')
                    if not cmd_re.match(_splited_cmd[1]):
                        print_log(f'Error parsing cmd: {_splited_cmd[1]}')
                        return False
                
                case 'GripperOpen' | 'GripperClose' | 'VacuumGrip' | 'VacuumRelease':
                    cmd_re = re.compile(r'\s*\)\s*')
                    if (not len(_splited_cmd) == 1) and (not cmd_re.match(_splited_cmd[1])):
                        print_log(f'Error parsing cmd: {_item} / {_splited_cmd}')
                        return False
                    
                case 'Delay' | 'SetVacuumPurgeDuration':
                    cmd_re = re.compile(r'\s*-?\d+(\.\d+)?\s*\)\s*')
                    if (not len(_splited_cmd) == 2) and (not cmd_re.match(_splited_cmd[1])):
                            print_log(f'Error parsing cmd: {_item} / {_splited_cmd}')
                            return False

                case _:
                    print_log(f'Unsupported cmd: {_cmd}')
                    return False
                

        return True
    
   

        
        

def str2float_tuple(_str:str) -> tuple:
    _str = re.sub(r"\s+", "", _str)
    return tuple(map(float, _str.split(',')))

def str2int_tuple(_str:str) -> tuple:
    _str = re.sub(r"\s+", "", _str)
    return tuple(map(int, _str.split(',')))
    
def str2ip_port(_str:str) -> tuple:
    _str = re.sub(r"\s+", "", _str)
    return tuple(map(str, _str.split(':')))



if __name__ == "__main__":

    if not len(sys.argv) == 2:
        print_log(f'Usage: python {sys.argv[0]} file_name')
        sys.exit()


    _parm = sys.argv[1]
    _fileName = None
    _ip = None
    _port = 0

    parm_re = re.compile(r'\b(?:\d{1,3}\.){3}\d{1,3}:\d{1,5}\b')
    if not parm_re.match(_parm) and not os.path.isfile(_parm):
        print_log(f'The parameter {_parm} is nor IP:port or existing file. Usage: python {sys.argv[0]} file_name|IP:port')
        sys.exit()


    if  os.path.isfile(_parm):
        _fileName = _parm
    else:
        _ip = _parm.split(':')[0]
        _port = int(_parm.split(':')[1])
        print(f'IP = {_ip}, port = {_port} --> {_parm}')
      



    robot = mdr.Robot()
    glRobot = robot

    # test_automatic_script(_fileName)
    # sys.exit()

    _model = robot.GetRobotInfo().model

    print_log(f'Robot: {_model}, scrtipt file: {_fileName}')

    callbacks = mdr.RobotCallbacks()
    callbacks.on_error = _errorCallback
    callbacks.on_connected = _on_connectedCallback
    callbacks.on_disconnected = _on_disconnectedCallback
    callbacks.on_activated = _on_activatedCallback
    callbacks.on_deactivated = _on_deactivatedCallback
    callbacks.on_homed = _on_homedCallback
 

    robot.RegisterCallbacks(callbacks=callbacks, run_callbacks_in_separate_thread=True)

    if not _connect(robot):
        print_log(f'Connection failed, exiting')
        sys.exit()

    if not _activate(robot):
        print_log(f'Activation failed, exiting')
        _stop_clear(robot)
        sys.exit()

    if _fileName:
        runScript(robot, _fileName)
    elif _ip and _port > 0:
        runAsyril(robot, _ip, _port)
    else:
        print('WTF error')



    _stop_clear(robot)
