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
from bs1_mcdmc_base import baseRobot, contexFuncParms, robotOpType


POS_UPDATE_RESOLUTION = 1

_meca500_posFields = ['x', 'y', 'z', 'alpha', 'beta', 'gamma']
_meca500_pos = namedtuple("_meca500_pos",  _meca500_posFields, defaults=[None,] * len(_meca500_posFields))
_meca500_pos.__annotations__={'x':float,  'y':float,  'z':float, 'alpha':float, 'beta':float, 'gamma':float}

class robotMeca500(baseRobot):

    def __init__(self, devName,  IP, parms):
        super().__init__(devName,  IP, parms)

        self.robot_pos:_meca500_pos = _meca500_pos(0,0,0,0,0,0)
        self.__workingPoint:_meca500_pos = None
        self.wd = None
        self.__3Dcoord:dict = None

        self.__stored_pos = _meca500_pos(0, 0, 0, 0, 0, 0)

        self.set_parms(parms = parms)

        self._init()

        self._connect()

        self._activate()
        
        self.robot.SetConf(elbow = self.__elbow)

    def __del__(self):
        pass

    def set_parms(self, parms)->bool:
        try:
            __3DPositions =  get_parm(self.devName, parms, '3DPOSITIONS')
            if __3DPositions is not None:
                if not isinstance(__3DPositions, dict):
                    print_err(f'Wrong Mecademic position structure in list: {__3DPositions}')
                    return False
                
                print_log(f'Mecademic pre-defined positions: {__3DPositions} ')
            
            __3DPositions =  get_parm(self.devName, parms, '3DOFFSET')
            if __3DPositions is not None:
                if not isinstance(__3DPositions, dict):
                    print_err(f'Wrong Mecademic position structure in list: {__3DPositions}')
                    return False

                print_log(f'Mecademic pre-defined offsets: {__3DPositions} ')
                

    
                parm_re = re.compile(r'(\s*((-?\d+(\.\d+)?)|(\.\.\.))\s*)(,\s*((-?\d+(\.\d+)?)|(\.\.\.))\s*){5}$')
                self.__3Dcoord = dict()
                for _pt, _coord in __3DPositions.items():
                    if not parm_re.match(_coord):
                        print_err(f'ERROR: Wrong coordinate format for point {_pt}: {_coord}')
                    else:
                        _coord_lst = _coord.split(',')
                        _4D = [0] * 6
                        for _ind, _axe in enumerate(_coord_lst):
                            if _axe.strip() == '...':
                                _4D[_ind] = None
                            else:
                                _4D[_ind] = float(_axe.strip())
                        self.__3Dcoord[_pt] = _meca500_pos(_4D[0], _4D[1], _4D[2], _4D[3], _4D[4], _4D[5])
                        print_log(f'Added point {_pt} with coordinates: {self.__3Dcoord[_pt]}')
                print_log(f'Avaliable points: {self.__3Dcoord}')


                parm_re = re.compile(r'(\s*(-?\d+(\.\d+)?)\s*)(,\s*(-?\d+(\.\d+)?)\s*){5}$')
                self.__3Doffset = dict()
                for _pt, _coord in __3DPositions.items():
                    if not parm_re.match(_coord):
                        print_err(f'ERROR: Wrong coordinate format for point {_pt}: {_coord}')
                    else:
                        _coord_lst = _coord.split(',')
                        _4D = [0] * 6
                        for _ind, _axe in enumerate(_coord_lst):
                            _4D[_ind] = float(_axe.strip())
                        self.__3Doffset[_pt] = _meca500_pos(_4D[0], _4D[1], _4D[2], _4D[3], _4D[4], _4D[5])
                        print_log(f'Added offsets {_pt} with coordinates: {self.__3Doffset[_pt]}')
                print_log(f'Avaliable offsets: {self.__3Doffset}')


        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'ERROR loading params.')
            return False


    @staticmethod
    def find_avilable(_ipAddr:str) -> bool:

        return baseRobot.find_avilable_robot(_ipAddr)


    def pickUpPart(self, __parm:contexFuncParms)->bool:
        raise(f'pickup command is not supported for device {self.devName} of type {self.type}')

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
            _current_pos = _meca500_pos(*_r_current_pos)
            print_log(f'Moving absolute to {_pos}, Current position = {_current_pos}')

            if __movePoint.x is None and __movePoint.y is None and __movePoint.z is None\
             and __movePoint.alpha is None and __movePoint.beta is None and __movePoint.gamma is None:
                print_log(f'Nothing to do for {_pos}: {__movePoint}')
                return True
                                                                        # if any exe is None use current pos
            _move_X = __movePoint.x if __movePoint.x is not None else _current_pos.x
            _move_Y = __movePoint.y if __movePoint.y is not None else _current_pos.y
            _move_Z = __movePoint.z if __movePoint.z is not None else _current_pos.z
            _move_ALPHA = __movePoint.alpha if __movePoint.alpha is not None else _current_pos.alpha
            _move_BETA = __movePoint.beta if __movePoint.beta is not None else _current_pos.beta
            _move_GAMMA = __movePoint.gamma if __movePoint.gamma is not None else _current_pos.gamma
            
            _vel = _velocity if _velocity is not None else self.VELOCITY
            self.robot.SetJointVel(_vel)
            self.robot.MovePose(_move_X, _move_Y, _move_Z, alpha=_move_ALPHA, beta=_move_BETA, gamma = _move_GAMMA)

            self.asyncCompletionAwait()
                        
        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err('ERROR: Moving failed. Error: {ex}')
            return False
        
        return True

    def moveSafe(self, __parm:contexFuncParms)->bool: 
        raise(f'moveSafe command is not supported for device {self.devName} of type {self.type}')



    def moveRel(self, __parm:contexFuncParms)->bool: 
        try:

            if __parm.coord is not None:
                _rel:_meca500_pos = __parm.coord
            elif  __parm.point is not None:
                _pos:str = __parm.point
                if not _pos in self.__3Doffset.keys():
                    raise Exception(f'No coordinates defined for {_pos}')
                _rel:_meca500_pos =  self.__3Doffset[_pos]
            else:
                print_err(f'Incorrect parameters for moving: {__parm}')
                return False

            _velocity:int = __parm.velocity
        
            _r_current_pos = self.robot.GetRtTargetCartPos()
            _current_pos = _meca500_pos(*_r_current_pos)

            print_log(f'Moving relative by {_rel}. Current position = {_current_pos}')

            if _rel.x == 0 and _rel.y == 0 and _rel.z == 0 and _rel.alpha == 0 and _rel.beta == 0 and _rel.gamma == 0:
                print_log(f'Nothing to do for {_rel}')
                return True
                                                                        # if any exe is None use current pos
            _move_X =  _current_pos.x + _rel.x
            _move_Y = _current_pos.y + _rel.y
            _move_Z = _current_pos.z + _rel.z
            _move_ALPHA = _current_pos.alpha + _rel.alpha 
            _move_BETA = _current_pos.beta + _rel.beta 
            _move_GAMMA = _current_pos.gamma + _rel.gamma 
            
            _vel = _velocity if _velocity is not None else self.VELOCITY
            self.robot.SetJointVel(_vel)
            self.robot.MovePose(_move_X, _move_Y, _move_Z, alpha=_move_ALPHA, beta=_move_BETA, gamma = _move_GAMMA)

            self.asyncCompletionAwait()

        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err('ERROR: Moving failed. Error: {ex}')
            return False
        
        return True

    def moveUp(self)->bool:
        raise(f'moveUp command is not supported for device {self.devName} of type {self.type}')

    @property
    def getPos(self)->_meca500_pos:
        _pos:_meca500_pos = _meca500_pos(0, 0, 0, 0, 0, 0)
        _current_time = time.time()     
        if _current_time - self._last_pos_update_time < POS_UPDATE_RESOLUTION:
            return self.__stored_pos
        
        try:
            # _r_pos = self.robot.GetPose()
            _r_pos = self.robot.GetRtTargetCartPos()
            _pos = _meca500_pos(*_r_pos)
        except Exception as ex:
            exptTrace(ex)
            print_log(f'Robot getting position failed: {ex}')
            return _meca500_pos(0, 0, 0, 0, 0, 0)
        
        self._last_pos_update_time = _current_time

        self.__stored_pos = _pos 

        return _pos
