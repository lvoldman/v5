from __future__ import annotations

__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["Leonid Voldman","Sasha Schechtman"]
__license__ = "SLA"
__version__ = "2.0.1"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Prototype"

import os, sys, time, re
import PySimpleGUI as sg

from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, real_num_validator, \
    int_num_validator, real_validator, int_validator, globalEventQ, smartLocker, clearQ, globalEventQ, event2GUI

from bs2_config import CDev, systemDevices

from enum import Enum
from queue import Queue 
from threading import Thread, Lock, get_ident 
from dataclasses import dataclass
from collections import namedtuple

# from playsound import playsound
import sounddevice as sd
import soundfile as sf




import os.path
from pathlib import Path

# from typing import List

import bs1_mecademic as mc

# from bs1_phidget import PhidgetRELAY


RunType = Enum("RunType", ["parallel", "serial", "single", "simultaneous"])


# CmdObj = namedtuple("CmdObj", ["device", "cmd"])
# CmdObj.__annotations__={'device':CDev, 'cmd':OpType}         # specify type of elements

OpType = Enum("OpType", ["go_to_dest", "go_fwrd_on", "go_bcwrd_off", "home", "vibrate", "stop", "stall", "delay", "halt", "trigger", \
                         "go_rnd", "calibrate", "check", "terminate","hotair", "measure", "phg_trigger", "phg_toggle", "phg_on", "phg_off", "phg_operate", \
                        "pick_n_put", "insert_pcb", "put_pcb", \
                        # "moveabs", "movesafe", "moverel", "moveup",  "pickup",\
                        "start_robot_op", "vacuum", \
                        "set_program", "single_shot", "play_media", "add_db", "nop", \
                        "unparsed_cmd"   ])

argsTypeFields = ["position", "end_position", "velocity", "time", "profile", "calibr_selector", "abs", "trigger_selector", "robot_op", "robot_parm" ,\
                  "start_stop", "curr", "volt", "light_on", "x_coord", "y_coord", "p3D", "z_coord", "gamma_coord", "media_file", "polarity", \
                 "cmd_num", "stall", "cmd_txt" ]
argsType = namedtuple("argsType",  argsTypeFields, defaults=[None,] * len(argsTypeFields))



@dataclass
class CmdObj:
    device:CDev = None          # device to perate
    cmd:OpType = None           # command to run
    args:argsType = None        # parameters for cms (position, time, etc)     

    @property
    def operation(self):        # cmd text fore unparsed_cmd
        return self.args.cmd_txt

# StatusType = Enum("StatusType", ["disabled", "in_motion", "available"])
# @dataclass
# class DevStatus:
#     device:CDev = None
#     event:int = None
#     status:StatusType = StatusType.disabled

@dataclass
class TaskObj:
    wTask:WorkingTask = None
    status:bool = False                     # True - in progress, False = not on progress
    threadID:Thread = None
    def __repr__(self) -> str:
        return self.wTask.__repr__()


taskResFields = ["wTaskID", "result", "device"]
taskRes = namedtuple("taskRes",  taskResFields, defaults=[None,] * len(taskResFields))
taskRes.__annotations__={'wTaskID':int,  'result':bool, 'device':str}         # specify type of elements


class WorkingTask:                                  # WorkingTask - self-recursive object structure where each object 
                                                    # is a single command or list of objects of WorkingTask type, that may be 
                                                    # operated in serial or paralel (simultaneously) manner
    def __init__(self, taskList = None, sType = RunType.parallel, stepTask:bool = False):
        self.__sub_tasks = list()                       # list of objects of type TaskObj or CmdObj (at __sub_tasks[0])
        self.__task_type:RunType = sType                       # running sequence: parallel/serial
        self.__reportQ: Queue =  Queue()                        # Queue() object for termination notification 
        self.__emergency_stop: bool = False
        self.__id = id(self)
        self.__stepTask:bool = stepTask

        if taskList == None:                    # empty task (i.e. no active dev)
            print_err(f'-WARNING- Empty task being loaded')
            return
        

        if type(taskList) == CmdObj:           # single command - end of recursion / otherwise list of commands
            self.__task_type = RunType.single
            # subTask = TaskObj(wTask=taskList, status=False)
            # self.__sub_tasks.append(subTask)
            self.__sub_tasks.append(taskList)



        elif (sType == RunType.parallel or sType == RunType.serial) and type(taskList) == list:
            self.__sub_tasks = [None] * len(taskList)
            for i, tsk in enumerate(taskList):
                subTask = TaskObj(wTask=tsk, status=False)
                self.__sub_tasks[i] = subTask
        else:
            print_err(f'ERROR task list initiation of {sType} run type')

    def isStep(self):
        return self.__stepTask
    
    def singleTaskRepr(self):
        retStr:str = None
        if self.is_single():
            # retStr = f'{self.__sub_tasks[0].device.get_device().devName}({self.__sub_tasks[0].cmd}, {self.__sub_tasks[0].args})'
            if self.__sub_tasks[0].device:
                retStr = f'{self.__sub_tasks[0].device.get_device().devName}({self.__sub_tasks[0].cmd})'
            else:
                retStr = f' {self.__sub_tasks[0].device}({self.__sub_tasks[0].cmd})'


        return retStr

    def __repr__(self) -> str:
        _tsk_repr = None
        _type:str = ''
        match self.__task_type:
            case RunType.parallel:
                _type = '<parallel>'
                _tsk_repr = self.__sub_tasks
    
            case RunType.serial:
                _type = '<serial>'
                _tsk_repr = self.__sub_tasks

            case RunType.single:
                _type = '<single>'
                _tsk_repr = self.singleTaskRepr()

            case _:
                _type = '<UNKNOWN>'


        # r_str = f'[WorkingTask: Type={self.__task_type}, sub={self.__sub_tasks}, id={self.__id}, step = {self.__stepTask}]'

        if self.__task_type == RunType.single:
            _tsk_inf = 'CMD =' + f'{_tsk_repr}'
        else:
            _tsk_inf = '\n >>> ' + f'{_tsk_repr}'


        r_str = f"[WorkingTask: {_type}, <<{self.__id}>>, {_tsk_inf}]"

        return (r_str)  

    def __del__(self):
        pass

    def id(self):
        return self.__id

    def is_single(self):
        return (self.__task_type == RunType.single) or (self.__sub_tasks == None)
    
    def exploreDevs(self)-> list[CDev]:
        dList:list[CDev] = list()
        dev:TaskObj = None
        if self.__task_type == RunType.single:
            dList.append(self.__sub_tasks[0].device)
        elif self.__sub_tasks and len(self.__sub_tasks) > 0:                                           # non single task object
            for dev in self.__sub_tasks:
                dList.extend(dev.wTask.exploreDevs())

        return dList

  
    def EmergencyStop(self):
                                                        # This method is generaly accessible from another context (thread)
                                                        # but it  PURPOSELY is not protected by mutex to enable emergency op 
                                                        # at waiting state. Even operation completed the device will be stoped 
                                                        # twice
        print_log(f'EmergencyStop for task {self}')
        self.__emergency_stop = True
        
        if self.__sub_tasks == None or len(self.__sub_tasks) == 0:                    # empty task. do nothing
            print_err(f'-WARNING- Empty task. Nothing to do with Emergency Stop. Exiting')
            return
        
        if self.__task_type == RunType.parallel:        # send stop to each dev
            for working_tsk in self.__sub_tasks:
                if working_tsk.status:
                    working_tsk.wTask.EmergencyStop()

        elif self.__task_type == RunType.serial:
            for working_tsk in self.__sub_tasks:
                if working_tsk.status:
                    working_tsk.wTask.EmergencyStop()
                    break                               # no simultanious run in serial mode
            
        elif self.__task_type == RunType.single:
            devTmp =  self.__sub_tasks[0].device
            if devTmp:
                devTmp.get_device().mDev_stop()
            else:
                print_err(f'-WARNING- null device - for {self.__sub_tasks[0]}')
                      
    def __parallelProcessingThread(self, wTask:WorkingTask, __reportQ:Queue, window:sg.Window):
        opResult = wTask.run(window=window)
        __reportQ.put(taskRes(wTaskID=wTask.id(), result=opResult.result, device= opResult.device))
        
    def __resolveTaskIndex(self, wTaskID:int) -> int:
        for index, working_tsk in enumerate(self.__sub_tasks):   
            if wTaskID == working_tsk.wTask.id():
                return index
        print_err(f'Cant resolve taskID {wTaskID} in tasks list. size =  {len(self.__sub_tasks)}, Tasks:')
        print_err(f'{self.__sub_tasks}')
        return -1

    def run(self, window:sg.Window = None) -> taskRes:

        print_log(f'Starting RUN at task {self}')
        if (self.__sub_tasks == None) or (len(self.__sub_tasks) == 0):                    # empty task. do nothing
            print_err(f'-WARNING-  enpty task, id = {self.__id}. Exiting.')
            return taskRes(result=True)
                
        self.__emergency_stop = False
        wTaskCounter = 0
        if self.__task_type == RunType.parallel:
            __paralelStatus = True                          # True if all paralel tasks succeeded
            print_log(f'WorkigTasks - paralel series')
            for working_tsk in self.__sub_tasks:
                if self.__emergency_stop:
                    break
                working_tsk.status = True
                working_tsk.threadID = Thread(target=self.__parallelProcessingThread, args=(working_tsk.wTask, self.__reportQ, window))
                working_tsk.threadID.start()
                wTaskCounter += 1
            
            print_log(f'Number of subtask in paralel: {wTaskCounter}')
            while wTaskCounter > 0: 
                print_log(f'Waiting subtask to complete (paralel proceeding)')
                tRes:taskRes = self.__reportQ.get()
                # self.__reportQ.task_done()
                wTaskID = tRes.wTaskID  
                opResult = tRes.result
                print_log(f'Subtask id = {wTaskID} completed opResult = {opResult} (paralel proceeding)')
                taskIndex = self.__resolveTaskIndex(wTaskID)

                if taskIndex < 0:
                    print_err(f'ERROR resolving sub task. ID = {wTaskID}')
                    return  taskRes(result=False, device='System error')
                
                working_tsk = self.__sub_tasks[taskIndex]
                working_tsk.threadID.join()                 # thread completed
                working_tsk.status = False
                if not opResult:                            # operation failed. Emeregency stop must be performed 
                    # return tRes                           
                    __paralelStatus = tRes.result                 # but it should not affect task running in paralel op
                    __device = tRes.device 
                                                              # continue loop
                wTaskCounter -= 1

            if not __paralelStatus:                         # if one of tasks failed
                return taskRes(result = False, device = __device)
            
        elif self.__task_type == RunType.serial:
            print_log(f'WorkigTasks - serial series')
            for working_tsk in self.__sub_tasks:
                working_tsk.status = True
                if self.__emergency_stop:
                    break
                tRes = working_tsk.wTask.run(window=window)
                working_tsk.status = False
                if not tRes.result:                                 # operation failed. Emeregency stop must be performed 
                     return tRes


            pass
        elif self.__task_type == RunType.single:
            print_log(f'WorkigTasks - single [device cmd] at {self.__sub_tasks[0].device} ')
            self.status = True
            if self.__sub_tasks[0].device:                              # if valid device (not None)
                                                                        # device cmd 
                devPtr = self.__sub_tasks[0].device.get_device()       #__sub_task of CmdObj type
                clearQ(devPtr.devNotificationQ)
                
            # opResult, toBlock = self.__runCmd(window=window)        # run command for device 
            opResult, toBlock = self.__runDevCmd(window=window)        # load and than Run  command for device                                                                     
                                                                                #  (the command will be parsed by device)
            
            print_log(f'WorkigTasks initiation done! with (block = {toBlock},result ={opResult}) [device cmd] at {self.__sub_tasks[0].device} ')
            
            if not opResult  :                           # operation succeded & toBlock = True (need wait for result)
                print_err(f'Device {self.__sub_tasks[0].device} returned ERROR on operation {self.__sub_tasks[0].cmd}')
          
            elif toBlock:
                print_log(f'Waiting device to complete the motion/operation. Device = {devPtr.devName}')
                opResult = devPtr.devNotificationQ.get()         # the ONLY block untill completed
                # devPtr.devNotificationQ.task_done()
                print_log(f'Device operation completed. Device = {devPtr.devName}. Result = {opResult}')
            
            self.__sub_tasks[0].status = False

            if not opResult:                                    # operation failed. Emeregency stop must be performed 
                print_err(f'Device {self.__sub_tasks[0].device} returned ERROR on THREAD operation {self.__sub_tasks[0].cmd}')
                return  taskRes(result = False, device = self.__sub_tasks[0].device.get_device().devName)
        elif self.__task_type == RunType.simultaneous: 
                                                                # 1) load all commands to devices (PLCDev:loadDevicesOp)
                                                                # 2) start runner (PLCDev:runDevicesOp) to proceed all devices assigned to the runner
            print_log(f'WorkigTasks - simultaneous series')
            # BUGBUG: implement simultaneous run type
            print_err(f'ERROR - simultaneous run type is not implemented yet')
            return  taskRes(result = False, device = 'System error')
        else:
            print_err(f'ERROR - undefined run type: {self.__task_type}')
            return taskRes(result = False, device = 'System error')
        

        return  taskRes(result = True)
    
    # def __clearQ(self, _Q:Queue):
    #     iCount = 0
    #     while not _Q.empty():
    #         _Q.get()
    #         iCount += 1
    #     if iCount > 0:
    #         print_log(f'reportQ had {iCount} unproceeded messages')



    # ["go_to_dest", "go_rnd",  "go_fwrd_on", "go_bcwrd_off", "home", "vibrate", "stop", "delay", "halt", "trigger", "calibrate", "check", "terminate","hotair", 
    # "phg_trigger", "phg_toggle", "phg_on", "phg_off", "phg_operate", "pick_n_put", "moveabs", "movesafe", "moverel", "moveup", "vacuum", 
    # "pickup", "insert_pcb", "put_pcb", set_program", "single_shot", "add_db","play_media"]
    # runType = single, task type of CmdObj
    # sg.Window is for events communication (i.e. 'Stop' event)
    # Obsolete, use __runDevCmd instead
    def __runCmd(self, window:sg.Window = None)-> tuple[bool, bool]:             
                                        # run command for single device
                                        # returns (opResult, toBlock) - operation result and toBlock flag

        wCmd:CmdObj = self.__sub_tasks[0]         # single command
        print_log(f'Proceeding cmd = {wCmd}')

        if wCmd.device:                          
            devPtr = wCmd.device.get_device()
        else:
            devPtr = None
            if not (wCmd.cmd == OpType.delay or  wCmd.cmd == OpType.halt or  wCmd.cmd == OpType.play_media or  wCmd.cmd == OpType.nop):
                print_err(f'ERROR- No device defined fot smd = {wCmd}')
                return False, False    

        toBlock:bool = True 
        opResult:bool = True

        match wCmd.cmd:
            case OpType.go_to_dest:
                toBlock = True
                if devPtr.devName[0] == 'T' or devPtr.devName[0] == 'R' or devPtr.devName[0] == 'G' or devPtr.devName[0] == 'Z' or devPtr.devName[0] == 'D':
                    opResult = devPtr.go2pos(new_position = wCmd.args.position, velocity = wCmd.args.velocity, stall = wCmd.args.stall )
                    print_log(f'-> go2pos(new_position = {wCmd.args.position}, velocity = {wCmd.args.velocity}, stall = {wCmd.args.stall})')
                else:
                    print_err(f'ERROR - No go_to_dest command available for [{devPtr.devName}] device')
                    opResult = False

            case OpType.go_rnd:
                toBlock = True
                if not (devPtr.devName[0] == 'Z'):
                    print_err(f'ERROR - No goRND command available for [{devPtr.devName}] device')
                    opResult =  False
                
                opResult = devPtr.rndMove(start = wCmd.args.position, end = wCmd.args.end_position)
                print_log(f'-> go_rnd(start = {wCmd.args.position}, end = {wCmd.args.end_position})')

            case OpType.go_fwrd_on:
                toBlock = True
                print_log(f'-> mDev_forward/gripper_on/timeRotaterFw()')

                if devPtr.devName[0] == 'T' :
                    if wCmd.args:
                        _stall = wCmd.args.stall
                        opResult = devPtr.mDev_forward(stall = _stall)
                    else:
                        opResult = devPtr.mDev_forward()
                
                elif devPtr.devName[0] == 'R':
                    _polarity = None
                    _vel = None
                    _time = None
                    if wCmd.args:
                        _time = wCmd.args.time
                        _vel = wCmd.args.velocity
                        if  not (wCmd.args.polarity == None):
                            _polarity = wCmd.args.polarity
                        
                    opResult = devPtr.mDev_forward(velocity=_vel, timeout=_time, polarity = _polarity)

                elif devPtr.devName[0] == 'G' or devPtr.devName[0] == 'D':
                    opResult = devPtr.gripper_on()

                elif devPtr.devName[0] == 'S':
                    opResult = devPtr.timeRotaterFw(time = wCmd.args.time, velocity = wCmd.args.velocity)
                else:
                    print_err(f'ERROR - No moving frwd command available for [{devPtr.devName}] device')
                    opResult = False
                

            case OpType.go_bcwrd_off:
                # print_log(f'-> mDev_backwrd/gripper_off/timeRotaterBw(time = { wCmd.args.time}, velocity = {wCmd.args.velocity})')
                # print_log(f'-> mDev_backwrd/gripper_off/timeRotaterBw(time = {nonNullV(wCmd.args, wCmd.args.time)}, velocity = {nonNullV(wCmd.args, wCmd.args.velocity)})')
                print_log(f'-> mDev_backwrd/gripper_off/timeRotaterBw')


                toBlock = True
                if devPtr.devName[0] == 'T'  :
                    opResult = devPtr.mDev_backwrd()

                elif  devPtr.devName[0] == 'R':
                    _polarity = None
                    _vel = None
                    _time = None
                    if wCmd.args:
                        _time = wCmd.args.time
                        _vel = wCmd.args.velocity
                        if  not (wCmd.args.polarity == None):
                            _polarity = wCmd.args.polarity
                        
                    opResult = devPtr.mDev_backwrd(velocity=_vel, timeout=_time, polarity = _polarity)

                elif devPtr.devName[0] == 'G'  or devPtr.devName[0] == 'D':
                    opResult = devPtr.gripper_off()

                elif devPtr.devName[0] == 'S':
                    opResult = devPtr.timeRotaterBw(time = wCmd.args.time, velocity = wCmd.args.velocity)
                else:
                    print_err(f'ERROR - No moving bkwd command available for [{devPtr.devName}] device')
                    opResult = False
            
            case OpType.home:
                print_log(f'-> home()')
                if devPtr.devName[0] == 'T' or devPtr.devName[0] == 'R' or devPtr.devName[0] == 'G' or devPtr.devName[0] == 'S' or  devPtr.devName[0] == 'D':
                    toBlock = False
                    opResult = devPtr.mDev_reset_pos()
                elif devPtr.devName[0] == 'Z':
                    toBlock = True
                    opResult = devPtr.move_home()
                else:
                    print_err(f'ERROR - No homing command available for [{devPtr.devName}] device')
                    opResult = False
            case OpType.vibrate:
                print_log(f'-> vibrate()')
                toBlock = True
                if devPtr.devName[0] == 'Z':            # implemented only for ZABER
                    opResult = devPtr.mDev_vibrate()
                else:
                    print_err(f'ERROR - No vibrate command available for [{devPtr.devName}] device')
                    opResult = False        

            case OpType.stop:
                print_log(f'-> stop()')
                toBlock = False
                opResult = devPtr.mDev_stop()

            case OpType.stall:
                print_log(f'-> stall()')
                toBlock = False
                opResult = devPtr.mDev_stall()


            case OpType.delay:
                toBlock = False
                print_log(f'-> delay({wCmd.args.time})')
                time.sleep(wCmd.args.time)
                opResult = True

            case OpType.play_media:
                toBlock = False
                print_log(f'-> Play file ({wCmd.args.media_file})')
                # playsound(wCmd.args.media_file)
                # playsound(wCmd.args.media_file, block = False)

                data, fs = sf.read(wCmd.args.media_file, dtype='float32')  
                sd.play(data, fs)
                # status = sd.wait()
                
                opResult = True

            case OpType.halt:
                print_log(f'HALT cmd')
                toBlock = False
                if window:
                    window.write_event_value(f'Stop', None)                           # Emergency stop event
                    opResult = True
                else:
                    print_err(f'ERROR- HALT cmd event could not be sent sent to main WINDOW [at script].')
                    opResult = False
                

            # case OpType.trigger:
            #     toBlock = False
            #     opResult = True
            #     if 'NI' in devPtr.devName:
            #         print_log(f'Trigger cmd -> {wCmd.args.trigger_selector} selector')
            #         if wCmd.args.trigger_selector:
            #             devPtr.selector(int(wCmd.args.trigger_selector))
            #         time.sleep(devPtr.SELECTOR_TRIGGER_DELAY)             # delay 
            #         devPtr.trigger()
            #     elif 'PHG' in devPtr.devName:
            #         print_log(f'operating Phidget triger relay')
            #         opResult = devPtr.trigerDev()
            #     else:
            #         print_err(f'Error discovering device type: {devPtr.devName} for operaton <<TRIGGER>>')
            #         opResult = False
                    

            # case OpType.hotair:
            #     toBlock = False
            #     opResult = True
            #     print_log(f'HOTAIR on/off -> {wCmd.args.start_stop}')
            #     if 'NI' in devPtr.devName:
            #         if wCmd.args.start_stop:
            #             devPtr.jtseStart()
            #         else:
            #             devPtr.jtseStop()
            #     elif 'PHG' in devPtr.devName:

            #         print_log(f'operating switch  {"ON" if wCmd.args.light_on == True else "OFF"} on Phidget relay')
            #         opResult = devPtr.relayOnOff(wCmd.args.light_on)
            #     else:
            #         print_err(f'Error discovering device type: {devPtr.devName} for operaton <<HOTAIR ON/OFF>>')
            #         opResult = False

            case OpType.start_robot_op:   
                toBlock = True
                opResult = True
                if  wCmd.args is not None:
                    print_log(f'Robot Operation operation {wCmd.args.robot_op} will with param  = { wCmd.args.robot_parm}')
                    # opResult = devPtr.runMecademicAsyrilOp(mc.partPos(wCmd.args.x_coord, wCmd.args.y_coord))
                    opResult = devPtr.robotOperation(wCmd.args.robot_op, wCmd.args.robot_parm)
                else:
                    print_err(f'Robot Operation error. No command provided')
                    toBlock = False
                    opResult = False

            case OpType.vacuum:   
                toBlock = False
                opResult = True
                if  wCmd.args:
                    print_log(f'Vacuum ON/OFF={wCmd.args.start_stop}, purge = {wCmd.args.velocity}')
                    # opResult = devPtr.runMecademicAsyrilOp(mc.partPos(wCmd.args.x_coord, wCmd.args.y_coord))
                    opResult = devPtr.Vacuum(onoff = wCmd.args.start_stop, purge = wCmd.args.velocity)
                else:
                    print_err(f'ERROR: No args passed to vacuum cmd')
                    opResult = False

            case OpType.pick_n_put:   
                toBlock = True
                if  wCmd.args:
                    if wCmd.args.p3D is not None:
                        print_log(f'PickUp and Put operation will be run to point = { wCmd.args.p3D}')
                        opResult = devPtr.startOP(mc.partPos(p3D = wCmd.args.p3D))
                    else:
                        print_log(f'PickUp and Put operation will be run to x= { wCmd.args.x_coord}, y = {wCmd.args.y_coord}')
                        # opResult = devPtr.runMecademicAsyrilOp(mc.partPos(wCmd.args.x_coord, wCmd.args.y_coord))
                        opResult = devPtr.startOP(mc.partPos(x = wCmd.args.x_coord, y = wCmd.args.y_coord))
                else:
                    print_log(f'PickUp and Put operation will be run to last position')
                    # opResult = devPtr.runMecademicAsyrilOp()
                    opResult = devPtr.startOP()
            
            case OpType.put_pcb:   
                toBlock = True
                opResult = True
                if  wCmd.args:
                    if wCmd.args.p3D is not None:
                        print_log(f'Pick and put PCB  operation will be run to point = { wCmd.args.p3D}')
                        opResult = devPtr.startCognexOP(mc.partPos(p3D = wCmd.args.p3D))
                    else:
                        print_log(f'Pick and put PCB operation will be run to x= { wCmd.args.x_coord}, y = {wCmd.args.y_coord}')
                        # opResult = devPtr.runMecademicAsyrilOp(mc.partPos(wCmd.args.x_coord, wCmd.args.y_coord))
                        opResult = devPtr.startCognexOP(mc.partPos(x=wCmd.args.x_coord, y=wCmd.args.y_coord))
                else:
                    print_err(f'ERROR: No args passed to put pcb cmd')
                    toBlock = False
                    opResult = False


            case OpType.insert_pcb:   
                toBlock = False
                opResult = True
                if  wCmd.args:
                    print_log(f'Insert PCB operation will be run to  Δx= { wCmd.args.x_coord}, Δy = {wCmd.args.y_coord}, Δz = {wCmd.args.z_coord}')
                    # opResult = devPtr.runMecademicAsyrilOp(mc.partPos(wCmd.args.x_coord, wCmd.args.y_coord))
                    opResult = devPtr.finalCorrection(wCmd.args.x_coord, wCmd.args.y_coord, wCmd.args.z_coord)
                else:
                    print_err(f'ERROR: No args passed to enter pcb cmd')
                    toBlock = False
                    opResult = False

            case OpType.single_shot:
                toBlock = False
                opResult = devPtr.single_shot(1)

            case OpType.set_program:
                toBlock = False
                opResult = True
                if  wCmd.args:
                    print_log(f'Set program to {wCmd.args.cmd_num}')
                    opResult = True if devPtr.program_control(wCmd.args.cmd_num) > 0 else False
                else:
                    print_log(f'Error setting program - no args!')
                    opResult = False

            case OpType.calibrate:
                toBlock = True
                print_log(f'calibrate on -Z-SELECTOR- = {wCmd.args.calibr_selector}, profile = {wCmd.args.profile}')
                opResult = devPtr.start_cam_tuning_operation(wCmd.args.calibr_selector, wCmd.args.profile, window, wCmd.args.abs)
       
            case OpType.check:
                toBlock = True
                print_log(f'check on profile = {wCmd.args.profile}')
                opResult = devPtr.start_cam_check_operation(wCmd.args.profile, window)                

            case OpType.terminate:
                toBlock = False
                print_log(f'Terminate {devPtr.devName}')
                opResult = devPtr.setTerminate()
            
            case OpType.measure:
                toBlock = False
                print_log(f'calculating R. C={wCmd.args.curr}, V={wCmd.args.volt}')
                _curr = None if wCmd.args.curr == '' else float(wCmd.args.curr)
                _volt = None if wCmd.args.volt == '' else float(wCmd.args.volt)
                opResult = devPtr.doMeasurement(_curr, _volt)

            # case OpType.phg_trigger:
            #     toBlock = False
            #     print_log(f'operating Phidget triger relay')
            #     opResult = devPtr.trigerDev()

            
            # case OpType.phg_toggle:
            #     toBlock = False
            #     if wCmd.args == None or wCmd.args.light_on == None:
            #         print_log(f'operating toggle on Phidget relay')
            #         opResult = devPtr.toggle()
            #     else:
            #         print_log(f'operating switch  {"ON" if wCmd.args.light_on == True else "OFF"} on Phidget relay')
            #         opResult = devPtr.relayOnOff(wCmd.args.light_on)

            # case OpType.phg_on | OpType.phg_off:
            #         toBlock = False
            #         print_log(f'operating switch  {"ON" if wCmd.cmd == OpType.phg_on else "OFF"} on Phidget relay')
            #         opResult = devPtr.relayOnOff(True if wCmd.cmd == OpType.phg_on else False ) 

            case OpType.phg_operate:               
                toBlock = False
                if wCmd.args and not (wCmd.args.light_on == None):
                    _onoff = wCmd.args.light_on
                else:
                    _onoff = None

                opResult = devPtr.devOperate(_onoff) 
                print_log(f'operating PHG device  ONOFF = {_onoff} on Phidget relay, result = {opResult}')

            case OpType.add_db:
                toBlock = False
                if wCmd.args and not (wCmd.args.light_on == None):
                    _onoff = wCmd.args.light_on
                else:
                    _onoff = None

                opResult = devPtr.recordDB(_onoff) 
                print_log(f'recordin into DB status = {_onoff}, result = {opResult}')

            case OpType.unparsed_cmd:
                runner = devPtr.loadDeviceOp(wCmd.operation)
                # devPtr.runDevicesOp(runner)
                toBlock, opResult = devPtr.__class__.runDevicesOp(runner)          # run the loaded commands
                print_log(f'Unparsed command loaded and run at device {devPtr.devName}, runner = {runner}')

            case OpType.nop:
                toBlock = False
                opResult = True
                print_log(f'No operation - do nothing')

            case _:
                toBlock = False
                print_err(f'ERROR - Unrecognized command {wCmd.cmd} for [{devPtr.devName}] device')
                opResult = False

        return opResult, toBlock


    # similar to __runCmd but passes entire command to device with no prior parsing by calling
    # loadDeviceOp method of CDev class. Do not run cmd itself. Use PLCDev:runDevicesOp to run all loaded commands
    def __runDevCmd(self, window:sg.Window = None)-> tuple[bool, bool]: 
        try:
            wCmd:CmdObj = self.__sub_tasks[0]
            print_log(f'Proceeding cmd = {wCmd}')

            toBlock:bool = True 
            opResult:bool = True

            if wCmd.cmd == OpType.halt:
                toBlock = False
                opResult = False
                if window:
                    window.write_event_value(f'Stop', None)                           # Emergency stop event
                    opResult = True
                else:
                    raise Exception (f'ERROR- HALT cmd event could not be sent  to main WINDOW [at script].')

            else:               #   device operations 
                if wCmd.device:                          
                    runner= wCmd.device.loadDeviceOp(wCmd)
                    toBlock, opResult = wCmd.device.__class__.runDevicesOp(runner)          # run the loaded commands

                    print_log(f'Device command loaded and run at device {wCmd.device.get_device().devName}, runner = {runner}')
                else:
                    print_err(f'ERROR- No device defined fot smd = {wCmd}')
                    return False, False
        
        except Exception as ex:
            exptTrace(ex)
            return False, False


        return opResult, toBlock

# @dataclass
# class task_ref:
#     ID:int = None
#     thread_id:Thread = None

# task_ref = namedtuple("task_ref", ["task", "ID", "thread_id"])
# task_ref.__annotations__={'task':WorkingTask, 'ID':int, 'thread_id':Thread}         # specify type of elements

task_ref = namedtuple("task_ref", ["task",  "thread_id"])
task_ref.__annotations__={'task':WorkingTask,  'thread_id':Thread}         # specify type of elements


# Process manager manages separate threads for each task
# All working task are run in separates thread
# The load_task API call is non-blocking
# Each workingTask (blocked untill completion) is operated from the separte thread and the thread terminates itself upon complition

class ProcManager:
    def __init__(self, window:sg.Window):
        self.__window:sg.Window = window                              # ref to returnt completion event   
        self.__tasks_list: list[task_ref] = list()                 # list of tasks (task_ref) refernces 
        self.__mLock:Lock = Lock()                                 # mutex for task list access control

          

    def __del__(self):

        for tsk in self.__tasks_list:
            tsk.task.EmergencyStop()
            tsk.thread_id.join()
        

    def EmergencyStop(self):
        for tsk in self.__tasks_list:
            tsk.task.EmergencyStop()

    def __proc_control_thread(self, wTask:WorkingTask, reportQ:Queue):
        tRes = wTask.run(self.__window)                                             # blocking
        print_log(f'Task completed with result = {tRes.result}, taskID = {wTask.id()}')

        current_thread_id = get_ident()
        self.__release_task(current_thread_id)

        print_log(f'Task completed. {"TASK_DONE" if tRes.result else "TASK_ERROR"} event will be sent')
        

        
        # if tRes.result:                                          
        #     self.__window.write_event_value(f'-TASK_DONE-', wTask.id())                           # TASK_DONE event to GUI context
        # else:
        #     self.__window.write_event_value(f'-TASK_ERROR-', wTask.id())

        if tRes.result:
            # globalEventQ.stepNotificationQ.put(event2GUI(event='-TASK_DONE-', value=wTask.id()))                           # TASK_DONE event to GUI context
            reportQ.put(event2GUI(event='-TASK_DONE-', value=wTask.id()))                           # TASK_DONE event to GUI context
        else:
            # globalEventQ.stepNotificationQ.put(event2GUI(event='-TASK_ERROR-', value=wTask.id())) 
            reportQ.put(event2GUI(event='-TASK_ERROR-', value=wTask.id(), device = tRes.device)) 
            
        print_log(f'Task done with result = {tRes.result}, dev = {tRes.device}')

    def __release_task(self, threadID):
        for tsk in self.__tasks_list:
            if tsk.thread_id.ident ==  threadID:
                print_log(f'Removing task (id={tsk.task.id()})')
                try:
                    l = smartLocker(self.__mLock)
                    self.__tasks_list.remove(tsk)
                    l.release()

                except Exception as ex:
                    exptTrace(ex)
            
                return
                            
        print_err(f'-WARNING - Cant find appropiate task to remove')


    def load_task(self, task_to_run:WorkingTask, reportQ:Queue):                   # non blocking
                                        # task_to_run - WorkingTask object
                                        # reportQ - Queue to send back the task completion event
    
        for tsk in self.__tasks_list:   # verify if single task is already running
            if (not tsk.task.is_single()) and (not task_to_run.is_single()):
                print_err(f'ERROR - the oly one script is allowed in time')
                print_err(f'1 ({task_to_run.id()}) -> {task_to_run}')
                print_err(f'2 {tsk.task.id()} -> {tsk.task}')
        wThread = Thread(target = self.__proc_control_thread, args =(task_to_run, reportQ,  )) 
        l = smartLocker(self.__mLock)
        # self.__tasks_list.append(task_ref(task = task_to_run, ID = WorkingTask.id(), thread_id = thread_id ))
        self.__tasks_list.append(task_ref(task = task_to_run, thread_id = wThread ))
        l.release()
        wThread.start() 
        print_log(f'Added new task to list. thread ID = {wThread.ident}')


        


# resolution device object by name
# OBSOLETE:
def resolve_device(dName:str, devs_list:list[CDev]) -> CDev:
    _devs = list()
    for dev in devs_list:
        _devName = dev.get_device().devName
        _devs.append(_devName)
        if _devName == dName:
            return dev
        
    print_err(f'-ERROR- Cant resolve the device >>{dName}<<')    
    
    print_err(f'Full list: {devs_list}<<')    
    print_err(f'Devices in the system: {_devs}<<')    
    return None

def _is_DEV_present( _devName:str, devs_list:list[CDev]) -> bool:
    for dev in reversed(devs_list):
        if dev.C_type == _devName:
            return True
    return False

# OBSOLETE
def resolve_SYS_device( opName:str, devs_list:list[CDev]) -> CDev:

    sub_dev = None
    dName = None
    match opName:
        # OBSOLETE:
        # case  'HOTAIR' | 'TRIGGER':
        #     dName = '--DAQ-NI--'

        # case  'HOTAIR':
        #     if _is_DEV_present('--DAQ-NI--', devs_list):
        #         dName = '--DAQ-NI--'
        #     elif _is_DEV_present('--PHG--', devs_list):
        #         dName = '--PHG--'
        #         sub_dev = 'JTSE_HOTAIR'
        #     else:
        #         print_err(f'Unrecognized device type for operation: {opName}')
        #         return None


        # case  'TRIGGER':
        #     if _is_DEV_present('--DAQ-NI--', devs_list):
        #         dName = '--DAQ-NI--'
        #     elif _is_DEV_present('--PHG--', devs_list):
        #         dName = '--PHG--'
        #         sub_dev = 'WELDER'
        #     else:
        #         print_err(f'Nonclear device for operation: {opName}')
        #         return None

        # case 'CALIBR':
        #     dName = '--CAM--'
        # case 'DISP':
        #     dName = '--PHG--'
        #     sub_dev = 'DISP'
        # case 'UV':
        #     dName = '--PHG--'
        #     sub_dev = 'UV'

        # case 'FRONTLIGHT':
        #     dName = '--PHG--'
        #     sub_dev = 'FRONTLIGHT'

        # case 'BACKLIGHT':
        #     dName = '--PHG--'
        #     sub_dev = 'BACKLIGHT'

        # case 'HOLELIGHT':
        #     dName = '--PHG--'
        #     sub_dev = 'HOLELIGHT'
        
        case 'HALT'  | 'DELAY' | 'PLAY':
            return None
        case _:
            print_err(f'-ERROR- Cant recognize the device by operation - {opName}')
            return None

    '''
    OBSOLETE:
    Resolving device by operation name
    1. find device type by operation name   
    2. find device instance by device type
    3. if sub_dev is set - find device instance by sub_dev name
    4. return device instance or None if not found

    '''
    for dev in reversed(devs_list):
        if dev.C_type == dName:            # e.g. '--DAQ-NI--', '--CAM--'
            if sub_dev:
                if dev.get_device().devName == sub_dev:
                    return dev
            else:
                return dev
        
    print_err(f'-ERROR- Cant resolve the device {opName}/{dName}')    
    return None

# def BuildComplexWorkingClass(script:dict, devs_list:List[CDev], stepTask:bool = False, tempTaskList: List[WorkingTask] = list())-> WorkingTask:
# def BuildComplexWorkingClass(script:dict, devs_list:list[CDev], key , stepTask:bool = False)-> WorkingTask:
def BuildComplexWorkingClass(script:dict, _sysDevs:systemDevices, key , stepTask:bool = False)-> WorkingTask:
  
    tempTaskList: list[WorkingTask] = list()

    print_DEBUG(f'Working on script = {script} key = {key}')
    for group_n, cmd in script.items():
        print_DEBUG(f'Procceeding cmd = {cmd} group_n = {group_n}')

        if isinstance(cmd, dict):     # complex command (sub-script) / cmd block, nested script
            print_DEBUG(f'Creating complex task for cmd = {cmd}')
            # woT:WorkingTask = BuildComplexWorkingClass(cmd, devs_list, group_n, stepTask)
            woT:WorkingTask = BuildComplexWorkingClass(cmd, _sysDevs, group_n, stepTask)
            if woT == None:
                return None
            else:
                tempTaskList.append(woT)
        else:                        # single command    
            _cmd =list(" ")
            _cmd.append(group_n) 
            if isinstance(cmd, str):    
                _cmd.extend(cmd.split())
            elif isinstance(cmd, bool):                   # bool
                 _cmd.append(cmd)
            else:
                print_err(f'Unexpected cmd {cmd} type: {type(cmd)} ')
                return None


            print_DEBUG(f'Creating single task for cmd = {_cmd}')
            # wTask:WorkingTask = Create_Single_Task(_cmd, devs_list) 
            wTask:WorkingTask = Create_Dev_Single_Task(_cmd, _sysDevs)
            print_DEBUG(f'Single_Task = {wTask}')
            if wTask is not None:     
                tempTaskList.append(wTask)
            else:    
                print_err(f'Dev {_cmd[1]} at script cmd {_cmd} is not active in the system')                               # device for cmd is not active in the system 
                return None
    # BUGBUG: code for simultaneous runner add HERE
    if key[-1] == 'P':
        sType = RunType.parallel
    elif key[-1] == 'S':
        sType = RunType.serial
    else:
        print_err(f'--WARNING Commands combination [{group_n}] that is not predefined group is treated as paralel')
        sType = RunType.parallel
    
    wT = WorkingTask(tempTaskList, sType=sType, stepTask=stepTask)
    print_DEBUG(f'completed wTask = {wT}')
            
    return wT          


'''
# BuildWorkingTask is OBSOLETE:
def BuildWorkingTask(script:list[str], devs_list:list[CDev],  sType:RunType = RunType.parallel, stepTask:bool = False) -> WorkingTask:
    
    tempTaskList: list[WorkingTask] = list()

# Only Paralel mode is currently supported for scrips!!!!!!!!
    for cmd in script:      
        wTask:WorkingTask = Create_Single_Task(cmd, devs_list)   
        if wTask:     
            tempTaskList.append(wTask)
        else:    
            print_err(f'Dev {cmd[1]} at script cmd {cmd[0]} is not active in the system')                               # device for cmd is not active in the system 
            return None 
    
    if sType == RunType.parallel:
        wTask = WorkingTask(tempTaskList, sType=sType, stepTask=stepTask)
    else:
        print_err(f'-ERROR- unsupported type: {sType}')
        wTask = None

    return wTask
'''

def validateScript(scriptStep:list[str])-> bool:
    print_log(f'-WARNING  validation rules  for script are not set so far')
    return True

# creating  WorkingTask for single (device direct) cmd (script cmd)
# OBSOLETE:use Create_Dev_Single_Task instead
def Create_Single_Task(cmd:str, devs_list:list[CDev]) -> WorkingTask:

    dev_type:str = cmd[1] if ((cmd[1] == 'SYS') or (cmd[1] == 'CMNT') or (cmd[1] == 'NOP') or (cmd[1] == 'MCDMC') or (cmd[1] == 'PHG') \
                or (cmd[1][:4] == 'PHG_') or (cmd[1][:-1] == 'DISP') or (cmd[1][:-1] == 'CAM') or (cmd[1] == 'DB')) else cmd[1][0]
    print_log(f'Creating "Single Task" from cmd = {cmd}, device = {cmd[1]} of type {dev_type} ')
    device = None
    if not (dev_type == 'SYS' or dev_type == 'CMNT' or dev_type == 'NOP'):
        if cmd[1] == 'PHG':
            device:CDev = resolve_device(cmd[2], devs_list)
        elif cmd[1][:4] == 'PHG_':
            device:CDev = resolve_device(cmd[1][4:], devs_list)
        else:
            device:CDev = resolve_device(cmd[1], devs_list)

        if device == None:
            print_err(f"No active device found for cmd: {cmd}")
            # return WorkingTask()                                # return empty task
            return None                                # return None
        
        print_log(f'Resolved device = {device}')

    wCmd = None
    
    

    print_log(f'Proceeding now: {cmd}, dev = {cmd[1]}, type = {dev_type}, len(cmd) = {len(cmd)}')

    match dev_type:
        case 'T' | 'R' :       
            device_n = (int)(cmd[1][1])                        # Trolley
            if cmd[2] == 'HO':
                wCmd = CmdObj(device=device, cmd=OpType.home)
                print_log(f'Device = {cmd[1]}, Reset  home pos')
                
            elif  cmd[2] == 'REL':
                wCmd = CmdObj(device=device, cmd=OpType.stop)
                print_log(f'Device = {cmd[1]},  Stop/Release')

            elif  cmd[2] == 'STALL':
                wCmd = CmdObj(device=device, cmd=OpType.stall)
                print_log(f'Device = {cmd[1]},  Stall device')
                
            elif  cmd[2] == 'MR' or cmd[2] == 'ML' or cmd[2] == 'MA' or cmd[2] == 'MLSTALL' or cmd[2] == 'MRSTALL':                                           # motion cmd
                if len(cmd) == 5:                          # Update rpm
                    print_log(f'Device = {cmd[1]}, RPM =  {cmd[4]}')
                    velocity = cmd[4]
                else:
                    velocity = None
                __pos = device.get_device().mDev_stored_pos()
                print_log(f'Device = {cmd[1]}, Curr pos = {__pos}, move pos = {(int)(cmd[3])}')
                move_pos:int = (int) (cmd[3])
                cur_pos:int = (int) (__pos)
                new_pos:int = 0
                _stall = False                               # stall used as STALL flag fro TROLLEY
                if cmd[2] == 'MR' or cmd[2] == 'MRSTALL':
                    new_pos = cur_pos - move_pos
                elif cmd[2] == 'ML' or cmd[2] == 'MLSTALL' :
                    new_pos = cur_pos + move_pos
                elif cmd[2] == 'MA':
                    new_pos = move_pos
                else:
                    print_err(f'-ERROR- Device = {cmd[1]}/cmd = {cmd[2]}, Error in cmd = {cmd}')
                    return None
                
                if cmd[2] == 'MLSTALL' or cmd[2] == 'MRSTALL':
                    _stall = True

                print_log(f'Device = {cmd[1]}, Curr pos = {cur_pos}, new pos = {new_pos}')
                wCmd = CmdObj(device=device, cmd = OpType.go_to_dest,  args = argsType(position = new_pos, velocity= velocity, stall = _stall))
            
            elif  cmd[2] == 'MRC' or cmd[2] == 'MLC':   
                _timeout: int = None                    # no default timeout
                _polarity = False                       # LOW is default for sensor
                if len(cmd) == 6:
                    print_log(f'Device = {cmd[1]}, RPM =  {cmd[3]}, Timeout = {cmd[4]}, Polarity = {cmd[5]}')
                    _polarity = True if str(cmd[5]).upper() == 'HIGH' else False
                    velocity = cmd[3]
                    _timeout = cmd[4]
                elif len(cmd) == 5:
                    print_log(f'Device = {cmd[1]}, RPM =  {cmd[3]}, Timeout = {cmd[4]}')
                    velocity = cmd[3]
                    _timeout = cmd[4]
                elif len(cmd) == 4:                          # Update rpm
                    print_log(f'Device = {cmd[1]}, RPM =  {cmd[3]}')
                    velocity = cmd[3]
                else:
                    velocity = None

                if cmd[2] == 'MLC':
                    print_log(f'Device = {cmd[1]}, Moving right continuously')

                    wCmd = CmdObj(device=device, cmd=OpType.go_fwrd_on,  args=argsType(
                        velocity=velocity, time=_timeout, polarity = _polarity))
                elif cmd[2] == 'MRC':
                    print_log(f'Device = {cmd[1]}, Moving left continuously')
                    wCmd = CmdObj(device=device, cmd=OpType.go_bcwrd_off,  args=argsType(
                        velocity=velocity, time=_timeout, polarity = _polarity))

                else:
                    print_err(f'-ERROR- Device = {cmd[1]}/cmd = {cmd[2]}, Error in cmd = {cmd}')
                    return None
                

            
                

        
        case 'S':

            if len(cmd) == 5:                          # Update rpm
                print_log(f'Device = {cmd[1]}, RPM =  {cmd[4]}')
                velocity = cmd[4]
            else:
                velocity = None

            movingTime = int(float(cmd[3]))
            if cmd[2] == 'MR':
                op:OpType = OpType.go_fwrd_on
            elif cmd[2] == 'ML':
                op:OpType = OpType.go_bcwrd_off
            else:
                print_err(f'-ERROR- Device = {cmd[1]}, Error S cmd = {cmd}')
                return None
            
            wCmd = CmdObj(device=device, cmd = op,  args = argsType(time = movingTime, velocity= velocity ))

        case 'G' :
            print_log(f'Device = {cmd[1]}, cmd = "{cmd[2]}", type={type(cmd[2])}, status = {device.get_device().gripper_onof}, type = {type(device.get_device().gripper_onof)}')

            if isinstance(cmd[2], bool):
                if (cmd[2]):
                    wCmd = CmdObj(device=device, cmd = OpType.go_fwrd_on)
                else:
                    wCmd = CmdObj(device=device, cmd = OpType.go_bcwrd_off)
            elif  isinstance(cmd[2], str):
                cmd[2].rstrip()
                print_log(f'cmd = {cmd}, cmd[2] =[{cmd[2]}]')
                print_log(f'[{cmd[2]}].upper() = [{cmd[2].upper()}]')
                if (cmd[2].upper() == 'TRUE'):
                    wCmd = CmdObj(device=device, cmd = OpType.go_fwrd_on)
                elif (cmd[2].upper() == 'FALSE'):
                    wCmd = CmdObj(device=device, cmd = OpType.go_bcwrd_off)
                else: 
                    print_err(f'-ERROR unrecognize parm: {cmd[2]}')
                    return None
            else:
                print_err(f'-ERROR wrong type: {type(cmd[2])}')
                return None

        case 'D':    

            if cmd[2] == 'HO':
                wCmd = CmdObj(device=device, cmd=OpType.home)
                print_log(f'Device = {cmd[1]}, Reset  home pos')
                
            elif  cmd[2] == 'MA' or cmd[2] == 'MR':                                           # motion cmd
                if len(cmd) == 5:                          # Update rpm
                    print_log(f'Device = {cmd[1]}, RPM =  {cmd[4]}')
                    velocity = cmd[4]
                else:
                    velocity = None
                __pos = device.get_device().mDev_stored_pos()
                print_log(f'Device = {cmd[1]}, Curr pos = {__pos}, move/shift pos = {(int)(cmd[3])}')
                move_pos:int = (int) (cmd[3])
                cur_pos:int = (int) (__pos)

                new_pos:int = 0

                if cmd[2] == 'MA':
                    new_pos = move_pos
                elif cmd[2] == 'MR':
                    new_pos = cur_pos + move_pos
                else:
                    print_err(f'-ERROR- Device = {cmd[1]}/cmd = {cmd[2]}, Error in cmd = {cmd}')
                    return None


                print_log(f'Device = {cmd[1]}, Curr pos = {cur_pos}, new pos = {new_pos}, velocity= {velocity}')
                wCmd = CmdObj(device=device, cmd = OpType.go_to_dest,  args = argsType(position = new_pos, velocity= velocity ))
            elif isinstance(cmd[2], bool):
                print_log(f'cmd = {cmd}, arg - bool')
                if (cmd[2]):
                    wCmd = CmdObj(device=device, cmd = OpType.go_fwrd_on)
                else:
                    wCmd = CmdObj(device=device, cmd = OpType.go_bcwrd_off)  
            elif isinstance(cmd[2], str): 
                print_log(f'cmd = {cmd}, arg - str')
                if cmd[2].upper() == 'TRUE':
                        wCmd = CmdObj(device=device, cmd = OpType.go_fwrd_on)
                elif cmd[2].upper() == 'FALSE':
                        wCmd = CmdObj(device=device, cmd = OpType.go_bcwrd_off)                
            else: 
                print_err(f'-ERROR- Device = {cmd[1]}/cmd = {cmd[2]}, Error in cmd = {cmd}')
                return None

        case 'Z':
            print_log(f'Zaber device = {cmd[1]}')

            if cmd[2] == 'HO':
                wCmd = CmdObj(device=device, cmd=OpType.home)
                print_log(f'Go to home pos')
            elif cmd[2] == 'RND':

                print_log(f'Device = {cmd[1]}, Random Start  = {cmd[3]}, End = {cmd[4]}')
                wCmd = CmdObj(device=device, cmd = OpType.go_rnd, args = argsType(position = float(cmd[3]), end_position = float(cmd[4]) ))

            elif cmd[2] == 'VIBRATE':
                print_log(f'Device = {cmd[1]}, Vibrate ZABER')
                wCmd = CmdObj(device=device, cmd = OpType.vibrate)
            else:
                if len(cmd) == 5:                          # Update rpm
                    print_log(f'Device = {cmd[1]}, Update ZABER Velocity =  {cmd[4][:-1]}% / {cmd[4]}')
                    velocity_in_percents = int(cmd[4][:-1])
                else:
                    velocity_in_percents = 100             # set to default 100%

                move_pos = (float)(cmd[3])
                cur_pos = device.get_device().GetPos()
                if cmd[2] == 'MA':
                    new_pos = move_pos
                elif cmd[2] == 'MR':
                        new_pos = cur_pos + move_pos

                else:
                    print_err(f'-ERROR- Error Zaber cmd = {cmd}')
                    return None
                
                print_log(f'Device = {cmd[1]}, Curr pos = {cur_pos}, new pos = {new_pos}')
                wCmd = CmdObj(device=device, cmd = OpType.go_to_dest,  args = argsType(position = new_pos, velocity= velocity_in_percents ))


        case 'MCDMC':
            if cmd[2] == 'PUT':
                if len(cmd) > 4:
                    wCmd = CmdObj(device=device, cmd=OpType.pick_n_put, args=argsType(x_coord=cmd[3], y_coord=cmd[4]))
                    print_log(f'Put part at X={cmd[3]}, Y={cmd[4]}')
                else:
                    wCmd = CmdObj(device=device, cmd=OpType.pick_n_put, args=argsType(p3D=cmd[3]))
                    print_log(f'Put part at point = {cmd[3]}')
            elif cmd[2] == 'PUTPCB':
                if len(cmd) > 4:
                    wCmd = CmdObj(device=device, cmd=OpType.put_pcb, args=argsType(x_coord=cmd[3], y_coord=cmd[4]))
                    print_log(f'Put PCB at X={cmd[3]}, Y={cmd[4]}')
                else:
                    wCmd = CmdObj(device=device, cmd=OpType.put_pcb, args=argsType(p3D=cmd[3]))
                    print_log(f'Put PCB part at point = {cmd[3]}')
            elif cmd[2] == 'INSERTPCB':
                wCmd = CmdObj(device=device, cmd=OpType.insert_pcb, args=argsType(x_coord=cmd[3], y_coord=cmd[4], z_coord = cmd[5]))
                print_log(f'Insert PCB with offset ΔX={cmd[3]}, ΔY={cmd[4]}, ΔZ={cmd[5]}')
            elif cmd[2] == 'VACUUM':
                wCmd = CmdObj(device=device, cmd=OpType.vacuum, args=argsType(start_stop = True if cmd[3].upper() == 'ON' else False, \
                                                                              velocity = None if len(cmd) < 5 else cmd[4]))
                print_log(f'Vacuum operated: {cmd}')
            else:
                print_log(f'Parsing robot command: {cmd}')
                match cmd[2]:
                    case 'MOVEABS':
                        _arg = argsType (robot_op = mc.robotOpType.moveabs, robot_parm = mc.contexFuncParms(point = cmd[3], \
                                                                                        velocity = None if len(cmd) < 5 else cmd[4]) )
                    case 'MOVESAFE':
                        _arg = argsType (robot_op = mc.robotOpType.movesafe, robot_parm = mc.contexFuncParms(point = cmd[3], \
                                                                                        velocity = None if len(cmd) < 5 else cmd[4]) )            
                    case 'MOVEUP':
                        _arg = argsType (robot_op = mc.robotOpType.moveup)            

                    case 'MOVEREL':
                        if len(cmd) < 7:
                            _arg = argsType (robot_op = mc.robotOpType.moverel, robot_parm = mc.contexFuncParms(point = cmd[3], \
                                                                                        velocity = None if len(cmd) < 5 else cmd[4]) )            

                        else:
                            _coord = mc._mcs500_pos(x=float(cmd[3]), y=float(cmd[4]), z=float(cmd[5]), gamma = float(cmd[6]))
                            _arg = argsType (robot_op = mc.robotOpType.moverel, robot_parm = mc.contexFuncParms(coord = _coord, \
                                                                                        velocity = None if len(cmd) < 8 else cmd[7]) )            
                    case 'PICKUP':
                        _arg = argsType (robot_op = mc.robotOpType.pickup, robot_parm = mc.contexFuncParms(point = cmd[3]))            

                wCmd = CmdObj(device=device, cmd=OpType.start_robot_op, args=_arg)
        
           

        case 'PHG':
            if len(cmd) == 3:
                wCmd = CmdObj(device=device, cmd = OpType.phg_operate)
                print_log(f'Operating toggle on Phidget device for {device.get_device().devName}')
            else:
                _onoff = True if str(cmd[3]).upper() == 'ON' else False
                wCmd = CmdObj(device=device, cmd = OpType.phg_operate, args=argsType(light_on = _onoff))
                print_log(f'Operating switch {"ON" if _onoff == True else "OFF"} on Phidget device for {device.get_device().devName}')
        
        case 'DB':
            _onoff = True if str(cmd[3]).upper() == 'TRUE' else False
            wCmd = CmdObj(device=device, cmd = OpType.add_db, args=argsType(light_on = _onoff))
            print_log(f'Adding record to DB {"TRUE" if _onoff == True else "FALSE"} at {device.get_device().devName}')
    
        case _:
            
            if  'DISP' in cmd[1]:
                if cmd[2] == 'SET' and cmd[3] == 'PROGRAM':
                    wCmd = CmdObj(device=device, cmd=OpType.set_program, args=argsType(cmd_num=cmd[4]))
                    print_log(f'Set program ={cmd[4]}')
                elif  cmd[2] == 'SINGLE_SHOT':
                    wCmd = CmdObj(device=device, cmd=OpType.single_shot)
                    print_log(f'Single shop is in progress')
                else:
                    print_err(f'Unrecognized DISP command: {cmd} ')
                

            elif cmd[1][:4] =='PHG_':
                if len(cmd) == 2:
                    wCmd = CmdObj(device=device, cmd = OpType.phg_operate)
                    print_log(f'Operating toggle on Phidget device for {device.get_device().devName}')
                else:
                    _onoff = True if str(cmd[2]).upper() == 'ON' else False
                    wCmd = CmdObj(device=device, cmd = OpType.phg_operate, args=argsType(light_on = _onoff))
                    print_log(f'Operating switch {"ON" if _onoff == True else "OFF"} on Phidget device for {device.get_device().devName}')

            elif 'CAM' in cmd[1]:
                if cmd[2] == 'CALIBR':
                    _abs = False

                    if len(cmd) > 4:
                        profile = cmd[4]
                        if (len(cmd) == 6) and (cmd[5] == 'ABS'):
                            _abs = True
                        
                    else:
                        profile = 'base'

                    print_log(f'CALIBR script cmd on -Z-SELECTOR- = {cmd[3]} motor, -CALIBR_PROFILE- = {profile}, ABSOLUTE MOVEMENT = {_abs}')

                    wCmd = CmdObj(device=device, cmd = OpType.calibrate, args = argsType(calibr_selector = cmd[3], profile = profile, abs = _abs ))
                elif cmd[2] == 'TERM':
                    print_log(f'Script cmd: Terminate CAM at {device} ')
                    wCmd = CmdObj(device=device, cmd = OpType.terminate)
                elif cmd[2] == 'CHECK':
                    print_log(f'Script cmd: CHECK CAM= {device} at profile {cmd[3]} ')
                    wCmd = CmdObj(device=device, cmd = OpType.check, args = argsType(profile = cmd[3]))
            
            elif cmd[1] == 'SYS':
                device = resolve_SYS_device(cmd[2], devs_list)
                if cmd[2] == 'PLAY':
                    _path = Path(cmd[3])
                    if _path.is_file():
                        print_log(f'Media {_path} will be played')
                        wCmd = CmdObj(device=None, cmd = OpType.play_media, args = argsType(media_file = _path)) 
                    else:
                        print_err(f'File to play is not exist: {_path}')
                    

                elif cmd[2] == 'HALT':
                    print_log(f'HALT cmd in script')
                    wCmd = CmdObj(device=None, cmd = OpType.halt)

                elif cmd[2] == 'DELAY':
                    print_log(f'Delay = {cmd[3]}')
                    wCmd = CmdObj(device=None, cmd = OpType.delay, args = argsType(time = float(cmd[3])))

                elif device == None:
                    print_log(f'No active device for {cmd[2]}, empty task')
                    # return WorkingTask() 
                    return None 
                
                # elif cmd[2] == 'HALT':
                #     print_log(f'HALT cmd in script')
                #     wCmd = CmdObj(device=None, cmd = OpType.halt)

                # elif cmd[2] == 'DELAY':
                #     print_log(f'Delay = {cmd[3]}')
                #     wCmd = CmdObj(device=None, cmd = OpType.delay, args = argsType(time = float(cmd[3])))


                
                # elif cmd[2] == 'HOTAIR':
                #     op = 'START' if cmd[3] == '1' else 'STOP'
                #     print_log(f'HOTAIR-{op}')

                #     if device.C_type == '--DAQ-NI--':
                #         opArg = argsType(start_stop=True) if cmd[3] == '1'  else argsType(start_stop=False)
                #         wCmd = CmdObj(device=device, cmd = OpType.hotair,  args = opArg)
                #     elif device.C_type == '--PHG--':
                #         light = True if cmd[3] == '1' else False
                #         wCmd = CmdObj(device=device, cmd = OpType.phg_toggle, args=argsType(light_on = light))
                #     else:
                #         print_err(f'Unrecognized device type for cmd:{cmd}')
                #         return None


                # elif cmd[2] == 'TRIGGER':
                    
                #     if device.C_type == '--DAQ-NI--':
                #         if len(cmd) == 4:
                #             opArg = argsType(trigger_selector=cmd[3])
                #             print_log(f'TRIGGER script cmd on [SC {cmd[3]}] schedule')
                #         else:
                #             opArg = argsType(trigger_selector=None)
                #             print_log(f'TRIGGER script cmd on *current* schedule')
                        
                    
                #         wCmd = CmdObj(device=device, cmd = OpType.trigger, args = opArg )
                #     elif device.C_type == '--PHG--':
                #         wCmd = CmdObj(device=device, cmd = OpType.phg_trigger )
                #     else:
                #         print_err(f'Unrecognized device type for cmd:{cmd}')
                #         return None


                # elif cmd[2] == 'DISP' or cmd[2] == 'UV':
                #     wCmd = CmdObj(device=device, cmd = OpType.phg_trigger )
                #     print_log(f'Operating trigger on Phidget device for {device.get_device().devName}')
                
                # elif cmd[2] == 'FRONTLIGHT' or cmd[2] == 'BACKLIGHT' or cmd[2] == 'HOLELIGHT':
                #     if len(cmd) == 3:
                #         wCmd = CmdObj(device=device, cmd = OpType.phg_toggle)
                #         print_log(f'Operating toggle on Phidget device for {device.get_device().devName}')
                #     else:
                #         light = True if str(cmd[3]).upper() == 'ON' else False
                #         # print_log(f'DEBUG - cmd = {cmd}, cmd[3]= {cmd[3]}, cmd[3].upper() = {cmd[3].upper()}, light = {light}')
                #         wCmd = CmdObj(device=device, cmd = OpType.phg_toggle, args=argsType(light_on = light))
                #         print_log(f'Operating switch {"ON" if light == True else "OFF"} on Phidget device for {device.get_device().devName}')
                # else:
                #     print_err(f'Unrecognized SYS cmd = {cmd}')
                #     return None
                
            elif cmd[1] == 'CMNT':
                print_log(f'CMNT command')
                wCmd = CmdObj(device=device, cmd = OpType.nop)

            elif cmd[1] == 'NOP':
                print_log(f'NOP command')
                wCmd = CmdObj(device=device, cmd = OpType.nop)
            

            else:
                print_err(f'Unrecognized script cmd = {cmd}')
                return None
    
    return WorkingTask(taskList = wCmd, sType=RunType.single)   

# creating  WorkingTask for single device with multiple cmds (script cmd)
# similar to Create_Single_Task but does not parsse the command (cmd will be parsed inside the device class)
def Create_Dev_Single_Task(cmd:str, _sysDevs: systemDevices) -> WorkingTask: 
    try:

        print_log(f'Creating "Device Single Task" from cmd = {cmd}, device = {cmd[1]} ')

        device:CDev = _sysDevs[cmd[1]] 

        if device == None and not (cmd[1] == 'SYS' or cmd[1] == 'CMNT' or cmd[1] == 'NOP'):
                        # device for cmd is not active in the system and not a cmd that does not require device
            raise(f"No active device {cmd[1]} found for cmd: {cmd}")
            # return WorkingTask()                                # return empty task
            
        print_log(f'Resolved device = {device} for cmd = {cmd}')

        wCmd = None

        wCmd = CmdObj(device=device, cmd=OpType.unparsed_cmd, args=argsType(cmd_txt=cmd[2:]))

    except KeyError as e:
        print_err(f'-ERROR- Cant create Single Task for device {cmd[1]} /  active device {cmd[1]} not found in system devices for cmd: {cmd}')
        exptTrace(e)
        return None
        
    return WorkingTask(taskList = wCmd, sType=RunType.single)



class WorkingTasksList:
    def __init__(self):
        self.tList:list[WorkingTask] = list()

    def __del__(self):
        if len(self.tList):
            print_err(f'-WARNING- There are unterminated tasks in the list')

    def addTask(self, wTask:WorkingTask):
        self.tList.append(wTask)

    def delTask(self, tID:int)-> bool:
        for t in self.tList:
            if tID == t.id():
                self.tList.remove(t)
                return True
        return False
    
    def getTask(self, tID:int) -> WorkingTask:
        for t in self.tList:
            if tID == t.id():
                return t
        return None
    
    def getAllTasks(self) -> list[WorkingTask]:
        return self.tList


if __name__ == "__main__":
    # fields = ('val', 'left', 'right')
    # Node = namedtuple('Node', fields, defaults=(None,) * len(fields))
    # Node = namedtuple('Node', ["a", "b", "c"], defaults=(None,) * len(fields))
    # Node = namedtuple('Node', ["a", "b", "c"], defaults=[None,] * len(fields))
    # print(Node())
    print(argsType())
    pass


