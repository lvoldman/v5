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

from bs1_asyril import AsyrilInterface

import time, re
import os.path

from typing import Optional

import logging, sys, datetime, yaml

from mecademic_error import mecademicErrorMsg

SYNC_MODE =  True                       # syncronious mode enabled
DISCONNECT_MODE = False                 # don't disconnect on exception

PUT_H = 92.0
PICK_H = 79.1403
WORKING_H = 44
VELOCITY = 25
PICK_DELAY = 0.5
PUT_DELAY = 0.5

PUT_X = 178.23324
PUT_Y = 30.56565
PUT_STEP = 5

MID_POS = (-67.69506, -55.04702, -65.00323, -1548.44055)

# -82.59525 - vacuum with no part
# -0.24776  - with no valuum

REPEATS = 3

PUT_ANGLE = 180

VACUUM_PURGE_DURATION = 0.5

PICK_PRESSURE = -84.5

ANGLE = 90

logFileDate = datetime.datetime.now().strftime(f"MECA500_SCRIPT_%Y_%m_%d_%H_%M.txt")

log_format = u'%(asctime)s: %(filename)s--%(funcName)s/%(lineno)d -- %(thread)d [%(threadName)s] %(message)s' 
logging.basicConfig(format=log_format, handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler(logFileDate, mode="w", encoding = 'utf-8')], encoding = "utf8", level=logging.DEBUG)
print_log = logging.debug


glRobot = None

def _connect(robot) -> bool:
    try:
        robot.Connect(address='192.168.0.100', enable_synchronous_mode = SYNC_MODE, \
                      disconnect_on_exception=DISCONNECT_MODE)
    except Exception as ex:
        print_log(f'Connection  failed: {ex}')
        return False
    
    return True

def _activate(robot) -> bool:
    try:
        robot.ActivateRobot()
        robot.Home()
    except Exception as ex:
        print_log(f'Activation  failed: {ex}')
        return False
    
    return True


def _stop_clear(robot):
    try:
        
        if not SYNC_MODE:
            print_log(f'Waiting to Idle')
            robot.WaitIdle()

        print_log(f'Deactivating')
        robot.DeactivateRobot()

        if not SYNC_MODE:
            print_log(f'Waiting for Deactivation')
            robot.WaitDeactivated()

        print_log(f'Disconnect now')
        robot.Disconnect()
        
    except Exception as ex:
        print_log(f'Stoping/clearing proc failed: {ex}')

def _errorRecovery(robot):                       # in general singularity is fixed right here
    pass

def _errorCallback():
    print_log(f'Error callback... error_status = {robot_status.error_status}, robot_status.error_code ={robot_status.error_code} ')
    robot_status = robot.GetStatusRobot(synchronous_update = True)
    if robot_status.error_status:
                print_log(f'Error detected. Code = {robot_status.error_code} / {mecademicErrorMsg(int(robot_status.error_code))} ')
                glRobot.ResetError()
                if robot_status.pause_motion_status:
                    glRobot.ResumeMotion()
                _errorRecovery(glRobot)

    else:
        print_log(f'Error???')


def _on_connectedCallback():
    print_log(f'-> Connected...')

def _on_disconnectedCallback():
    print_log(f'-> Disconnected...')

def _on_activatedCallback():
    print_log(f'-> Activated...')

def _on_deactivatedCallback():
    print_log(f'-> Deactivated...')

def _on_homedCallback():
    print_log(f'-> Homed...')


def _validateScript(_script:dict) -> bool:
    _cmdList = ['SetJointVel', 'MovePose', 'MoveJoints', 'GripperOpen', 'GripperClose', 'VacuumGrip', 'VacuumRelease', 'Delay', 'SetVacuumPurgeDuration' ]

    _model = glRobot.GetRobotInfo().robot_model
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
                
            # case 'MoveJoints':
            #     if not len(_splited_cmd) == 2:
            #         print_log(f'Error parsing cmd: {_item} / {_splited_cmd}')
            #         return False
            #     if tools.robot_model_is_meca500(_model):
            #         cmd_re = re.compile(r'\s*-?\d{0,3},\s*-?\d{0,3},\s*-?\d{0,3},\s*-?\d{0,3},\s*-?\d{0,3},\s*-?\d{0,3}\)\s*')
            #     elif tools.robot_model_is_mcs500(_model):
            #         cmd_re = re.compile(r'\s*-?\d{0,3},\s*-?\d{0,3},\s*-?\d{0,3},\s*-?\d{0,3}\)\s*')
            #     else:
            #         print_log(f'Unsupported robot model: {_model}')    

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

def _runRobotCmd (robot, r_cmd):
    print(f'Runnung: {r_cmd}')
    try:
        _splited_cmd = r_cmd.split("(")
        _cmd = _splited_cmd[0]
        
        parm:Optional[str] = None

        if len(_splited_cmd) > 1:
            parm = _splited_cmd[1].split(")")[0]

        
        match _cmd:
            case 'SetJointVel':
                robot.SetJointVel(int(parm))
                pass
            case 'MovePose':
                float_tuple_parm = tuple(map(float, parm.split(',')))
                robot.MovePose(*float_tuple_parm)
            case 'MoveJoints':
                int_tuple_parm = tuple(map(float, parm.split(',')))
                robot.MoveJoints(*int_tuple_parm)
                pass
            case 'GripperOpen':
                robot.GripperOpen()
            case 'GripperClose':
                robot.GripperClose()
            
            case 'VacuumGrip':
                robot.VacuumGrip()

            case 'VacuumRelease':
                robot.VacuumRelease()

            case 'Delay':
                float_tuple_parm = float(parm)
                robot.Delay(float_tuple_parm)
            
            case 'SetVacuumPurgeDuration':
                float_tuple_parm = float(parm)
                robot.SetVacuumPurgeDuration(float_tuple_parm)


            case _:     
                print_log(f'Unsupported command: {_cmd}')
        
        print_log(f'Jonts = {robot.GetJoints()} Position = {robot.GetPose()}')    

    except KeyboardInterrupt as ex:
        print_log(f'Exiting by ^C \n{ex}')
        return
    
    except Exception as ex:
        print_log(f'Script robot cmd failed: {ex}')




def runScript(robot, scriptFile):
    try:
        with open(scriptFile) as script_file:

            subScript = yaml.safe_load(script_file)

            print_log(f'Script doc = {subScript}\n\n')

            if not _validateScript(subScript):
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
                        robot.SetJointVel(int(parm))
                        pass
                    case 'MovePose':
                        float_tuple_parm = tuple(map(float, parm.split(',')))
                        robot.MovePose(*float_tuple_parm)
                    case 'MoveJoints':
                        int_tuple_parm = tuple(map(float, parm.split(',')))
                        robot.MoveJoints(*int_tuple_parm)
                        pass
                    case 'GripperOpen':
                        robot.GripperOpen()
                    case 'GripperClose':
                        robot.GripperClose()
                    
                    case 'VacuumGrip':
                        robot.VacuumGrip()

                    case 'VacuumRelease':
                        robot.VacuumRelease()

                    case 'Delay':
                        float_tuple_parm = float(parm)
                        robot.Delay(float_tuple_parm)
                    
                    case 'SetVacuumPurgeDuration':
                        float_tuple_parm = float(parm)
                        robot.SetVacuumPurgeDuration(float_tuple_parm)


                    case _:     
                        print_log(f'Unsupported command: {_cmd}')
                
                print_log(f'Jonts = {robot.GetJoints()} Position = {robot.GetPose()}')

        
        while True:
            print_log(f'Jonts = {robot.GetJoints()} Position = {robot.GetPose()}')
            time.sleep(0.5)
            response_codes = [mdr_def.MxRobotStatusCode.MX_ST_GET_STATUS_ROBOT]
            response = robot.SendCustomCommand('GetStatusRobot', expected_responses=response_codes, timeout=10)
            _stat_res = tuple(map(int, response.data.split(',')))
            print_log(f'Status response = {response.data} / {_stat_res}')
            
            if _stat_res[0] == 1 and _stat_res[1] == 1 and  _stat_res[5] == 1 and _stat_res[6] == 1  :
                print_log(f'Exiting. Status = {response.data}/{_stat_res}')
                break


    except KeyboardInterrupt as ex:
        print_log(f'Exiting by ^C \n{ex}')
        return
    

    except Exception as ex:
        print_log(f'Script run proc failed: {ex}')


def _getPressure(robot)->float:
    response_codes = [mdr_def.MxRobotStatusCode.MX_ST_RT_VACUUM_PRESSURE]
    response = robot.SendCustomCommand('GetRtvacuumPressure', expected_responses=response_codes, timeout=10)
    _stat_res = tuple(map(float, response.data.split(',')))
    timestamp = _stat_res[0]
    pressure = _stat_res[1]
    print_log(f'PRESSURE: timestamp = {timestamp} / pressure = {pressure}')
    return float(pressure)

def test_automatic_script(scriptFile):
    try:
        with open(scriptFile) as script_file:

            subScript = yaml.safe_load(script_file)

            print_log(f'Script doc = {subScript}\n\n')

            if not _validateScript(subScript):
                print_log(f'Incorrect script format')
                sys.exit()
    except KeyboardInterrupt as ex:
        print_log(f'Exiting by ^C \n{ex}')
        return
    

    except Exception as ex:
        print_log(f'Script run proc failed: {ex}')


def runAsyril(robot, _ip:str, _port:int):
        
    try:
        _runRobotCmd(robot, f'SetJointVel({VELOCITY})')
        _runRobotCmd(robot, f'SetVacuumPurgeDuration({VACUUM_PURGE_DURATION})')
        _asiryl = AsyrilInterface(_ip, _port)


        for _i in range (10):
            for _rep in range (REPEATS):    
                _x, _y, _rz = _asiryl.get_pos()
                print(f'x= {_x}, y= {_y}, rz = {_rz}')
                print (f'Try # {_rep}')
                _runRobotCmd(robot, f'VacuumRelease()')
                _runRobotCmd(robot, f'MovePose({_x}, {_y}, {WORKING_H}, {ANGLE})')
                _runRobotCmd(robot, f'MovePose({_x}, {_y}, {PICK_H},  {_rz})')

                _runRobotCmd(robot, 'VacuumGrip()')
                _runRobotCmd(robot, f'Delay({PICK_DELAY})')
            
                _runRobotCmd(robot, f'MovePose({_x}, {_y}, {WORKING_H}, {_rz})')

                _runRobotCmd(robot, f'MoveJoints{MID_POS}')
                # _runRobotCmd(robot, f'MoveJoints(-67.69506, -55.04702, -65.00323, -1548.44055)')

                if _getPressure(robot) < PICK_PRESSURE:
                    break

            
            _runRobotCmd(robot, f'MoveJoints(-60.17617, 66.7551, -65.00227, -1507.78408)')
            # MovePose(173.9134, -72.42768, 44, 180)
            _runRobotCmd(robot, f'MovePose({PUT_X}, {PUT_Y - _i*5}, {WORKING_H}, {PUT_ANGLE})')
            _runRobotCmd(robot, f'MovePose({PUT_X}, {PUT_Y - _i*5}, {PUT_H}, {PUT_ANGLE})')
            _runRobotCmd(robot, f'VacuumRelease()')
            _runRobotCmd(robot, f'Delay({PUT_DELAY})')

            _runRobotCmd(robot, f'MovePose({PUT_X}, {PUT_Y - _i*5}, {WORKING_H}, {PUT_ANGLE})')

        while True:
            print_log(f'Jonts = {robot.GetJoints()} Position = {robot.GetPose()}')
            time.sleep(0.5)
            response_codes = [mdr_def.MxRobotStatusCode.MX_ST_GET_STATUS_ROBOT]
            response = robot.SendCustomCommand('GetStatusRobot', expected_responses=response_codes, timeout=10)
            _stat_res = tuple(map(int, response.data.split(',')))
            print_log(f'Status response = {response.data} / {_stat_res}')
            
            if _stat_res[0] == 1 and _stat_res[1] == 1 and  _stat_res[5] == 1 and _stat_res[6] == 1  :
                print_log(f'Exiting. Status = {response.data}/{_stat_res}')
                break

    except KeyboardInterrupt as ex:
        print_log(f'Exiting by ^C \n{ex}')
        return
    
    except Exception as ex:
        print_log(f'Script run proc failed: {ex}')



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
