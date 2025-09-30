__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "1.0.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Tool"


from curses.ascii import isdigit
from weakref import finalize
import serial as serial
import sys, os
import time
import threading
import ctypes
from threading import Lock
from collections import namedtuple

from inputimeout  import inputimeout , TimeoutOccurred
from dataclasses import dataclass
from queue import Queue 



from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, num2binstr, set_parm, get_parm, globalEventQ, smartLocker
import momanlibpy

#=========================== block  __name__ == "__main__" ================
if __name__ == "__main__":
    import datetime, logging
    logFileDate = datetime.datetime.now().strftime(f"LOG_%Y_%m_%d_%H_%M.txt")
    format = "%(asctime)s: %(filename)s--%(funcName)s/%(lineno)d -- %(thread)d [%(threadName)s] %(message)s" 
    logging.basicConfig(format=format, handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler(logFileDate, mode="w")], encoding='utf-8', level=logging.DEBUG, datefmt="%H:%M:%S")
#---------------  end of block  __name__ == "__main__" -------------------


# # void_f = lambda a : a
void_f = lambda a : None
print_log = void_f
# # print_log = logging.debug
# print_inf = logging.debug
# print_err = logging.debug
# print_DEBUG = logging.debug



'''
1 SLOBJ 6403.00 313032344B3031325352 (1024K012SR)  ?????

 Controlword

Bit Function Commands for the device control state machine


                       Shut Switch Disable  Quick  Disable Enable Fault 
					   Down    On  Voltage   Stop   Op       Op   Reset
0 Switch On            0       1      X       X     1        1      X
1 Enable Voltage       1       1      0       1     1        1      X
2 Quick Stop           1       1      X       0     1        1      X
3 Enable Operation     X       0      X       X     0        1      X
4 Operation Mode Specific
5 Operation Mode Specific
6 Operation Mode Specific
7 Fault Reset                                                     0 →1
8 Halt
----------------------------------------------------------------------
Statusword  
Bit Function State of the device control state machine
								  Not        Switch     Ready to   Switched   Operation  Quick    Fault     Fault
								Ready to       On        Switch       on       Enabled   Stop    Reaction 
								 Switch      Disabled      On                            Active   Active
								  On

0 Ready to Switch On              0            0             1          1          1        1       1         0
1 Switched On                     0            0             0          1          1        1       1         0
2 Operation Enabled               0            0             0          0          1        1       1         0
3 Fault                           0            0             0          0          0        0       1         1
4 *Voltage enabled                X            X             X          X          X        X       X         X
5 Quick Stop                      X            X             1          1          1        0       X         X
6 Switch On Disabled              0            1             0          0          0        0       0         0
7 Warning
...
10 Target reached
-------------------------------------------------------------------

0x6080, 0, MAX_SPEED, 4 			#max speed
0x6081, 0, SPEED, 4 				#profile velocity

0x6040,0,6, 2						# shutdown
0x6040,0,F, 2						# enable


0x6060, 0, 0x1, 1					# Modes of Operation = Profile Position
0x6060, 0, 0x3, 1					# Modes of Operation = Profile Velocity

>
0x607A, 0,  TARGET_POS, 4			# Target Position 
0x6040,0, 0xF, 2					# enable
0x6040,0, 0x3F, 2					# absolute 
---
0x607A, 0,  TARGET_POS, 4			# Target Position 
0x6040,0, 0xF, 2					# enable
0x6040,0, 0x7F, 2					# relative
>


#stop:
0x6040, 0, 0x010F, 2				# stop?? / bit 9   # BUGBUG ????
0x607A, 0, POS, 4					# set cur position
0x6040, 0, 3F, 2					# absolute

0x6078, 0                           # Current Actual Value


'''



# MEASUREMENT_DELAY:float = 0.25
# MINIMAL_OP_DURATION:float = 0.25
# GRIPPER_TIMEOUT:float = 10
# DEFAULT_CURRENT_LIMIT:int = 300
# DEFAULT_ROTATION_TIME:float = 5
# DEAFULT_VELOCITY_EV_VOLTAGE:int = 5000
# DevMaxSPEED:int = 15000
# DevOpSPEED:int = 640

# CMD_TIMEOUT_TRIALS:float = 5
# CMD_TIMEOUT_DELAY:float = 0.5

# MAX_SPEED_SET_CMD = (0x6080, 0x0, DevMaxSPEED, 0x4)
# # DEFAULT_SPEED_SET_CMD = (0x6081, 0x0, DevOpSPEED, 0x4)
# DEFAULT_SPEED_SET_CMD = (0x6081, 0x0, DevMaxSPEED, 0x4)


# FHv3_INIT_CMD_LST = [
#     MAX_SPEED_SET_CMD,                  # max speed
#     DEFAULT_SPEED_SET_CMD,              # default operational speed
#     SHUTDOWN_CMD,                       # shutdown
#     ENABLE_CMD,                         # enable
#     DISABLE_CMD                         # disable / unknown faulhaber huinya (copied from Motion manager)

# ]


GLOBAL_EX_LIMIT = 100
HOMING_ATTEMPTS = 3
HOMING_DELAY = 0.25

FHv3POSITION_ACTUAL_VALUE_QUERY = (0x6064, 0x0)
FHv3VELOCITY_ACTUAL_VALUE_QUERY = (0x606C, 0x0)
FHv3POSITION_QUERY = (0x2315, 0x3)
FHv3SN_QUERY = (0x1018, 0x04)
FHv3CURRENT_ACTUAL_VALUE_QUERY = (0x6078, 0x0 )
FHv3STATUS_QUERY = (0x6041, 0x00)



SHUTDOWN_CMD = (0x6040, 0x0, 0x6, 0x2)
ENABLE_CMD =  (0x6040, 0x0, 0xF, 0x2)
DISABLE_CMD = (0x6040, 0x0, 0xD, 0x2)
PROFILE_POSITION_MODE = (0x6060, 0x0, 0x1, 0x1)
PROFILE_VELOCITY_MODE = (0x6060, 0x0, 0x3, 0x1)
GO_TO_ABS_POS = (0x6040, 0x0, 0x3F, 0x2)    # Load the set-point immediately or after the end of a current movement task:
                                        # Bit 5 = 1: Movement towards the position starts immediately.
                                        # Bit 5 = 0: Movement towards the new position does not start until the preceding positioning task has been completed.
GO_TO_REL_POS = (0x6040, 0x0, 0x7F, 0x2)    # Same as above

TARGET_VELOCITY = (0x60FF, 0x0)           # 4 bytes value
TARGET_POSITION = (0x607A, 0x0)           # 4 bytes
SET_SPEED = (0x6081, 0x0)                 # 4 bytes - profile velocity

STALL_CMD_LST =[
    (0x6040, 0x00, 0x010F, 0x2),
    (0x60FF, 0x00, 0x00000000, 0x04),      # zero speed
    ENABLE_CMD
]


RESET_POS_CMD_LST = [
    (0x6060, 0x0, 0x6, 0x1),
    (0x6040, 0x0, 0xE, 0x2),
    (0x6040, 0x0, 0xF, 0x2),	
    (0x6098, 0x0, 0x25, 0x1),         # Homing method 37
    (0x6040, 0x0, 0xF, 0x2),
    (0x6040, 0x0, 0x1F, 0x2),
    DISABLE_CMD	

]


FHv3_REENABLE_CMD_LST = [
    (0x6040, 0, 0xE, 0x2),
    ENABLE_CMD
]

FAULT_RESET_LST =[
    (0x6040, 0x0, 0x80, 0x2),
    (0x6040, 0x0, 0x06, 0x2),
    (0x6040, 0x0, 0x0F, 0x2)
]

@dataclass 
class  interfaceType: 
    port: int
    channel: int
    devinfo: str
    serialN: int


'''
devName = device name: G1 / G2/ ... T1/ T2 / T3 ... R1 / R2 ... S1
devParms =- device parameters (dictionary) devParms[MEASUREMENT_DELAY] / devParms[DEAFULT_VELOCITY_EV_VOLTAGE] /....

'''

class FH_Motor_v3: 
    resultType = namedtuple("resultType", ["res", "answData", "query"])
    activated_devs = []                                     # port numbers
    protocol = None
    devices = None                                          # list of interfaceType
    intf = None
    protocol_dll=momanlibpy.DLL_PATH_CO_USB, 
    interface_dll=momanlibpy.DLL_PATH_MC3USB 
    fh_lock = Lock()                               # COM port access mutex 



    def __init__(self, fhv3port, channel, parms, devName):
#################################  conf parms per device ###########################
        self.MEASUREMENT_DELAY:float = 0.25
        self.MINIMAL_OP_DURATION:float = 0.25
        self.GRIPPER_TIMEOUT:float = 10
        self.DEFAULT_CURRENT_LIMIT:int = 300
        self.DEFAULT_ROTATION_TIME:float = 5
        self.DEAFULT_VELOCITY_EV_VOLTAGE:int = 5000
        self.DevMaxSPEED:int = 15000
        self.DevOpSPEED:int = 640
        self.EX_LIMIT = GLOBAL_EX_LIMIT
        self.__title = None
#########################################################################
        self.fhv3_port:int = fhv3port                           # 1,2,3..
        self.fhv3_chan:int = channel                            # 1,2,3..
        self.mDev_type = None                                 # devise type (--ROTATOR-- / --GRIPPERv3-- / --DIST_ROTATOR-- / --TIME_ROTATOR-- (SPINNER))
        self.mDev_pos:int = 0                                 #  current position 
        self.el_current_limit:int = 0                       # electrical current limit to stop 
        self.el_current_on_the_fly:int = 0                  # On-the-fly current                    
        self.wd = None                                      # watch dog identificator
        self.mDev_SN = None                                   # Serial N (0x1018:0x04)
        self.mDev_status = False                              # device status (bool) / used for succesful initiation validation
        self.mDev_in_motion = False                           # is device in motion
        self.possition_control_mode = False                 # TRUE - control possition, FALSE - don't
        self.time_control_mode = False                      # TRUE - time control
        self.mDev_pressed = False                             # is motion button pressed
        self.gripper_onof = True                            # True means off )))
#------- Bad practice --------
        self.el_voltage:int = self.DEAFULT_VELOCITY_EV_VOLTAGE       # forCOMPATABILTY ONLY (BUGBUG)
        self.rpm:int = self.DevOpSPEED 
#------- Bad practice --------
        self.start_time: float = 0                                   # Start thread time
        self.success_flag = True                            # end of op flag
        self.rotationTime:float = 0                               # rotation time
        self.diameter = None
        self.gear = None
        self.new_pos = 0
        self.devName = devName
        self.dev_lock =  Lock()                 # device lock to avoid multiply access
        self.devNotificationQ = Queue()
        
        try:

            print_log(f'Starting devName = {self.devName}')

            # answA = FH_Motor_v3.FH_cmd(self.fhv3_port, [FHv3POSITION_QUERY], lock=FH_Motor_v3.fh_lock)
            answA = FH_Motor_v3.FH_cmd(self.fhv3_port, [FHv3POSITION_ACTUAL_VALUE_QUERY], lock=FH_Motor_v3.fh_lock)
            
            if len(answA):
                self.mDev_pos = s32(answA[0].answData)
            else:
                print_err(f'FHv3 ({devName}) __init__ ERROR. Cant retrive current position')
            
            answA = FH_Motor_v3.FH_cmd(self.fhv3_port, [FHv3SN_QUERY], lock = FH_Motor_v3.fh_lock)
            if len(answA):
                self.mDev_SN = answA[0].answData 
            else:
                print_err(f'FHv3 ({devName}) __init__ ERROR. Cant retrive S/N')

            print_inf(f'({devName}) Serial number = {self.mDev_SN} Possition = {self.mDev_pos}')

        except Exception as ex:

            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
   
            print_err(f"({devName}) ERROR. Connection to FHv3 port {self.fhv3_port} was lost. Unexpected Exception: {ex}")
            return                                  # no valid FAULHABBER motor can be added
        
        else:
            self.mDev_status = True
            pass

        if self.fhv3_port in FH_Motor_v3.activated_devs:
            print_err(f"**ERROR** . ({devName}) Device with port = {self.fhv3_port} already activated")
        else:
            FH_Motor_v3.activated_devs.append(self.fhv3_port)

                
        # self.MEASUREMENT_DELAY = parms[devName]['MEASUREMENT_DELAY'] if ((devName in parms.keys()) and ('MEASUREMENT_DELAY' in parms[devName].keys())) else parms['DEAFULT']['MEASUREMENT_DELAY']
        # self.MINIMAL_OP_DURATION = parms[devName]['MINIMAL_OP_DURATION'] if ((devName in parms.keys()) and ('MINIMAL_OP_DURATION' in parms[devName].keys())) else parms['DEAFULT']['MINIMAL_OP_DURATION']
        # self.GRIPPER_TIMEOUT  = parms[devName]['GRIPPER_TIMEOUT'] if ((devName in parms.keys()) and ('GRIPPER_TIMEOUT' in parms[devName].keys())) else parms['DEAFULT']['GRIPPER_TIMEOUT']
        # self.DEFAULT_CURRENT_LIMIT = parms[devName]['DEFAULT_CURRENT_LIMIT'] if ((devName in parms.keys()) and ('DEFAULT_CURRENT_LIMIT' in parms[devName].keys())) else parms['DEAFULT']['DEFAULT_CURRENT_LIMIT']
        # self.DEFAULT_ROTATION_TIME  = parms[devName]['DEFAULT_ROTATION_TIME'] if ((devName in parms.keys()) and ('DEFAULT_ROTATION_TIME' in parms[devName].keys())) else parms['DEAFULT']['DEFAULT_ROTATION_TIME']
        # self.DEAFULT_VELOCITY_EV_VOLTAGE = parms[devName]['DEAFULT_VELOCITY_EV_VOLTAGE'] if ((devName in parms.keys()) and ('DEAFULT_VELOCITY_EV_VOLTAGE' in parms[devName].keys())) else parms['DEAFULT']['DEAFULT_VELOCITY_EV_VOLTAGE']
        # self.DevMaxSPEED = parms[devName]['DevMaxSPEED'] if ((devName in parms.keys()) and ('DevMaxSPEED' in parms[devName].keys())) else parms['DEAFULT']['DevMaxSPEED']
        # self.DevOpSPEED  = parms[devName]['DevOpSPEED'] if ((devName in parms.keys()) and ('DevOpSPEED' in parms[devName].keys())) else parms['DEAFULT']['DevOpSPEED']
        # self.EX_LIMIT = int(get_parm(devName, parms, 'TOLERANCE_LIMIT')) 
        # self.__title = get_parm(devName, parms, 'TITLE')
        
        # self.el_voltage =  self.DEAFULT_VELOCITY_EV_VOLTAGE
        # self.rpm = self.DevOpSPEED

        self.set_parms(parms)

    def __del__(self):
        print_inf(f'Releasing FAULHABER v3 on port {self.fhv3_port}/{FH_Motor_v3.devices[self.fhv3_port-1]}')

        self.mDev_in_motion = False

        try:
            FH_Motor_v3.FH_cmd(self.fhv3_port, [DISABLE_CMD], FH_Motor_v3.fh_lock)  # disable motor
            print_inf(f'({self.devName}) FAULHABER disabled on port {self.fhv3_port}.')
            FH_Motor_v3.activated_devs.remove(self.fhv3_port)
        except  Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'({self.devName})  FAULHABER on port {self.fhv3_port} could not be closed. Exception: {ex} of type: {type(ex)}.')
        finally:
            pass


        if not len(FH_Motor_v3.activated_devs):         # if no more active devices - deactivate
            print_inf(f'No more active FHv3 devices. Exiting.')
            try:
                # trying to close,
                # as described in EN_7000_05064.pdf 3.1.1 Synchronous communication
                FH_Motor_v3.devices = None
                FH_Motor_v3.protocol.mmProtCloseInterface()
                pass

            except OSError:
                print_err(f'OS ERROR.  FHv3 on port {self.fhv3_port} could not be closed cortrectly. Exception: {ex} of type: {type(ex)}.') 
                pass  # we are done here anyways
            except  Exception as ex:
                e_type, e_filename, e_line_number, e_message = exptTrace(ex)
                print_err(f'ERROR.  FHv3 on port {self.fhv3_port} could not be closed cortrectly. Exception: {ex} of type: {type(ex)}.')
            finally:
                pass



    @staticmethod
    def enum_devs(interface_dll = momanlibpy.DLL_PATH_MC3USB):

        FH_Motor_v3.devices = []
        try:   
            FH_Motor_v3.intf = momanlibpy.MomanIntf(interface_dll)
            port_amount, ports, channels, deviceinfos = FH_Motor_v3.intf.mmIntfEnumPorts()
        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)

            print_err(f"Error enumerating FaulHaber v3 devices. Exception: {ex} of type: {type(ex)}")
            FH_Motor_v3.devices = None
            
        else:
            print_inf (f'Found {port_amount} ports: list={ports}, channels={channels}')
            for index, inf in enumerate(deviceinfos):
                print_inf(f'{inf}, s/n = 0x{inf[-8:]} = {int(inf[-8:], 16)}')
                new_dev = interfaceType(port = int(ports[index]),      
                                        channel=int(channels[index]),   
                                        devinfo=deviceinfos[index], serialN = int(inf[-8:], 16))
                FH_Motor_v3.devices.append(new_dev)
        
        finally:
            return FH_Motor_v3.devices

        


    @staticmethod
    def init_devices(protocol_dll = momanlibpy.DLL_PATH_CO_USB, interface_dll = momanlibpy.DLL_PATH_MC3USB):
        FH_Motor_v3.protocol_dll = protocol_dll
        FH_Motor_v3. interface_dll = interface_dll
        init_lock = Lock()


        try:
            FH_Motor_v3.enum_devs(interface_dll)

            if FH_Motor_v3.devices is None or len(FH_Motor_v3.devices) == 0:
                print_inf("No v3 devices detected in the system")
                FH_Motor_v3.devices = None
                return None

            FH_Motor_v3.protocol = momanlibpy.MomanProt(protocol_dll=protocol_dll)

            interface_result = FH_Motor_v3.protocol.mmProtInitInterface(interface_dll)
            # if not (interface_result is momanlibpy.eMomanprot.eMomanprot_ok):  # i.e. eMomanprot_error
            if not (interface_result == momanlibpy.eMomanprot.eMomanprot_ok):  # i.e. eMomanprot_error
                print_err(f"ERROR loading Momanprot interface: {interface_result} ")
                FH_Motor_v3.devices = None
                return None


            print_inf(f'Active Faulhaber v3 devices:\n{FH_Motor_v3.devices}')
                    
                
            
        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f"Error initiatioin FaulHaber v3 devices. Exception: {ex} of type: {type(ex)}")
            FH_Motor_v3.devices = None
            return None
        
        return FH_Motor_v3.devices
            
    @staticmethod
    def deactivate_inteface():
        FH_Motor_v3.devices = None
        FH_Motor_v3.protocol.mmProtCloseInterface()



    @staticmethod
    def res2text(res):
        ret = 'Unrecognized error'
        # if res == momanlibpy.eMomanprot.eMomanprot_ok:
        if res is momanlibpy.eMomanprot.eMomanprot_ok: 
           ret = f'res= eMomanprot_ok'
        elif res is momanlibpy.eMomanprot.eMomanprot_error_timeout:
            ret = f'res= eMomanprot_error_timeout (No response)'
        elif res  is momanlibpy.eMomanprot.eMomanprot_error_cmd:
            ret=f'res= eMomanprot_error_cmd (Error message from device)'
        elif res is momanlibpy.eMomanprot.eMomanprot_error:
            ret = f'res= eMomanprot_error (Other error)'
        else:
            ret = f'res={res} (Unknown error)'



        return ret
    

##############################   
#  FH_cmd() 
#
#
##############################

    @staticmethod
    def FH_cmd(port, arr, lock = None, node = 1):

        retValues = []
        
        if len(arr) == 0:
            print_err(f'Empty CMD array = {arr}')
            return retValues
        

        # sL = smartLocker(lock)                  #  mutex for FH channel
        sL = smartLocker(FH_Motor_v3.fh_lock)                  #  mutex for FH channel
        
        try:

            print_log(f'mmIntfIsPortAvailable({port}, {FH_Motor_v3.devices[port-1].channel}), intf = {FH_Motor_v3.intf}, protocol = {FH_Motor_v3.protocol}')
            if FH_Motor_v3.intf.mmIntfIsPortAvailable(port, FH_Motor_v3.devices[port-1].channel) and FH_Motor_v3.protocol:
                print_log(f"Port {port} is available. Interface: {FH_Motor_v3.devices[port-1]}")
                baudrate = 0
                # baudrates_list = FH_Motor_v3.protocol.mmProtSupportedBaudratesList()
                # print_log(f'baudrates_list={baudrates_list}')
                # baudrate = baudrates_list[0] if len(baudrates_list) > 0 else 0
                print_log(f'mmProtOpenCom({port}, {FH_Motor_v3.devices[port-1].channel},{baudrate})')
                open_result = FH_Motor_v3.protocol.mmProtOpenCom(
                    port,
                    FH_Motor_v3.devices[port-1].channel,
                    baudrate,  # 0 for usb
                )
                
                if not (open_result == momanlibpy.eMomanprot.eMomanprot_ok):

                    print_err(f"ERROR. Port {port} failed to open. Interface: {FH_Motor_v3.devices[port-1]}")
                    return retValues

            else:
                print_err(f"ERROR. Port {port} is not available. Interface: {FH_Motor_v3.devices[port-1]}")
                return retValues
            
            print_log(f"Port {port} was opened successfuly. Interface: {FH_Motor_v3.devices[port-1]}")

        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'Exception: {ex} of type: {type(ex)} for interface {FH_Motor_v3.devices[port-1]}.')
            # raise ex
            return retValues

        print_log(f'CMD array = {arr}')
        for _cmd in arr:
            res = None
            answData = None
            try:      
                if len(_cmd) == 2:
                        res, answData = FH_Motor_v3.protocol.mmProtGetObj(node, _cmd[0], _cmd[1])            # Indication of the device type
                        print_log(f"mmProtGetObj({node}, 0x{_cmd[0]:04x}, 0x{_cmd[1]:04x}) = {res} / answData=0x{answData:04x} ({answData})")
                elif len(_cmd) == 4:
                        res, fU = FH_Motor_v3.protocol.mmProtSetObj(node, _cmd[0], _cmd[1], _cmd[2], _cmd[3])            # Indication of the device type
                        print_log(f"mmProtSetObj({node}, 0x{_cmd[0]:04x}, 0x{_cmd[1]:04x}, 0x{_cmd[2]:04x}, 0x{_cmd[3]:04x}) = {res} ")
                else:
                    print_err(f'Error: Wrong command/query format: {_cmd}')
                    continue
                
                if res is  momanlibpy.eMomanprot.eMomanprot_ok:
                    retValues.append(FH_Motor_v3.resultType(res = res, answData = answData, query=_cmd))
                else:
                    print_err(f'Error executing CMD = {_cmd} with res = {res}')

            except Exception as ex:
                    e_type, e_filename, e_line_number, e_message = exptTrace(ex)
                    print_err(f"Exception: {ex} of type: {type(ex)} on cmd {_cmd}")
                    continue
                ############
        try:
            print_log(f"mmProtCloseCom()")
            FH_Motor_v3.protocol.mmProtCloseCom()

        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f"Exception at mmProtCloseCom: {ex} of type: {type(ex)}")
            
        

        return retValues


    def init_dev(self, type) -> bool:
        MAX_SPEED_SET_CMD = (0x6080, 0x0, self.DevMaxSPEED, 0x4)
        # DEFAULT_SPEED_SET_CMD = (0x6081, 0x0, DevOpSPEED, 0x4)
        DEFAULT_SPEED_SET_CMD = (0x6081, 0x0, self.DevMaxSPEED, 0x4)

        FHv3_INIT_CMD_LST = [
            MAX_SPEED_SET_CMD,                  # max speed
            DEFAULT_SPEED_SET_CMD,              # default operational speed
            SHUTDOWN_CMD,                       # shutdown
            ENABLE_CMD,                         # enable
            DISABLE_CMD                         # disable / unknown faulhaber huinya (copied from Motion manager)
        ]

        self.mDev_type = type
        print_inf(f'Initiating FHv3 devise of {self.mDev_type} type on port {self.fhv3_port}')
        try:
            if self.mDev_type == '--DIST_ROTATOR--' or self.mDev_type == '--TIME_ROTATOR--' or self.mDev_type == '--TROLLEY--' or self.mDev_type == '--GRIPPERv3--':
                self.el_current_limit = self.DEFAULT_CURRENT_LIMIT
                res = FH_Motor_v3.FH_cmd(self.fhv3_port, FHv3_INIT_CMD_LST,lock = FH_Motor_v3.fh_lock )
                    
                print_inf (f"Port= {self.fhv3_port},  init commands =  {FHv3_INIT_CMD_LST} with res=  {res}" )
            
            if self.mDev_type == '--TIME_ROTATOR--':
                self.rotationTime = self.DEFAULT_ROTATION_TIME

        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'FAULHABER v3 initiation on port {self.fhv3_port} failed. Exception: {ex} of type: {type(ex)}.')
            return False

        return True

    def  mDev_watch_dog_thread(self):
        print_inf (f'>>> WatchDog FHv3  started on FHv3 port = {self.fhv3_port}, dev = {self.devName}, position = {self.mDev_pos}')
        time.sleep(self.MEASUREMENT_DELAY)                 # waif for a half of sec
        self.success_flag = True

        self.mDev_in_motion = True
        max_GRC:int = 0
        while (self.mDev_in_motion):
            try:
                answA = FH_Motor_v3.FH_cmd(self.fhv3_port, [FHv3CURRENT_ACTUAL_VALUE_QUERY], lock=FH_Motor_v3.fh_lock)
                print_log(f'WatchDog FHv3: FHv3CURRENT_ACTUAL_VALUE_QUERY={answA}, len = {len(answA)}')
                if len(answA):
                    answ:int = s16(answA[0].answData)
                    print_log(f'WatchDog FHv3: FHv3CURRENT_ACTUAL_VALUE_QUERY={answ}')

                    self.el_current_on_the_fly = answ

                    max_GRC = abs(answ) if abs(answ) > max_GRC else max_GRC

                    if (int(abs(answ)) > int(self.el_current_limit)):
                        print_inf(f' WatchDog FHv3: Actual Current Value = {answ}, Limit = {self.el_current_limit}')
                        if (self.mDev_type == '--TROLLEY--' or self.mDev_type == '--DIST_ROTATOR--') and self.possition_control_mode:
                            _pos = self.mDev_get_cur_pos()
                            if abs(_pos - self.new_pos) > self.EX_LIMIT:
                                print_log(f'Desired position [{self.new_pos}] is not reached. Current position = {_pos}')
                                self.success_flag = False

                        break


                else:
                    print_err(f'WatchDog FHv3 failed read FHv3CURRENT_ACTUAL_VALUE_QUERY on port  {self.fhv3_port}.')


                
                if self.mDev_type == '--GRIPPERv3--':
                    end_time = time.time()
                    if end_time - self.start_time > self.GRIPPER_TIMEOUT:
                        print_inf(f' WatchDog FHv3: GRIPPER operation canceled by timeout, port = {self.fhv3_port}, current GRC = {answ}, Limit = {self.el_current_limit}, max GRC = {max_GRC} ')
                        break

                if self.mDev_type == '--TIME_ROTATOR--':
                    end_time = time.time()
                    if end_time - self.start_time > self.rotationTime:
                        print_inf(f' WatchDog FHv3: TIME ROTATOR operation completed, port = {self.fhv3_port}, current GRC = {answ}, Limit = {self.el_current_limit}, max GRC = {max_GRC} ')
                        break


                if (self.mDev_type == '--TROLLEY--' or self.mDev_type == '--DIST_ROTATOR--') and self.possition_control_mode :
                    answA = FH_Motor_v3.FH_cmd(self.fhv3_port, [FHv3STATUS_QUERY], lock=FH_Motor_v3.fh_lock)
                    print_log(f'WatchDog FHv3: FHv3STATUS_QUERY={answA}, len = {len(answA)}')
                    if len(answA):
                        answ = answA[0].answData
                        fh_op_status = answ
                        if fh_op_status & 0b00000000010000000000:        # Position reached - bit 10 at statusword 

                            _pos = self.mDev_get_cur_pos()
                            if abs(_pos - self.new_pos) < GLOBAL_EX_LIMIT:          # WORKAROUND Since FAULHABER fails and informs about "target position is achieved"
                                                                                    # althought it's NOT!
                                print_inf(f'Operational status on WatchDog FHv3 port {self.fhv3_port} = {num2binstr(fh_op_status)}, position = {_pos}, target = {self.new_pos}')

                                break
                    else:
                        print_err(f'WatchDog FHv3 failed read FHv3STATUS_QUERY on port = {self.fhv3_port}.')
                    


            except Exception as ex:
                e_type, e_filename, e_line_number, e_message = exptTrace(ex)
                print_err(f'WatchDog FHv3 failed on port = {self.fhv3_port}. Exception: {ex} of type: {type(ex)}.')
                self.success_flag = False
                break
            finally:
                pass

            time.sleep(0.1)
            
        end_time = time.time()
        _pos = self.mDev_get_cur_pos()
        print_inf (f'>>> WatchDog FHv3  completed on FHv3 port = {self.fhv3_port}, dev = {self.devName}, position = {_pos}, target = {self.new_pos}')
        print_inf(f' WatchDog FHv3: Start time = {self.start_time}, end time ={end_time}, delta = {end_time - self.start_time}')
        if not (self.mDev_type == '--DIST_ROTATOR--') and end_time - self.start_time - self.MEASUREMENT_DELAY < self.MINIMAL_OP_DURATION:
            print_inf(f' WatchDog FHv3: Abnormal FHv3 termination on port = {self.fhv3_port}')
            self.success_flag = False

        
    
        if self.mDev_in_motion:
            print_inf(f'Thread is being stoped')
            # if self.mDev_type == '--DIST_ROTATOR--' or self.mDev_type == '--GRIPPERv3--':     # stop as is with no realeasing for ROTATOR devs
            #     self.mDev_stall()
            # else:
            #     self.mDev_stop()
            self.mDev_stop()
        else:
            print_log(f'Operation (watchdog thread) stoped by external event')
            if self.dev_lock.locked():
                self.dev_lock.release()
            else:
                print_err(f'-WARNING unlocket mutual access mutex')

            
        
        print_log(f'Exithing watch dog thread with {self.success_flag} result')
        print(f'***DEBUG*** Exithing watch dog thread with {self.success_flag} result. mDev_in_motion = {self.mDev_in_motion}')
        self.mDev_get_cur_pos()
        self.devNotificationQ.put(self.success_flag)
        
        return
    

    def  mDev_watch_dog(self):
        self.start_time = time.time()
        self.wd = threading.Thread(target=self.mDev_watch_dog_thread)
        self.wd.start()
        return self.wd

    def  mDev_stop(self)->bool:
        FH_Motor_v3.FH_cmd(self.fhv3_port, [DISABLE_CMD], lock = FH_Motor_v3.fh_lock)  # disable motor
        self.mDev_in_motion = False
        print_inf(f'Motor of {self.mDev_type} type is being disabled on interface: {FH_Motor_v3.devices[self.fhv3_port-1]}')

        if self.dev_lock.locked():
            self.dev_lock.release()

        self.mDev_get_cur_pos()
        return True


    def gripper_on(self)->bool:
        if not self.mutualControl():
            return False

        self.success_flag = True
        print_inf(f'gripper_on on interface: {FH_Motor_v3.devices[self.fhv3_port-1]}, velocity = self.rpm')

        try:
            FH_Motor_v3.FH_cmd(self.fhv3_port, [
                                                DISABLE_CMD,   ##### RELEASE previous stall mode
                                                PROFILE_VELOCITY_MODE, 
                                                SHUTDOWN_CMD, 
                                                ENABLE_CMD, 
                                                (*TARGET_VELOCITY, int(self.rpm), 4)], 
                                                lock = FH_Motor_v3.fh_lock)
                                                                                                          
        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'FHv3 gripper on failed on port = {self.fhv3_port}. Exception: {ex} of type: {type(ex)}.')
            self.success_flag = False
            if self.dev_lock.locked():
                self.dev_lock.release()
            return False
            
        self.gripper_onof = True    
        self.mDev_watch_dog()
        return True

    def gripper_off(self)->bool:
        if not self.mutualControl():
            return False

        self.success_flag = True
        print_inf(f'gripper_on on interface: {FH_Motor_v3.devices[self.fhv3_port-1]}, velocity = self.rpm')

        try:
            FH_Motor_v3.FH_cmd(self.fhv3_port, [
                                                DISABLE_CMD,   ##### RELEASE previous stall mode
                                                PROFILE_VELOCITY_MODE, 
                                                SHUTDOWN_CMD, 
                                                ENABLE_CMD, 
                                                (*TARGET_VELOCITY, (-1)*int(self.rpm), 4)], 
                                                lock = FH_Motor_v3.fh_lock)
                                                                                                          # disable motor
        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'FHv3 gripper on failed on port = {self.fhv3_port}. Exception: {ex} of type: {type(ex)}.')
            self.success_flag = False
            if self.dev_lock.locked():
                self.dev_lock.release()
            return False
            
        self.gripper_onof = True    
        self.mDev_watch_dog()
        return True

    def timeRotaterFw(self, _time, velocity = None)->bool:
        if not self.mutualControl():
            return False

        if velocity == None:
            velocity= self.rpm


        self.success_flag = True
        self.possition_control_mode = False
        self.time_control_mode = True 
        self.rotationTime = _time
        print_inf(f'Going forward (time) on port = {self.fhv3_port} for {_time} seconds, velocity = {velocity}')

        
        try:

            FH_Motor_v3.FH_cmd(self.fhv3_port, [
                                                DISABLE_CMD,   ##### RELEASE previous stall mode
                                                PROFILE_VELOCITY_MODE, 
                                                SHUTDOWN_CMD, 
                                                ENABLE_CMD, 
                                                (*TARGET_VELOCITY, int(velocity), 4)], 
                                                lock = FH_Motor_v3.fh_lock)
                                                                                          
        except Exception as ex:
                e_type, e_filename, e_line_number, e_message = exptTrace(ex)
                print_err(f'FHv3 time FW failed on port = {self.fhv3_port}. Exception: {ex} of type: {type(ex)}.')
                self.success_flag = False
                if self.dev_lock.locked():
                    self.dev_lock.release()
                return False
        else:
            print_inf (f"FHv3 time FW started on port = {self.fhv3_port} " )
        
        self.mDev_watch_dog()
        return True
        

    def timeRotaterBw(self, _time, velocity = None)->bool:
        if not self.mutualControl():
            return False

        if velocity == None:
            velocity = self.rpm

        self.success_flag = True
        self.possition_control_mode = False
        self.time_control_mode = True 
        self.rotationTime = _time
        print_inf(f'Going backward (time) on port = {self.fhv3_port}  for {_time} seconds, velocity = {velocity}')

      
        try:
            FH_Motor_v3.FH_cmd(self.fhv3_port, [
                                                DISABLE_CMD,   ##### RELEASE previous stall mode
                                                PROFILE_VELOCITY_MODE, 
                                                SHUTDOWN_CMD, 
                                                ENABLE_CMD, 
                                                (*TARGET_VELOCITY, (-1)*int(velocity), 4)], 
                                                lock = FH_Motor_v3.fh_lock)


            
        except Exception as ex:
                e_type, e_filename, e_line_number, e_message = exptTrace(ex)
                print_err(f'FHv3 Time BW failed on port = {self.fhv3_port}. Exception: {ex} of type: {type(ex)}.')
                self.success_flag = False
                if self.dev_lock.locked():
                    self.dev_lock.release()
                return False
        else:
            print_inf (f"FHv3 Time BW started on port = {self.fhv3_port} " )
 

        self.mDev_watch_dog()
        return True

    def go2pos(self, new_position, velocity = None, stall=None)->bool:
        if not self.mutualControl():
            return False

        self.new_pos = new_position
        if velocity == None:
            velocity = self.rpm

        self.success_flag = True
        self.possition_control_mode = True
        self.time_control_mode = False 
        print_inf(f'FHv3 GO2POS {new_position} velocity = {velocity} ')
        try:
            FH_Motor_v3.FH_cmd(self.fhv3_port, [
                                                DISABLE_CMD,   ##### RELEASE previous stall mode
                                                PROFILE_POSITION_MODE, 
                                                (*SET_SPEED,  int(velocity), 4),  
                                                SHUTDOWN_CMD, 
                                                # ENABLE_CMD, 
                                                (*TARGET_POSITION, int(new_position), 4),
                                                GO_TO_ABS_POS], 
                                                lock = FH_Motor_v3.fh_lock)




        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'FHv3 gripper on failed on port = {self.fhv3_port}. Exception: {ex} of type: {type(ex)}.')
            self.success_flag = False
            if self.dev_lock.locked():
                self.dev_lock.release()
            return False
            
        self.mDev_watch_dog()  
        return True

    def  mDev_stall(self)->bool:
        # safeRPM = self.rpm
        # self.rpm = 0
        # self.mDev_forward()
        # self.rpm = safeRPM
        

        try:
            FH_Motor_v3.FH_cmd(self.fhv3_port, STALL_CMD_LST, lock = FH_Motor_v3.fh_lock)




        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'FHv3 dev failed to stall on port = {self.fhv3_port}. Exception: {ex} of type: {type(ex)}.')
            return False

        self.mDev_in_motion = False
        pass

        if self.dev_lock.locked():
            self.dev_lock.release()
        return True

    def  mDev_forward(self, velocity = None, timeout=None, polarity = None, stall = None)->bool:
        if not self.mutualControl():
            return False

        self.success_flag = True
        self.possition_control_mode = False
        self.time_control_mode = False 
        if self.rpm == 0:
            print_inf(f'Going stall on port = {self.fhv3_port}')
        else:
            print_inf(f'Going forward on port = {self.fhv3_port}, velocity = {self.rpm}')
        
        try:

            FH_Motor_v3.FH_cmd(self.fhv3_port, [
                                                DISABLE_CMD,   ##### RELEASE previous stall mode
                                                PROFILE_VELOCITY_MODE, 
                                                SHUTDOWN_CMD, 
                                                ENABLE_CMD, 
                                                (*TARGET_VELOCITY, int(self.rpm), 4)], 
                                                lock = FH_Motor_v3.fh_lock)
                                                                                          
        except Exception as ex:
                e_type, e_filename, e_line_number, e_message = exptTrace(ex)
                print_err(f'FHv3 forward failed on port = {self.fhv3_port}. Exception: {ex} of type: {type(ex)}.')
                self.success_flag = False
                if self.dev_lock.locked():
                    self.dev_lock.release()
                return False
        else:
            print_inf (f"FHv3 forward/stall started on port = {self.fhv3_port}" )
        
        if  self.rpm:                       # no need watchdog for zero speed
            self.mDev_watch_dog()
        return True

    def  mDev_backwrd(self, velocity = None, timeout=None, polarity = None, stall = None)->bool:
        if not self.mutualControl():
            return False

        self.success_flag = True
        self.possition_control_mode = False
        self.time_control_mode = False 
        print_inf(f'Going backward on port = {self.fhv3_port}, velocity = {self.rpm}')

      
        try:
            FH_Motor_v3.FH_cmd(self.fhv3_port, [
                                                DISABLE_CMD,   ##### RELEASE previous stall mode
                                                PROFILE_VELOCITY_MODE, 
                                                SHUTDOWN_CMD, 
                                                ENABLE_CMD, 
                                                (*TARGET_VELOCITY, (-1)*int(self.rpm), 4)], 
                                                lock = FH_Motor_v3.fh_lock)


            
        except Exception as ex:
                e_type, e_filename, e_line_number, e_message = exptTrace(ex)
                print_err(f'FHv3 backward failed on port = {self.fhv3_port}. Exception: {ex} of type: {type(ex)}.')
                self.success_flag = False
                if self.dev_lock.locked():
                    self.dev_lock.release()
                return False
        else:
            print_inf (f"FHv3 backward started on port = {self.fhv3_port} " )
 

        if  self.rpm:                       # no need watchdog for zero speed
            self.mDev_watch_dog()

        return True

   
    def mDev_stored_pos(self): 
        return self.mDev_pos


    def mDev_get_cur_pos(self) -> int:
        try:
            # answA = FH_Motor_v3.FH_cmd(self.fhv3_port, [FHv3POSITION_QUERY], lock=FH_Motor_v3.fh_lock)
            answA = FH_Motor_v3.FH_cmd(self.fhv3_port, [FHv3POSITION_ACTUAL_VALUE_QUERY], lock=FH_Motor_v3.fh_lock)

            
            if len(answA):
                answ = s32(answA[0].answData)
                self.mDev_pos = answ
            else:
                print_err(f'ERROR retriving position on port {self.fhv3_port} Ret Array = answA ')

        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f"ERROR retriving position on port {self.fhv3_port},  Unexpected Exception: {ex}")
            return 0         
        else:
            return self.mDev_pos        

    def  mDev_reset_pos(self)->bool:

        self.mDev_stop()
        try:
            _pos = GLOBAL_EX_LIMIT + 10
            _attempts = 0
            while  _attempts < HOMING_ATTEMPTS:
                FH_Motor_v3.FH_cmd(self.fhv3_port, RESET_POS_CMD_LST, lock=FH_Motor_v3.fh_lock)
                print_log (f"FHv3 HOME attempt {_attempts + 1} on port = {self.fhv3_port}, dev = {self.devName} " )
                _pos = self.mDev_get_cur_pos()
                if abs(_pos) < GLOBAL_EX_LIMIT:
                    break
                time.sleep(HOMING_DELAY)
                _attempts += 1

            print_inf (f"FHv3 HOME done on port = {self.fhv3_port} , position = {_pos}" )

        except Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f"FHv3 HOME faled on port {self.fhv3_port}, Unexpected Exception: {ex} ")
            return False         
        else:
            return True  


    def getTitle(self)->str:
        return self.__title

    def set_parms(self, parms):
        self.MEASUREMENT_DELAY = get_parm(self.devName, parms, 'MEASUREMENT_DELAY')
        self.MINIMAL_OP_DURATION = get_parm(self.devName, parms, 'MINIMAL_OP_DURATION')
        self.GRIPPER_TIMEOUT  = get_parm(self.devName, parms, 'GRIPPER_TIMEOUT')
        self.DEFAULT_CURRENT_LIMIT = get_parm(self.devName, parms, 'DEFAULT_CURRENT_LIMIT')
        self.DEFAULT_ROTATION_TIME  = get_parm(self.devName, parms, 'DEFAULT_ROTATION_TIME')
        self.DEAFULT_VELOCITY_EV_VOLTAGE = get_parm(self.devName, parms, 'DEAFULT_VELOCITY_EV_VOLTAGE')
        self.DevMaxSPEED = get_parm(self.devName, parms, 'DevMaxSPEED')
        self.DevOpSPEED  = get_parm(self.devName, parms, 'DevOpSPEED')
        self.EX_LIMIT = int(get_parm(self.devName, parms, 'TOLERANCE_LIMIT')) 
        self.__title = get_parm(self.devName, parms, 'TITLE')

        # self.diameter = int(get_parm(self.devName, parms, 'DIAMETER')) if (_parm := get_parm(self.devName, parms, 'DIAMETER')) is not None and _parm.isdigit()  else None
        # self.gear = int(get_parm(self.devName, parms, 'GEAR')) if (_parm := get_parm(self.devName, parms, 'GEAR')) is not None and _parm.isdigit()  else None
        
        # self.diameter = int(_parm := get_parm(self.devName, parms, 'DIAMETER')) if  _parm is not None and _parm.isdigit()  else None
        # self.gear = int(_parm := get_parm(self.devName, parms, 'GEAR')) if  _parm is not None and _parm.isdigit()  else None


        _diam_parm:str = str(get_parm(self.devName, parms, 'DIAMETER'))
        _gear_parm:str = str(get_parm(self.devName, parms, 'GEAR'))

        if _diam_parm is not None and _diam_parm.isdigit():
            self.diameter = int(_diam_parm)

        if _gear_parm is not None and _gear_parm.isdigit():
            self.gear = int(_gear_parm)

        self.el_voltage =  self.DEAFULT_VELOCITY_EV_VOLTAGE
        self.rpm = self.DevOpSPEED

        print_inf (f'Loading parms for {self.devName} -> {self}')


    def mutualControl(self):
        if self.dev_lock.locked():
            print_err(f'ERROR- The device {self.devName} on port {self.fhv3_port} is active. Cant allow multiply activations')
            return False
        else:
            self.dev_lock.acquire()
            return True





#------------------------- U N I T E S T ----------------------------
    
if __name__ == "__main__":

    import PySimpleGUI as sg

    image_left = './Images/button_left_c.png'
    image_right = './Images/button_right_c.png'

# OFF button diagram
    toggle_btn_off = b'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'

# ON button diagram
    toggle_btn_on = b'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'


    trolley  = [[sg.Push(), sg.Text('Trolley', font='Any 20'),  sg.Push()],
            [sg.T('Position'), sg.Text("_", size=(10, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                border_width = 2, key='-TROLLEY_POSSITION-'), sg.Button(button_text = "Reset", key='-TROLLEY_TARGET_RESET-') ],
            [sg.Text('Target'), sg.Input(size=(10, 1), enable_events=True, key='-TROLLEY_TARGET-', \
                font=('Arial Bold', 10), justification='left'), sg.Button(button_text = "Set & Go", key='-TROLLEY_POS_SET-')],
            [sg.Button( button_color=sg.TRANSPARENT_BUTTON, image_filename = image_left, image_size=(55, 60), \
                image_subsample=2, border_width=0, key='-TROLLEY_LEFT-'),
                sg.Frame('',[[sg.Text('Velocity (RPM)')], [sg.Input(size=(15, 1), enable_events=True, key='-TROLLEY_VELOCITY-', \
                    font=('Arial Bold', 10), justification='left')]], border_width=0),
            sg.Button( button_color=sg.TRANSPARENT_BUTTON, image_filename = image_right, image_size=(55, 60), \
                image_subsample=2, border_width=0, key='-TROLLEY_RIGHT-')], 
            [sg.Button(button_text = 'Stop',  key='-TROLLEY_STOP-')]]
    
    la_gripper = [[sg.Button('', image_data=toggle_btn_off, key='ONOFF', \
                button_color=(sg.theme_background_color(), sg.theme_background_color()), border_width=0)], 
                [sg.Button('Exit')]]





    def main_loop():
        from bs1_faulhaber import FH_Motor 
        from bs1_FHv3 import FH_Motor_v3

    # Layout (COM / on / off)
        layout = [
            [sg.Frame('', la_gripper,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                    border_width=3, vertical_alignment='center', element_justification = "center")],
            [sg.Frame('', trolley,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                    border_width=3, vertical_alignment='center', element_justification = "center")]

        ]

        window = sg.Window('Unitest', layout, finalize = True)

        from bs1_config import port_scan

        print_inf(f'Scanning ports')
        devs = port_scan()



        down = graphic_off = True

        dev_trolley = None
        dev_gripper =  None

        for fh_dev in devs:
            if fh_dev.C_type == '--TROLLEY--':
                if fh_dev.dev_mDC:
                    dev_trolley = fh_dev.dev_mDC 
                elif fh_dev.dev_mDC:
                    dev_trolley = fh_dev.dev_mDC
                else: 
                    print_err(f'Unsupported TROLLEY device: {fh_dev}') 

            elif fh_dev.C_type == '--GRIPPERv3--':
                if fh_dev.dev_mDC:
                    dev_gripper = fh_dev.dev_mDC 
                elif fh_dev.dev_mDC:
                    dev_gripper = fh_dev.dev_mDC
                else: 
                    print_err(f'Unsupported GRIPPER device: {fh_dev}') 

                # dev_gripper = fh_dev.dev_mDC 

        if dev_trolley:
            dev_trolley.mDev_pressed = False

        if dev_gripper:
            dev_gripper.mDev_pressed = False
        


    #-----------------------------------------------------------   
        if dev_trolley:
            window['-TROLLEY_POSSITION-'].update(value = dev_trolley.mDev_pos)
            window['-TROLLEY_VELOCITY-'].update(value = dev_trolley.rpm)

        while True:

            #get event
            event, values = window.read(timeout=100)

            # print(event, values)

            #When the window is closed or the Exit button is pressed
            if event in (sg.WIN_CLOSED, 'Exit'):
                print_inf(f'Exiting')
                # sys.exit()
                break


            #When '-TOGGLE-GRAPHIC-' button is presseddisab
            elif event == 'ONOFF':
                graphic_off = not graphic_off
                window['ONOFF'].update(image_data=toggle_btn_off if graphic_off else toggle_btn_on)
                if dev_gripper: 
                    if graphic_off:
                        dev_gripper.gripper_on()
                        print_log(f'Switching on')
                    else:
                        dev_gripper.gripper_off()
                        print_log(f'Switching off')

                
                window['ONOFF'].update(disabled = True)
                
                if dev_gripper:
                    dev_gripper.mDev_pressed = True

            elif  event == '-TROLLEY_TARGET-':
                if  len(values['-TROLLEY_TARGET-']) > 0 and values['-TROLLEY_TARGET-'][-1] not in ('-0123456789'):
                    sg.popup("Only digits allowed")
                    window['-TROLLEY_TARGET-'].update(values['-TROLLEY_TARGET-'][:-1])
                    # window['-TROLLEY_TARGET-'].update(disabled = True)
            elif event == '-TROLLEY_POS_SET-':
                window['-TROLLEY_POSSITION-'].update(value = values['-TROLLEY_TARGET-'])


                if dev_trolley:
                    dev_trolley.go2pos(values['-TROLLEY_TARGET-'])
                    dev_trolley.mDev_pressed = True
                
                window['-TROLLEY_POS_SET-'].update(disabled = True)
                window['-TROLLEY_RIGHT-'].update(disabled = True)
                window['-TROLLEY_LEFT-'].update(disabled = True)

            elif event == '-TROLLEY_TARGET_RESET-':
                window['-TROLLEY_POSSITION-'].update(value = 0)
                window['-TROLLEY_TARGET-'].update(value = 0) 
                if dev_trolley:
                    dev_trolley.mDev_reset_pos()
                
            elif event == '-TROLLEY_RIGHT-':
                window['-TROLLEY_POS_SET-'].update(disabled = True)
                window['-TROLLEY_RIGHT-'].update(disabled = True)
                window['-TROLLEY_LEFT-'].update(disabled = True)
                if dev_trolley:
                    dev_trolley.mDev_forward()
                    dev_trolley.mDev_pressed = True

            elif event == '-TROLLEY_LEFT-':
                window['-TROLLEY_POS_SET-'].update(disabled = True)
                window['-TROLLEY_RIGHT-'].update(disabled = True)
                window['-TROLLEY_LEFT-'].update(disabled = True)
                if dev_trolley:
                    dev_trolley.mDev_backwrd()
                    dev_trolley.mDev_pressed = True
            
            elif event == '-TROLLEY_STOP-':
                if dev_trolley:
                    dev_trolley.mDev_stop()


                window['-TROLLEY_POS_SET-'].update(disabled = False)
                window['-TROLLEY_RIGHT-'].update(disabled = False)
                window['-TROLLEY_LEFT-'].update(disabled = False)
                

            if dev_gripper  and dev_gripper.mDev_pressed and not dev_gripper.wd.is_alive():
                print_log(f'Complete')
                window['ONOFF'].update(disabled = False)
                dev_gripper.mDev_pressed = False

            if dev_trolley and dev_trolley.mDev_pressed and not dev_trolley.wd.is_alive():
                print_log(f'Trolley Complete')
                window['-TROLLEY_POS_SET-'].update(disabled = False)
                window['-TROLLEY_RIGHT-'].update(disabled = False)
                window['-TROLLEY_LEFT-'].update(disabled = False)
                dev_trolley.mDev_pressed = False
            

            if  dev_trolley and dev_trolley.mDev_pressed and dev_trolley.wd.is_alive():
                new_pos = dev_trolley.mDev_get_cur_pos()
                # print_log(f'Updating possition: {new_pos}')
                window['-TROLLEY_POSSITION-'].update(value = new_pos)

        
        if dev_trolley:
            del  dev_trolley
        if dev_gripper:
            del dev_gripper    

        window.close()
        


if __name__ == "__main__":
    main_loop()