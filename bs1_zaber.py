#!/usr/bin/env python
# coding: utf-8
__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "1.0.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Tool"


from zaber_motion import Units
from zaber_motion.ascii import Connection
from zaber_motion import Library, DeviceDbSourceType, LogOutputMode, MotionLibException
import threading
import time
import logging
import PySimpleGUI as sg

from threading import Lock
from queue import Queue 
import random

from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, set_parm, get_parm, \
    assign_type_parm, assign_parm


default_unit = Units.LENGTH_MILLIMETRES
default_velocity_unit = Units.VELOCITY_MILLIMETRES_PER_SECOND


PRESITION_IND = 100
DEFAULT_RND_STEP = 0.1
DEFAULT_AMP = 25
DEFAULT_PERIOD = 25
DEFAULT_COUNT = 2
DEFAULT_MICROSTEP_RESOLUTION = 64
VIBRATION_MICROSTEP_RESOLUTION = 1

# print_log = lambda x: print(x) if __name__ == "__main__" else None
# print_log = lambda x: print(x) if __name__ == "__main__" else print(x)


# format = "%(asctime)s: %(filename)s--%(funcName)s/%(lineno)d -- %(thread)d [%(threadName)s] %(message)s" 
# logging.basicConfig(format=format, level=logging.DEBUG, datefmt="%H:%M:%S")
# print_log = logging.debug

REPEARTS2SUCCESS = 2

# # ZABER settings to avoid stalls 
MAX_ACCEL:int = 205                 # 2147483647 
ACCEL_PERCENT:int = 5
RAMP_TIME:int = 50
CURRENT_RUN:int = 17
DEFAULT_VELOCITY_PERCENTAGE:int = 100

'''
Zaber_Motor.device_port_list -> static nested dictionary 

device_port_list -	"COM1" : 	"device_list"[] 		(zaber_motion.ascii.Connection.detect_devices())
								"connection" 			(zaber_motion.ascii.Connection)

					"COM2" : 	"device_list"[] 		(zaber_motion.ascii.Connection.detect_devices())
								"connection" 			(zaber_motion.ascii.Connection)
					
					
					"COMn" : 	"device_list"[]			(zaber_motion.ascii.Connection.detect_devices())
								"connection" 			(zaber_motion.ascii.Connection)


            "device_list"[].axis_count
            "device_list"[].device_id
            "device_list"[].identity
            "device_list"[].name
            "device_list"[].serial_number

'''


class Zaber_Motor:
    device_port_list=dict()
    # portno = None                   # working port number
    # device_list = None              # recognized devices list
    # connection = None               # connection ref
    # init_done = False                  # initiation completed
    

    def __init__ (self, z_id, dev_port, parms, devName):

        if (not dev_port in Zaber_Motor.device_port_list) or (not Zaber_Motor.device_port_list[dev_port]["device_list"]):
            print_log(f'No initiated devices found')
            return
        
        # if not Zaber_Motor.init_done:
        #     print_log(f'No initiation done')
        #     return

        # self.device_list = None              # recognized devices list
        # self.connection = None               # connection ref
    
#####################################  parameter per device ################
        # ZABER settings to avoid stalls 
        self.MAX_ACCEL:int = 205                 # 2147483647 
        self.ACCEL_PERCENT:int = 5
        self.RAMP_TIME:int = 50
        self.CURRENT_RUN:int = 17
        self.DEFAULT_VELOCITY_PERCENTAGE:int = 100
        self.Z_MAXSPEED:int = 10
        self.rnd_step:float = 1
        self.__title:str = '' 
#####################################  
        self.portno = dev_port                   # working port number

        self.device_id= z_id                      #conf. device ID
        self.z_device = Zaber_Motor.device_port_list[dev_port]["device_list"][z_id]
        self.z_serialN = Zaber_Motor.device_port_list[dev_port]["device_list"][z_id].serial_number
        self.axis = self.z_device.get_axis(1)
        self.current_pos = self.GetPos()
        self.z_in_motion = False                            # is device in motion
        self.z_pressed = False                              # is motion button pressed
        self.wd = None                                      # watch dog identificator
        self.success_flag = True                            # end of op flag
        self.target_pos = self.GetPos()                     # Target move position (for motion control)
        self.velocity_in_percents = self.DEFAULT_VELOCITY_PERCENTAGE                     # default velocity
        self.max_velocity = self.axis.settings.get("maxspeed", default_velocity_unit)
        self.devName = devName
        self.dev_lock =  Lock()                 # device lock to avoid multiply access
        self.devNotificationQ = Queue()
        self.__stopFlag:bool = False
        self.rnd_step:int = None
        self.__rnd_gen = random.Random()
        self.__sin_amp:float = DEFAULT_AMP
        self.__sin_period:float = DEFAULT_PERIOD
        self.__sin_count:int = DEFAULT_COUNT



        try:
            self.__rnd_gen.seed()
            self.__title = devName

            # self.MAX_ACCEL  = parms[devName]['MAX_ACCEL'] if ((devName in parms.keys()) and ('MAX_ACCEL' in parms[devName].keys())) else parms['DEAFULT']['MAX_ACCEL']
            # self.ACCEL_PERCENT  = parms[devName]['ACCEL_PERCENT'] if ((devName in parms.keys()) and ('ACCEL_PERCENT' in parms[devName].keys())) else parms['DEAFULT']['ACCEL_PERCENT']
            # self.RAMP_TIME  = parms[devName]['RAMP_TIME'] if ((devName in parms.keys()) and ('RAMP_TIME' in parms[devName].keys())) else parms['DEAFULT']['RAMP_TIME']
            # self.CURRENT_RUN  = parms[devName]['CURRENT_RUN'] if ((devName in parms.keys()) and ('CURRENT_RUN' in parms[devName].keys())) else parms['DEAFULT']['CURRENT_RUN']
            # self.DEFAULT_VELOCITY_PERCENTAGE  = parms[devName]['DEFAULT_VELOCITY_PERCENTAGE'] if ((devName in parms.keys()) and ('DEFAULT_VELOCITY_PERCENTAGE' in parms[devName].keys())) else parms['DEAFULT']['DEFAULT_VELOCITY_PERCENTAGE']
            # self.Z_MAXSPEED = get_parm(devName, parms, 'Z_MAXSPEED')
            # self.__title = get_parm(devName, parms, 'TITLE')
            # self.velocity_in_percents = self.DEFAULT_VELOCITY_PERCENTAGE

            # _rnd_step = get_parm(devName, parms, 'RND_STEP')
            # if _rnd_step:
            #     self.rnd_step = int(round(_rnd_step * PRESITION_IND))
            # else:
            #     self.rnd_step = DEFAULT_RND_STEP

            # new_accel = self.MAX_ACCEL * self.ACCEL_PERCENT / 100
            # print_log(f'Current/Default motion.accelonly value = {self.axis.settings.get("motion.accelonly")}/{self.axis.settings.get_default_string("motion.accelonly")}')
            # print_log(f'Current/Default motion.accel.ramptime value = {self.axis.settings.get("motion.accel.ramptime")}/{self.axis.settings.get_default_string("motion.accel.ramptime")}')
            # print_log(f'Current/Default driver.current.run value = {self.z_device.settings.get("driver.current.run")}/{self.z_device.settings.get_default_string("driver.current.run")}')
            # # print_log(f'Current/Default zaber maxspeed value = {self.z_device.settings.get("maxspeed", default_velocity_unit)}/{self.z_device.settings.get_default_string("maxspeed", default_velocity_unit)}')
            # print_log(f'Current/Default zaber maxspeed value = {self.axis.settings.get("maxspeed", default_velocity_unit)}/{self.axis.settings.get("maxspeed", default_velocity_unit)}')
            
            # self.axis.settings.set("motion.accelonly", new_accel)
            # self.axis.settings.set("motion.accel.ramptime", self.RAMP_TIME)
            # self.z_device.settings.set("driver.current.run", self.CURRENT_RUN)
            # self.axis.settings.set("maxspeed", self.Z_MAXSPEED, unit = default_velocity_unit)
            # # self.z_device.settings.set("maxspeed", self.Z_MAXSPEED)

            # print_log(f'New motion.accelonly value = {self.axis.settings.get("motion.accelonly")}')
            # print_log(f'New motion.accel.ramptime value = {self.axis.settings.get("motion.accel.ramptime")}')
            # print_log(f'New driver.current.run value = {self.z_device.settings.get("driver.current.run")}')
            # # print_log(f'New zaber maxspeed value = {self.z_device.settings.get("maxspeed", default_velocity_unit)}')
            # self.max_velocity = self.axis.settings.get("maxspeed", default_velocity_unit)
            # print_log(f'New zaber maxspeed value = {self.max_velocity}')

            self.set_parms(parms=parms)

        except Exception as ex:
            exptTrace(ex)
            print_log(f'Setting failed on ZABER device {self.device_id}/{self.devName},\nException = {ex}')
         
                                    # device ref
        print_log(f"ZABER device added name= {self.z_device.name} device current position = {self.current_pos}")

            
    @staticmethod
    def init_devs(z_port) -> bool:
        # Zaber_Motor.portno = z_port
        # try:
        #     Library.set_device_db_source(DeviceDbSourceType.FILE, "./db/devices-public.sqlite")
        #     if Zaber_Motor.device_list == None:
        #         Zaber_Motor.connection = Connection.open_serial_port(z_port)  
        #         Zaber_Motor.device_list = Zaber_Motor.connection.detect_devices()     
        #         Zaber_Motor.connection.enable_alerts()
        #         print_log (f"Zaber initiaton succeded on port {z_port}, Found {len(Zaber_Motor.device_list)}\{Zaber_Motor.connection.renumber_devices()} devices")
        #         return True
        # except Exception as ex:
        #     print_log(f"Error initiatioin Zaber devices. Exception: {ex} of type: {type(ex)}")
        #     print_log(f"Trying online, will take a while...")
        #     return False
        device_list = []
        # portno = z_port
        try:
            # Library.set_log_output(LogOutputMode.FILE, "Motors_Control_Dashboard.log")
            Library.set_device_db_source(DeviceDbSourceType.WEB_SERVICE)
            if (not z_port in Zaber_Motor.device_port_list) or (not Zaber_Motor.device_port_list[z_port]["device_list"]):
            # if Zaber_Motor.device_list == None:
                # Zaber_Motor.connection = Connection.open_serial_port(z_port)  
                # Zaber_Motor.device_list = Zaber_Motor.connection.detect_devices()
                # Zaber_Motor.connection.enable_alerts()

                connection = Connection.open_serial_port(z_port)
                print_log(f'Created connection {connection} on port {z_port}')
                connection.enable_alerts()
                connection.renumber_devices()
                print_log(f'Discovering device list....')
                device_list = connection.detect_devices()
                print_log(f'Device list = {device_list}')


                Zaber_Motor.device_port_list[z_port]={"connection":connection, "device_list": device_list}
                print_log(f'Zaber_Motor.device_port_list[{z_port} = {Zaber_Motor.device_port_list[z_port]}')
                

                print_log (f'Zaber initiaton succeded on port {z_port}, Found {len(Zaber_Motor.device_port_list[z_port]["device_list"])}')
                # print_log(f'of {Zaber_Motor.device_port_list[z_port]["connection"].renumber_devices()} devices')
                return True
        except Exception as ex:
            print_log(f"Error initiatioin Zaber devices. Exception: {ex} of type: {type(ex)}, Port={z_port}")
            if z_port in Zaber_Motor.device_port_list:
                Zaber_Motor.device_port_list[z_port]["connection"] = None
            return False

  

    def readStoredPos(self):
        return self.current_pos


    def GetPos(self, unit = default_unit):
        try:
            self.current_pos = round(self.axis.get_position(unit ), 2)
        except Exception as ex:
            print_log(f"Error on Zaber device {self.device_id} ({self.devName}) S/N = {self.z_serialN}, port {self.portno}. Exception: {ex} of type: {type(ex)}")

        return self.current_pos

       
    @staticmethod
    def GetInfo(ind, z_port):

        if not z_port in Zaber_Motor.device_port_list:
            print_log(f'WARNING: No device found for port = {z_port}')
            

        try:
            Zaber_Motor.device_port_list[z_port]["connection"].detect_devices()             
                                                                # Have no idea why but with no this nonsense call access to devices fails
                                                                # ok, it needs to retrive device identification, 
                                                                # but the info is saved so this is needed be called again and again
                                                                # unbelievably stupid identification implementation
                                                                # even adding local db did not help

            # Zaber_Motor.device_list[ind].identity()             # calling a special function as recomended also does not work

            # axes = Zaber_Motor.device_list[ind].axis_count
            # device_id = Zaber_Motor.device_list[ind].device_id
            # identity = Zaber_Motor.device_list[ind].identity
            # name = Zaber_Motor.device_list[ind].name
            # serial_number = Zaber_Motor.device_list[ind].serial_number
            axes = Zaber_Motor.device_port_list[z_port]["device_list"][ind].axis_count
            device_id = Zaber_Motor.device_port_list[z_port]["device_list"][ind].device_id
            identity = Zaber_Motor.device_port_list[z_port]["device_list"][ind].identity
            name = Zaber_Motor.device_port_list[z_port]["device_list"][ind].name
            serial_number = Zaber_Motor.device_port_list[z_port]["device_list"][ind].serial_number
        
        except Exception as ex:
            print_log(f"Error on Zaber devices. Exception: {ex} of type: {type(ex)}")


        return axes, device_id, identity, name, serial_number

    # @staticmethod
    def reset_devs (self):
        try:
            if self.portno in Zaber_Motor.device_port_list and Zaber_Motor.device_port_list[self.portno]["connection"]:
                Zaber_Motor.device_port_list[self.portno]["connection"].home_all()
            else:
                print_log(f'Cant perform RESET on port {self.portno}')
        except Exception as ex:
            print_log(f"Error on Zaber devices. Exception: {ex} of type: {type(ex)}")

    def mDev_stop(self) -> bool:                            # for compatability 
        self.__stopFlag = True
        return self.stop_motion()

    def stop_motion(self) -> bool:
        try:
            if self.axis.is_busy():
                print_log(f'Stoping {self.devName}')
                self.axis.stop( wait_until_idle = False)
            else:
                print_log(f'Stop ignored since device {self.devName} is not in motion')

        except Exception as ex:
            print_log(f'Stop motion action on ZABER failed at {self.device_id} ({self.devName}) ZABER device. Exception: {ex} of type: {type(ex)}.')
            return False
        
        return True

    def repeatTry(self, position:float, velocity_in_percents = None, unit = default_unit ) -> bool:
        if velocity_in_percents == None:
            velocity_in_percents = self.velocity_in_percents
        try:    
            self.axis.move_absolute (float(position), unit, wait_until_idle=False, \
                                        velocity = self.max_velocity * int(velocity_in_percents)/100, \
                                        velocity_unit = default_velocity_unit)
            
            
        except: 
            print_err(f'Repeat move failed at exeption')
            return False
        else:
            return True
        
    def z_watch_dog_thread(self):
        try:
            print_log (f'>>> Watch dog started on {self.device_id} ({self.devName}) ZABER device, position = {self.current_pos}')
            self.z_in_motion = True
            tryRes = False
            self.__stopFlag = False
            __working_resolution = self.axis.settings.get("resolution")
            for _it in range(REPEARTS2SUCCESS):
                reductedVelocity = round(int(self.velocity_in_percents)/2**(_it+1))          # pow(2, _tr+2)
                reductedVelocity = reductedVelocity if reductedVelocity >= 1 else 1
                self.success_flag = True
                try:
                    self.axis.wait_until_idle()
                    time.sleep(0.05)
                except Exception as ex:
                    currentPos = self.GetPos()
                    print_log(f'Try # {_it}: WatchDog ZABER failed on {self.device_id} ({self.devName}) ZABER device (pos= {currentPos}). Exception: {ex} of type: {type(ex)}.')
                    self.success_flag = False  
                    tryRes = self.repeatTry(self.target_pos, reductedVelocity)
                    continue
                else:
                    if  not (__working_resolution  == DEFAULT_MICROSTEP_RESOLUTION):
                        print_log(f'Reset resolution to {DEFAULT_MICROSTEP_RESOLUTION} on {self.device_id} ({self.devName}) ZABER device')
                        self.axis.settings.set("resolution", DEFAULT_MICROSTEP_RESOLUTION)  
                        break

                    currentPos = self.GetPos()
                    print_log(f'Try # {_it}: {self.devName}: ZABER motion completed. Position = {currentPos}')
                    if abs(float(self.target_pos) - float(currentPos)) > 0.01:  
                        
                        print_err(f'Try # {_it}: ZABER motion operation failed on {self.device_id} ({self.devName}) ZABER device. Target position = {self.target_pos}, real position = {currentPos} .')
                        if self.__stopFlag:
                            print_log(f'Enforced stop at ZABER {self.devName}')
                            self.success_flag = True
                            break
                        else:
                            self.success_flag = False
                            
                        tryRes = self.repeatTry(self.target_pos, reductedVelocity)
                        continue
                    else:
                        self.success_flag = True
                        break                       # success

                
            
            if self.z_in_motion:                          # moved to manage GUI control activation 
                self.z_in_motion = False

            
            if self.dev_lock.locked():
                self.dev_lock.release()
            else:
                print_err(f'-WARNING unlocket mutual access mutex')
                
            self.GetPos()  
            
            print_log (f'>>> Watch dog completed on {self.device_id} ({self.devName}) ZABER device,  position = {self.current_pos}, ststus = {self.success_flag}')
            

        except Exception as ex:
            exptTrace(ex)
            self.devNotificationQ.put(False)
        else:
            self.devNotificationQ.put(self.success_flag)
        return

    def go2pos(self, new_position, velocity = None, stall = None)->bool:                          # for compatability
        return self.move_abs(position=new_position, velocity_in_percents = velocity)


    def rndMove(self, start:float=0, end:float=0)->bool:
        if start == end:
            print_log(f'No movement. Start = {start}, End = {end}')
            return True
        
        

        # random.seed()                                               # re-nitialize the random number generator
        _start_int = int(round(start * PRESITION_IND))
        _end_int = int(round(end * PRESITION_IND))
        # _dest:float = random.randrange(_start_int, _end_int, self.rnd_step) / PRESITION_IND

        _dest:float = self.__rnd_gen.randrange(start=_start_int, stop=_end_int, step=self.rnd_step)  / PRESITION_IND
        

        print_log(f'Random movement to pos = {_dest}. Start = {start}, End = {end}, Step = {self.rnd_step/PRESITION_IND}')
        return self.go2pos(_dest)

    def move_abs (self, position, velocity_in_percents = None, unit = default_unit ) -> bool:

        if not self.mutualControl():
            return False

        if velocity_in_percents == None:
            velocity_in_percents = self.velocity_in_percents

        self.success_flag = True 
        self.target_pos = position
        # if position == 0:
        #     if self.dev_lock.locked():
        #         self.dev_lock.release()
        #     return self.move_home()
        try:
            if velocity_in_percents == '': velocity_in_percents = '100' 
            elif int(velocity_in_percents) > 100: velocity_in_percents = '100' 
            
            print_log(f'Max velocity = {self.max_velocity}, selected velocity = {velocity_in_percents}% / {self.max_velocity * int(velocity_in_percents)/100}')
            self.axis.move_absolute (float(position), unit, wait_until_idle=False, \
                                     velocity = self.max_velocity * int(velocity_in_percents)/100, \
                                     velocity_unit = default_velocity_unit)
        except Exception as ex:
            print_log(f'move_absolute() ZABER failed on{self.device_id}({self.devName}) ZABER device. Exception: {ex} of type: {type(ex)}.')
            self.success_flag = False 
            if self.dev_lock.locked():
                self.dev_lock.release()
            return False

        self.wd = threading.Thread(target=self.z_watch_dog_thread)
        self.wd.start()
        return True
        
    
    def move_rel (self, position, unit = default_unit) -> bool:
        if not self.mutualControl():
            return False

        self.target_pos = position
        self.success_flag = True 
        try:
            if self.velocity_in_percents == '': self.velocity_in_percents = '100' 
            elif int(self.velocity_in_percents) > 100: self.velocity_in_percents = '100' 

            self.axis.move_relative (float(position), unit,  wait_until_idle=False, \
                                     velocity = self.max_velocity * int(self.velocity_in_percents)/100, \
                                     velocity_unit = default_velocity_unit)
        except Exception as ex:
            print_log(f'move_relative() ZABER failed on {self.device_id} ({self.devName}) ZABER device. Exception: {ex} of type: {type(ex)}.')
            self.success_flag = False
            if self.dev_lock.locked():
                self.dev_lock.release()
            return False

        self.wd = threading.Thread(target=self.z_watch_dog_thread)
        self.wd.start()
        return True

    def move_home(self)-> bool:
        if not self.mutualControl():
            return False

        self.target_pos = 0
        self.success_flag = True 
        try:
            self.axis.home(wait_until_idle = False)
        except Exception as ex:
            print_log(f'home() ZABER failed on {self.device_id} ({self.devName}) ZABER device. Exception: {ex} of type: {type(ex)}.')
            self.success_flag = False
            if self.dev_lock.locked():
                self.dev_lock.release()
            return False

        self.wd = threading.Thread(target=self.z_watch_dog_thread)
        self.wd.start()
        return True

    def mutualControl(self):
        if self.dev_lock.locked():
            print_err(f'ERROR- The device {self.devName} on port {self.portno} / id = {self.device_id}  is active. Cant allow multiply activations')
            return False
        else:
            self.dev_lock.acquire()
            return True


    def __del__ (self):
        port_no = self.portno
        Zaber_Motor.device_port_list[self.portno]["device_list"].remove(self.z_device)
        # Zaber.connection.close() method does not work properly so stop() is skiped
        # if len(Zaber_Motor.device_port_list[port_no]["device_list"]) == 0:
        #     Zaber_Motor.stop(port_no)
        


    @staticmethod
    def stop(port_no):
        print_inf (f'Stoping ZABER on port {port_no}')
        try:
            if port_no in Zaber_Motor.device_port_list and Zaber_Motor.device_port_list[port_no]["connection"]:
                print_inf(f'Delete ZABER connection {Zaber_Motor.device_port_list[port_no]["connection"]} on port {port_no}')
                Zaber_Motor.device_port_list[port_no]["connection"].close()
                print_inf(" ZABER connection closed")
        except Exception as ex:
            print_log(f'Stopping ZABER failed on {port_no} ZABER device port. Exception: {ex} of type: {type(ex)}.')

    @staticmethod
    def park_all():

        try:
            if len(Zaber_Motor.device_port_list) == 0:
                print(f'No active Zaber devices found')
                sg.popup_auto_close('No active Zaber devices found', no_titlebar = True, auto_close_duration = 2, button_color= 'tomato', 
                                    background_color = 'blue')
                return
            

            for con in Zaber_Motor.device_port_list:
                print_log(f'Parking devices on port {con}')
                # for dev in con["device_list"]:
                for dev in Zaber_Motor.device_port_list[con]["device_list"]:
                    print_log(f'Trying park {con}:{dev}')
                    ax = dev.get_axis(1)
                    ax.park()
                    # dev.axis.park()
                    print_log(f'Zaber {dev.serial_number} / Z{dev.identity} is parked')            
        except Exception as ex:
            print_err(f'Zaber park proc exception')
            exptTrace(ex)
        

    @staticmethod
    def unpark_all():
        try:
            if len(Zaber_Motor.device_port_list) == 0:
                print_log(f'No active Zaber devices found')
                sg.popup_auto_close('No active Zaber devices found', no_titlebar = True, auto_close_duration = 2, button_color= 'tomato', 
                                    background_color = 'blue')
                return
            
            devs_all = 0
            devs_unp = 0 
            for con in Zaber_Motor.device_port_list:
                print_log(f'Unparking devices on port {con}')
                # for dev in con["device_list"]:
                for dev in Zaber_Motor.device_port_list[con]["device_list"]:
                    print_log(f'Trying unpark {con}:{dev} / Z{dev.identity}')
                    devs_all += 1
                    ax = dev.get_axis(1)

                    # if dev.axis.is_parked():
                    if ax.is_parked():
                        devs_unp += 1
                        # dev.axis.unpark()
                        ax.unpark()
                        print_log(f'Zaber {dev.serial_number} / Z{dev.identity} is unparked')
            sg.popup_auto_close(f'{devs_unp} devices unparkef of {devs_all} devices', no_titlebar = True, auto_close_duration = 2, 
                                button_color= 'green', background_color = 'blue')
        except Exception as ex:
            print_err(f'Zaber unpark proc exception')
            exptTrace(ex)
        




    def is_parked(self) -> bool:
        try:
            ax = self.z_device.get_axis(1)
            return ax.is_parked()
        except Exception as ex:
            print_err(f'Zaber is_parked proc exception')
            exptTrace(ex)
        
    
    def mDev_vibrate(self)->bool: 
        if not self.mutualControl():
            return False

        try:
            if self.__sin_amp <= 0 or self.__sin_period < 0.2 or self.__sin_count < 1:
                print_log(f'Vibration parameters are not set properly, sin_amp = {self.__sin_amp}, sin_period = {self.__sin_period}, sin_count = {self.__sin_count}')
                return False

            print_log(f'Starting vibration on {self.devName} with amp = {self.__sin_amp}, period = {self.__sin_period}, count = {self.__sin_count}')
            self.target_pos = self.GetPos()
            self.axis.settings.set("resolution", VIBRATION_MICROSTEP_RESOLUTION) 
            # self.axis.generic_command_no_response(f'move sin {self.__sin_amp} {self.__sin_period} {self.__sin_count} ')
            # self.axis.generic_command(f'move sin {self.__sin_amp} {self.__sin_period} {self.__sin_count} ')
            self.axis.move_sin(self.__sin_amp, Units.NATIVE, self.__sin_period, Units.NATIVE, count = self.__sin_count, wait_until_idle = False)
            print_log(f'Running vibration sin_amp = {self.__sin_amp}, sin_period = {self.__sin_period}, sin_count = {self.__sin_count}, resolution= {VIBRATION_MICROSTEP_RESOLUTION}, target = {self.target_pos}, pos = {self.current_pos} ')
            self.wd = threading.Thread(target=self.z_watch_dog_thread)
            self.wd.start()
        except Exception as ex:
            print_err(f'Zaber vibrate proc exception')
            exptTrace(ex)
            return False
        
        return True
    

    def getTitle(self)->str:
        return self.__title

    def set_parms(self, parms):
        try:
            self.MAX_ACCEL  = assign_parm(self.devName, parms, 'MAX_ACCEL', MAX_ACCEL)
            self.ACCEL_PERCENT  = assign_parm(self.devName, parms, 'ACCEL_PERCENT', ACCEL_PERCENT)
            self.RAMP_TIME  = assign_parm(self.devName, parms, 'RAMP_TIME', RAMP_TIME)
            self.CURRENT_RUN  = assign_parm(self.devName, parms, 'CURRENT_RUN', CURRENT_RUN)
            self.DEFAULT_VELOCITY_PERCENTAGE  = assign_parm(self.devName, parms, 'DEFAULT_VELOCITY_PERCENTAGE', DEFAULT_VELOCITY_PERCENTAGE)
            self.Z_MAXSPEED = get_parm(self.devName, parms, 'Z_MAXSPEED')
            self.__title = assign_parm(self.devName, parms, 'TITLE', '')
            print_log(f'Zaber {self.devName} TITLE = {self.__title}')
            # self.__title = get_parm(self.devName, parms, 'TITLE')
            self.velocity_in_percents = self.DEFAULT_VELOCITY_PERCENTAGE
            self.__sin_amp = float(assign_type_parm(self.devName, parms, 'SIN_AMP', float, DEFAULT_AMP))
            self.__sin_period = float(assign_type_parm(self.devName, parms, 'SIN_PERIOD', float, DEFAULT_PERIOD))
            self.__sin_count = int(assign_type_parm(self.devName, parms, 'SIN_COUNT ', int, DEFAULT_COUNT))

            self.axis.settings.set("resolution", DEFAULT_MICROSTEP_RESOLUTION)
            
            _rnd_step = get_parm(self.devName, parms, 'RND_STEP')
            if _rnd_step:
                self.rnd_step = int(round(_rnd_step * PRESITION_IND))
            else:
                self.rnd_step = DEFAULT_RND_STEP

            new_accel = self.MAX_ACCEL * self.ACCEL_PERCENT / 100

            self.axis.settings.set("motion.accelonly", new_accel)
            self.axis.settings.set("motion.accel.ramptime", self.RAMP_TIME)
            self.z_device.settings.set("driver.current.run", self.CURRENT_RUN)
            self.axis.settings.set("maxspeed", self.Z_MAXSPEED, unit = default_velocity_unit)
            self.max_velocity = self.axis.settings.get("maxspeed", default_velocity_unit)

            print_log(f'Current/Default motion.accelonly value = {self.axis.settings.get("motion.accelonly")}/{self.axis.settings.get_default_string("motion.accelonly")}')
            print_log(f'Current/Default motion.accel.ramptime value = {self.axis.settings.get("motion.accel.ramptime")}/{self.axis.settings.get_default_string("motion.accel.ramptime")}')
            print_log(f'Current/Default driver.current.run value = {self.z_device.settings.get("driver.current.run")}/{self.z_device.settings.get_default_string("driver.current.run")}')
            # print_log(f'Current/Default zaber maxspeed value = {self.z_device.settings.get("maxspeed", default_velocity_unit)}/{self.z_device.settings.get_default_string("maxspeed", default_velocity_unit)}')
            print_log(f'Current/Default zaber maxspeed value = {self.axis.settings.get("maxspeed", default_velocity_unit)}/{self.axis.settings.get("maxspeed", default_velocity_unit)}')
            
            
            # self.z_device.settings.set("maxspeed", self.Z_MAXSPEED)

            print_log(f'New motion.accelonly value = {self.axis.settings.get("motion.accelonly")}')
            print_log(f'New motion.accel.ramptime value = {self.axis.settings.get("motion.accel.ramptime")}')
            print_log(f'New driver.current.run value = {self.z_device.settings.get("driver.current.run")}')
            # print_log(f'New zaber maxspeed value = {self.z_device.settings.get("maxspeed", default_velocity_unit)}')
            print_log(f'New zaber maxspeed value = {self.max_velocity}')

            print_inf (f'Loading parms for device = {self.devName}')
       
        except Exception as ex:
            print_err(f'Error setting zaber parms')
            exptTrace(ex)