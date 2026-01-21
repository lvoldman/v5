__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["Leonid Voldman","Sasha Schechtman"]
__license__ = "SLA"
__version__ = "2.0.1"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Prototype"

import struct
import time
import os
import socket



from edcon.edrive.com_modbus import ComModbus
from edcon.edrive.motion_handler import MotionHandler
from edcon.utils.logging import Logging

import time
import sys
import threading
from collections import namedtuple
from dataclasses import dataclass
from queue import Queue 
import numpy as np




from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, set_parm, get_parm, unsigned_16 
from bs1_base_motor import BaseMotor

''' 
 Data Types (IEC 61158-5-10)

Data type	Lower bound	Upper bound	Memory space
BOOL           TRUE (1)     FALSE (0)   8 bit
BYTE	        0	        255	        8 bit
WORD	        0	        65535	    16 bit
DWORD	        0	        4294967295	32 bit
LWORD	        0	        2^64-1	    64 bit
SINT	        -128	    127	        8 bit
USINT	        0	        255	        8 bit
INT	           -32768	    32767	    16 bit
UINT	        0	        65535	    16 bit
DINT	    -2147483648	    2147483647	32 bit
UDINT	        0	        4294967295	32 bit
LINT	        -2^63	    2^63-1	    64 bit
ULINT	        0	        2^64-1	    64 bit

Data type	Lower limit	                Upper limit         Smallest absolute value	            Storage space
REAL	    -3.402823e+38	            3.402823e+38            1.0e-44	                            32-bit
LREAL	-1.7976931348623158e+308	1.7976931348623158e+308	4.94065645841247e-324	                64-bit

FESTO TYPES:
Data type PN/EP/EC 	Data type plug-in 		Variable and sign 		Value range
BOOL 			        BOOL			 8-bit without sign		    0 … 255
USINT 			        UINT8			 8-bit without sign 		0 … 255
SINT 			        SINT8 			8-bit with sign 		    -128 … 127
UINT                    UINT16          16-bit without sign         0 … 65535
INT                     SINT16          16-bit with sign            -32768 … 32767
UDINT                   UINT32          32-bit without sign         0 … (232 -1)
DINT                    SINT32          32-bit with sign            -231 … (231 -1)
ULINT                   UINT64          64-bit without sign         0 … (264 -1)
LINT                    SINT64          64-bit with sign            -263 … (263 -1)
REAL                    FLOAT32         32-bit with 
                                        floating decimal                1.17 * 10-38 … 3.4 * 1038
LREAL                   FLOAT64         64-bit with floating decimal    2.2 * 10-308 … 1.8 * 10308
STRING(X)               CHAR            X * 8-bit without sign X *      0 … 255


'''

DATA_TYPES= {
"BYTE": {"size": 1, "format": "b"},
"WORD": {"size": 2, "format": "H"}, 
"DWORD": {"size": 4, "format": "I"}, 
"LWORD": {"size": 8, "format": "Q"}, 
"SINT": {"size": 1, "format": "b"}, 
"USINT": {"size": 1, "format": "B"}, 
"INT": {"size": 2, "format": "h"}, 
"UINT": {"size": 2, "format": "H"}, 
"DINT": {"size": 4, "format": "i"}, 
"UDINT": {"size": 4, "format": "I"}, 
"LINT": {"size": 8, "format": "q"}, 
"ULINT": {"size": 8, "format": "Q"}, 
"REAL": {"size": 4, "format": "f"}, 
"LREAL": {"size": 8, "format": "d"}, 
"STR": {"size": 128, "format": "s"}, 
"BOOL": {"size": 1, "format": "b"}, 
    
}


'''
Special address blocks
Address block	Address range	Number of addresses	Scope	Description
0.0.0.0/8	    0.0.0.0 - 0.255.255.255	16777216	Software	Current (local, "this") network[1]
10.0.0.0/8	    10.0.0.0 - 10.255.255.255	16777216	Private network	Used for local communications within a private network[3]
100.64.0.0/10	100.64.0.0 - 100.127.255.255	4194304	Private network	Shared address space[4] for communications between a service provider and its subscribers when using a carrier-grade NAT
127.0.0.0/8	    127.0.0.0 - 127.255.255.255	16777216	Host	Used for loopback addresses to the local host[1]
169.254.0.0/16	169.254.0.0 - 169.254.255.255	65536	Subnet	Used for link-local addresses[5] between two hosts on a single link when no IP address is otherwise specified, such as would have normally been retrieved from a DHCP server
172.16.0.0/12	172.16.0.0 - 172.31.255.255	1048576	Private network	Used for local communications within a private network[3]
192.0.0.0/24	192.0.0.0 - 192.0.0.255	256	Private network	IETF Protocol Assignments, DS-Lite (/29)[1]
192.0.2.0/24	192.0.2.0 - 192.0.2.255	256	Documentation	Assigned as TEST-NET-1, documentation and examples[6]
192.88.99.0/24	192.88.99.0 - 192.88.99.255	256	Internet	Reserved.[7] Formerly used for IPv6 to IPv4 relay[8] (included IPv6 address block 2002::/16).
192.168.0.0/16	192.168.0.0 - 192.168.255.255	65536	Private network	Used for local communications within a private network[3]
198.18.0.0/15	198.18.0.0 - 198.19.255.255	131072	Private network	Used for benchmark testing of inter-network communications between two separate subnets[9]
198.51.100.0/24	198.51.100.0 - 198.51.100.255	256	Documentation	Assigned as TEST-NET-2, documentation and examples[6]
203.0.113.0/24	203.0.113.0 - 203.0.113.255	256	Documentation	Assigned as TEST-NET-3, documentation and examples[6]
224.0.0.0/4	    224.0.0.0 - 239.255.255.255	268435456	Internet	In use for multicast[10] (former Class D network)
233.252.0.0/24	233.252.0.0 - 233.252.0.255	256	Documentation	Assigned as MCAST-TEST-NET, documentation and examples (This is part of the above multicast space.)[10][11]
240.0.0.0/4	240.0.0.0 - 255.255.255.254	268435455	Internet	Reserved for future use[12] (former Class E network)
255.255.255.255/32	255.255.255.255	1	Subnet	Reserved for the "limited broadcast" destination address[1]


'''

IP_RESERVED = [
     ["0.0.0.0", "0.255.255.255"],
     ["100.64.0.0", "100.127.255.255"],
     ["127.0.0.0", "127.255.255.255"],
     ["192.0.2.0", "192.0.2.255"],
     ["198.18.0.0", "198.19.255.255"],
     ["224.0.0.0", "239.255.255.255"],
     ["255.255.255.255", "255.255.255.255"]
]




_dev_cmdFields = ["cmd",  "res"]
_dev_cmd = namedtuple("_dev_cmd",  _dev_cmdFields, defaults=[None,] * len(_dev_cmdFields))
_dev_cmd.__annotations__={'cmd':dict, 'res':list}         # specify type of elements

_dev_lst_Fields = ["IP",  "SN"]
_dev_lst = namedtuple("_dev_lst",  _dev_lst_Fields, defaults=[None,] * len(_dev_cmdFields))
_dev_lst.__annotations__={'IP':str, 'SN':int}         # specify type of elements


M2MM = 1000

RESOLUTION_POSITION = {'cmd':[11724, 0], 'type':'SINT', 'txt': 'Resolution position'}
RESOLUTION_VELOCITY = {'cmd':[11725, 0], 'type':'SINT', 'txt': 'Resolution velocity'}
RESOLUTION_ACCELERATION = {'cmd':[11726, 0], 'type':'SINT', 'txt': 'Resolution acceleration'}
ACTUAL_CURRENT = {'cmd':[11190, 0], 'type':'REAL', 'txt': 'Actual active current value'}
POSITION = {'cmd':[2939, 0], 'type':'LINT', 'txt': 'Absolute position in user units'}
SERIAL_NUMBER = {'cmd':[2221, 0, 19], 'type':'STR', 'txt': 'Serial number'}
ACTUAL_POSITION = {'cmd':[11067, 0], 'type':'LINT', 'txt':"Actual position value encoder channel 1"}
ACTUAL_NOMINAL_CURRENT = {'cmd':[11691, 0], 'type':'REAL', 'txt': 'Actual nominal current'}
ENCODER_RESOLUTION = {'cmd':[2837, 0], 'type':'UINT', 'txt': 'Encoder resolution'}
REFERENCING_STATUS = {'cmd':[11202, 0], 'type':'UDINT', 'txt': 'Referencing status'}     # 100: Drive NOT referenced, 
                                                                                        # 103: Save homing data, 
                                                                                        # 200: Drive referenced
ACCELERATION = {'cmd':[12325, 0], 'type':'REAL', 'txt': 'Acceleration'}
DECELERATION = {'cmd':[12326, 0], 'type':'REAL', 'txt': 'Deceleration'}
ACTUAL_VELOCITY = {'cmd':[11311, 0], 'type':'REAL', 'txt': 'Actual velocity value encoder channel 1'}
ACTUAL_MAX_VELOCITY = {'cmd':[11695, 0], 'type':'REAL', 'txt': 'Actual maximum velocity'}

MOTION_MANAGER_STATUS = {'cmd':[11071, 0], 'type':'UDINT', 'txt': 'Motion Manager status'} 
ACTIVE_MOTION_TASK = {'cmd':[11072, 0], 'type':'UDINT', 'txt': 'Active motion task'} 
ACTIVE_MOTION_TASK_STATUS = {'cmd':[11073, 0], 'type':'UDINT', 'txt': 'Active motion task status'} 

PRODUCT_NAME = {'cmd':[1, 7], 'type':'STR', 'txt': 'Product Name'} 


'''
format:
< - little-endian
> - big-endian
c	char                        1
h	short	            integer	2
H	unsigned short	    integer	2
i	int	                integer	4
I	unsigned int	    integer	4
l	long	            integer	4
L	unsigned long	    integer	4
q	long long	        integer	8
Q   unsigned long long	integer	8

'''

class Festo_Motor (BaseMotor): 

    devices:list[_dev_lst] = list()
    @staticmethod
    def enum_devices()->list[_dev_lst]:
        try:
            os.system('arp -a > scan.tmp')
            scan = np.loadtxt('.\scan.tmp', dtype='str', comments= ['Interface', 'Internet'] , usecols=0,  delimiter=None)
            os.system('del scan.tmp')
            empty_nodes = np.where(np.char.find(scan, 'incomplete', ) > 0)[0]
            scan = np.delete(scan, empty_nodes)
            hostname = socket.gethostname()
            print_log(f'hostname = {hostname}')
            ips = socket.gethostbyname_ex(hostname)[2]
            print_log(f'ips = {ips}')
            neighbour_ip = list()
            for _arp_ip in scan:
                for _local_ip in ips:
                    _arp_ip_spl = _arp_ip.split('.')
                    _local_ip_spl = _local_ip.split('.')
                    if (_arp_ip_spl[0] != '255') and (_arp_ip_spl[3] != '255') and \
                        (_arp_ip_spl[0] == _local_ip_spl[0]) and (_arp_ip_spl[1] == _local_ip_spl[1]) and (_arp_ip_spl[2] == _local_ip_spl[2]):
                        neighbour_ip.append(_arp_ip)

            print_log (f'neighbour_ip = {neighbour_ip}, found {len(neighbour_ip)} hosts, type = {type(neighbour_ip)}')


            for __count, __ip in enumerate(neighbour_ip):
                print_log(f'Looking FESTO at IP={__ip} # {__count}')
                __sn:str = Festo_Motor.find_dev(__ip)
                if  __sn is not None: 
                    __dev = _dev_lst(IP = __ip, SN = __sn)
                    # print_log(f'Adding FESTO ip = {__ip}, sn = {__sn} of type {type(__sn)} // dev = {__dev} // {__dev.IP}, {__dev.SN} of type {type(__dev.SN)}' )
                    Festo_Motor.devices.append(__dev)

            print_log(f'Found devices: {Festo_Motor.devices}')

        except Exception as ex:
            print_log(f'Error listing FESTO devs in neighbour nodes. Exception = {ex}')
            exptTrace(ex)

        return Festo_Motor.devices
         

    @staticmethod
    def find_dev(IP:str)->str:
        mot = None
        _sn = None
        com = None
        try:
            _sn = None
            com = ComModbus(IP, timeout_ms=500)
            if not com.connected():
                print_err(f'Festo Combus connecting fail on IP {IP}')
                return None

            _res_lst = _CMD(com, [SERIAL_NUMBER])
            _el = _res_lst[0]


            _sn = _el.res

            mot = MotionHandler(com)
            if mot is None:
                print_err(f'Can not connect motion control. com = {com}, mot = {mot}')
                del com
                return None
                
            mot.acknowledge_faults()
            _ps = mot.enable_powerstage()

            if not _ps:
                print_err(f'Can not overtake motion control. com = {com}, mot = {mot}')
                return None
            
            
        
        except Exception as ex:
            print_log(f'Error operating FESTO devs at  ip:{IP}. Exception = {ex}')
            _sn = None
            exptTrace(ex)
         
        if mot is not None:
            del mot

        if com is not None:
            del com
            
        return _sn


    def __init__ (self, sn, port, d_name, parms):               # port -> IP
        super().__init__(port, d_name, parms)
        self.DEFAULT_VELOCITY_PERCENTAGE:int = 100
        self.DEFAULT_CURRENT_LIMIT:int = 250                # in mA
        self.HOMING_VELOCITY = -25

        self.__lock = threading.Lock()                               # device access mutex 
        self.__mot:MotionHandler = None
        self.__com:ComModbus = None
        self._mDev_SN = str(sn)
        self._mDev_port = port
        self.devName = d_name
        self._mDev_pos:int = 0
        self.el_current_on_the_fly:int = 0                  # On-the-fly current  -- for compatability only      
        self.__stopFlag = False
        self.__max_velocity =  None
        self.resolution_position = None
        self.velocity = self.DEFAULT_VELOCITY_PERCENTAGE                     # default velocity
        self.possition_control_mode = False                 # TRUE - control possition, FALSE - don't
        self.homing_mode_active = False
        self.el_current_limit:int = self.DEFAULT_CURRENT_LIMIT


        self.target_position = 0            
        # self.__target_velocity = None

        self.devNotificationQ:Queue = Queue()
        self._title = None

        try:
            print_log(f'Connecting Festo to Modbus server/slave at IP = {self._mDev_port}')
            self.__com = ComModbus(self._mDev_port, timeout_ms=500)
            if not self.__com.connected():
                raise Exception(f'Festo ComModbus couldnot connect')
            
            d_info = self.__com.read_device_info()
            print_log(f'{self.devName} connected to Modbus serv at {self._mDev_port}. INFO:\n{d_info}')
            _read_parms = self._op_CMD([ACTUAL_MAX_VELOCITY, RESOLUTION_POSITION, RESOLUTION_VELOCITY])
            self.__max_velocity =  _read_parms[0].res
            self.resolution_position = _read_parms[1].res
            self.resolution_velocity = _read_parms[2].res
            self.current_pos = self.GetPos()

            self.__mot = MotionHandler(self.__com)

            self.__mot.acknowledge_faults()

            if not self.__mot.plc_control_granted:
               raise Exception(f'No PLC control granted. com = {com}, mot = {mot}')    
                        
            _ps = self.__mot.enable_powerstage()

            if not _ps:
               raise Exception(f'Can not enable motion control. com = {com}, mot = {mot}')
                
                        
            self.set_parms(parms=parms)


        except Exception as ex:
            if self.mot is not None:
                del self.mot
            if self.com is not None:
                del self.com
            print_log(f'ModBus initialization failed. Exception = {ex}')
            exptTrace(ex)
            raise ex
        else:
            self._mDev_status = True
            print_log(f'ModBus successfuly connected with the server at port={self._mDev_port}')
            self.mot.acknowledge_faults()


    def __del__(self):
        if self.__mot is not None:
                try:
                    self.__mot.disable_powerstage()
                except Exception as ex:
                    exptTrace(ex)

                del self.__mot
        if self.com is not None:
            del self.com
        return super().__del__()
    
    def  mDev_stall(self)->bool:
        try:
            self.__mot.configure_brake(True)
        except Exception as ex:
            exptTrace(ex)
            return False
        return True

    
    def readStoredPos(self):
        return self.current_pos


    def GetPos(self):
        try:
            self.current_pos = round(self.__mot.current_position() /(10**self.resolution_position) / M2MM, 2)
        except Exception as ex:
            exptTrace(ex)
            print_log(f"Error on Zaber device {self.device_id} ({self.devName}) S/N = {self.z_serialN}, port {self.portno}. Exception: {ex} of type: {type(ex)}")

        return self.current_pos

    def _op_CMD(self, arr:list)->list:
        retValues = list()
        _com = self.com

        if len(arr) == 0:
            print_err(f'Empty CMD array = {arr}')
            return retValues
        
        if self.__lock.locked():
                print_err(f'WARNING: CMD semaphore is locked!')

        self.__lock.acquire()    

        print_log(f'CMD array = {arr}')

        retValues = _CMD(_com, [arr])

        self.__lock.release()
        return retValues

    def init_dev(self, dev_type) -> bool:
        self._mDev_type = dev_type
        if not self._mDev_status:                      # the device is not active
            return False
        print_log(f'Initiating ({ self._devName})  FESTO device of {self._mDev_type} type on port {self._mDev_port}')
        try:

            pass
              
            
        except  Exception as ex:
            e_type, e_filename, e_line_number, e_message = exptTrace(ex)
            print_err(f'FESTO ({self._devName}) failed to initialize on port {self._mDev_port}. Exception: {ex} of type: {type(ex)}.')
            return False


        return True
    
    def _success_run(self)->bool:
        if self.__mot.fault_present():
                print_err(f'ERROR (FAULT): {self.__mot.fault_string()}')
                self._reset()
                return False
        return True


    def _reset(self):
        try:
            if self.__mot.fault_present():
                print_err(f' Previuse operation caused FAULT: {self.__mot.fault_string()}')
                self.__mot.acknowledge_faults()
                self.__mot.enable_powerstage()
        except  Exception as ex:
            exptTrace(ex)
            print_err(f'FESTO ({self._devName}) failed to reset state on port {self._mDev_port}. Exception: {ex} of type: {type(ex)}.')


    def mDev_stop(self) -> bool:                            # for compatability 
        self.__stopFlag = True
        return self.stop_motion()

    def stop_motion(self) -> bool:
        try:
            self._reset()
            _res = self._op_CMD([ACTIVE_MOTION_TASK_STATUS])
            Active_motion_task_status = _res[0].res
            if Active_motion_task_status != 3:
                self.__mot.stop_motion_task()
                # __pos = self.__mot.current_position() 
                # self.__mot.position_task(__pos, 5, absolute=True)
            else:
                print_log(f'Stop ignored since FESTO device {self.devName} is not in motion')

        except Exception as ex:
            exptTrace(ex)
            print_log(f'Stop motion action on failed at {self.device_id} ({self.devName}) FESTO STEPPER device. Exception: {ex} of type: {type(ex)}.')
            return False
        
        return True

    def gripper_on(self)-> bool:
        print_err(f'No gripper operation is currently supported for FESTO device {self.devName}')
        return False

    def gripper_off(self)-> bool:
        print_err(f'No gripper operation is currently supported for FESTO device {self.devName}')
        return False

    
    def mDev_watch_dog_thread(self):
        print_log (f'>>> Watch dog started on {self.device_id} ({self.devName}) FESTO device, position = {self.current_pos}')
        self.__stopFlag = False
        self.success_flag = True
        __cur = self._op_CMD([ACTUAL_CURRENT])[0].res * M2MM
        try:
            while not self.__stopFlag and not (_fault_status := self.__mot.fault_present()):
                if (__cur := self._op_CMD([ACTUAL_CURRENT])[0].res * M2MM)*1000 > int(self.el_current_limit):       # in mA
                     print_log(f'FESTO {self.devName} current exeeded alloved limit')
                     break
                if self.possition_control_mode and self.__mot.target_position_reached():
                     print_log(f'FESTO {self.devName} reached target')
                     break
                time.sleep(0.05)
            
            if _fault_status:
                self.success_flag = False

            print_log(f'STOPING FESTO dev {self.devName}: position = {self.GetPos()}, current = {__cur}, fault = {_fault_status}')


        except Exception as ex:

            self.success_flag = False  
        else:
            self.success_flag = True
            
        
        self.__stopFlag = False
        
        if self.homing_mode_active:
            self.homing_mode_active = False
            self.__mot.referencing_task()
        
        if not self._success_run():
            self.success_flag = False

        if self.dev_lock.locked():
            self.dev_lock.release()
        else:
            print_err(f'-WARNING unlocked mutual access mutex')
            
        self.possition_control_mode = False
          
        print_log (f'>>> Watch dog completed on {self.device_id} ({self.devName}) FESTO device,  position = {self.current_pos}, status = {self.success_flag}, actual current = {__cur}')
        self.devNotificationQ.put(self.success_flag)
        return

    def rndMove(self, start:float=0, end:float=0)->bool:
        import random
        if start == end:
            print_log(f'No movement. Start = {start}, End = {end}')
            return True
        _new_pos = random.randint(int(start*M2MM), int(end*M2MM))
        return self.go2pos(_new_pos)


    # velocity in %
    # position in mm
    def go2pos(self, new_position, velocity = None, stall=None)->bool:
        if not self.mutualControl():
            return False
       
        if velocity == None:
            velocity = self.velocity
        else:
            if velocity == '': velocity = 100
            elif int(velocity) > 100: velocity = 100

        self.success_flag = True 
        self.target_position = new_position
        self.possition_control_mode = True

        try:
            self._reset()
            if not self.__mot.ready_for_motion():
                raise Exception (f'ERROR: FESTO {self.devName} is not ready for motion')

            _set_velocity = self.__max_velocity * (velocity/100) * M2MM
            print_log(f'Moving device {self.devName} to {new_position  / M2MM / (10**self.resolution_position)} ({new_position}mm) velocity = {_set_velocity} ({_set_velocity*(10**self.resolution_velocity)*M2MM} mm/s)')
            self.__mot.position_task(new_position  / M2MM / (10**self.resolution_position), _set_velocity, nonblocking=True)

            if not self._success_run():
                self.success_flag = False
                raise Exception(f'position_task failes during mDev_forward() op on dev {self.devName}')
            
        except Exception as ex:
            exptTrace(ex)
            self.success_flag = False
            print_log(f'go2pos() FESTO failed on{self.device_id}({self.devName}) FESTO device. Exception: {ex} of type: {type(ex)}.')
            if self.dev_lock.locked():
                self.dev_lock.release()
            return False
        self.wd = threading.Thread(target=self.mDev_watch_dog_thread)
        self.wd.start()
        return True
     

    def mDev_stored_pos(self)->int:
        return self.readStoredPos()
    


    def  move_home(self)->bool:
        if not self.mutualControl():
            return False
        self.success_flag = True 

        try:
            self._reset()
            if not self.__mot.ready_for_motion():
                raise Exception (f'ERROR: FESTO {self.devName} is not ready for motion')

            if self.__mot.referenced():
                self.__mot.position_task(0, self.HOMING_VELOCITY, nonblocking=True)
                self.homing_mode_active = False
            else:
                self.__mot.velocity_task(self.HOMING_VELOCITY)
                self.homing_mode_active = True

            if not self._success_run():
                raise Exception(f'motion task failed during move_home() op on dev {self.devName}')
            
        except Exception as ex:
            exptTrace(ex)
            self.success_flag = False
            print_log(f'Homing of FESTO failed on{self.device_id}({self.devName}) FESTO device. Exception: {ex} of type: {type(ex)}.')
            if self.dev_lock.locked():
                self.dev_lock.release()
            self.homing_mode_active = False
            return False
        
        self.wd = threading.Thread(target=self.mDev_watch_dog_thread)
        self.wd.start()
        return True

    def  mDev_forward(self, velocity = None, timeout=None, polarity:bool=None, stall = None)->bool:
        if not self.mutualControl():
            return False
       
        if velocity == None:
            velocity = self.velocity
        else:
            if velocity == '': velocity = 100
            elif int(velocity) > 100: velocity = 100

        self.success_flag = True 
        self.possition_control_mode = True

        try:
            self._reset()
            if not self.__mot.ready_for_motion():
                raise Exception (f'ERROR: FESTO {self.devName} is not ready for motion')

            _set_velocity = self.__max_velocity * (velocity/100) * M2MM
            print_log(f'Moving device {self.devName} forward, velocity = {_set_velocity} ({_set_velocity*(10**self.resolution_velocity)*M2MM} mm/s)')
            self.__mot.velocity_task(_set_velocity)
            if not self._success_run():
                raise Exception(f'velocity_task failes during mDev_forward() op on dev {self.devName}')
            
        except Exception as ex:
            exptTrace(ex)
            self.success_flag = False
            print_log(f'forward opefration at FESTO failed on{self.device_id}({self.devName}) FESTO device. Exception: {ex} of type: {type(ex)}.')
            if self.dev_lock.locked():
                self.dev_lock.release()
            return False
        self.wd = threading.Thread(target=self.mDev_watch_dog_thread)
        self.wd.start()
        return True


    def  mDev_backwrd(self, velocity = None, timeout=None, polarity:bool=None, stall = None)->bool:
        if not self.mutualControl():
            return False
       
        if velocity == None:
            velocity = self.velocity
        else:
            if velocity == '': velocity = 100
            elif int(velocity) > 100: velocity = 100

        self.success_flag = True 
        self.possition_control_mode = True

        try:
            self._reset()
            if not self.__mot.ready_for_motion():
                raise Exception (f'ERROR: FESTO {self.devName} is not ready for motion')

            _set_velocity = self.__max_velocity * (velocity/100) * M2MM * (-1)
            print_log(f'Moving device {self.devName} forward, velocity = {_set_velocity} ({_set_velocity*(10**self.resolution_velocity)*M2MM} mm/s)')
            self.__mot.velocity_task(_set_velocity)
            if not self._success_run():
                raise Exception(f'velocity_task failes during mDev_forward() op on dev {self.devName}')
            
        except Exception as ex:
            exptTrace(ex)
            self.success_flag = False
            print_log(f'forward opefration at FESTO failed on{self.device_id}({self.devName}) FESTO device. Exception: {ex} of type: {type(ex)}.')
            if self.dev_lock.locked():
                self.dev_lock.release()
            return False
        self.wd = threading.Thread(target=self.mDev_watch_dog_thread)
        self.wd.start()
        return True

    def mDev_get_cur_pos(self)->float:
        return self.GetPos()

    def set_parms(self, parms:dict):
        pass

    def getTitle(self)->str:
        return self.__title

    @staticmethod
    def park_all():
        print_log('Parking all FESTO devices is not implemented yet')
    
    @staticmethod
    def unpark_all():
        print_log('Unparking all FESTO devices is not implemented yet')

    def is_parked(self) -> bool:
        print_log('Parking state checking for FESTO devices is not implemented yet')
        return False
    
    def mDev_vibrate(self)->bool: 
        print_log('Vibration operation for FESTO devices is not implemented yet')
        return True

#-------------- class ^^^ ------------------    

def _CMD(_com:ComModbus, arr:list, lock = None)->list:
    retValues = list()
        
    if len(arr) == 0:
        print_err(f'Empty CMD array = {arr}')
        return retValues
   
    # print_log(f'CMD array = {arr}')
    for _op in arr:
        _cmd = _op['cmd']
        answData = None
        _format = DATA_TYPES[_op['type']]['format']

        try:      
            if len(_cmd) == 2:          # read cmd: PNU, index, number
                    answData = _com.read_pnu_raw(pnu=_cmd[0], subindex=_cmd[1])        
                    # print_log(f"read_pnu_raw(0x{_cmd[0]:04x}/{_cmd[0]},, 0x{_cmd[1]:04x}) = answData={answData}")
            elif len(_cmd) == 3:          # read cmd: PNU, index, number
                    answData = _com.read_pnu_raw(pnu=_cmd[0], subindex=_cmd[1], num_elements=_cmd[2])        
                    # print_log(f"read_pnu_raw(0x{_cmd[0]:04x}/{_cmd[0]}, 0x{_cmd[1]:04x}, 0x{_cmd[2]:04x}) = answData={answData}")
            elif len(_cmd) == 4:
                    _send_data = struct.pack(_format, _cmd[3])
                    answData = _com.write_pnu_raw(pnu=_cmd[0], subindex=_cmd[1], num_elements=_cmd[2],value=_send_data)        
                    # print_log(f"write_pnu_raw(0x{_cmd[0]:04x}/{_cmd[0]}, 0x{_cmd[1]:04x}, 0x{_cmd[2]:04x}, 0x{_cmd[3]:04x}/{_cmd[3]}/{_send_data}) = answData=0x{answData:04x} ({answData})")
                    if not answData:
                         print_err(f'ERROR writing MODBUS')
            else:
                print_err(f'Error: Wrong command/query format: {_cmd}')

            if answData is not None:
                if not isinstance(answData, bool):
                
                    if DATA_TYPES[_op['type']]['size'] <= 8:
                        answData = answData[0:DATA_TYPES[_op['type']]['size']]

                    _data = struct.unpack(_format, answData)[0] if  _op['type'] != 'STR' else answData.decode('utf-8').rstrip('\x00')


                    retValues.append(_dev_cmd(cmd=_op, res=_data))
                else:
                    retValues.append(_dev_cmd(cmd=_op, res=answData))
            
            

        except Exception as ex:
                e_type, e_filename, e_line_number, e_message = exptTrace(ex)
                print_err(f"Exception: {ex} of type: {type(ex)} on cmd {_op}. answData = {answData}")
                continue
            ############

    return retValues


#============================================================
#--------------------------------------------------
# UNITEST ZONE
#
#--------------------------------------------------



# Enable loglevel info
if __name__ == "__main__":

    def _status(com)->str:
        _res = _CMD(com, [MOTION_MANAGER_STATUS, ACTIVE_MOTION_TASK, ACTIVE_MOTION_TASK_STATUS, ACTUAL_VELOCITY])
        Motion_Manager_status  = _res[0].res
        Active_motion_task = _res[1].res
        Active_motion_task_status = _res[2].res
        Actual_Velocity = _res[3].res
        # return f'[Motion Manager status = {Motion_Manager_status}, Active motion task = {Active_motion_task}, Active motion task status= {Active_motion_task_status}] '
        return f'[Active motion task status= {Active_motion_task_status}, Vel = {Actual_Velocity}/{mot.current_velocity()}] '


    def _test_arp():
        import os
        import socket
        # Perform LAN scan
        os.system('arp -a > scan.tmp')
        scan = np.loadtxt('.\scan.tmp', dtype='str', comments= ['Interface', 'Internet'] , usecols=0,  delimiter=None)
        os.system('del scan.tmp')
        empty_nodes = np.where(np.char.find(scan, 'incomplete', ) > 0)[0]
        scan = np.delete(scan, empty_nodes)
        hostname = socket.gethostname()
        print(f'hostname = {hostname}')
        ips = socket.gethostbyname_ex(hostname)[2]
        print(f'ips = {ips}')
        neighbour_ip = list()
        for _arp_ip in scan:
             for _local_ip in ips:
                _arp_ip_spl = _arp_ip.split('.')
                _local_ip_spl = _local_ip.split('.')
                if (_arp_ip_spl[0] != '255') and (_arp_ip_spl[3] != '255') and \
                    (_arp_ip_spl[0] == _local_ip_spl[0]) and (_arp_ip_spl[1] == _local_ip_spl[1]) and (_arp_ip_spl[2] == _local_ip_spl[2]):
                    neighbour_ip.append(_arp_ip)

        print (f'neighbour_ip = {neighbour_ip}, found {len(neighbour_ip)} hosts, type = {type(neighbour_ip)}')


        for __count, __ip in enumerate(neighbour_ip):
            print(f'Looking FESTO at IP={__ip} # {__count}')
            Festo_Motor.find_dev(__ip)


    def stop(mot:MotionHandler):
        pos = mot.current_position() 
        print(f'Start stoping: {time.time()}, pos = {pos*10**position_resolution} ')
        mot.position_task(pos, 5, absolute=True)
        print(f'End stoping: {time.time()}, pos = {mot.current_position()*10**position_resolution}')



    # _test_arp()
    # devs = Festo_Motor.enum_devices()
    # print(devs)
    # sys.exit()

    import logging


    _lg = Logging()
    _lg.logger.setLevel(logging.WARNING)

    Festo_Motor.find_dev('192.168.0.1')
    # sys.exit()

    _read_lst = [
        RESOLUTION_POSITION,
        RESOLUTION_VELOCITY,
        ACTUAL_NOMINAL_CURRENT,
        ACTUAL_CURRENT, 
        ACTUAL_POSITION, 
        POSITION,
        ENCODER_RESOLUTION,
        {'cmd':[4318, 0], 'type':'BOOL', 'txt': 'Activation of extended encoder resolution'},
        REFERENCING_STATUS,
        SERIAL_NUMBER,
        {'cmd':[2444, 0, 12], 'type':'STR', 'txt': 'Serial number motor reference configuration'},
        ACTUAL_VELOCITY,
        ACTUAL_MAX_VELOCITY,
    ]

    com = None
    mot = None

    try:
        com = ComModbus('192.168.0.1', timeout_ms=500)
        if not com.connected():
            raise Exception(f'Combus not connected')
        d_info = com.read_device_info()


        _res_lst = _CMD(com, _read_lst)

        for _el in _res_lst:
            print(f"_el -> cmd = {_el.cmd['cmd']}, {_el.cmd['txt']} = {_el.res}")



        _pos = _CMD(com, [POSITION])[0]
        _factor = _CMD(com, [RESOLUTION_POSITION])[0]
        print (f'position = {_pos.res}, factor = {_factor.res}  => {_pos.res*(10**_factor.res)} // {_pos.res/(10**7)} mm // {_pos.res/(10**16)}')
        # sys.exit()

        
        # _CMD(com, [{'cmd':[*(RESOLUTION_POSITION['cmd']), 1, -6], 'type':'SINT', 'txt': 'Resolution position'}])

        # _pos = _CMD(com, [POSITION])[0]
        # _factor = _CMD(com, [RESOLUTION_POSITION])[0]

        # print (f'position = {_pos.res}, factor = {_factor.res}  => {_pos.res*(10**_factor.res)} // {_pos.res/(10**7)} mm // {_pos.res/(10**16)}')

        # sys.exit()
        

        mot = MotionHandler(com)
        # with MotionHandler(com) as mot:
        mot.acknowledge_faults()


        if not mot.plc_control_granted:
            raise Exception(f'No PLC control granted. com = {com}, mot = {mot}')    
                    
        _ps = mot.enable_powerstage()

        if not _ps:
            raise Exception(f'Can not enable motion control. com = {com}, mot = {mot}')


        # mot.disable_powerstage()


        # mot.referencing_task()    
        # mot.jog_task(duration=10)

        # mot.position_task(0, 60000, absolute=True)
        # mot.position_task(10000, 60000)
        # mot.position_task(-10000, 60000)
        # mot.position_task(20000, 60000, absolute=True)
        print(f'position_info_string = {mot.position_info_string()}')
        print(f'velocity_info_string = {mot.velocity_info_string()}')


        position_resolution = _CMD(com, [RESOLUTION_POSITION])[0].res
# homing
        print ('HOMING')

        if not mot.referenced():
            mot.velocity_task(-5, 20.0)
            if mot.fault_present():
                print(f'ERROR (FAULT): {mot.fault_string()}')
                mot.acknowledge_faults()
                mot.enable_powerstage()

            mot.referencing_task() 
            if mot.fault_present():
                print(f'ERROR (FAULT): {mot.fault_string()}')
                mot.acknowledge_faults()
                mot.enable_powerstage()
        else:
            print('Already homed. Do nothing')

        if  mot.ready_for_motion():
            print ('READY to motion')
        else:
            print('NON READY (probably no PLC control garnted or error present)')
        
        mot.position_task(100, 35, absolute=True, nonblocking=True)
        print (f'GO to absolute 100, {_status(com)}')
        while not mot.target_position_reached():
            time.sleep(0.5)
            print(f'pos = {mot.current_position()*10**position_resolution} , current = {_CMD(com, [ACTUAL_CURRENT])[0].res}, {_status(com)}')
            if mot.fault_present():
                print(f'ERROR (FAULT): {mot.fault_string()}')
                mot.acknowledge_faults()
                mot.enable_powerstage()
                break
        if mot.target_position_reached():
            print(f'Target reached, {_status(com)} ')

        
        mot.position_task(10000, 35, nonblocking=True)
        print (f'GO to relative 10000, {_status(com)}')
        while not mot.target_position_reached():
            print(f'pos = {mot.current_position()*10**position_resolution}, current = {_CMD(com, [ACTUAL_CURRENT])[0].res}, {_status(com)}')
            time.sleep(0.5)        
            if mot.fault_present():
                print(f'ERROR (FAULT): {mot.fault_string()}')
                mot.acknowledge_faults()
                mot.enable_powerstage()
                break
        if mot.target_position_reached():
            print(f'Target reached , {_status(com)}')


        mot.position_task(-10000, 35, nonblocking=True)
        print (f'GO to relative -10000, {_status(com)}')
        while not mot.target_position_reached():
            print(f'pos = {mot.current_position()*10**position_resolution}, current = {_CMD(com, [ACTUAL_CURRENT])[0].res}, {_status(com)}')
            time.sleep(0.5)        
            if mot.fault_present():
                print(f'ERROR (FAULT): {mot.fault_string()}, {_status(com)}')
                mot.acknowledge_faults()
                mot.enable_powerstage()
                break

        if mot.target_position_reached():
            print(f'Target reached , {_status(com)}')


        
        mot.position_task(30000, 35, absolute=True, nonblocking=True)
        print (f'GO to absolute 30000, {_status(com)}')
        while not mot.target_position_reached():
            print(f'pos = {mot.current_position()*10**position_resolution}, current = {_CMD(com, [ACTUAL_CURRENT])[0].res}, {_status(com)}')
            time.sleep(0.5)
            if mot.fault_present():
                print(f'ERROR (FAULT): {mot.fault_string()}, {_status(com)}')
                mot.acknowledge_faults()
                mot.enable_powerstage()
                break
        if mot.target_position_reached():
            print(f'Target reached, {_status(com)} ')


            start_time = time.time()
            mot.velocity_task(-35 )
            print(f'Moving up to barrier from pos= {mot.current_position()} , {_status(com)}')
            while not mot.fault_present() and abs(__cur := _CMD(com, [ACTUAL_CURRENT])[0].res) < 0.1:
                print(f'pos = {mot.current_position()*10**_CMD(com, [RESOLUTION_POSITION])[0].res}, current = {__cur}, {_status(com)}')
                if (time.time() - start_time) > 0.5:
                    stop(mot)
                    break
                time.sleep(0.1)


            if mot.fault_present():
                print(f'ERROR (FAULT): {mot.fault_string()}, {_status(com)}')
                mot.acknowledge_faults()
                mot.enable_powerstage()

        print('DONE')
        
        mot.configure_brake(False)
        mot.disable_powerstage()

        # mot.velocity_task(5, 6.0)
        # time.sleep(1.0)
        # mot.velocity_task(-3, 10.0)

        
    except Exception as ex:
        exptTrace(ex)
        print_err(f"Exception: {ex} of type: {type(ex)}, com = {com}")

    finally:
        if mot is not None:
            del mot
        if com is not None:
            del com

    