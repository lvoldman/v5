__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "1.0.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Tool"


import time 
from pyModbusTCP.client import ModbusClient
import serial as serial
import time, sys
from inputimeout  import inputimeout , TimeoutOccurred
import threading
from threading import Lock
from queue import Queue 
from collections import namedtuple
from typing import List

from numpy import uint32


MARCO_TIMEOUT = 10

#password = 413222 / Service

from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, set_parm, get_parm 

pulseDataFields = ["pulse_on", "cycle_rate", "pulse_count"]
pulseData = namedtuple("pulseData",  pulseDataFields, defaults=[0,] * len(pulseDataFields))
pulseData.__annotations__={'pulse_on':uint32,  'cycle_rate':uint32, 'pulse_count':uint32}


# Holding registers
# SET_TEMPERATURE = 40031      # WRITE ONLY - set temp
# ACTUAL_TEMPERATURE = 40032   # READ ONLY - current temp

# PULSE_DATA_R = 40040
# PULSE_DATA_SIZE = 6

# PULSE_ON_H = 40040           # R/W  pulse opening time [50 .. 10ˆ9 us]
# PULSE_ON_L = 40041           # R/W  pulse opening time [50 .. 10ˆ9 us]

# PULSE_CYCL_H = 40042        # R/W repetition rate; PulseCycl > PulseOn + 100us
# PULSE_CYCL_L = 40043        # R/W repetition rate; PulseCycl > PulseOn + 100us

# PULSE_COUNT_H = 40044       #  R/W number of pulses per trigger [0 .. 10ˆ9]
# PULSE_COUNT_L = 40045       #  R/W number of pulses per trigger [0 .. 10ˆ9]

# PURGE_STATUS_CMD = 40046     # R/W 0: closed, 1:open  // 1: toggle valve, 0: close valve
# PURGE_TIME = 40049           # R/W  maximum purge time [s * 10]  { 0.0s ... 286.3s }

# PULSE_RESET_COUNT_H  = 40071       # R/W pulses generated since last reset of counter in driver  // transmit 1 to reset counter in driver.
# PULSE_RESET_COUNT_L  = 40072       # R/W pulses generated since last reset of counter in driver // transmit 1 to reset counter in driver.


SET_TEMPERATURE = 31      # WRITE ONLY - set temp
ACTUAL_TEMPERATURE = 32   # READ ONLY - current temp

PULSE_DATA_R = 40
PULSE_DATA_SIZE = 6


PURGE_STATUS_CMD = 46     # R/W 0: closed, 1:open  // 1: toggle valve, 0: close valve
PURGE_TIME = 49           # R/W  maximum purge time [s * 10]  { 0.0s ... 286.3s }

PULSE_RESET_COUNT_H  = 70       # R/W pulses generated since last reset of counter in driver  // transmit 1 to reset counter in driver.
PULSE_RESET_COUNT_L  = 71  

SINGLE_SHOT = 48

SELECT_SINGLE_SHOT_PROGRAM = 4002   # 0-10
SELECT_EXT_PROGRAM = 4003           # 1- 11


class Marco_modbus:
    @staticmethod
    def find_server(t_ip, t_port) -> bool:
        try:
            print_log(f'Looking MARCO ModBus server at IP:port = {t_ip}:{t_port}')
            _client = ModbusClient(host=t_ip,            # Initiating modbus client connection
                    port = int(t_port), unit_id = 1, debug=False, timeout = MARCO_TIMEOUT, auto_open=True)
            print_log (f'client = {_client}')
             
            if not _client:
                print_log(f'ModBus server with IP:port = {t_ip}:{t_port} was not found')
                return False

            _res = _client.open()
            print_log(f'Connecting to Modbus server at IP:port = {t_ip}:{t_port} has {"SUCCEDED" if _res else "FAILED"}')

            if not _client.is_open:
                print_log(f'ModBus client can not connect to server >> IP:port = {t_ip}:{t_port} ')
                _client.close()
                del _client
                return False
            
            _client.close()
            del _client
            return True

            
        except Exception as ex:
            print_err(f'Error detecting ModBus server with IP:port = {t_ip}:{t_port}')
            exptTrace(ex)
            return False
        

    def __init__ (self, mb_ip, mb_port, parms, d_name = 'MARCO', mb_unit_id = 2):
        self.set_temp_ = 0
        self.pulse_on_ = 0
        self.cycle_rate_ = 0
        self.pulse_count_ = 0
        self.get_purge_time_ = 0
        self.__prog:int = 0
        self.__single_shot_status:bool = False
        self.m_client = None
        self.devNotificationQ:Queue = Queue()         # for compatability

        self.devName = d_name
        try:
            print_log(f'Connecting ModBus server at IP:port = {mb_ip}:{mb_port}')
            self.m_client = ModbusClient(host=mb_ip,            # Initiating modbus client connection
                    port = int(mb_port),
                    unit_id = mb_unit_id, 
                    timeout = MARCO_TIMEOUT,
                    debug=False, auto_open=True)
        
            self.set_parms(parms=parms)


        except Exception as ex:
            self.m_client = None
            print_log(f'ModBus initialization failed. Exception = {ex}')
            exptTrace(ex)
        else:
            self.__prog = self.program_control()
            print_log(f'ModBus connected at server={mb_ip}, port={mb_port}')


    @property
    def progNum(self)->int:
        # return self.__prog - 1
        return self.__prog

    # @progNum.setter
    # def progNum(self, value):
    #     print_err(f'ERROR: Internal value: progNum')

    def set_parms(self, parms):
        pass

    def mDev_stop(self):
        pass

    def program_control(self, _pNum:int = None)->bool:

        if not self.m_client:
            print_err(f'ERROR: ModBus client is not initiazed')
            return False

        _new_pr = None
        if _pNum is not None:
            _pNum = int(_pNum) - 1
            _new_pr = [0] * 1
            _new_pr[0] = int(_pNum)
            try:

                res = self.m_client.write_multiple_registers(int(SELECT_SINGLE_SHOT_PROGRAM), _new_pr)
                if not res:
                    print_log(f'Error setting program number for SingleShot {_new_pr}  at address {SELECT_SINGLE_SHOT_PROGRAM}')


                _new_pr[0] += 1                         # do not know but it works this way (num + 1) -
                                                        # SINGLE_SHOT [0 .. 10] active program for SingleShot (1st program is 0), 
                                                        # EXT_PROGRAM [1 .. 11] active program for external trigger (1st program is 1)

                res = self.m_client.write_multiple_registers(int(SELECT_EXT_PROGRAM), _new_pr)
                if not res:
                    print_log(f'Error setting program number {_new_pr}  at address {SELECT_EXT_PROGRAM}')
                
                

            except Exception as ex:
                print_log(f'Error seting program number {_new_pr}  at address {SELECT_EXT_PROGRAM}/{SELECT_SINGLE_SHOT_PROGRAM}')
                exptTrace(ex)
                

        try:
            val:List[int] = [0] * 1
            ctrl_val:List[int] = [0] * 1
            val = self.m_client.read_holding_registers(SELECT_EXT_PROGRAM, 1)
            if val is None:
                print_err(f'Error reading MODBUS value. Please verify MODBUS is set active')
                return False
            ctrl_val = self.m_client.read_holding_registers(SELECT_SINGLE_SHOT_PROGRAM, 1)
            if ctrl_val is None:
                print_err(f'Error reading MODBUS value. Please verify MODBUS is set active')
                return False
            if val[0] != ctrl_val[0] + 1:
                print_err(f'WARNING: EXT_PROGRAM is set to {val[0]} while SINGLE_SHOT_PROGRAM to {ctrl_val[0]}')


        except Exception as ex:
            print_log(f'Error reading program number at address {SELECT_EXT_PROGRAM}')
            exptTrace(ex)
            return False

        print_log(f'MARCO Program is set to {val[0]}/{ctrl_val[0]} ({_new_pr}/{_pNum})')

        self.__prog = int(val[0])
        
        # return val[0] - 1
        return True


    def set_temp(self, temp:int) -> bool:
        if not self.m_client:
            print_err(f'ERROR: ModBus client is not initiazed')
            return False
        
        print_log(f'Set temperature = {temp}')
        self.set_temp_ = temp
        try: 
            _temperatute = [0] * 1
            _temperatute[0] = int(temp)
            res = self.m_client.write_multiple_registers(int(SET_TEMPERATURE), _temperatute)
            if not res:
                print_log(f'Error Setting temperature {temp}  at address {SET_TEMPERATURE}')
                return False
            
        except Exception as ex:
            print_log(f'Error Setting temperature {temp}  at address {SET_TEMPERATURE}')
            exptTrace(ex)
            return False
        
        return True
    
    def get_set_temp(self) -> int:
        if not self.m_client:
            print_err(f'ERROR: ModBus client is not initiazed')
            return 0
        try:
            val:List[int] = list()
            val = self.m_client.read_holding_registers(SET_TEMPERATURE, 1)
            if val is None:
                print_err(f'Error reading MODBUS value. Please verify MODBUS is set active')
                return 0

        except Exception as ex:
            print_log(f'Error reading set temperature at address {SET_TEMPERATURE}')
            exptTrace(ex)
            return 0
        
        self.set_temp_ = val[0]
        return self.set_temp_

    def get_temp(self) -> int:
        if not self.m_client:
            print_err(f'ERROR: ModBus client is not initiazed')
            return 0
        try:
            val:List[int] = list()
            val = self.m_client.read_holding_registers(ACTUAL_TEMPERATURE, 1)
            if val is None:
                print_err(f'Error reading MODBUS value. Please verify MODBUS is set active')
                return 0

        except Exception as ex:
            print_log(f'Error reading temperature at address {ACTUAL_TEMPERATURE}')
            exptTrace(ex)
            return 0
        
        # print_log(f'Reading temp = {val} / {val[0]}')
        return val[0]
    
    def set_pulse_data (self, _on_time:uint32, _cycl_rate:uint32, _pulse_count:uint32):
        if not self.m_client:
            print_err(f'ERROR: ModBus client is not initiazed')
            return False

        self.pulse_on_ = _on_time
        self.cycle_rate_ = _cycl_rate
        self.pulse_count_ = _pulse_count
        
        _data_lst = [0] * 6
        _data_lst[0] = uint32(_on_time) >> 16
        _data_lst[1] = uint32(_on_time)  & 0x0000FFFF
        _data_lst[2] = uint32(_cycl_rate) >> 16
        _data_lst[3] = uint32(_cycl_rate)  & 0x0000FFFF
        _data_lst[4] = uint32(_pulse_count) >> 16
        _data_lst[5] = uint32(_pulse_count)  & 0x0000FFFF

        print_log(f'Set pulse data: pulse opening time = {_on_time}, pulse repetition rate = {_cycl_rate}, number of pulses per trigger = {_pulse_count} ')
        
        try:
             
            res = self.m_client.write_multiple_registers(PULSE_DATA_R, _data_lst)
            if not res:
                print_log(f'Error Setting pulse data  at address {PULSE_DATA_R} size = {PULSE_DATA_SIZE}')
                return False
            
        except Exception as ex:
            print_log(f'Error Setting pulse data  at address {PULSE_DATA_R} size = {PULSE_DATA_SIZE}')
            exptTrace(ex)
            return False
        
        return True
    
    def get_pulse_data (self) -> pulseData:
        if not self.m_client:
            print_err(f'ERROR: ModBus client is not initiazed')
            return 0
        
        print_log(f'Reading pulse data. Start at {PULSE_DATA_R}, size = {PULSE_DATA_SIZE}')

        try:
            val:List[uint32] = list()
            val = self.m_client.read_holding_registers(PULSE_DATA_R, PULSE_DATA_SIZE)
            if val is None:
                print_log(f'Error reading pulse data at address  {PULSE_DATA_R}, size = {PULSE_DATA_SIZE}. Check MODBUS mose turned on')
                return None
            
            _res = pulseData(pulse_on = uint32((val[0] << 16) | val[1]), \
                             cycle_rate = uint32((val[2] << 16) | val[3]),  \
                             pulse_count = uint32((val[4] << 16) | val[5])) 

            print_log(f'BUGBUG == {val}, {_res.pulse_on}/{_res.cycle_rate}/{_res.pulse_count}')
            self.pulse_on_ = _res.pulse_on
            self.cycle_rate_ = _res.cycle_rate
            self.pulse_count_ = _res.pulse_count
            return _res
           
        except Exception as ex:
            print_log(f'Error reading pulse data at address  {PULSE_DATA_R}, size = {PULSE_DATA_SIZE}')
            exptTrace(ex)
            return None
        
        

    # def set_pulse_on (self, _time:uint32) -> bool:
    #     if not self.m_client:
    #         print_err(f'ERROR: ModBus client is not initiazed')
    #         return False
        
    #     print_log(f'Set SET_TEMPERATURE = {_time}')
    #     try:
             
    #         res = self.m_client.write_multiple_registers(PULSE_ON_H, [int(_time >> 16), int(_time & 0x0000FFFF)])
    #         if not res:
    #             print_log(f'Error Setting pulse opening time {_time}  at address {PULSE_ON_H} /{PULSE_ON_H+1}')
    #             return False
            
    #     except Exception as ex:
    #         print_log(f'Error Setting pulse opening time {_time}  at address  {PULSE_ON_H} /{PULSE_ON_H+1}')
    #         exptTrace(ex)
    #         return False
        
    #     return True
    
    # def get_pulse_on (self) -> uint32:
    #     if not self.m_client:
    #         print_err(f'ERROR: ModBus client is not initiazed')
    #         return 0
        
    #     try:
             
    #         val:List[int] = list(2)
    #         val = self.m_client.read_holding_registers(PULSE_ON_H, 2)
    #         _time_h:uint32 = val[0]
    #         _time_l:uint32 = val[1]
    #         _time:uint32 = (_time_h << 16) & _time_l
           
    #     except Exception as ex:
    #         print_log(f'Error reading pulse opening time at address  {PULSE_ON_H} /{PULSE_ON_H+1}')
    #         exptTrace(ex)
    #         return 0
        
    #     return _time




    # def set_cycle_rate (self, _rate:uint32) -> bool:
    #     if not self.m_client:
    #         print_err(f'ERROR: ModBus client is not initiazed')
    #         return False
        
    #     print_log(f'set repetition rate = {_rate}')
    #     try:
            
             
    #         res = self.m_client.write_multiple_registers(PULSE_CYCL_H, [int(_rate >> 16), int(_rate & 0x0000FFFF)])
    #         if not res:
    #             print_log(f'Error Setting pulse repetition rate {_rate}  at address {PULSE_CYCL_H} /{PULSE_CYCL_H+1}')
    #             return False
            
    #     except Exception as ex:
    #         print_log(f'Error Setting repetition rate {_rate}  at address  {PULSE_CYCL_H} /{PULSE_CYCL_H+1}')
    #         exptTrace(ex)
    #         return False
        
    #     return True
    
    
    # def get_cycle_rate (self) -> uint32:
    #     if not self.m_client:
    #         print_err(f'ERROR: ModBus client is not initiazed')
    #         return 0
        
    #     try:
             
    #         val:List[int] = list(2)
    #         val = self.m_client.read_holding_registers(PULSE_ON_H, 2)
    #         _rate_h:uint32 = val[0]
    #         _rate_l:uint32 = val[1]
    #         _rate:uint32 = (_rate_h << 16) & _rate_l
           
    #     except Exception as ex:
    #         print_log(f'Error reading pulse repetition rate at address  {PULSE_ON_H} /{PULSE_ON_H+1}')
    #         exptTrace(ex)
    #         return 0
        
    #     return _rate
    
    # def set_pulse_count (self, _count:uint32) -> bool:
    #     if not self.m_client:
    #         print_err(f'ERROR: ModBus client is not initiazed')
    #         return False
        
    #     print_log(f'Set number of pulses per trigger = {_count}')
    #     try:
            
             
    #         res = self.m_client.write_multiple_registers(PULSE_COUNT_H, [int(_count >> 16), int(_count & 0x0000FFFF)])
    #         if not res:
    #             print_log(f'Error Setting number of pulses per trigger {_count}  at address {PULSE_COUNT_H} /{PULSE_COUNT_H+1}')
    #             return False
            
    #     except Exception as ex:
    #         print_log(f'Error Setting number of pulses per trigger {_count}  at address  {PULSE_CYCL_H} /{PULSE_COUNT_H+1}')
    #         exptTrace(ex)
    #         return False
        
    #     return True
    
    
    # def get_pulse_count (self) -> uint32:
    #     if not self.m_client:
    #         print_err(f'ERROR: ModBus client is not initiazed')
    #         return 0
        
    #     try:
             
    #         val:List[int] = list(2)
    #         val = self.m_client.read_holding_registers(PULSE_CYCL_H, 2)
    #         _count_h:uint32 = val[0]
    #         _count_l:uint32 = val[1]
    #         _count:uint32 = (_count_h << 16) & _count_l
           
    #     except Exception as ex:
    #         print_log(f'Error reading number of pulses per trigger at address  {PULSE_CYCL_H} /{PULSE_CYCL_H+1}')
    #         exptTrace(ex)
    #         return 0
        
    #     return _count


    def set_purge(self, _status:int) -> bool:
        if not self.m_client:
            print_err(f'ERROR: ModBus client is not initiazed')
            return False
        
        print_log(f' Purge cmd to excecute =  {"close" if _status == 0 else "open" }')

        try: 
            res = self.m_client.write_multiple_registers(PURGE_STATUS_CMD, [int(_status)])
            if not res:
                print_log(f'Error Setting purge to {_status} =  {"close" if _status == 0 else "open" } at address {PURGE_STATUS_CMD}')
                return False
            
        except Exception as ex:
            print_log(f'Error Setting purge to {_status} =  {"close" if _status == 0 else "open" } at address {PURGE_STATUS_CMD}')
            exptTrace(ex)
            return False
        
        return True
    
    def get_purge_status(self) -> int:

        if not self.m_client:
            print_err(f'ERROR: ModBus client is not initiazed')
            return 0
        try:
            val:List[int] = list()
            val = self.m_client.read_holding_registers(PURGE_STATUS_CMD, 1)
            if val is None:
                print_err(f'Error reading MODBUS value. Please verify MODBUS is set active')
                return 0

        except Exception as ex:
            print_log(f'Error reading  purge status at address {PURGE_STATUS_CMD}')
            exptTrace(ex)
            return 0
        
        print_log(f'purge status = {val[0]}')
        return val[0]
    
    
    
    def single_shot(self, _status:int) -> bool:
        if not self.m_client:
            print_err(f'ERROR: ModBus client is not initiazed')
            return False
        
        print_log(f' Single shot CMD to excecute =  {"stop" if _status == 0 else "start" } on program {self.__prog}')

        try: 
            res = self.m_client.write_multiple_registers(SINGLE_SHOT, [int(_status)])
            if not res:
                print_log(f'Error operating single shot e to {_status} =  {"stop" if _status == 0 else "start" } at address {SINGLE_SHOT}')
                return False
            
        except Exception as ex:
            print_log(f'Error operating single shot e to {_status} =  {"stop" if _status == 0 else "start" } at address {SINGLE_SHOT}')
            exptTrace(ex)
            return False
        
        if self.get_shot_status():
            __sh_ctl_thread = threading.Thread(target=self.__shoting_seq_cntl_thread)
            __sh_ctl_thread.start()
        return True
    
    def get_shot_status(self):
        
        if not self.m_client:
            print_err(f'ERROR: ModBus client is not initiazed')
            return 0
        try:
            val:List[int] = list()
            val = self.m_client.read_holding_registers(SINGLE_SHOT, 1)
            if val is None:
                print_err(f'Error reading MODBUS value. Please verify MODBUS is set active')
                return 0

        except Exception as ex:
            print_log(f'Error reading  sequence status at address {SINGLE_SHOT}')
            exptTrace(ex)
            return 0
        self.__single_shot_status = True if val[0] > 0 else False
        print_log(f'sequence status = {val[0]}')
        return val[0]

    def  __shoting_seq_cntl_thread(self):
        while self.__single_shot_status:
            self.get_shot_status()
            time.sleep(0.5)

    @property
    def shoting_status(self)->bool:
        return self.__single_shot_status
    
    




    def reset_pulse_count (self) -> bool:
        if not self.m_client:
            print_err(f'ERROR: ModBus client is not initiazed')
            return False
        
        print_log(f'Reseting pulse counter')
        try:
            res = self.m_client.write_multiple_registers(PULSE_RESET_COUNT_H, [0, 1])
            if not res:
                print_log(f'Error reseting pulse counter  at address {PULSE_RESET_COUNT_H} /{PULSE_RESET_COUNT_H+1}')
                return False
            
        except Exception as ex:
            print_log(f'Error reseting pulse counter  at address {PULSE_RESET_COUNT_H} /{PULSE_RESET_COUNT_H+1}')
            exptTrace(ex)
            return False
        
        return True
    
    
    def get_pulse_count_since_last_reset (self) -> uint32:
        if not self.m_client:
            print_err(f'ERROR: ModBus client is not initiazed')
            return 0
        
        try:
             
            val = [0] * 2
            val = self.m_client.read_holding_registers(PULSE_RESET_COUNT_H, 2)
            if val is None:
                print_err(f'Error reading MODBUS value. Please verify MODBUS is set active')
                return 0
            
            _count_h = val[0]
            _count_l = val[1]
            _count:uint32 = (_count_h << 16) | _count_l
           
        except Exception as ex:
            print_log(f'Error reading number of pulses per trigger at address  {PULSE_RESET_COUNT_H} /{PULSE_RESET_COUNT_H+1}')
            exptTrace(ex)
            return 0
        
        return _count


    def set_purge_time(self, _time:int) -> bool:
        if not self.m_client:
            print_err(f'ERROR: ModBus client is not initiazed')
            return False
        
        print_log(f'Set purge time = {_time}')
        self.get_purge_time_ = _time
        try: 
            res = self.m_client.write_multiple_registers(PURGE_TIME, [int(_time)])
            if not res:
                print_log(f'Error Setting purge time {_time}  at address {PURGE_TIME}')
                return False
            
        except Exception as ex:
            print_log(f'Error Setting  purge time {_time}  at address {PURGE_TIME}')
            exptTrace(ex)
            return False
        
        return True
    
    def get_purge_time(self) -> int:
        if not self.m_client:
            print_err(f'ERROR: ModBus client is not initiazed')
            return 0
        try:
            val:List[int] = list()
            val = self.m_client.read_holding_registers(PURGE_TIME, 1)
            if val is None:
                print_err(f'Error reading MODBUS value. Please verify MODBUS is set active')
                return 0

        except Exception as ex:
            print_log(f'Error reading set  purge time at address {PURGE_TIME}')
            exptTrace(ex)
            return 0
        
        self.get_purge_time_ = val[0]
        return self.get_purge_time_


    def __del__ (self):
        self.m_client.close()
        del self.m_client




#------- UNIT TEST -------------------
if __name__ == "__main__":

    def ret2text(ret) -> str:
        if ret == 0:
            return('operation fail')
        elif ret == 1:
            return('operation pass')
        elif ret == 2:
            return('operation errorl')
        elif ret == 3:
            return('not ready')
        elif ret == 4:
            return('timeoutl')
        elif ret == 5:
            return('move')
        else:
            return('unknown')




    try:
        test_client = Cam_modbus.find_server('192.168.230.51', 1502)
        print_log(f'test_client = {test_client}')

        # calibration_cam = Cam_modbus('192.168.230.51', 1502)

        # calibration_in_process = calibration_cam.start_tuning('Z4')

        # if not calibration_in_process:
        #     print_log(f'Calibration on Z4 failed to start')
        #     sys.exit()
        # else:
        #     print_log(f'calibration will be operated on Z4')

        # while True:
        #     time.sleep(0.1)
        #     try:
        #         i_cmd = inputimeout(prompt='Press Enter to continue....', timeout=1000)
        #         # i_cmd = inputimeout(prompt='Press Enter to continue....', timeout=5)
        #         # print(">", end='')
        #         # i_cmd = inputimeout(timeout=5)


        #     except TimeoutOccurred:
        #         print_log(f'Input timeout ')
        #         continue
        #     else:
        #         pass

             
        #     ret, dist = calibration_cam.calibration_step()
        #                             # operation fail = 0 //  operation pass = 1  // operation error = 2 //  not ready = 3 // 
        #                             # timeout = 4 // move = 5
        #                             # second return value - moving distance 
        #     print_log(f'Calibration step returned ret={ret} ({ret2text(ret)}), dist={dist}')
        #     if ret == 5:
        #         print_log(f'MOVE, distance = {dist}')
        #         sys.exit()

        #     elif ret == 0:
        #         print_log(f'Calibration on {calibration_cam.motor} failed. Go to next iteration..')
        #         continue
                
        #     elif ret == 1:
        #         print_log(f'Calibration on {calibration_cam.motor} succeded.')
        #         calibration_cam.stop_tuning()
        #         calibration_in_process = False
        #         break
                
        #     elif ret == 2:
        #         print_log(f'Calibration on {calibration_cam.motor} critical error')
        #         calibration_cam.stop_tuning()
        #         calibration_in_process = False
        #         break
                
        #     elif ret == 3:
        #         pass

        #     elif ret == 4:
        #         print_log(f'Calibration on {calibration_cam.motor} timeout')
        #         calibration_cam.stop_tuning()
        #         calibration_in_process = False
        #         break

           
    except KeyboardInterrupt:
            print_log(f'Cntr-C pressed. Exit.')