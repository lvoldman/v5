__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "1.0.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Tool"

from daqmx import NIDAQmxInstrument
import sys, time
from collections import namedtuple
from queue import Queue 


from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, set_parm, get_parm

# first, allocate the hardware using the automatic hardware
# allocation available to the instrument; this is safe when there
# is only one NIDAQmx instrument, but you may wish to specify a
# serial number or model number for a safer experience



class NI6002:
    dev = namedtuple('dev', ['device', 'model', 'sn'])

    @staticmethod
    def find_devs(sn):
        try:
            t_daq =  NIDAQmxInstrument(serial_number = sn)
            print_log(f'Found NI device: Device = {t_daq._device} PN={t_daq.model} SN={hex(t_daq.sn)}')
            found_dev = NI6002.dev(t_daq._device, t_daq.model, t_daq.sn)
            if int(t_daq.sn) == int(sn):
                return found_dev
            else:
                print_err(f'Error detecting NI device with SN = {hex(sn)}')
                return None
        except Exception as ex:
            print_err(f'Error detecting NI device with SN = {hex(sn)}')
            return None
        
    def __repr__(self):
        return f'NI: type: {self.devType}, name: {self.devName}, s/n: {self.sn }'

    def __init__ (self, _dname, sn, parms):
        try:
            print_log(f'Initiating {_dname}, s/n = {sn} (type={type(sn)})')
            self.devType = 'NI6002'
            self.devName = _dname
            self.sn = sn
            # self.daq = NIDAQmxInstrument(serial_number = f'{int(sn, 16):x}')
            self.daq = NIDAQmxInstrument(serial_number = sn)
            print_log(f'Activating DAQ NI device {self.daq}')
            # self.daq = NIDAQmxInstrument(model_number='USB-6002')
            self.devNotificationQ:Queue = Queue()         # for compatability 

            self.set_parms(parms=parms)
            self.__allOFF()

            print_log(f'DAQ = {self.daq}')
        except Exception as ex:
            exptTrace(ex)
            self.daq = None
            print_log(f'DAQ aquisition/initialization failed. Exception = {ex}')

    def __allOFF(self):
        for _port in range (3):
            if _port == 0:
                for _line in range (8):
                    self.setSignal ( _port, _line, False)
            elif _port == 1:
                for _line in range (4):
                    self.setSignal ( _port, _line, False)
            elif _port == 2:
                self.setSignal ( _port, 0, False)
            else:
                print_err(f'WARNING reseting unexistent port = {_port}')


    def setSignal (self, _port, _line, _signal:bool = True) -> bool:
        # if _signal:
        #     _signal_int = 1
        # else:
        #     _signal_int = 0
        print_log(f'Setting DAQ {_port},{_line} -> {_signal} for sn = {self.sn} / {self.devName}')
        try:
            match int(_port):
                case 0:
                    match int(_line):
                        case 0: 
                            self.daq.port0.line0 = _signal
                        case 1:
                            self.daq.port0.line1 = _signal
                        case 2:
                            self.daq.port0.line2 = _signal
                        case 3:
                            self.daq.port0.line3 = _signal
                        case 4:
                            self.daq.port0.line4 = _signal
                        case 5:
                            self.daq.port0.line5 = _signal
                        case 6:
                            self.daq.port0.line6 = _signal
                        case 7:
                            self.daq.port0.line7 = _signal
                        case _:
                            print_err(f'Wrong line number - {_line} for port = {_port}')
                            return False

                case 1:
                    match int(_line):
                        case 0: 
                            self.daq.port1.line0 = _signal
                        case 1:
                            self.daq.port1.line1 = _signal
                        case 2:
                            self.daq.port1.line2 = _signal
                        case 3:
                            self.daq.port1.line3 = _signal
                      
                        case _:
                            print_err(f'Wrong line number - {_line} for port = {_port}')
                            return False

                case 2:
                    match int(_line):
                        case 0: 
                            self.daq.port2.line0 = _signal
                        case _:
                            print_err(f'Wrong line number - {_line} for port = {_port}')
                            return False
                case _:
                            print_err(f'Wrong port number = {_port}')
                            return False
            
        except Exception as ex:
            # self.daq = None
            print_log(f'DAQ set {_port},{_line} for sn = {self.sn} / {self.devName} failed. Exception = {ex}')
            return False
        
        return True
    
    def getSignal(self,  _port:int, _line:int) -> tuple:
        _signal = 0
        try:
            match int(_port):
                case 0:
                    match int(_line):
                        case 0: 
                            _signal = self.daq.port0.line0 
                        case 1:
                            _signal = self.daq.port0.line1
                        case 2:
                            _signal = self.daq.port0.line2
                        case 3:
                            _signal = self.daq.port0.line3
                        case 4:
                            _signal = self.daq.port0.line4
                        case 5:
                            _signal = self.daq.port0.line5
                        case 6:
                            _signal = self.daq.port0.line6
                        case 7:
                            _signal = self.daq.port0.line7
                        case _:
                            print_err(f'Wrong line number - {_line} for port = {_port}')
                            return False, _signal

                case 1:
                    match int(_line):
                        case 0: 
                            _signal = self.daq.port1.line0
                        case 1:
                            _signal = self.daq.port1.line1
                        case 2:
                            _signal = self.daq.port1.line2
                        case 3:
                            _signal = self.daq.port1.line3
                      
                        case _:
                            print_err(f'Wrong line number - {_line} for port = {_port}')
                            return False, _signal

                case 2:
                    match int(_line):
                        case 0: 
                            _signal = self.daq.port2.line0
                        case _:
                            print_err(f'Wrong line number - {_line} for port = {_port}')
                            return False, _signal
                case _:
                            print_err(f'Wrong port number = {_port}')
                            return False, _signal
                
        except Exception as ex:
            # self.daq = None
            exptTrace(ex)
            print_log(f'DAQ: {self.daq}, sn = {self.sn}, port = {_port}/{type(_port)}, line = {_line}/{type(_line)} failed. Exception = {ex}')
            return False, _signal
        
        return True, _signal



    def __del__ (self):
        print_log(f'Exiting DAQ: {self.daq}')
        # self.__allOFF()


    def mDev_stop(self):                # for compitability with CDev
        pass

    def set_parms(self, parms):       
        pass
        # self.SELECTOR_TRIGGER_DELAY = get_parm('TRIGGER', parms, 'SELECTOR_TRIGGER_DELAY')


    # def jtseStart(self):
    #     if not self.daq:
    #         print_log(f'No DAQ card configured')
    #         return
    #     print_inf(f'NI6002 is to start operating LTSE Hot Air Station on {self.daq}')
        
    #     try:
    #         self.daq.port0.line2 =  True            # start operating
    #         time.sleep(0.5)                         # delay to start
    #     except Exception as ex:
    #         self.daq = None
    #         print_log(f'DAQ trigger operation failed. Exception = {ex}')

    


    # def jtseStop(self):
    #     if not self.daq:
    #         print_log(f'No DAQ card configured')
    #         return
    #     print_inf(f'DI6002 is about to stop operating LTSE Hot Air Station on {self.daq}')
        
    #     try:
    #         self.daq.port0.line2 =  False            # stop operating
    #         time.sleep(0.5)                          # delay to start
    #     except Exception as ex:
    #         self.daq = None
    #         print_log(f'DAQ trigger operation failed. Exception = {ex}')

    # def trigger(self):
    #     print_inf(f'DI6002 Trigger operated on {self.daq}')
    #     if not self.daq:
    #         print_log(f'No DAQ card configured')
    #         return
        
    #     try:
    #         self.daq.port0.line3 =  True            # shot!
    #         time.sleep(0.5)
    #         self.daq.port0.line3 =  False           # back to unarmed state
    #     except Exception as ex:
    #         self.daq = None
    #         print_log(f'DAQ trigger operation failed. Exception = {ex}')


    # def selector(self, sel):
    #     if not self.daq:
    #         print_log(f'No DAQ card configured')
    #         return
        
    #     try:
    #         dig = int(sel)
    #     except Exception as ex:
    #         print_log(f'Wrong input: {sel}. Exception = {ex}')
    #         return
        
    #     if (dig < 0) or  (dig > 3):
    #         print_log(f'Wrong input: {sel}. Only 0/1/2/3 are allowed')
    #         return
    #     bit1 = bool(dig & 0b01)
    #     bit2 = bool(dig & 0b10)
    #     print_inf(f'DI6002 Selector (value = {sel}) is set: bit1 = {bit1}, bit2 = {bit2}')

    #     try:
    #         self.daq.port0.line4 = bool(bit1)
    #         self.daq.port0.line5 = bool(bit2)
    #         time.sleep(0.5)
            
    #     except Exception as ex:
    #         self.daq = None
    #         print_log(f'DAQ trigger operation failed. Exception = {ex}')


if __name__ == "__main__":
    import re
    if not len(sys.argv) == 2:
        print_log(f'Usage: python {sys.argv[0]} S/N')
        sys.exit()
    _re =  re.compile(r'[0-9a-fA-F]+')
    if not _re.match(sys.argv[1]):
        print_log (f'Invalid S/N: {sys.argv[1]}')
        sys.exit()


    _ni = NI6002('test', int(sys.argv[1]), dict())

    import random
    random.seed(5)
    _r = random.randint(0, 100)
    _v = True if  _r%2 == 0 else False

    for _port in range (3):
        if _port == 0:
            for _line in range (8):
                _r = random.randint(0, 100)
                _v = True if  _r%2 == 0 else False
                _ni.setSignal ( _port, _line, _v)
                print_log(f'Port = {_port}, Line={0} -> {_v}')
        elif _port == 1:
            for _line in range (4):
                _r = random.randint(0, 100)
                _v = True if  _r%2 == 0 else False
                _ni.setSignal ( _port, _line, _v)
                print_log(f'Port = {_port}, Line={_line} -> {_v}')
        elif _port == 2:
            _r = random.randint(0, 100)
            _v = True if  _r%2 == 0 else False
            _ni.setSignal ( _port, 0, _v)
            print_log(f'Port = {_port}, Line={0} -> {_v}')
        else:
            print_err(f'WARNING reseting unexistent port = {_port}')

    _sigs = list()
    
    for _port in range (3):
        if _port == 0:
            for _line in range (8):
                _r, _v =_ni.getSignal ( _port, _line)
                print_log(f'Port = {_port}, Line={_line}, signal = {_v}')
        elif _port == 1:
            for _line in range (4):
                _r, _v = _ni.getSignal ( _port, _line)
                print_log(f'Port = {_port}, Line={_line}, signal = {_v}')
        elif _port == 2:
            _r, _v = _ni.getSignal ( _port, 0)
            print_log(f'Port = {_port}, Line={0}, signal = {_v}')
        else:
            print_err(f'WARNING reseting unexistent port = {_port}')
        

        

