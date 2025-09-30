__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["Leonid Voldman","Sasha Schechtman"]
__license__ = "SLA"
__version__ = "2.0.1"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Prototype"

import time, time 
import serial as serial
import time, sys
from inputimeout  import inputimeout , TimeoutOccurred
import threading
from threading import Lock
from queue import Queue 
from collections import namedtuple
import easy_scpi as scpi
from typing import List


from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, set_parm, get_parm


# pip install ->
# zeroconf                      0.132.2
# easy_scpi                     0.1.4
# PyVISA                        1.14.1
# PyVISA-py                     0.7.2

# in serials.yml ->
# HMP:
#    H1: 100026215996A

# in params.yml ->
# HMP:
#     HMPDelay: 10          # in msec
#     HMPTimes: 10


hmpResFields = ["curr", "volt"]
hmpRes = namedtuple("hmpRes",  hmpResFields, defaults=[None,] * len(hmpResFields))
hmpRes.__annotations__={'curr':float,  'volt':float}         # specify type of elements

ROUND_DIGITS = 3

STATRUP_CURR:float = 10
STARTUP_VOLT:float = 0.02

RESET_CMD = '*RST'
OUTP_ON_CMD = 'OUTP ON'
OUTP_OFF_CMD = 'OUTP OFF'

class HMP_PS:
    def __init__ (self, sn, port, parms):
        self.__port = port
        self.devNotificationQ = Queue()
        self.DELAY:float =  0
        self.times = 1
        
        self.set_parms(parms=parms)

        self.__lastAve:str = '∞'
        self.sn = sn

        print_log(f'Initiating HMP on port {port}, s/n = {sn}')
        self.__instrument = scpi.Instrument(self.__port)

        if self.__instrument == None:
            raise Exception(f'Cant initialize instrument on port {port}')
        
        self.__instrument.connect()
        self.__instrument.init()

        _resp = self.__instrument.query( '*IDN?' )
        self.devName = _resp.split(',')[1]

        print_log(f'Device = {self.devName} of [{_resp}]')
        self.__instrument.write(RESET_CMD)
        self.__instrument.write(OUTP_OFF_CMD)

        self.__voltRT = 0
        self.__currRT = 0

        self.setVoltCurr(volt = STARTUP_VOLT, curr  = STATRUP_CURR)



    def __del__ (self):
        self.__instrument.write(OUTP_OFF_CMD)
        pass


    def getVoltRT(self) -> float:
        return self.__voltRT


    def getCurrRT(self) -> float:
        return self.__currRT

    def getAve(self) -> str:
        return self.__lastAve


    def setVoltCurr(self, volt:float  = None, curr:float  = None):
        print_log(f'Setting new voltage/current: curr = {curr}, volt = {volt}')
        # self.__instrument.write(OUTP_ON_CMD)
        if curr:
            self.__instrument.source.current(curr)
        if volt:
            self.__instrument.source.voltage(volt)
            time.sleep(0.5)
        self.__currRT = self.__instrument.measure.current.dc()
        self.__voltRT = self.__instrument.measure.voltage.dc()
        print_log(f'Updated  voltage/current: curr = {self.__currRT}, volt = {self.__voltRT}')
        # self.__instrument.write(OUTP_OFF_CMD)



    def __singleMeasurement(self, curr:float = None, volt:float = None) -> hmpRes:

        print_log(f'Changing (input) before meadurement: curr = {curr}, volt = {volt}')

        if curr:
            self.__instrument.source.current(curr)
        if volt:
            self.__instrument.source.voltage(volt)


        # str.strip()  ???

        self.__currRT = self.__instrument.measure.current.dc()
        self.__voltRT = self.__instrument.measure.voltage.dc()
        print_log(f'Intermediate measuremenr result: curr = {self.__currRT}, volt = {self.__voltRT}')
        _curr = float(self.__currRT)
        _volt = float(self.__voltRT)
        print_log(f'Measuremenr result: curr = {_curr}, volt = {_volt}')

        

        return hmpRes(curr = _curr, volt = _volt)




    def doMeasurement(self, curr = None, volt = None) -> str:
        _res = '∞'
        resLst:List[float] = list()

        self.__instrument.write(OUTP_ON_CMD)
        time.sleep(0.5)

        for _ in range(self.times):
            _res  = self.__singleMeasurement(curr=curr, volt=volt)
            time.sleep(0.1)
            if not (_res.curr == 0):
                resLst.append(_res.volt/_res.curr)
            time.sleep(self.DELAY)
        
        time.sleep(0.5)
        self.__instrument.write(OUTP_OFF_CMD)
        
        _ave:float = 0

        if len(resLst) == 0:            # all results
            self.__lastAve = '∞'
        else:
            for _val in resLst:
                _ave += _val
            
            _res_v = _ave/len(resLst)

            if  _res_v < 1:
                self.__lastAve = str(round(_res_v*1000, ROUND_DIGITS)) + 'm'  + 'Ω'
            elif _res_v > 10000:
                self.__lastAve = str(round(_res_v/1000, ROUND_DIGITS)) + 'k'  + 'Ω'
            else:
                self.__lastAve = str(round(_res_v, ROUND_DIGITS))  + 'Ω'

        return True

    def mDev_stop(self):
        self.__instrument.write(OUTP_OFF_CMD)


    def set_parms(self, parms):
        self.DELAY =  int(get_parm('HMP', parms, 'HMPDelay')) / 1000
        self.times = int(get_parm('HMP', parms, 'HMPTimes'))

################ UNITEST module ###############

if __name__ == "__main__":
    import serial.tools.list_ports

    def SimpleTest(_port):
        try:
            _inst = scpi.Instrument(_port)
    
            if _inst == None:
                print(f' Error initiation = {_inst}')
                sys.exit()
            _inst.connect()
            _inst.init()

            print(f"Device ={_inst.query( '*IDN?' )}")
            # print(f'Timeout = {_inst.timeout()}')
        except Exception as ex:
            print(f'Exception operation error on {ex}')   
            sys.exit()

        while(True):
            try:
                _val = input("Enter cmd: [Q]/cmd ").split('/')


                try:
                    if len(_val) == 2 and _val[0].upper() == 'Q':
                        _resp = _inst.query(_val[1])
                        print(f'cmd [{_val[1]}] returned {_resp}** V={_inst.measure.voltage.dc()}, I={_inst.measure.current.dc()}')
                    elif len(_val) > 1:
                        print(f'Error entering cmd')
                    else:
                        _inst.write(_val[0])
                        print(f'cmd [{_val[0]}] done** V={_inst.measure.voltage.dc()}, I={_inst.measure.current.dc()}')

                except Exception as ex:
                    print(f'Something goes wrong. Exception = {ex}')
                    continue

            except KeyboardInterrupt as ex:
                print(f'Exiting by ^C \n{ex}')
                _inst.disconnect()
                sys.exit()
            except Exception as ex:
                print(f'Exception operation error on {ex}')   
           



    for ps in list(serial.tools.list_ports.comports()):
        print ("========================================")
        print (f"name={ps.name}")
        print (f"device={ps.device}")
        print (f"description={ps.description}")
        print (f"hwid={ps.hwid}")
        print (f"vid={ps.vid}")
        print (f"serial_number={ps.serial_number}")
        print (f"location={ps.location}")
        print (f"manufacturer={ps.manufacturer}")
        print (f"product={ps.product}")
        print (f"interface={ps.interface}")

    _inst = None
    try:

        print(f'{len(sys.argv)} -> {sys.argv}')
        if not len(sys.argv) == 2  or  not sys.argv[1].isdecimal():
            print (f"Usage: python {sys.argv[0]} port  (1,2,3..)")
            sys.exit()

        _port = f'COM{sys.argv[1]}'

        SimpleTest(_port)

    except Exception as ex:
                print(f'Exception operation error on {ex}') 
        
        
        