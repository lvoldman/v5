import serial as serial
import serial.tools.list_ports
import sys
from queue import Queue 
import re, time


from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, set_parm, get_parm, assign_parm

# _baudrate = 9600
_baudrate = 19200
_timeout = 1
_break_duration = 0.25
_parity='N'
_bytesize=8 
_stopbits=1

STX = b'\x02'
ETX = b'\x03'
RAT_CODE = 'RAT'
REFRESH_TIMEOUT = 1

_cmd_re = re.compile(r'\s*(R|W|A|N)(ST|SF|SE|AT|ET|PS|CT|SMN|MAT|MIT|MAF|MIF|MAE|MIE|SER|RSP|PP)\s*(\s+\d)?$')
_cnt_re = re.compile(r'\s*(R|W|A|N)(SMN|MAT|MIT|MAF|MIF|MAE|MIE|SER|RSP)\s*(\s+\d)?$')


class JTSEcontrol:
    def __init__(self, _devName, _port:str):
        self.devName = _devName
        self.devNotificationQ = Queue()                 # for compatability
        self.port = _port
        self.REFRESH_TIMEOUT = REFRESH_TIMEOUT
        self.ser = serial.Serial(port=self.port, baudrate = _baudrate, timeout = _timeout, parity=_parity, bytesize= _bytesize, stopbits=_stopbits)
        self.last_update = time.time()
        self.rat = 0
        _rat_str = JTSEcontrol.SendRcvCMD(self.ser, RAT_CODE)
        if  _rat_str.isdigit():
            self.rat = int(_rat_str)
        




    def __del__(self):
        if self.ser is not None:
            self.ser.close

    @property
    def RAT(self)->str:
        __c_time = time.time()
        if  __c_time - self.last_update < self.REFRESH_TIMEOUT:
            return self.rat
        else: 
            _rat_str = JTSEcontrol.SendRcvCMD(self.ser, RAT_CODE)
            if  _rat_str.isdigit():
                self.rat = int(_rat_str)
            self.last_update = __c_time
        
        return str(self.rat) 
        # return str(self.rat) + '°'

    '''
    
    '''
    @staticmethod
    def recognizeDev(ps:serial.tools.list_ports.ListPortInfo, dev_name:str = 'JTSE' )->str:
        _res:str = None
        _ser = None
        try:
            print_log(f'Looking for JTSE {dev_name} at port {ps.device} ')
            _ser = serial.Serial(port=ps.device, baudrate = _baudrate, timeout = _timeout, parity=_parity, bytesize= _bytesize, stopbits=_stopbits)
            _dev = JTSEcontrol.SendRcvCMD(_ser, 'RSMN')
            if _dev == dev_name:
                _res = ps.device
                return _res
        
            if _ser is not None:
                _ser.close
                _ser.__del__()
            
    
        except Exception as ex:
            exptTrace(ex)
            
            
        return None        


    @staticmethod
    def findDev(dev_name:str = 'JTSE' )->str:
        _res:str = None
        _ser = None
        for ps in list(serial.tools.list_ports.comports()):
            try:
                print_log(f'Looking for {dev_name} at port {ps.device} ')
                _ser = serial.Serial(port=ps.device, baudrate = _baudrate, timeout = _timeout, parity=_parity, bytesize= _bytesize, stopbits=_stopbits)
                _dev = JTSEcontrol.SendRcvCMD(_ser, 'RSMN')
                if _dev == dev_name:
                    _res = ps.device
                    break
            
                if _ser is not None:
                    _ser.close
                    _ser.__del__()
                
        
            except Exception as ex:
                exptTrace(ex)
                # return None
                continue
        
        return _res

    @staticmethod
    def SendRcvCMD(_ser, code:str)->str:
        try:
            _val = code
            _cmd:bytearray = bytearray()
            _cmd.extend(STX)
            if not _cnt_re.match(_val):
                _cmd.extend(bytearray(_val[0:3], 'utf-8'))
                _cmd.extend(bytearray('1', 'utf-8'))

            else:
                _cmd.extend(bytearray(_val[0:4], 'utf-8'))

            _cmd.extend(ETX)

            _bcc = 0
            for _i, _b in  enumerate(_cmd):
                if _i == 0:
                    _bcc = _b
                else:
                    _bcc = _bcc ^ _b
            
            _cmd.extend(bytes([_bcc]))
            _cmd_str = ''.join(f'0x{x:02x} ' for x in _cmd)
            # print_log (f'JTSE cmd  = {_cmd} ** {_cmd_str}')
            
            _ser.write(_cmd)

            answ = _ser.read_until(ETX)
            _bcc_answ = _ser.read()
            _str_answ = answ[5:10].decode("utf-8").strip()
            # print(f'JTSE reply: {answ} [{_str_answ}] -- {int(_str_answ) if _str_answ.isdigit() else _str_answ}, BCC = {_bcc_answ}')
            return _str_answ

        except Exception as ex:
            exptTrace(ex)
            return ''
    
    def  mDev_stop(self)->bool:                 # for compatability
        return True
    
    def set_parms(self, parms)->bool:
        self.REFRESH_TIMEOUT = assign_parm(self.devName, parms, 'REFRESH_TIMEOUT', self.REFRESH_TIMEOUT)
        return True



#-------------------  UNITEST ZONE ----------------------------------       
#   


if __name__ == "__main__":

    def _cmd_str_test_print()->str:
        try:

            _cmd_re = re.compile(r'\s*(R|W|A|N)(ST|SF|SE|AT|ET|PS|CT|SMN|MAT|MIT|MAF|MIF|MAE|MIE|SER|RSP|PP)\s*(\s+\d)?$')
            _cnt_re = re.compile(r'\s*(R|W|A|N)(SMN|MAT|MIT|MAF|MIF|MAE|MIE|SER|RSP)\s*(\s+\d)?$')
            while True:
                _val = input("Enter cmd: ")
                if not _cmd_re.match(_val):
                    print(f'Wrong command - {_val}')
                    continue

                _cmd:bytearray = bytearray()

                _cmd.extend(STX)
                if not _cnt_re.match(_val):
                    print(f'{_val[0:3]} ', end='')
                    _cmd.extend(bytearray(_val[0:3], 'utf-8'))

                    _cmd.extend(bytearray('1', 'utf-8'))

                else:
                    print(f'{_val[0:4]} ', end='')
                    _cmd.extend(bytearray(_val[0:4], 'utf-8'))

                _cmd.extend(ETX)

                print(f'cmd before bcc = {_cmd}')

                _bcc = 0
                for _i, _b in  enumerate(_cmd):
                    if _i == 0:
                        _bcc = _b
                        print(f'{_b} ', end = '')
                    else:
                        _bcc = _bcc ^ _b
                        print(f'^ {_b} ', end = '')

                print (f' = {_bcc}')

                
                _cmd.extend(bytes([_bcc]))

                print(f'cmd after bcc = {_cmd} ** ', end='')
                print (''.join(f'0x{x:02x} ' for x in _cmd))

        except KeyboardInterrupt as ex:
            exptTrace(ex)
            print ('^C exiting')
            

        except Exception as ex:
            exptTrace(ex)
            print (f'Exception - {ex}')



    # _j_p = JTSEcontrol.findDev()
    # if _j_p is not None:
    #     print(f'JTSE found om port: {_j_p}')
    #     _J = JTSEcontrol('JTSE', _j_p)
    #     time.sleep(2)
    #     print(f'temp= {_J.RAT}')
    # else:
    #     print(f'No JTSE found')   


         
    
    # sys.exit()

    if len(sys.argv) == 1:
        print(f'Usage: python {sys.argv[0]} COMx')
        print(f'if no COM present: test print, no write to COM')
        _cmd_str_test_print()

    for ps in list(serial.tools.list_ports.comports()):
        
        

        print_log ("========================================")
        print_log (f"name={ps.name}")
        print_log (f"device={ps.device}")
        print_log (f"description={ps.description}")
        print_log (f"hwid={ps.hwid}")
        print_log (f"vid={ps.vid}")
        print_log (f"serial_number={ps.serial_number}")
        print_log (f"location={ps.location}")
        print_log (f"manufacturer={ps.manufacturer}")
        print_log (f"product={ps.product}")
        print_log (f"interface={ps.interface}")    

    if not len(sys.argv) == 2:
        print(f'Usage: python {sys.argv[0]} COMx')
        sys.exit()
    _re =  re.compile(r'(COM|com)[0-9]+$')
    if not _re.match(sys.argv[1]):
        print(f'Invalid COM: {sys.argv[1]}')
        sys.exit()

    _port = sys.argv[1]



    try:
        serial.Serial()
        ser = serial.Serial(port=_port, baudrate = _baudrate, timeout = _timeout, parity=_parity, bytesize= _bytesize, stopbits=_stopbits)
        print (f"name = {ser.name}")
        _cmd_re = re.compile(r'\s*(R|W|A|N)(ST|SF|SE|AT|ET|PS|CT|SMN|MAT|MIT|MAF|MIF|MAE|MIE|SER|RSP|PP)\s*(\s+\d)?$')
        _cnt_re = re.compile(r'\s*(R|W|A|N)(SMN|MAT|MIT|MAF|MIF|MAE|MIE|SER|RSP)\s*(\s+\d)?$')
        while True:
            _val = input("Enter cmd: ")
            if not _cmd_re.match(_val):
                print(f'Wrong command - {_val}')
                continue

            _cmd:bytearray = bytearray()
            # bytes()
            # _cmd.append(STX)
            _cmd.extend(STX)
            if not _cnt_re.match(_val):
                print(f'{_val[0:3]} ', end='')
                _cmd.extend(bytearray(_val[0:3], 'utf-8'))
                # _cmd.append(b'\x01')
                # _cmd.append(0x01)
                _cmd.extend(bytearray('1', 'utf-8'))

            else:
                print(f'{_val[0:4]} ', end='')
                _cmd.extend(bytearray(_val[0:4], 'utf-8'))

            _cmd.extend(ETX)

            print(f'cmd before bcc = {_cmd}')

            _bcc = 0
            for _i, _b in  enumerate(_cmd):
                if _i == 0:
                    _bcc = _b
                    print(f'{_b} ', end = '')
                else:
                    _bcc = _bcc ^ _b
                    print(f'^ {_b} ', end = '')

            print (f' = {_bcc}')

            
            _cmd.extend(bytes([_bcc]))

            print(f'cmd after bcc = {_cmd} ** ', end='')
            print (''.join(f'0x{x:02x} ' for x in _cmd))

            ser.write(_cmd)
            print(f'Result -> ', end='')
            # answ = ser.read(size=12)

            answ = ser.read_until(b'\x03')
            _bcc_answ = ser.read()
            print(f'{answ} [{answ[5:10].decode("utf-8")}] -- {int(answ[5:10].decode("utf-8")) if answ[5:10].decode("utf-8").isdigit() else answ[5:10].decode("utf-8").strip()}, BCC = {_bcc_answ}')

    except KeyboardInterrupt as ex:
        exptTrace(ex)
        print ('^C exiting')
        sys.exit()

    except Exception as ex:
        exptTrace(ex)
        print (f'Exception - {ex}')
