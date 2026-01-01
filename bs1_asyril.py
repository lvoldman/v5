
__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["Leonid Voldman"]
__license__ = "SLA"
__version__ = "2.0.1"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Prototype"

# import telnetlib3
import telnetlib
import asyncio, time
from collections import namedtuple
import asyril_error as ae 

from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, set_parm, get_parm, unsigned_16 


DEFAULT_RECIPE =  61013         # 13690  
PREPARE_PART = 'prepare_part '
GET_PART = 'get_part'
STATE_QUERY = 'get_parameter state'
START_PRODUCTION = 'start production'
STOP_PRODUCTION = 'stop production'
FORCE_TAKE_IMAGE = 'force_take_image'

_end_char = '\r\n'
_port1 = 7171
_port2 = 171 
_port3 = 17171


_partPosFields = ["x", "y"]
_partPos = namedtuple("_partPos",  _partPosFields, defaults=[None,] * len(_partPosFields))
_partPos.__annotations__={'x':float,  'y':float} 


class AsyrilInterface:
    def find_device(t_ip, t_port, _unit_id = 1) -> bool:
        try:
            print_log(f'Looking ASYRIL at IP:port = {t_ip}:{t_port}')
            test_tn =  telnetlib.Telnet(t_ip, t_port)
            print_log (f'ASYRIL test_client = {test_tn}')
             
            
            if test_tn is None:
                print_err(f'ASYRIL with IP:port = {t_ip}:{t_port} not found')
                return False

            _res = test_tn.open()
            print_log(f'Connecting to ASYRIL at IP:port = {t_ip}:{t_port} has {"SUCCEDED" if _res else "FAILED"}')

            
            test_client.close()
            del test_client
            return True
        
        except Exception as ex:
            print_err(f'Error connecting  ASYRIL with IP:port = {t_ip}:{t_port}')
            exptTrace(ex)
            return False
        
    def __init__(self, _ip_addr:str, _port:int, _recipe:int = None) -> None:
        self.ip_addr = _ip_addr
        self.port = _port
        self.__recipe = _recipe if _recipe is not None else DEFAULT_RECIPE
        self.tn =  telnetlib.Telnet(self.ip_addr, self.port)
        self._clean_buff()
        self._run_cmd(STATE_QUERY)
        res, resp = self._get_resp()
        if 'ready' in resp:
            _cmd = f'{START_PRODUCTION} {self.__recipe}'
            self._run_cmd(_cmd)
            self._get_resp()
        elif 'production' in resp:
            pass
        else:
            print(f'ERROR operating ASYRIL')
        
        self.__retakeImage()

    def __del__(self):
        self.tn.close()
        del self.tn

    def __retakeImage(self):
        self._run_cmd(FORCE_TAKE_IMAGE)
        self._get_resp()
        self._run_cmd(PREPARE_PART)
        self._get_resp()

    def _clean_buff (self):
        _exp:list = list()
        _exp.append(b'200')
        _ind, _code, _str =  self.tn.expect(_exp, timeout=1)
        print(f'Cleaning buffer:')
        print(f'[ind = {_ind}, code={_code} -> {_str}]')

    def _run_cmd(self, _cmd:str):
        _buf = f'{_cmd}{_end_char}'.encode('utf-8')
        print(f'CMD: {_buf}.  ', end = '')
        self.tn.write(_buf)
        pass

    def _get_resp(self)->tuple[int, str]:
        response = ''
        _exp:list = list()
        codes_str = '(2|4|5)[0-9]{2}'.encode('utf-8')
        _exp.append(codes_str)
        # _exp.append(b'^(2|4|5)[0-9]{2}')
        # _exp.append(b'^\b(6|7|8)[0-9]{2}\b')
        # _exp.append(b'200')
        _ind, _code, _str =  self.tn.expect(_exp, timeout=None)
        print_log(f'[ind = {_ind}, code={_code} -> {_str}]')
        
        t_str = _str
        while not t_str == b'':
            t_str =  self.tn.read_very_eager()
            response += t_str.decode('utf-8')
        
        if _ind < 0 or not (int(_str.decode('utf-8')) == 200):
            if _ind >= 0: 
                print(f"Error {_str.decode('utf-8')}: {ae.asyrilErrorCodes[int(_str.decode('utf-8'))]} >> {response}")
            return int(_str.decode('utf-8')), None
        else:
            print_log(f"CMD RESULT:\n{response}")  

        return int(_str.decode('utf-8')), response
    

    def get_pos (self, _try:int = 0) -> tuple:
        self.__retakeImage()
        self._run_cmd(GET_PART)
        res, _str_pos = self._get_resp()
        _pos = _str_pos.strip().split(' ')
        _x = float(_pos[0][2:])
        _y = float(_pos[1][2:])
        _rz = float(_pos[2][3:])
        print_log(f'Before correction: {_pos} -> x= {_x}, y= {_y}, rz = {_rz}')
        if _rz > 180:
            _rz = _rz - 180
        print_log(f'After correction: {_pos} -> x= {_x}, y= {_y}, rz = {_rz}')
        return _x, _y, _rz, 1, 1                  # last values are offset sign gor axes x and y , for compatability
        
    @staticmethod
    def is_avilable(_ip:str, _port:int) -> bool:
        status = False
        _tn = None
        try:
            _tn =  telnetlib.Telnet(_ip, _port)
                                        # cleaning buffer
            _exp:list = list()
            _exp.append(b'200')
            _ind, _code, _str =  _tn.expect(_exp, timeout=1)
            

            _tn.write(f'{STATE_QUERY}{_end_char}'.encode('utf-8'))

            resp =_get_resp(_tn)
            if 'ready' in resp or  'production' in resp:
                status =  True
            else:
                print(f'ERROR operating ASYRIL at {_ip}:{int}')
                status = False

        except Exception as ex:
            print_log(f'Attempt to access Asyril device at {_ip}:{int} was failed: {ex}')
            exptTrace(ex)
            status = False

        if _tn is not None:
            _tn.close()
            
        return status


def _clean_buff (tn:telnetlib.Telnet):
    _exp:list = list()
    _exp.append(b'200')
    _ind, _code, _str =  tn.expect(_exp, timeout=0.5)
    print(f'[ind = {_ind}, code={_code} -> {_str}]')



####################################################################################################


def _get_resp(_tn:telnetlib.Telnet)->str:
    response = ''
    _exp:list = list()
    codes_str = '(2|4|5)[0-9]{2}'.encode('utf-8')
    _exp.append(codes_str)

    _ind, _code, _str =  _tn.expect(_exp, timeout=None)
    print_log(f'[ind = {_ind}, code={_code} -> {_str}]')
    
    t_str = _str
    while not t_str == b'':
        t_str =  _tn.read_very_eager()
        response += t_str.decode('utf-8')
    
    if _ind < 0 or not (int(_str.decode('utf-8')) == 200):
        if _ind >= 0: 
            print_err(f"Error {_str.decode('utf-8')}: {ae.asyrilErrorCodes[int(_str.decode('utf-8'))]} >> {response}")
        return None
    else:
        print_log(f"CMD RESULT:\n{response}")  

    return response



if __name__ == "__main__":

    


    def get_resp(tn:telnetlib.Telnet)->str:
        response = ''
        _exp:list = list()
        test_str = '(2|4|5)[0-9]{2}'.encode('utf-8')
        _exp.append(test_str)
        _exp.append(b'^(2|4|5)[0-9]{2}')
        _exp.append(b'^\b(6|7|8)[0-9]{2}\b')
        _exp.append(b'200')
        _ind, _code, _str =  tn.expect(_exp, timeout=None)
        print(f'[ind = {_ind}, code={_code} -> {_str}]')
        
        while not _str == b'':
            _str =  tn.read_very_eager()
            response += _str.decode('utf-8')
        
        # print(response.decode('utf-8'))     
        return response

    def _get_pos(tn:telnetlib.Telnet) ->_partPos:
        pass



    def test_asyril():
        try:
            # Connect to a Telnet server (replace with your server and port)

            tn =  telnetlib.Telnet('192.168.1.50', 7171)
            print(f'Con = {tn}')
            # tn.write(b'\r\n')
            _clean_buff(tn)

            while True:
                # response = ''
                # _str =  tn.read_very_eager()
                # response += _str.decode('utf-8')
                # while not _str == b'':
                #     _str =  tn.read_very_eager()
                #     response += _str.decode('utf-8')
                
                # response = get_resp(tn)
                # print(response)


                _val = input(">")
                print(f'>>>>{_val}')
                _buf = f'{_val}{_end_char}'.encode('utf-8')
                tn.write(_buf)

                response = get_resp(tn)
                print(response)

        except KeyboardInterrupt:
                print(f'Cntr-C pressed. Exit.')



            # t_con = telnetlib3.open_connection('192.168.1.50', 7171, shell=shell)
# Run the main function
    
    # test_asyril()
    _asyril = AsyrilInterface('192.168.1.50', 7171)
    _asyril.get_pos()
