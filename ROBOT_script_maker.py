
__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["Leonid Voldman","Sasha Schechtman"]
__license__ = "SLA"
__version__ = "2.0.1"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Prototype"

import time, re
import os.path

from typing import Optional

import logging, sys, datetime, yaml

logFileDate = datetime.datetime.now().strftime(f"MECA500_SCRIPT_%Y_%m_%d_%H_%M.txt")

log_format = u'%(asctime)s: %(filename)s--%(funcName)s/%(lineno)d -- %(thread)d [%(threadName)s] %(message)s' 
logging.basicConfig(format=log_format, handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler(logFileDate, mode="w", encoding = 'utf-8')], encoding = "utf8", level=logging.DEBUG)
print_log = logging.debug


def is_float(string):
    try:
        float(string)
        return True
    except ValueError:
        return False
    except Exception as ex:
        print_log(f'Unexpexted exception: {ex}')
        return False
    
BLACK_start = (-162.10705, -146.77473, 43.1000, -149.72264)
WHITE_start = (204.77551, -15.28232, 54.88549, 26.46612)
BLACK_DELTA = 2.85
WHITE_DELTA = 5
OPERATIONAL_DELAY = 0.1


if __name__ == "__main__":

    if not len(sys.argv) == 4:
        print_log(f'Usage: python {sys.argv[0]} file_name Z1 Z2')
        sys.exit()


    _fileName = sys.argv[1]

    file_re = re.compile(r'(\s*[A-Za-z0-9_]+(\.[A-Za-z]{3})?\s*$)')
    if not file_re.match(_fileName):
        print_log(f'{_fileName} is invalid file name. Usage: python {sys.argv[0]} file_name Z1 Z2')
        sys.exit()

    # if not os.path.isfile(_fileName):
    #     print_log(f'File {_fileName} not exist. Usage: python {sys.argv[0]} file_name Z1 Z2')
    #     sys.exit()

    if not is_float(sys.argv[2]):
        print_log(f'Coordinate Z1 has incorrect value. Usage: python {sys.argv[0]} file_name Z1 Z2')
        sys.exit()
    
    if not is_float(sys.argv[3]):
        print_log(f'Coordinate Z2 has incorrect value. Usage: python {sys.argv[0]} file_name Z1 Z2')
        sys.exit()

    _script = list()

    _script.append("10000:  SetJointVel(25)")
    _script.append("10001: SetVacuumPurgeDuration(0.5)")
    _script.append("10002:  MoveJoints(0, 0, 0, 0)")

    (_x, _y, _z, _t) = BLACK_start

    if not (float(sys.argv[2]) == 0):
        _z = float(sys.argv[2])
    
    (_xE, _yE, _zE, _tE) = WHITE_start
    if not (float(sys.argv[3]) == 0):
        _zE = float(sys.argv[3])


    _line = 1
    for it in range(10):
        _script.append(f"# Start forward iteration # {it}")
        _script.append(f"{it*100+1}:    MoveJoints(-125.63881, -0.0047, -0.00347, 0.07769 )")
        _script.append(f"{it*100+2}:    MovePose({_x}, {_y+it*BLACK_DELTA:.5f}, 80, {_t})")
        _script.append(f"{it*100+11}:   Delay({OPERATIONAL_DELAY})")
        _script.append(f"{it*100+3}:    MovePose({_x}, {_y+it*BLACK_DELTA:.5f}, {_z}, {_t})")
        _script.append(f"{it*100+13}:   Delay({OPERATIONAL_DELAY})")
        _script.append(f"{it*100+4}:    VacuumGrip")
        _script.append(f"{it*100+15}:   Delay({OPERATIONAL_DELAY})")
        _script.append(f"{it*100+5}:    MovePose({_x}, {_y+it*BLACK_DELTA:.5f}, 80, {_t})")
        _script.append(f"{it*100+17}:   Delay({OPERATIONAL_DELAY})")
        _script.append(f"{it*100+6}:    MoveJoints(-31.42492, 48.57322, -29, 9.31781)")
        _script.append(f"{it*100+7}:    MovePose({_xE}, {_yE+it*WHITE_DELTA:.5f}, 80, {_tE})")
        _script.append(f"{it*100+19}:   Delay({OPERATIONAL_DELAY})")
        _script.append(f"{it*100+8}:    MovePose({_xE}, {_yE+it*WHITE_DELTA:.5f}, {_zE}, {_tE})")
        _script.append(f"{it*100+21}:   Delay({OPERATIONAL_DELAY*3})")
        _script.append(f"{it*100+9}:    VacuumRelease")
        _script.append(f"{it*100+23}:   Delay({OPERATIONAL_DELAY})")
        _script.append(f"{it*100+10}:    MovePose({_xE}, {_yE+it*WHITE_DELTA:.5f}, 80, {_tE})")
        _script.append(f"# End forward iteration # {it}")
 
    _script.append(f"{it*100+30}:   Delay({OPERATIONAL_DELAY*3})")


    for it in range(10):
        _script.append(f"# Start forward iteration # {it}")
        if not it == 0:
            _script.append(f"{it*100+31}:    MoveJoints(-125.63881, -0.0047, -0.00347, 0.07769 )")
        _script.append(f"{it*100+32}:    MovePose({_xE}, {_yE+it*WHITE_DELTA:.5f}, 80, {_tE})")
        _script.append(f"{it*100+41}:   Delay({OPERATIONAL_DELAY})")
        _script.append(f"{it*100+33}:    MovePose({_xE}, {_yE+it*WHITE_DELTA:.5f}, {_zE}, {_tE})")
        _script.append(f"{it*100+43}:   Delay({OPERATIONAL_DELAY})")
        _script.append(f"{it*100+34}:    VacuumGrip")
        _script.append(f"{it*100+45}:   Delay({OPERATIONAL_DELAY})")
        _script.append(f"{it*100+35}:    MovePose({_xE}, {_yE+it*WHITE_DELTA:.5f}, 80, {_tE})")
        _script.append(f"{it*100+47}:   Delay({OPERATIONAL_DELAY})")
        _script.append(f"{it*100+36}:    MoveJoints(-31.42492, 48.57322, -29, 9.31781)")
        _script.append(f"{it*100+37}:    MovePose({_x}, {_y+it*BLACK_DELTA:.5f}, 80, {_t})")
        _script.append(f"{it*100+49}:   Delay({OPERATIONAL_DELAY})")
        _script.append(f"{it*100+38}:    MovePose({_x}, {_y+it*BLACK_DELTA:.5f}, {_z}, {_t})")
        _script.append(f"{it*100+51}:   Delay({OPERATIONAL_DELAY*3})")
        _script.append(f"{it*100+39}:    VacuumRelease")
        _script.append(f"{it*100+53}:   Delay({OPERATIONAL_DELAY})")
        _script.append(f"{it*100+40}:    MovePose({_x}, {_y+it*BLACK_DELTA:.5f}, 80, {_t})")
        _script.append(f"# End forward iteration # {it}")

    _script.append("10003:  MoveJoints(0, 0, 0, 0)")

    _f = open(_fileName, "w")
    for _str in _script:
        print_log(f'{_str}\n')
        _f.write(f'{_str}\n')
    
    
    
    
    
    
    _f.close()






