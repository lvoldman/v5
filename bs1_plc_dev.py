__author__ = "Leonid Voldman"
__created_on__ = "2025-10-06"  
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "1.0.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Tool"

from threading import Lock
import threading

from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, \
                                        s32, num2binstr, set_parm, get_parm, void_f

from bs1_base_motor import BaseMotor, BaseDev


class PLCDev(BaseDev):
    def __init__(self, dev_name:str, devAPI:dict, devINFO:dict):
        super().__init__(dev_name=dev_name)
        self.__devAPI: dict = devAPI
        self.__devINFO:dict = devINFO
        
