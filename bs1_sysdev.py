__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "5.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Tool"


from queue import Queue 
from bs1_base_motor import BaseDev


from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, num2binstr, set_parm, get_parm, globalEventQ, smartLocker



class sysDevice(BaseDev):
    
    def __init__(self):
        super().__init__('SYS')

    def devQuery(self, query:str, timeout:float=0)-> str:
                                                        # no query avilable for system device
        return ''
    
    def operateDevice(self, command:str)-> tuple[bool, bool]:
        pass
        return (False, True)                        #  no blocking, result = True

    def play_media(media_file:str)->bool:
        try:
            pass

        except Exception as ex:
            exptTrace(ex)
            return False

        return True


    def  mDev_stop(self)->bool:                 # for compatability
        return True


#------------------------   UNITEST SECTION -----------------
if __name__ == "__main__":


    def test():
        try:
            pass
        except Exception as ex:
            print(f'Exception : {ex}')
            
    print("Running bs1_sysdev.py unitest ...")
    test()

     
