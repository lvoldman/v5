__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["Leonid Voldman","Sasha Schechtman"]
__license__ = "SLA"
__version__ = "2.0.1"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Prototype"


from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, \
    real_num_validator, int_num_validator, real_validator, int_validator
from bs1_mecademic import _mcs500_pos 
from bs1_cognex_modbus import statisticData
from threading import Thread
import os, sys, time, re
from queue import Queue 
from enum import Enum
from bs1_DH_RB_modbus import MAX_DH_ANGLE
from Motors_Control_Dashboard import SetLED

from typing import TYPE_CHECKING

from bs2_config import DevType

tSync = Enum("tSync", ["start", "stop", "cont"])

# MAX_DH_ANGLE = 65536


class StatusMonitor:
    def __init__(self, window, devs_list, statusPub, timeout = 0.1):
        self.__devs_list = devs_list
        self.__statusPub = statusPub
        self.__window = window
        # self.__timeout = timeout
        # self.__syncQ:Queue = Queue()
        # print_log(f'Starting Status Monitor')
        # self.__thread_id = Thread(target = self.__monitorThread)          # pySImpleGUI package fails process multiply events
        # self.__thread_id.start()
        pass 

    # def __monitorThread(self):
    #     cmd:tSync = tSync.cont 
    
    #     while not (cmd == tSync.stop):
    #         if not self.__syncQ.empty():
    #             cmd = self.__syncQ.get()
    #         self.__window.write_event_value(f'-UPDATE_STATUS-', self)
    #         # self.__monitorUpdate()
    #         time.sleep(self.__timeout)

    #     print_log(f'Status monitor terminated')

    def monitorUpdate(self, realTime = False):

        statusData = dict() 
        new_pos = 0
    
        for m_dev in  self.__devs_list:

            if m_dev.C_type == DevType.TROLLEY or m_dev.C_type == DevType.GRIPPER \
                    or m_dev.C_type == DevType.GRIPPERv3 \
                    or m_dev.C_type == DevType.DIST_ROTATOR or m_dev.C_type == DevType.TIME_ROTATOR \
                    or m_dev.C_type == DevType.DH :
                
                # if realTime and not (m_dev.C_type == DevType.DH) :
                if realTime :
                    new_pos = m_dev.dev_mDC.mDev_get_cur_pos()
                else:
                    new_pos = m_dev.dev_mDC.mDev_stored_pos()

                otf_cur = m_dev.dev_mDC.el_current_on_the_fly
                if m_dev.C_type == DevType.TROLLEY:
                    self.__window[f'-{m_dev.c_gui}-TROLLEY_POSSITION-'].update(value = new_pos)
                    self.__window[f'-{m_dev.c_gui}-TROLLEY_CUR_DISPLAY-'].update(value = otf_cur)

                elif m_dev.C_type == DevType.DH:
                    # if int(new_pos) > (MAX_DH_ANGLE/2):
                    #     new_pos = new_pos - MAX_DH_ANGLE
                    self.__window[f'-{m_dev.c_gui}-DH_GRIPPER_POSSITION-'].update(value = new_pos)    

                

                elif m_dev.C_type == DevType.GRIPPERv3:
                    self.__window[f'-{m_dev.c_gui}-GRIPPER_POSSITION-'].update(value = new_pos)
                    self.__window[f'-{m_dev.c_gui}-GRIPPER_CUR_DISPLAY-'].update(value = otf_cur)
                elif m_dev.C_type == DevType.GRIPPER:
                    self.__window[f'-{m_dev.c_gui}-GRIPPER_CUR_DISPLAY-'].update(value = otf_cur)
                elif m_dev.C_type == DevType.DIST_ROTATOR:
                    self.__window[f'-{m_dev.c_gui}-DIST_ROTATOR_POSSITION-'].update(value = new_pos)
                    self.__window[f'-{m_dev.c_gui}-DIST_ROTATOR_CUR_DISPLAY-'].update(value = otf_cur)
                elif m_dev.C_type == 'DevType.TIME_ROTATOR':
                    # self.__window[f'-{m_dev.c_gui}-TIME_ROTATOR_CUR_DISPLAY-'].update(value = otf_cur)
                    self.__window[f'-TIME_ROTATOR_CUR_DISPLAY-'].update(value = otf_cur)
            elif m_dev.C_type == DevType.HMP:
                _ave = m_dev.dev_hmp.getAve()
                self.__window[f'-HMP-RES-'].update(value = _ave)
                self.__window[f'-HMP-CURR-RES-'].update(value = m_dev.dev_hmp.getCurrRT())
                self.__window[f'-HMP-VOLT-RES-'].update(value = m_dev.dev_hmp.getVoltRT())


            elif m_dev.C_type == DevType.ZABER:
                
                if realTime:
                    new_pos = m_dev.dev_zaber.GetPos()
                else:
                    new_pos = m_dev.dev_zaber.readStoredPos()
                    
                self.__window[f'-{m_dev.c_gui:02d}-ZABER-POSSITION-'].update(value = new_pos)
            
            elif  DevType.MARCO == m_dev.C_type:
                self.__window[f'-MARCO_ACTUAL_TEMP-'].update(value = m_dev.dev_marco.get_temp())  
                self.__window[f'-MARCO_PULSE_COUNT_AFTER_RESET-'].update(value = m_dev.dev_marco.get_pulse_count_since_last_reset())  
                # self.__window[f'-MARCO_PROGRAM-'].update(value = m_dev.dev_marco.progNum)  
                # self.__window[f'-MARCO_PROGRAM-'].update(value = 'Prog '+ str(m_dev.dev_marco.program_control()))
                self.__window[f'-MARCO_PROGRAM-'].update(value = f'Prog {m_dev.dev_marco.progNum}')  
                if not m_dev.dev_marco.shoting_status:
                    self.__window[f'-MARCO_SINGLE_SHOT_OFF-'].update(button_color='tomato on red')
                    self.__window[f'-MARCO_SINGLE_SHOT_ON-'].update(button_color='white on green')
            
            elif DevType.MCDMC == m_dev.C_type:
                if m_dev.dev_mcdmc.isCognexOnline:
                    SetLED (self.__window, '_cognex_', 'green')
                else:
                    SetLED (self.__window, '_cognex_', 'red')

                if m_dev.dev_mcdmc.isActive:
                    SetLED (self.__window, '_mcdmc_active_', 'green')
                else:
                    SetLED (self.__window, '_mcdmc_active_', 'red')

                _r_pos:_mcs500_pos =  m_dev.dev_mcdmc.getPos
                self.__window[f'-MCDMC_POS_X-'].update(value = _r_pos.x)
                self.__window[f'-MCDMC_POS_Y-'].update(value = _r_pos.y)
                self.__window[f'-MCDMC_POS_Z-'].update(value = _r_pos.z)
                self.__window[f'-MCDMC_POS_GAMMA-'].update(value = _r_pos.gamma)
                _info:statisticData = m_dev.dev_mcdmc.statisticINFO   
                self.__window[f'-CGNX_VALID_PRODUCT-'].update(value = _info.valid_products)
                self.__window[f'-CGNX_WRONG_PRODUCT-'].update(value = _info.wrong_products)
                self.__window[f'-CGNX_UP_SIDE_DOWN-'].update(value = _info.up_side_down)
                self.__window[f'-CGNX_OUT_OF_RANGE-'].update(value = _info.out_of_range)
            
            elif (DevType.DB  == m_dev.C_type) and ('-CGNX_PROCEEDED_PRODUCT-' in self.__window.AllKeysDict):
                self.__window[f'-CGNX_PROCEEDED_PRODUCT-'].update(value = m_dev.dev_DB.success_counter)
            
            elif (DevType.JTSE == m_dev.C_type) and ('-JBC_TEMP-' in self.__window.AllKeysDict): 
                self.__window[f'-JBC_TEMP-'].update(value = m_dev.dev_jtse.RAT)
                
            
                
            statusData = self.__updatePayloadStatus(m_dev, statusData, new_pos)    # update status for external apps

        self.__statusPub.publishJsonMsg(statusData)


    def __updatePayloadStatus(self, m_dev, statusData, pos):
        if m_dev.C_type == DevType.TROLLEY:
            statusData[f'T{m_dev.c_gui}'] = dict()
            statusData[f'T{m_dev.c_gui}']["encoder"] = pos
        elif m_dev.C_type == DevType.GRIPPER:
            statusData[f'G{m_dev.c_gui}'] = dict()
            statusData[f'G{m_dev.c_gui}']["encoder"] = 0 if m_dev.dev_mDC.gripper_onof else 1
        elif m_dev.C_type == DevType.GRIPPERv3:
            statusData[f'G{m_dev.c_gui}'] = dict()
            statusData[f'G{m_dev.c_gui}']["encoder"] = pos
        elif m_dev.C_type == DevType.DIST_ROTATOR:
            statusData[f'R{m_dev.c_gui}'] = dict()
            statusData[f'R{m_dev.c_gui}']["encoder"] = pos
        elif m_dev.C_type == 'DevType.TIME_ROTATOR':
            pass
        elif m_dev.C_type == DevType.ZABER:
            statusData[f'Z{m_dev.c_gui}'] = dict()
            statusData[f'Z{m_dev.c_gui}']["encoder"] = pos

        return statusData

    def __del__(self):
        # if self.__thread_id.is_alive():
        #     self.__syncQ.put(tSync.stop)
        
        # self.__thread_id.join()
        pass

        
    def StopMonitor(self):
        print_log(f'Stoping monitor')
        # self.__syncQ.put(tSync.stop)

