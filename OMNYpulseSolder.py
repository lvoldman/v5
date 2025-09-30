__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["Leonid Voldman","Elad Diukman"]
__license__ = "SLA"
__version__ = "2.0.1"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Prototype"

from operator import pos, truediv
from re import T
from weakref import finalize
from threading import Condition, Thread, Lock
import PySimpleGUI as sg
from typing import List
from queue import Queue 



import bs1_proc_manager as pm
import bs1_status_monitor as sm 


from bs1_config import port_scan, CDev, gif103, read_params, get_dev
from bs1_faulhaber import FH_Motor
from bs1_zaber import Zaber_Motor
from bs1_script import Tbl, Log_tbl, groups, GetSubScript, LoadScriptFile, ClearLog, SaveLog, GetSubScriptDict
from bs1_ni6002 import NI6002
from bs1_cam_modbus import Cam_modbus, camRes
from bs1_FHv3 import FH_Motor_v3
from bs1_maxon import MAXON_Motor
from bs1_anim_0MQ import anim_0MQ


import os, sys, time, re
import math

from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, real_num_validator, \
    int_num_validator, real_validator, int_validator, file_name_validator, void_f, clearQ, removeElementQ, event2GUI

R1_GEAR = 64
R2_GEAR = 64
R1_PITCH = 1.27
R2_DIAMETER = 6


theme_dict = {'BACKGROUND': '#2B475D',
                'TEXT': '#FFFFFF',
                'INPUT': '#F2EFE8',
                'TEXT_INPUT': '#000000',
                'SCROLL': '#F2EFE8',
                'BUTTON': ('#000000', '#C2D4D8'),
                'PROGRESS': ('#FFFFFF', '#C7D5E0'),
                'BORDER': 0,'SLIDER_DEPTH': 0, 'PROGRESS_DEPTH': 0}

sg.theme_add_new('Dashboard', theme_dict)
sg.theme('Dashboard')

BORDER_COLOR = '#C7D5E0'
DARK_HEADER_COLOR = '#1B2838'
BPAD_TOP = ((20,20), (20, 10))
BPAD_LEFT = ((20,10), (0, 0))
BPAD_LEFT_INSIDE = (0, (10, 0))
BPAD_RIGHT = ((10,20), (10, 0))

image_left = './Images/button_left_c.png'
image_right = './Images/button_right_c.png'



# ====   GUI configuration
zaber_motors = 2
gripper_motors = 2
distance_rotators = 2


trolley_motors = 0
time_rotators = 0
dh_grippers = 0


# ==== Global vars
emergency_stop_pressed = False
calibration_in_process = False
step_in = False                                 # used to sync non motion devices (DELAY, CALIBR, etc)
event_procedure_in_progress = False

ZMQ_PORT = 6001


CUSTOM_MENU_RIGHT_CLICK_VER_LOC_EXIT = ['', ['Version', 'File Location', 'Exit']]


#================  GUI utils start here =======================
def activateScriptControl(window, disableInd = False):
    print_log(f'ScriptControl: activation = {"ON" if not disableInd else "OFF"}')
    window['Step'].update(disabled = disableInd)
    window['Open File'].update(disabled = disableInd)
    window['Run'].update(disabled = disableInd)
    
def deActivateScriptControl(window):
    activateScriptControl(window = window, disableInd = True)

def activationControl (window, type, disableInd = False, ledColor = 'green', index = None):
    if type == '--TIME_ROTATOR--' and '_time_rotator_' in window.AllKeysDict: 
        SetLED (window, f'_time_rotator_', ledColor)
        window[f'-TIME_ROTATOR_LEFT-'].update(disabled = disableInd)
        window[f'-TIME_ROTATOR_RIGHT-'].update(disabled = disableInd)
        window[f'-TIME_ROTATOR_TARGET-'].update(disabled = disableInd)
        window[f'-TIME_ROTATOR_VELOCITY-'].update(disabled = disableInd)
        window[f'-TIME_ROTATOR-CURR-'].update(disabled = disableInd)
    elif type == '--DIST_ROTATOR--' and f'_dist_rotator_{index}_' in window.AllKeysDict:
        SetLED (window, f'_dist_rotator_{index}_', ledColor)
        window[f'-{index}-DIST_ROTATOR_POS_SET-'].update(disabled = disableInd)
        window[f'-{index}-DIST_ROTATOR_TARGET_RESET-'].update(disabled = disableInd)
        window[f'-{index}-DIST_ROTATOR_LEFT-'].update(disabled = disableInd)
        window[f'-{index}-DIST_ROTATOR_RIGHT-'].update(disabled = disableInd)
        window[f'-{index}-DIST_ROTATOR_TARGET-'].update(disabled = disableInd)
        window[f'-{index}-DIST_ROTATOR_VELOCITY-'].update(disabled = disableInd)
        window[f'-{index}-DIST_ROTATOR-CURR-'].update(disabled = disableInd)
    elif type == '--TROLLEY--' and f'_trolley_{index}_' in window.AllKeysDict:
        SetLED (window, f'_trolley_{index}_', ledColor)
        window[f'-{index}-TROLLEY_POS_SET-'].update(disabled = disableInd)
        window[f'-{index}-TROLLEY_TARGET_RESET-'].update(disabled = disableInd)
        window[f'-{index}-TROLLEY_LEFT-'].update(disabled = disableInd)
        window[f'-{index}-TROLLEY_RIGHT-'].update(disabled = disableInd)
        window[f'-{index}-TROLLEY_TARGET-'].update(disabled = disableInd)
        window[f'-{index}-TROLLEY_VELOCITY-'].update(disabled = disableInd)
        window[f'-{index}-TROLLEY-CURR-'].update(disabled = disableInd)
    elif type == '--GRIPPER--' and f'_gripper_{index}_' in window.AllKeysDict:                                 # no more used for activation/deactivation. use GRIPPERv3 instead
        SetLED (window, f'_gripper_{index}_', ledColor)
        window[f'-{index}-GRIPPER-RPM-'].update(disabled = disableInd)
        window[f'-{index}-GRIPPER-CURR-'].update(disabled = disableInd)
        window[f'-{index}-GRIPPER-OFF-'].update(disabled = disableInd)
        window[f'-{index}-GRIPPER-ON-'].update(disabled = disableInd)
    elif type == '--GRIPPERv3--' and f'_gripper_{index}_' in window.AllKeysDict:  
        SetLED (window, f'_gripper_{index}_', ledColor)
        window[f'-{index}-GRIPPER_POS_SET-'].update(disabled = disableInd)
        window[f'-{index}-GRIPPER_TARGET_RESET-'].update(disabled = disableInd)
        window[f'-{index}-GRIPPER_TARGET-'].update(disabled = disableInd)
        window[f'-{index}-GRIPPER-RPM-'].update(disabled = disableInd)
        window[f'-{index}-GRIPPER-CURR-'].update(disabled = disableInd)
        window[f'-{index}-GRIPPER-OFF-'].update(disabled = disableInd)
        window[f'-{index}-GRIPPER-ON-'].update(disabled = disableInd)
    elif type == '--ZABER--' and f'_zaber_{index}_' in window.AllKeysDict:  
        SetLED (window, f'_zaber_{index}_', ledColor)
        window[f'-{index:02d}-ZABER-GTD-'].update(disabled = disableInd)
        window[f'-{index:02d}-ZABER-GH-'].update(disabled = disableInd)
        window[f'-{index:02d}-ZABER-DST-'].update(disabled = disableInd)
        window[f'-{index:02d}-ZABER-VELOCITY-'].update(disabled = disableInd)
    elif type == '--DAQ-NI--'  and f'_trigger_' in window.AllKeysDict:  
        SetLED (window, f'_trigger_', ledColor)
        window[f'TRIGGER'].update(disabled = disableInd)
    elif type == '--DAQ-NI--'  and f'_hotair_' in window.AllKeysDict:  
        SetLED (window, f'_hotair_', ledColor)
        window[f'HOTAIR-START'].update(disabled = disableInd)
        window[f'HOTAIR-STOP'].update(disabled = disableInd)
    elif type == '--CAM--' and f'_calibr_' in window.AllKeysDict:
        SetLED (window, f'_calibr_', ledColor)
        window[f'-CALIBRATE-'].update(disabled = disableInd)
        window[f'-CALIBR_PROFILE-'].update(disabled = disableInd)
    elif type == '--HMP--' and f'_hmp_' in window.AllKeysDict:
        SetLED (window, f'_hmp_', ledColor)
        window[f'-HMP-CURRENT-'].update(disabled = disableInd)
        window[f'-HMP-VOLTAGE-'].update(disabled = disableInd)
        window[f'-HMP-RES-CALC-'].update(disabled = disableInd)
        window[f'-HMP-CURR-SET-'].update(disabled = disableInd)
        window[f'-HMP-VOLT-SET-'].update(disabled = disableInd)

    elif type == '--DH--'  and f'_dh_gripper_{index}_' in window.AllKeysDict:  
        SetLED (window, f'_dh_gripper_{index}_', ledColor)
        window[f'-{index}-DH_GRIPPER_POS_SET-'].update(disabled = disableInd)
        window[f'-{index}-DH_GRIPPER_TARGET_RESET-'].update(disabled = disableInd)
        window[f'-{index}-DH_GRIPPER_TARGET-'].update(disabled = disableInd)
        window[f'-{index}-DH-GRIPPER-RPM-'].update(disabled = disableInd)
        window[f'-{index}-DH-GRIPPER-OFF-'].update(disabled = disableInd)
        window[f'-{index}-DH-GRIPPER-ON-'].update(disabled = disableInd)

    else:
        print_err(f'PANIC - wrong type {type}')



def ActivateMotorControl(window, type, index = None):
    activationControl(window, type,  False, 'green', index)

def DeActivateMotorControl(window, type, index = None):
    activationControl(window, type,  True, 'red', index)
    pass    


    

def initGUIDevs(window, devs_list):
    for m_dev in devs_list:
        if m_dev.C_type == '--TROLLEY--':
            ActivateMotorControl(window, '--TROLLEY--', m_dev.c_gui)
            window[f'-{m_dev.c_gui}-TROLLEY_POSSITION-'].update(value = m_dev.dev_mDC.mDev_pos)
            window[f'-{m_dev.c_gui}-TROLLEY_VELOCITY-'].update(value = m_dev.dev_mDC.rpm)
            window[f'-{m_dev.c_gui}-TROLLEY-CURR-'].update(value = m_dev.dev_mDC.el_current_limit)
        elif m_dev.C_type == '--DIST_ROTATOR--':
            ActivateMotorControl(window, '--DIST_ROTATOR--', m_dev.c_gui)
            window[f'-{m_dev.c_gui}-DIST_ROTATOR_POSSITION-'].update(value = m_dev.dev_mDC.mDev_pos)
            window[f'-{m_dev.c_gui}-DIST_ROTATOR_VELOCITY-'].update(value = m_dev.dev_mDC.rpm)
            window[f'-{m_dev.c_gui}-DIST_ROTATOR-CURR-'].update(value = m_dev.dev_mDC.el_current_limit)
        elif m_dev.C_type == '--TIME_ROTATOR--':
            ActivateMotorControl(window, '--TIME_ROTATOR--', m_dev.c_gui)
            window[f'-TIME_ROTATOR_TARGET-'].update(value = m_dev.dev_mDC.rotationTime)
            window[f'-TIME_ROTATOR_VELOCITY-'].update(value = m_dev.dev_mDC.el_voltage)
            window[f'-TIME_ROTATOR-CURR-'].update(value = m_dev.dev_mDC.el_current_limit)
        elif m_dev.C_type == '--GRIPPER--':
            ActivateMotorControl(window, f'--GRIPPER--', m_dev.c_gui)
            window[f'-{m_dev.c_gui}-GRIPPER-RPM-'].update(value = m_dev.dev_mDC.el_voltage)
            window[f'-{m_dev.c_gui}-GRIPPER-CURR-'].update(value = m_dev.dev_mDC.el_current_limit)
        elif m_dev.C_type == '--GRIPPERv3--':
            ActivateMotorControl(window, f'--GRIPPERv3--', m_dev.c_gui)
            window[f'-{m_dev.c_gui}-GRIPPER-RPM-'].update(value = m_dev.dev_mDC.rpm)
            window[f'-{m_dev.c_gui}-GRIPPER-CURR-'].update(value = m_dev.dev_mDC.el_current_limit)
        elif m_dev.C_type == '--ZABER--':
            window[f'-{m_dev.c_gui:02d}-ZABER-POSSITION-'].update(value = m_dev.dev_zaber.current_pos)
            window[f'-{m_dev.c_gui:02d}-ZABER-VELOCITY-'].update(value = m_dev.dev_zaber.velocity_in_percents)
            ActivateMotorControl(window, f'--ZABER--', m_dev.c_gui)
        elif m_dev.C_type == '--DAQ-NI--':
            ActivateMotorControl(window, f'--DAQ-NI--')
        elif m_dev.C_type == '--CAM--':
            window[f'-CALIBR_PROFILE-'].update(value = 'base')
            ActivateMotorControl(window, f'--CAM--')
        elif m_dev.C_type == '--HMP--':
            ActivateMotorControl(window, f'--HMP--' )
        elif m_dev.C_type == '--DH--':
            ActivateMotorControl(window, f'--DH--', m_dev.c_gui)
            window[f'-{m_dev.c_gui}-DH-GRIPPER-RPM-'].update(value = m_dev.dev_mDC.DevOpSPEED)
        else:
            print_err(f'ERROR PANIC - wrong device {m_dev.C_type} in the list at {m_dev.c_gui} position')

    # SetLED (window, '_calibr_', 'green')
            
    # window[f'-1-ZABER-TITLE-'].update('שלום')
    # window[f'-2-ZABER-TITLE-'].update('עליכם')
    # window[f'-1-GRIPPER-TITLE-'].update('מלכי')
    # window[f'-2-GRIPPER-TITLE-'].update('השרת')
    # window[f'-1-DIST_ROTATOR-TITLE-'].update('מלכי')
    # window[f'-2-DIST_ROTATOR-TITLE-'].update('עליון')

def LEDIndicator(key=None, radius=30):
    return sg.Graph(canvas_size=(radius, radius),
             graph_bottom_left=(-radius, -radius),
             graph_top_right=(radius, radius),
             pad=(0, 0), key=key)

def SetLED(window, key, color):
    graph = window[key]
    graph.erase()
    graph.draw_circle((0, 0), 12, fill_color=color, line_color=color)


def gripper_block(i):
    return (
            [[sg.Text(f'Gripper {i} (G{i})', font='Any 10', key = f'-{i}-GRIPPER-TITLE-'), LEDIndicator(f'_gripper_{i}_'), sg.Text('On/Off'), \
              sg.Button(button_text ='On', size=(2, 1), button_color='white on green', key=f'-{i}-GRIPPER-ON-', 
                        disabled_button_color = 'light slate gray on navajo white'),
              sg.Button(button_text ='Off', size=(2, 1), button_color='white on red', key=f'-{i}-GRIPPER-OFF-', 
                        disabled_button_color = 'light slate gray on navajo white')],
            [ sg.Text(f'Velocity (mV/rpm) ->>', key = f'-{i}-VOLT-RPM-'), sg.Input(size=(6, 1), enable_events=True, key=f'-{i}-GRIPPER-RPM-')], 
            #  [sg.Text(f'Stop (mA)'), sg.Input(size=(3, 1), enable_events=True, key=f'-{i}-GRIPPER-CURR-')],
             [sg.Button(button_text = "Reset", key=f'-{i}-GRIPPER_TARGET_RESET-'), 
              sg.Text("_", size=(8, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', border_width = 2, key=f'-{i}-GRIPPER_POSSITION-'),
             sg.Input(size=(10, 1), enable_events=True, key=f'-{i}-GRIPPER_TARGET-', \
            font=('Arial Bold', 8), justification='left'), sg.Button(button_text = "Go", key=f'-{i}-GRIPPER_POS_SET-')],
            [sg.Text("_", size=(4, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
            border_width = 2, key=f'-{i}-GRIPPER_CUR_DISPLAY-'),
            sg.Text(f'Stop (mA) >\n< Curr. (mA)'), sg.Input(size=(4, 1), enable_events=True, key=f'-{i}-GRIPPER-CURR-'), sg.Button(button_text = 'Stop',  key=f'-{i}-GRIPPER_STOP-')],
             ]

    )


def dist_rotators_block(i):
    return ([
        [sg.Push(), sg.Text(f'Distance Rotator {i} ({i}8)', font='Any 10', key = f'-{i}-DIST-ROTATOR-TITLE-'), LEDIndicator(f'_dist_rotator_{i}_'), sg.Push()],
        [sg.T('Position'), sg.Text("_", size=(8, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
            border_width = 2, key=f'-{i}-DIST_ROTATOR_POSSITION-'), sg.Button(button_text = "Reset", key=f'-{i}-DIST_ROTATOR_TARGET_RESET-') ],
        [sg.Text('Target'), sg.Input(size=(10, 1), enable_events=True, key=f'-{i}-DIST_ROTATOR_TARGET-', \
            font=('Arial Bold', 8), justification='left'), sg.Button(button_text = "Set & Go", key=f'-{i}-DIST_ROTATOR_POS_SET-')],
        [sg.Button( button_color=sg.TRANSPARENT_BUTTON, image_filename = image_left, image_size=(22, 24), \
            image_subsample=2, border_width=0, key=f'-{i}-DIST_ROTATOR_LEFT-'),
            sg.Frame('',[[sg.Text('Velocity (RPM)')], [sg.Input(size=(15, 1), enable_events=True, key=f'-{i}-DIST_ROTATOR_VELOCITY-', \
                font=('Arial Bold', 8), justification='left')]], border_width=0),
        sg.Button( button_color=sg.TRANSPARENT_BUTTON, image_filename = image_right, image_size=(22, 24), \
            image_subsample=2, border_width=0, key=f'-{i}-DIST_ROTATOR_RIGHT-')], 
        [sg.Text("_", size=(6, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
            border_width = 2, key=f'-{i}-DIST_ROTATOR_CUR_DISPLAY-'),
            sg.Text(f'Stop (mA) >\n< Curr. (mA)'), sg.Input(size=(8, 1), enable_events=True, key=f'-{i}-DIST_ROTATOR-CURR-')],
        [sg.Button(button_text = 'Stop/Release',  key=f'-{i}-DIST_ROTATOR_STOP-')]]

    )

def dh_grippers_block(i):
    return ([
        [sg.Text(f'DH Gripper {i} (D{i})', font='Any 10', key = f'-{i}-DH-GRIPPER-TITLE-'), LEDIndicator(f'_dh_gripper_{i}_'), sg.Text('On/Off'), \
              sg.Button(button_text ='On', size=(2, 1), button_color='white on green', key=f'-{i}-DH-GRIPPER-ON-', 
                        disabled_button_color = 'light slate gray on navajo white'),
              sg.Button(button_text ='Off', size=(2, 1), button_color='white on red', key=f'-{i}-DH-GRIPPER-OFF-', 
                        disabled_button_color = 'light slate gray on navajo white')],
        [ sg.Text(f'Velocity (0-100) %  ->>', key = f'-{i}-DH_VELOCITY-'), sg.Input(size=(6, 1), enable_events=True, key=f'-{i}-DH-GRIPPER-RPM-')], 
             [sg.Button(button_text = "Home", key=f'-{i}-DH_GRIPPER_TARGET_RESET-'), 
              sg.Text("_", size=(8, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', border_width = 2, key=f'-{i}-DH_GRIPPER_POSSITION-')],
             [sg.Text(f'Move (in degrees) ->>', key = f'-{i}-DH_VELOCITY-'), sg.Input(size=(10, 1), enable_events=True, key=f'-{i}-DH_GRIPPER_TARGET-', \
            font=('Arial Bold', 8), justification='left'), sg.Button(button_text = "Go", key=f'-{i}-DH_GRIPPER_POS_SET-')],
            [ sg.Button(button_text = 'Stop',  key=f'-{i}-DH_GRIPPER_STOP-')],
             ]

    )

def trolley_block(i):
    return ([
        [sg.Push(), sg.Text(f'Trolley {i}', font='Any 10', key = f'-{i}-TROLLEY-TITLE-'), LEDIndicator(f'_trolley_{i}_'), sg.Push()],
        [sg.T('Position'), sg.Text("_", size=(8, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
            border_width = 2, key=f'-{i}-TROLLEY_POSSITION-'), sg.Button(button_text = "Reset", key=f'-{i}-TROLLEY_TARGET_RESET-') ],
        [sg.Text('Target'), sg.Input(size=(10, 1), enable_events=True, key=f'-{i}-TROLLEY_TARGET-', \
            font=('Arial Bold', 8), justification='left'), sg.Button(button_text = "Set & Go", key=f'-{i}-TROLLEY_POS_SET-')],
        [sg.Button( button_color=sg.TRANSPARENT_BUTTON, image_filename = image_left, image_size=(22, 24), \
            image_subsample=2, border_width=0, key=f'-{i}-TROLLEY_LEFT-'),
            sg.Frame('',[[sg.Text('Velocity (RPM)')], [sg.Input(size=(15, 1), enable_events=True, key=f'-{i}-TROLLEY_VELOCITY-', \
                font=('Arial Bold', 8), justification='left')]], border_width=0),
        sg.Button( button_color=sg.TRANSPARENT_BUTTON, image_filename = image_right, image_size=(22, 24), \
            image_subsample=2, border_width=0, key=f'-{i}-TROLLEY_RIGHT-')], 
        [sg.Text("_", size=(6, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
            border_width = 2, key=f'-{i}-TROLLEY_CUR_DISPLAY-'),
            sg.Text(f'Stop (mA) >\n< Curr. (mA)'), sg.Input(size=(8, 1), enable_events=True, key=f'-{i}-TROLLEY-CURR-')],
        [sg.Button(button_text = 'Stop',  key=f'-{i}-TROLLEY_STOP-')]]

    )


def trolley_frames(line_s, line_e):
    return (
            [[sg.Frame('', trolley_block(i), border_width=3, \
                                    expand_x=True, expand_y=True, element_justification = "center")]  for i in range (line_s, line_e+1) ]
    )

def dist_rotators_frames(line_s, line_e):
    return (
            [[sg.Frame('', dist_rotators_block(i), border_width=3, \
                                    expand_x=True, expand_y=True, element_justification = "center")]  for i in range (line_s, line_e+1) ]
    )

def dh_grippers_frames(line_s, line_e):
    return (
            [[sg.Frame('', dh_grippers_block(i), border_width=3, \
                                    expand_x=True, expand_y=True, element_justification = "center")]  for i in range (line_s, line_e+1) ]
    )

def gripper_frames(line_s, line_e):
    return (
            [[sg.Frame('', gripper_block(i), border_width=3, \
                                    expand_x=True, expand_y=True, element_justification = "center")]  for i in range (line_s, line_e+1) ]
    )




def zaber_block(i):
    return (
            [[sg.Text(f'Zaber {i}', font='Any 10', key = f'-{i:02d}-ZABER-TITLE-'), LEDIndicator(f'_zaber_{i}_')],
            [ sg.T('Pos'), sg.Text("_", size=(8, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                border_width = 2, key=f'-{i:02d}-ZABER-POSSITION-'), 
            sg.Text(f'Dest'), sg.Input(size=(7, 1), enable_events=True, key=f'-{i:02d}-ZABER-DST-')],
            [sg.Button(button_text = 'Dest', key = f'-{i:02d}-ZABER-GTD-'), \
             sg.Button(button_text = 'Home', key = f'-{i:02d}-ZABER-GH-'),
             sg.T('Velocity'), sg.Input(size=(3, 1), enable_events=True, key=f'-{i:02d}-ZABER-VELOCITY-'), sg.T('%')]  
             ]

    )




def zaber_frames(line_s, line_e):
    return (
            [[sg.Frame('', zaber_block(i), border_width=2, \
                                    expand_x=True, expand_y=True, element_justification = "center")]  for i in range (line_s, line_e+1) ]
    )



#=========================================  Proc utils ========================

def LocateDevice (ev_name, devs_list) -> int:

    d_index = 0
    for m_dev in devs_list:
        if 'TROLLEY' in ev_name and m_dev.C_type == '--TROLLEY--':
            if   m_dev.c_gui == int (ev_name[1]):
                break
        if 'TIME_ROTATOR' in ev_name and m_dev.C_type == '--TIME_ROTATOR--':
            break
        elif 'GRIPPER' in ev_name and (m_dev.C_type == '--GRIPPER--' or m_dev.C_type == '--GRIPPERv3--'):
            if   m_dev.c_gui == int (ev_name[1]):
                break
        elif 'DIST_ROTATOR' in ev_name and m_dev.C_type == '--DIST_ROTATOR--' :
            if   m_dev.c_gui == int (ev_name[1]):
                break

        elif '-CALIBR' in ev_name and m_dev.C_type == '--CAM--' :
            break

        elif ('HOTAIR' in ev_name or 'TRIGGER' in  ev_name) and m_dev.C_type == '--DAQ-NI--':
            break

        elif ('HMP' in ev_name) and m_dev.C_type == '--HMP--':
            break

        elif  'ZABER' in ev_name and m_dev.C_type == '--ZABER--':
            if  m_dev.c_gui == int (ev_name[1:3]):
                print_DEBUG(f'Found device at d_index = {d_index} for event "{ev_name}"')
                break
        
        elif '-DH' in ev_name and (m_dev.C_type == '--DH--'):
            if   m_dev.c_gui == int (ev_name[1]):
                break

        d_index += 1

    if d_index >= len(devs_list):                                   # active device not found
            print_err(f'Non device assosiated event -> {ev_name}')
            return -1
        
    # print_log(f'Device located at index {d_index}')
    return (d_index)

def Correct_ZB_possition_after_unpark(window, devs_list):
    for m_dev in devs_list:
        if m_dev.C_type == '--ZABER--':
            new_pos = m_dev.dev_zaber.GetPos()
            window[f'-{m_dev.c_gui:02d}-ZABER-POSSITION-'].update(value = new_pos)


    

def TaskCloseProceedure(window:sg.Window, wTask:pm.WorkingTask):

    dev_list:List[CDev] = wTask.exploreDevs()
    print_log(f'{len(dev_list)} devices completed at task # {wTask.id()}')

    for dev in dev_list:
        if dev:
            ActivateMotorControl(window, dev.C_type, dev.c_gui)
    

def preTaskProceedure(window:sg.Window, wTask:pm.WorkingTask):

    dev_list:List[CDev] = wTask.exploreDevs()
    print_log(f'{len(dev_list)} devices will be operated at task # {wTask.id()}')

    for dev in dev_list:
        if dev:
            print_log(f'Deactivating {dev.get_device().devName}')
            DeActivateMotorControl(window, dev.C_type, dev.c_gui)
        else:
            print_log(f'Non device associated cmd')

# 
ScrRunLock = Lock()
def ScriptRunner(window:sg.Window, devs_list:List[CDev], selected_group:List[str], syncQ:Queue, retEvent:str, eventQ:Queue):

    if ScrRunLock.locked():
        print_err(f'-WARNING another script is still in progress. Trying to kill it')
        syncQ.put('-EMERGENCY_STOP-')
        time.sleep(0.5)
        if ScrRunLock.locked():
            print_err(f"-ERROR: failed eliminate another script that is still in progress. ")
            return
    
    ScrRunLock.acquire()

    clearQ(syncQ)
    gr_index = -1

    print_log (f'Selection: {selected_group}')

    if selected_group == []:
        print_err(f'Empty script')
        ScrRunLock.release()
        return
    
    

    if selected_group in groups:                 # probably block is not selected correctly, find correct group index
        gr_index = groups.index(selected_group)
    else:
        for gr_index, gr in enumerate(groups):
            if selected_group[0] in gr:
                break
            
    print_log (f'Starting script at group # {gr_index} of {len (groups)} groups')   

    clearQ(syncQ)

    while gr_index < len (groups):
        print_log(f'Running step # {gr_index} ')
        print_log(f'Group -> {groups[gr_index]}')
        # window.write_event_value(f'-SCRIPT_STEP-', groups[gr_index])
        # globalEventQ.stepNotificationQ.put(event2GUI(event='-SCRIPT_STEP-', value = groups[gr_index]))
        eventQ.put(event2GUI(event='-SCRIPT_STEP-', value = groups[gr_index]))
        
        print_log(f'Step # {gr_index} is waiting for completion')
        msg:str = syncQ.get()
        # syncQ.task_done()
        print_log(f'Script runner get msg = {msg}, Step # {gr_index}')

        if msg == '-EMERGENCY_STOP-':
            removeElementQ(eventQ, '-SCRIPT_STEP-')         # remove -SCRIPT_STEP- if still not proceeded 
            print_log(f'Emergency Stop. Group = {gr_index}')
            break
        elif msg == '-STEP_DONE-':
            print_log(f'Step # {gr_index} done')
            # window.write_event_value(f'-SCRIPT_STEP-', groups[gr_index])    
            gr_index += 1
        else:
            print_err(f'Unexpected msg = {msg}')

    window.write_event_value(retEvent, None)
    print_log(f'Script done')
    ScrRunLock.release()
    return



def  proc_tuning(window, devs_list, motor, dist)-> pm.WorkingTask:               # calibration
    
    wTask:pm.WorkingTask = None

    if motor[0] == 'Z':
        device_n = (int)(motor[1:])
        print_log(f'Zaber device = {device_n:02d} moving dist = {dist}')
        d_index = LocateDevice(f'-{device_n:02d}-ZABER-POSSITION-', devs_list)
        if d_index < 0:                     # No device
            print_err(f'No active Z device ({motor}) to execute cmd ')
            return wTask

        move_pos = (float)(dist)
        cur_pos = devs_list[d_index].dev_zaber.GetPos()
        new_pos = cur_pos + move_pos

        print_log(f'Calibration in process. Curr pos = {cur_pos}, new pos = {new_pos}')
        # window.write_event_value(f'-{device_n:02d}-ZABER-DST-', str(new_pos))
        # window.write_event_value(f'-{device_n:02d}-ZABER-GTD-', None)
        wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.go_to_dest, args=pm.argsType(position=new_pos)), sType = pm.RunType.single)
    elif motor[0] == 'R':
        device_n = (int)(motor[1:])
        print_log(f'Router device = {device_n} moving dist = {dist}')
        d_index = LocateDevice(f'-{device_n}-DIST_ROTATOR_TARGET-', devs_list)
        if d_index < 0:                     # No device
            print_err(f'No active R device to execute cmd ')
            return
        if dist == 0:
            # window.write_event_value(f'-{device_n}-DIST_ROTATOR_STOP-', None)
            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.stop), sType = pm.RunType.single)
        else:
            if motor == 'R1':
                new_dist = (dist * 4 * 1024 * R1_GEAR) / R1_PITCH
            elif motor == 'R2':
                new_dist = (dist * 4 * 1024 * R2_GEAR) / (R2_DIAMETER * math.pi)
            else:
                print_err(f'Unsupported dev = {motor}')

            move_pos = (float)(new_dist)
            cur_pos = devs_list[d_index].dev_mDC.mDev_get_cur_pos()
            new_pos = round(cur_pos + move_pos)


            print_log(f'Curr pos = {cur_pos}, new pos = {new_pos}')
            window[f'-{device_n}-DIST_ROTATOR_TARGET-'].update(str(new_pos))

            # window.write_event_value(f'-{device_n}-DIST_ROTATOR_POS_SET-', None)
            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.go_to_dest, args=pm.argsType(position=new_pos)), sType = pm.RunType.single)
    else:
         print_err(f'Unsupported dev = {motor}')
    
    return wTask

        



# def no_motion(devs_list):
#     global step_in
#     global event_procedure_in_progress
    
#     if step_in or event_procedure_in_progress:
#         return False

#     for m_dev in devs_list:
#         if m_dev.dev_zaber and m_dev.dev_zaber.z_in_motion  or   m_dev.dev_mDC and m_dev.dev_mDC.mDev_in_motion:
#             print_log(f'Device {m_dev.C_type} # {m_dev.c_gui} is still in motion. No script/log operation allowed')
#             return False

#     return True


#========================================= Utils end here =======================


# ===  Initiation GUI topology, etc
def InitGUI():
    top_banner = [
                [sg.Text('Dashboard', font='Any 12', background_color=DARK_HEADER_COLOR, enable_events=True, \
                            grab=False), sg.Push(background_color=DARK_HEADER_COLOR),
                sg.Text('© 2023', font='Any 12', background_color=DARK_HEADER_COLOR)],
                ]

    # trolley  = [[sg.Push(), sg.Text('Trolley', font='Any 10'), LEDIndicator('_trolley_'), sg.Push()],
    #             [sg.T('Position'), sg.Text("_", size=(8, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
    #                 border_width = 2, key='-TROLLEY_POSSITION-'), sg.Button(button_text = "Reset", key='-TROLLEY_TARGET_RESET-') ],
    #             [sg.Text('Target'), sg.Input(size=(10, 1), enable_events=True, key='-TROLLEY_TARGET-', \
    #                 font=('Arial Bold', 8), justification='left'), sg.Button(button_text = "Set & Go", key='-TROLLEY_POS_SET-')],
    #             [sg.Button( button_color=sg.TRANSPARENT_BUTTON, image_filename = image_left, image_size=(22, 24), \
    #                 image_subsample=2, border_width=0, key='-TROLLEY_LEFT-'),
    #                 sg.Frame('',[[sg.Text('Velocity (RPM)')], [sg.Input(size=(15, 1), enable_events=True, key='-TROLLEY_VELOCITY-', \
    #                     font=('Arial Bold', 8), justification='left')]], border_width=0),
    #             sg.Button( button_color=sg.TRANSPARENT_BUTTON, image_filename = image_right, image_size=(22, 24), \
    #                 image_subsample=2, border_width=0, key='-TROLLEY_RIGHT-')], 
    #             [sg.Text("_", size=(6, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
    #                 border_width = 2, key='-TROLLEY_CUR_DISPLAY-'),
    #                 sg.Text(f'Stop (mA) >\n< Curr. (mA)'), sg.Input(size=(8, 1), enable_events=True, key=f'-TROLLEY-CURR-')],
    #             [sg.Button(button_text = 'Stop',  key='-TROLLEY_STOP-')]]

    timeRotator = [[sg.Push(), sg.Text('Time Rotator (Spinner)', font='Any 10', key = f'-TIME-ROTATOR-TITLE-'), LEDIndicator('_time_rotator_'), sg.Push()],
                [sg.Text('Duration (in sec)'), sg.Input(size=(10, 1), enable_events=True, key='-TIME_ROTATOR_TARGET-', \
                    font=('Arial Bold', 8), justification='left')],
                [sg.Button( button_color=sg.TRANSPARENT_BUTTON, image_filename = image_left, image_size=(22, 24), \
                    image_subsample=2, border_width=0, key='-TIME_ROTATOR_LEFT-'),
                    sg.Frame('',[[sg.Text('Velocity (Volt)')], [sg.Input(size=(15, 1), enable_events=True, key='-TIME_ROTATOR_VELOCITY-', \
                        font=('Arial Bold', 8), justification='left')]], border_width=0),
                sg.Button( button_color=sg.TRANSPARENT_BUTTON, image_filename = image_right, image_size=(22, 24), \
                    image_subsample=2, border_width=0, key='-TIME_ROTATOR_RIGHT-')], 
                [sg.Text("_", size=(6, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                    border_width = 2, key='-TIME_ROTATOR_CUR_DISPLAY-'),
                    sg.Text(f'Stop (mA) >\n< Curr. (mA)'), sg.Input(size=(8, 1), enable_events=True, key=f'-TIME_ROTATOR-CURR-')],
                [sg.Button(button_text = 'Stop',  key='-TIME_ROTATOR_STOP-')]]



    script = [
                [sg.Text("Command script")],
                [sg.Button('Run'), sg.Button(button_text = 'Emergency Stop', button_color = 'white on firebrick4', border_width = 10, key='Stop')], 
                [Tbl],
                [sg.Button('Step'), sg.Button('Open File')
                ], 
                [sg.Text('Run = Run the whole script now')],
                [sg.Text('Stop = Stop runing the script')],
                [sg.Text('Step = Run selected command block')],
                [sg.Text('Open File = Load command script'), sg.Sizegrip()]
                ]
    
    
    measurement_block = [[sg.Push(), sg.Text('Measurement panel', font='Any 10'), LEDIndicator('_hmp_'), sg.Push()],
                [sg.Text('Current'), sg.Input(size=(10, 1), enable_events=True, key='-HMP-CURRENT-', \
                    font=('Arial Bold', 8), justification='left'), sg.Button(button_text = "Set", key='-HMP-CURR-SET-'), sg.Text("_", size=(8, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                    border_width = 2, key='-HMP-CURR-RES-')],   
                [sg.Text('Voltage'), sg.Input(size=(10, 1), enable_events=True, key='-HMP-VOLTAGE-', \
                    font=('Arial Bold', 8), justification='left'), sg.Button(button_text = "Set", key='-HMP-VOLT-SET-'), sg.Text("_", size=(8, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                    border_width = 2, key='-HMP-VOLT-RES-')],                       
                [sg.T('Resistance'), sg.Text("_", size=(8, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                    border_width = 2, key='-HMP-RES-'), sg.Button(button_text = "Calculate", key='-HMP-RES-CALC-') ]

                ]



    script_col=[]
    script_col.append([sg.Frame('', script, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                        border_width=1, vertical_alignment='down', element_justification = "center")])

    script_col.append([sg.Push(), sg.Button('Park', border_width = 3, button_color = 'red'), 
                    sg.Button('Unpark', border_width = 3, button_color = 'green'), sg.Push()])



    calibration = [[sg.Push(), sg.Button('Calibrate', border_width = 5, button_color = 'dodger blue', expand_x = True, expand_y = True, key  = '-CALIBRATE-',), 
                    sg.OptionMenu(values=('R1', 'R2'), default_value = 'R1',  key='-Z-SELECTOR-'),
                    LEDIndicator('_calibr_'),
                    sg.Push()],
                    [sg.Text('Profile/Filter:')], [sg.Input(size=(15, 1), enable_events=True, key='-CALIBR_PROFILE-', \
                        font=('Arial Bold', 8), justification='left')]
                    ]

    hot_air = [[sg.Push(), sg.Button('Start', border_width = 5, button_color = 'orange', expand_x = True, expand_y = True, key  = 'HOTAIR-START'), 
                sg.Button('Stop', border_width = 5, button_color = 'green', expand_x = True, expand_y = True, key  = 'HOTAIR-STOP'), LEDIndicator('_hotair_'),
                sg.Push()]]

    dist_rotators_list = list()
    for i in range (1, distance_rotators+1):
        dist_rotators_list.append([sg.Frame('', dist_rotators_block(i), border_width=3, \
                                        expand_x=True, expand_y=True, element_justification = "center")])


    dist_rotators_list.append([sg.Frame('Camera Calibration', calibration, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                        border_width=3, vertical_alignment='top', element_justification = "center", expand_y = False)])
    dist_rotators_list.append([sg.Frame('JTSE Hot Air Station', hot_air, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                        border_width=3, vertical_alignment='top', element_justification = "center", expand_y = False)])

    zaber_list = []
    for i in range (1, zaber_motors+1):
        zaber_list.append([sg.Frame('', zaber_block(i), border_width=3, \
                                        expand_x=True, expand_y=True, element_justification = "center")])

    for i in range (1, gripper_motors+1):
        zaber_list.append([sg.Frame('', gripper_block(i), border_width=3, \
                                        expand_x=True, expand_y=True, element_justification = "center")])







    layout = [
            [sg.Frame('', top_banner,   pad=(0,0), background_color=DARK_HEADER_COLOR,  expand_x=True, \
                        border_width=0, grab=True)],
            
            [   
                sg.Column(script_col, vertical_alignment = 'top', expand_y = 'True', scrollable=False, vertical_scroll_only=True, \
                        key='COLUMN-SC', justification='center', pad=0),
                sg.Column(zaber_list, scrollable=False, vertical_scroll_only=True, \
                        key='COLUMN-Z1', vertical_alignment='center', justification='center',pad=0, expand_y = 'True'), 
                sg.Column(dist_rotators_list, scrollable=False, vertical_scroll_only=True, \
                        key='COLUMN_GT', vertical_alignment='center', justification='center',pad=0, expand_y = 'True'), 
                
                ],
            [sg.Sizegrip(background_color=BORDER_COLOR)]
            ]


    return layout

# === end InitGUI settings


def formFillProc(event, values, window, realNum = False, positiveNum = True, defaultValue = 0):
    print_DEBUG(f'DEBUG: Event received =  {event}, values = {values[event]}, positiveNum = {positiveNum}, default value = {defaultValue}')
    if realNum:
        valValidator = real_validator
        numValidator = real_num_validator
        cast = float
    else:
        valValidator = int_validator
        numValidator = int_num_validator
        cast = int


    if not valValidator(values[event], positive = positiveNum):    
        window[event].update(values[event][:-1])
        print_DEBUG(f'DEBUG: if not valValidator -> window[{event}].update({values[event][:-1]})')

        if not numValidator(values[event][:-1], positive = positiveNum):    #invalid edited value
            if defaultValue:
                window[event].update(str(defaultValue))   
                print_DEBUG(f'DEBUG: if not numValidator and there is default value -> window[{event}].update({str(defaultValue)})')
        
            else:                       # there is no defail value
                window[event].update('')   
                print_DEBUG(f'DEBUG: if not numValidator and there     NO default value -> window[{event}].update(EMPTY_STR)')
        else:                           # valid edited value
            print_DEBUG(f'DEBUG: if YES numValidator -> Leave it as is and DO NOITHING -> values[event][:-1]= {values[event][:-1]}')


    else:
        window[event].update(values[event])
        print_DEBUG(f'DEBUG: if YES  valValidator -> window[{event}].update({values[event]})')

    ret_value = values[event]
    ret_value = cast(ret_value) if numValidator(ret_value) else cast(defaultValue)

    return ret_value

#========================= GUI code starts here =============================

def StartGUI (layout):
    window = sg.Window('Dashboard', layout, margins=(0,0), background_color=BORDER_COLOR,  \
                    # no_titlebar=False, resizable=False, right_click_menu=sg.MENU_RIGHT_CLICK_EDITME_VER_LOC_EXIT, \
                    no_titlebar=False, resizable=True, right_click_menu=CUSTOM_MENU_RIGHT_CLICK_VER_LOC_EXIT, \
                    element_justification='c', finalize = True)

    window.read(timeout=0) 
    window.Finalize()

    for i in range (1, trolley_motors + 1):
        DeActivateMotorControl(window, '--TROLLEY--', i)

    for i in range (1, zaber_motors + 1):
        DeActivateMotorControl(window, '--ZABER--', i)

    for i in range (1, gripper_motors + 1):
        DeActivateMotorControl(window, '--GRIPPERv3--', i)

    for i in range (1, distance_rotators + 1):
        DeActivateMotorControl(window, '--DIST_ROTATOR--', i)

    for i in range (1, dh_grippers + 1):
        DeActivateMotorControl(window, '--DH--', i)

    DeActivateMotorControl(window, '--TIME_ROTATOR--')

    DeActivateMotorControl(window, '--DAQ-NI--')

    DeActivateMotorControl(window, '--CAM--')

    DeActivateMotorControl(window, '--HMP--')


    sg.PopupAnimated(image_source=gif103, text_color='blue', 
                    message='Loading...',background_color='grey', time_between_frames=100)    

    return window

def SetParmUtility(devs_list, window):
    parms_table = read_params()

    ni_dev = None
    calibration_cam = None
    hmp_dev = None

    for dev in devs_list:
        if dev.C_type == '--GRIPPERv3--':
            window[f'-{dev.c_gui}-VOLT-RPM-'].update('Velocity (rpm) ->>')
            dev.dev_mDC.set_parms(parms_table, f'G{dev.c_gui}')
        elif dev.C_type == '--GRIPPER--':
            window[f'-{dev.c_gui}-VOLT-RPM-'].update('Velocity DC (mV) ->>')
            dev.dev_mDC.set_parms(parms_table, f'G{dev.c_gui}')
        elif dev.C_type == '--ZABER--':
            if dev.dev_zaber.is_parked():
                DeActivateMotorControl(window, '--ZABER--', dev.c_gui)
                SetLED (window, f'_zaber_{dev.c_gui}_', 'blue')
            dev.dev_zaber.set_parms(parms_table, f'Z{dev.c_gui}')
        elif dev.C_type == '--TIME_ROTATOR--':
            dev.dev_mDC.set_parms(parms_table, f'S{dev.c_gui}')
        elif dev.C_type == '--DIST_ROTATOR--':
            dev.dev_mDC.set_parms(parms_table, f'R{dev.c_gui}')
        elif dev.C_type == '--TROLLEY--':
            dev.dev_mDC.set_parms(parms_table, f'T{dev.c_gui}')
        elif dev.C_type == '--DAQ-NI--':
            ni_dev = dev.dev_ni
        elif dev.C_type == '--CAM--':
            calibration_cam = dev.dev_cam
        elif dev.C_type == '--HMP--':
            hmp_dev = dev.dev_hmp


    return ni_dev, calibration_cam


def workingCycle (window):

    global emergency_stop_pressed
                                            # init devices and pereferials 
    devs_list = port_scan()
    parms_table = read_params()
    initGUIDevs(window, devs_list)

    scriptRunning:bool = False

    statusPub = anim_0MQ()
    statusPub._publisher(ZMQ_PORT)


    ni_dev = None
    calibration_cam = None

    ni_dev, calibration_cam = SetParmUtility(devs_list, window)

    sg.PopupAnimated(image_source=None)

    print_log(f'{len(devs_list)} devices found')
    scriptQ:Queue = Queue()

    # script_condition = Condition()

    # last_subscript = []                         # workaround for pySimpleGUI bug / Table.update( failes to update)

    control_monitor = sm.StatusMonitor(window = window, devs_list = devs_list, statusPub = statusPub, timeout = 1)
    # control_monitor = sm.StatusMonitor(window = window, devs_list = devs_list, statusPub = statusPub, timeout = 60)  #BUGBUGBUG
    process_manager = pm.ProcManager(window=window)
    gui_task_list = pm.WorkingTasksList()

    eventQ:Queue = Queue()

    while True:                                                             # Event Loop
        
        # event, values = window.read()   # blocking!!!
        event, values = window.read(timeout=100)   # NON blocking!!!
        
        control_monitor.monitorUpdate(realTime = not scriptRunning)             #BUGBUGBUG
        
        # if not window[f'Run'].WidgetDisabled():    # Update real time values only when the script is not running
        #     control_monitor.monitorUpdate()  

        wTask:pm.WorkingTask = None
        
        if event == '__TIMEOUT__':

            # if not globalEventQ.stepNotificationQ.empty():
            if not eventQ.empty():
                # recvEvent:event2GUI = globalEventQ.stepNotificationQ.get()
                recvEvent:event2GUI = eventQ.get()
                print_log(f'event = {recvEvent.event} of type = {type(recvEvent.event)}, values = {recvEvent.value}, dev = {recvEvent.device} of type = {type(recvEvent.value)}')
                if recvEvent.event == '-TASK_DONE-':                              # TASK_DONE/success
                    taskID = recvEvent.value
                    completedTask = gui_task_list.getTask(taskID)
                    print_log(f'-TASK_DONE- event received (task id = {taskID}). Task = {completedTask} ')
                    if completedTask:
                        TaskCloseProceedure(window, completedTask)
                        if completedTask.isStep():
                            print_log(f'STEP task DONE')
                            scriptQ.put('-STEP_DONE-')
                        else:
                            print_log(f'Single task DONE')
                        res = gui_task_list.delTask(taskID)
                    else:
                        print_err(f'-ERROR- No task # {taskID} found in the tasks list')
                        continue

                    activateScriptControl(window)
                    
                    continue
                
                elif recvEvent.event == '-TASK_ERROR-':
                    taskID = recvEvent.value
                    completedTask = gui_task_list.getTask(taskID)
                    print_log(f'-TASK_ERROR- event received (task id = {taskID}). Device = {recvEvent.device}. Task = {completedTask}. ')
                    if completedTask:
                        completedTask.EmergencyStop()
                        TaskCloseProceedure(window, completedTask)
                        res = gui_task_list.delTask(taskID)
                        # if completedTask.isStep():
                        activateScriptControl(window)
                    else:
                        print_err(f'-ERROR- No task # {taskID} found in the tasks list')
                        
                    scriptQ.put('-EMERGENCY_STOP-')    
                    sg.popup_error(f'Attention.\nAbnormal termination of {recvEvent.device}', background_color = 'orange')
                    
                    
                    continue

                elif recvEvent.event == '-SCRIPT_STEP-':
                    event = '-SCRIPT_STEP-'
                    steptask:bool = False
                    deActivateScriptControl(window)
                    # scriptStep:List[str] = GetSubScript(window, event, values, recvEvent.value)
                    scriptStep:dict = GetSubScriptDict(window, event, values, recvEvent.value)
                    print_DEBUG(f'scriptStep = {scriptStep}')
                    if pm.validateScript(scriptStep):
                        if event == '-SCRIPT_STEP-':
                            steptask = True
                        else:
                            steptask = False

                        # wTask = pm.BuildWorkingTask(scriptStep, devs_list, pm.RunType.parallel , steptask) 
                        key = list(scriptStep.keys())[0]
                        script = scriptStep[key]
                        wTask = pm.BuildComplexWorkingClass(script, devs_list, key, steptask) 
                        print_DEBUG(f'wTask = {wTask}')
                        if wTask == None:
                            print_err(f'Cant perform STEP, since one of listed commands operate device that is not active. Go next')
                            if steptask:                                            # if it's part of script 
                                scriptQ.put('-STEP_DONE-')

                            activateScriptControl(window)
                            continue
                    else:
                        print_err(f'Script validation failed for script: {scriptStep} ')

                else:
                    print_err(f'-ERROR- Unrecognized event received {recvEvent.event}')       

            else:
                continue        


        # if event == '-UPDATE_STATUS-':
        #     values['-UPDATE_STATUS-'].monitorUpdate()
        #     continue


        


        if event == sg.WIN_CLOSED or event == 'Exit':

            print_log(f'Exit received. Stop all activity')
            for tsk in gui_task_list.getAllTasks():
                tsk.EmergencyStop()

            if control_monitor:
                control_monitor.StopMonitor()

            sg.PopupAnimated(image_source='Images/attention_image.gif',message='Wait for Zaber devices be parked',no_titlebar=False)
            # Zaber_Motor.park_all()

            sg.PopupAnimated(image_source=None)
            break

        # elif emergency_stop_pressed == True and no_motion(devs_list):
        #     emergency_stop_pressed = False
        #     sg.popup_auto_close(f'Emergency stop\ndone', non_blocking = True)

                      

            
        # if event == '__TIMEOUT__':
        #     ThreadCloseProceedure(window, devs_list, statusPub)
        #     continue

        elif event == '-TASK_DONE-':
            taskID = values['-TASK_DONE-']
            completedTask = gui_task_list.getTask(taskID)
            print_log(f'-TASK_DONE- event received (task id = {taskID}). Task = {completedTask} ')
            if completedTask:
                TaskCloseProceedure(window, completedTask)
                if completedTask.isStep():
                    print_log(f'STEP task DONE')
                    scriptQ.put('-STEP_DONE-')
                else:
                    print_log(f'Single task DONE')
                res = gui_task_list.delTask(taskID)
            else:
                print_err(f'-ERROR- No task # {taskID} found in the tasks list')
                continue

            activateScriptControl(window)

            # window.write_event_value(f'-UPDATE_STATUS-', control_monitor)  #BUGBUGBUG 
            control_monitor.monitorUpdate(realTime = True)

            continue
        
        elif event == '-TASK_ERROR-':
            taskID = values['-TASK_ERROR-']
            completedTask = gui_task_list.getTask(taskID)
            if completedTask:
                completedTask.EmergencyStop()
                TaskCloseProceedure(window, completedTask)
                res = gui_task_list.delTask(taskID)
                # sg.popup_auto_close(f'Emergency stop \noperated', non_blocking = True)
            else:
                print_err(f'-ERROR- No task # {taskID} found in the tasks list')
                continue
            
            if completedTask.isStep():
                # scriptQ.put('-EMERGENCY_STOP-')
                activateScriptControl(window)


            # window.write_event_value(f'-UPDATE_STATUS-', control_monitor)  #BUGBUGBUG 
            control_monitor.monitorUpdate(realTime = True)

            continue
        
        elif event == 'Park':
            # if not no_motion(devs_list):
            #     continue
            print_log(f'Parking all devices')
            Zaber_Motor.park_all()
            for i in range (1, zaber_motors + 1):
                DeActivateMotorControl(window, '--ZABER--', i)
                SetLED (window, f'_zaber_{i}_', 'blue')
            continue
        elif event == 'Unpark':
            # if not no_motion(devs_list):
            #     continue
            print_log(f'Unparking all devices')
            Zaber_Motor.unpark_all()
            Correct_ZB_possition_after_unpark(window, devs_list)
            for i in range (1, zaber_motors + 1):
                ActivateMotorControl(window, '--ZABER--', i)
            continue

        elif event == 'Version':
            sg.popup_scrolled(sg.get_versions(), keep_on_top=True)
            continue
        elif event == 'File Location':
            sg.popup_scrolled('This Python file is:', __file__)   
            continue 



# Control panel section
#
        if event == 'Stop':

            print_log(f'Emergency stop button was pressed')
            scriptQ.put('-EMERGENCY_STOP-')
            removeElementQ(eventQ, '-SCRIPT_STEP-')
            
            for tsk in gui_task_list.getAllTasks():
                tsk.EmergencyStop()

            emergency_stop_pressed = True
            sg.popup_auto_close(f'Emergency stop\ndone', non_blocking = True)
            # window[f'Run'].update(disabled = False)

           
            continue

        elif event == 'Run':
            # window[f'Run'].update(disabled = True)
            deActivateScriptControl(window)
            # script_thread = Thread(target=run_script_cont_thread, args=(window, devs_list, values['-TABLE-'], '-SCRIPT-RUN-DONE-', script_condition, ))
            # 
            # window.perform_long_operation(lambda :
            #     run_script_cont(window, devs_list, values['-TABLE-']), '-SCRIPT-RUN-DONE-')
            print_log(f'Running new script thread!')
            script_thread = Thread(target=ScriptRunner, args=(window, devs_list, values['-TABLE-'], \
                                                                        scriptQ, '-SCRIPT_DONE-', eventQ,  ))
            script_thread.start()
            print_log(f'Running new script thread = {script_thread}/{script_thread.ident}')

            scriptRunning = True
            continue
        
        elif event == '-SCRIPT_DONE-':
            print_log(f'Script completed. Activating controls')
            activateScriptControl(window)
            # sg.popup_auto_close('Scritp done')
            scriptRunning = False
            continue



        elif event in ['Step', '-TABLE-']:
        # elif event in ['Step', '-TABLE-', '-SCRIPT_STEP-']:
        # elif event in ['Step',  '-SCRIPT_STEP-']:

            steptask:bool = False
            deActivateScriptControl(window)
            # scriptStep:List[str] = GetSubScript(window, event, values)
            scriptStep:dict = GetSubScriptDict(window, event, values)
            print_DEBUG(f'scriptStep = {scriptStep}')
            if scriptStep == []:
                activateScriptControl(window)
                continue
            if pm.validateScript(scriptStep):
                if event == '-SCRIPT_STEP-':
                    steptask = True
                else:
                    steptask = False

                # wTask = pm.BuildWorkingTask(scriptStep, devs_list, pm.RunType.parallel , steptask) 
                key = list(scriptStep.keys())[0]
                script = scriptStep[key]
                wTask = pm.BuildComplexWorkingClass(script, devs_list, key, steptask) 
                # wTask = pm.BuildComplexWorkingClass(scriptStep, devs_list, steptask)
                print_DEBUG(f'wTask = {wTask}')
                if wTask == None:
                    print_err(f'Cant perform STEP, since one of listed commands operate device that is not active. Go next')
                    if steptask:                                            # if it's part of script 
                        scriptQ.put('-STEP_DONE-')

                    activateScriptControl(window)
                    continue
            else:
                print_err(f'Script validation failed for script: {scriptStep} ')

            # if no_motion(devs_list):
            #     event_procedure_in_progress = True
            #     script_proc(window, devs_list, event, values)
            #     event_procedure_in_progress = False
            
        
        elif event == 'Open File':
            LoadScriptFile()
            # window.Finalize()
            window.refresh()

            continue 
        elif event == 'Clear Log':
            ClearLog()
            continue
        elif event == 'Save Log':
            SaveLog()
            continue

#
#        

        elif event == '-SCRIPT-RUN-DONE-':
            window[f'Run'].update(disabled = False)
            # sg.popup_auto_close('Scritp done')
            continue
        


        d_index = LocateDevice(event, devs_list)


        if (not ((event == 'Step') or (event == '-SCRIPT_STEP-') or (event == '-TABLE-'))) and d_index < 0:
        # if (not ((event == 'Step') or (event == '-SCRIPT_STEP-'))) and d_index < 0:
            continue


# Peripheral devices section
#

        ni_dev = get_dev('--DAQ-NI--', devs_list)
        calibration_cam = get_dev('--CAM--', devs_list)
        hmp_dev = get_dev('--HMP--', devs_list)

        if event == 'TRIGGER':
            if ni_dev:
                DeActivateMotorControl(window, '--DAQ-NI--')
                print_log(f'Trigger - {values["-SELECTOR-"]} ')

                wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.trigger, args=pm.argsType(trigger_selector=int(values["-SELECTOR-"][-1]))), sType = pm.RunType.single)

                # ni_dev.selector(int(values["-SELECTOR-"][-1]))
                # time.sleep(ni_dev.SELECTOR_TRIGGER_DELAY)             # delay 
                # ni_dev.trigger()


                print_log(f'{int(values["-SELECTOR-"][-1])} schedule trigger will be sent')
                ActivateMotorControl(window, '--DAQ-NI--')
            else:
                print_err(f'No NI device found to operate TRIGGER control')
            



        elif event == '-HMP-CURRENT-':
            if values[event] == '':
                # devs_list[d_index].dev_hmp.curr = 0
                continue

            new_val = formFillProc(event, values, window, realNum = True, positiveNum = True, defaultValue = 0)
            # devs_list[d_index].dev_hmp.current = new_val
            continue

        elif event == '-HMP-VOLTAGE-':
            if values[event] == '':
                # devs_list[d_index].dev_hmp.volt = 0
                continue
            
            new_val = formFillProc(event, values, window, realNum = True, positiveNum = True, defaultValue = 0)
            # devs_list[d_index].dev_hmp.volt = new_val
            continue
            

        elif event == '-HMP-RES-CALC-':
            if (hmp_dev):
                print_log(f'Operating  HMP power station')
                DeActivateMotorControl(window, '--HMP--')
                wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.measure, \
                                                args=pm.argsType(curr=values[f'-HMP-CURRENT-'], volt=values[f'-HMP-VOLTAGE-'])),\
                                                sType = pm.RunType.single)
                    
                ActivateMotorControl(window, '--HMP--')
            else:
                print_err(f'No HMP device found to calculate resistance')
            

        elif event == '-HMP-VOLT-SET-':
            if values["-HMP-VOLTAGE-"] == '':
                continue

            if (hmp_dev):
                print_log(f'Setting new voltage = {values["-HMP-VOLTAGE-"]}')
                devs_list[d_index].dev_hmp.setVoltCurr(volt = values['-HMP-VOLTAGE-'])

            continue

        elif event == '-HMP-CURR-SET-':
            if values["-HMP-CURRENT-"] == '':
                continue

            if (hmp_dev):
                print_log(f'Setting new current = {values["-HMP-CURRENT-"]}')
                devs_list[d_index].dev_hmp.setVoltCurr(curr = values['-HMP-CURRENT-'])
                
            continue

        elif event == 'HOTAIR-START':
            if (ni_dev):
                DeActivateMotorControl(window, '--DAQ-NI--')
                print_log(f'Operating  JTSE Hot Air Station')

                wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.hotair, args=pm.argsType(start_stop=True)), sType = pm.RunType.single)
                # ni_dev.jtseStart()
                
                ActivateMotorControl(window, '--DAQ-NI--')
            else:
                print_err(f'No NI device found to operate JTSE control')

            

        elif event == 'HOTAIR-STOP':
            if (ni_dev):
                DeActivateMotorControl(window, '--DAQ-NI--')
                print_log(f'Stoping JTSE Hot Air Station')

                wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.hotair, args=pm.argsType(start_stop=False)), sType = pm.RunType.single)
                # ni_dev.jtseStop()

                ActivateMotorControl(window, '--DAQ-NI--')
            else:
                print_err(f'No NI device found to operate JTSE control')

            

        elif event == '-CALIBRATE_CONTINUE-':
            # camRes(res=True, dist=dist, repQ=reportQ)
            calibrRes:camRes = values["-CALIBRATE_CONTINUE-"]
            dist = calibrRes.dist
            if not calibrRes.res:                               # Operation failed
                emergency_stop_pressed =  True
                print_err(f'Calibration error. Stoping.')
                sg.popup_error('Calibration failed.\n Vision Error', font=("Arial Bold", 60))
            else:
                wTask = proc_tuning(window, devs_list, calibration_cam.motor, dist)
                # runTask(wTask, gui_task_list, process_manager, calibrRes.repQ)
                process_manager.load_task(wTask, calibrRes.repQ)
                continue



        elif event == '-CALIBRATE_DONE-':
            dist = values["-CALIBRATE_DONE-"]

###############################################

            if dist == sys.maxsize:
                emergency_stop_pressed =  True
                print_err(f'Calibration error. Stoping.')
                sg.popup_error('Calibration failed.\n Vision Error', font=("Arial Bold", 60))
            #     pass
            # else:
            #     wTask = proc_tuning(window, devs_list, calibration_cam.motor, dist)
    ###############################################

            ActivateMotorControl(window, '--CAM--')
            calibration_in_process = False


        elif event == '-CALIBRATE-':
            if calibration_cam:
                print_log(f'Calibrate button pressed for {values["-Z-SELECTOR-"]} profile = {values[f"-CALIBR_PROFILE-"]}  ')
                DeActivateMotorControl(window, '--CAM--')
                SetLED (window, '_calibr_', 'DeepPink2')
                window.Finalize()
                
                calibration_in_process = True

                if calibration_cam.cam_status():
                    wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.calibrate, \
                                                    args=pm.argsType(calibr_selector = values["-Z-SELECTOR-"], profile = values[f'-CALIBR_PROFILE-'])), \
                                                    sType = pm.RunType.single)
                else:
                    calibration_in_process = False

                # calibration_in_process = calibration_cam.start_cam_tuning_operation(values["-Z-SELECTOR-"], values[f'-CALIBR_PROFILE-'], window)

                
                if not calibration_in_process:
                    print_err(f'Calibration on {values["-Z-SELECTOR-"]} failed to start/no modbus client found')
                    ActivateMotorControl(window, '--CAM--')
                    sg.popup_auto_close('Calibration failed to start')
                    continue
                    
                print_log(f'calibration will be operated on ZABER Motor {int(values["-Z-SELECTOR-"][-1])}')
                
            else:
                print_err(f'No ModBus/Camera server connection established. Calibration could not be performed')
                
            
                
        elif event == '-CALIBR_PROFILE-':
            if not file_name_validator(values[f'-CALIBR_PROFILE-']):    
                    window[f'-CALIBR_PROFILE-'].update(values[f'-CALIBR_PROFILE-'][:-1])
            else:
                pass
            continue



# Motor Devices section 
#        

    # ZABER
        if '-ZABER-DST-' in event  :
            formFillProc(event, values, window, realNum = True, positiveNum = True, defaultValue = devs_list[d_index].dev_zaber.GetPos())
            continue

        elif '-ZABER-VELOCITY-' in event:
            new_val = formFillProc(event, values, window, realNum = False, positiveNum = True, defaultValue = devs_list[d_index].dev_zaber.DEFAULT_VELOCITY_PERCENTAGE)
            devs_list[d_index].dev_zaber.velocity_in_percents = new_val
            continue


        elif '-ZABER-GTD-' in event:
            i = (int)(event[1:3])
            if not real_num_validator(values[f'-{i:02d}-ZABER-DST-'], positive=True):
                                                                    # the value is invalid
                window[f'-{i:02d}-ZABER-DST-'].update(str(devs_list[d_index].dev_zaber.GetPos())) 
            else:
                DeActivateMotorControl(window, f'--ZABER--', devs_list[d_index].c_gui)
                wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.go_to_dest, args=pm.argsType(position=values[f'-{i:02d}-ZABER-DST-'])), sType = pm.RunType.single)
                # devs_list[d_index].dev_zaber.move_abs(values[f'-{i:02d}-ZABER-DST-'])

        elif 'ZABER-GH-' in event:
            DeActivateMotorControl(window, f'--ZABER--', devs_list[d_index].c_gui)
            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.home), sType = pm.RunType.single)
            # devs_list[d_index].dev_zaber.move_home()
            pass


    # DH Gripper
        elif '-DH-GRIPPER-RPM-' in event:
            
            update_val = str(devs_list[d_index].dev_mDC.DevOpSPEED)
            new_val = formFillProc(event, values, window, realNum = False, positiveNum = True, defaultValue = update_val)
            devs_list[d_index].dev_mDC.DevOpSPEED = new_val
            continue

        elif f'-DH-GRIPPER-ON-' in event:
            window[f'-{devs_list[d_index].c_gui}-DH-GRIPPER-ON-'].update(button_color='dark green on green')
            window[f'-{devs_list[d_index].c_gui}-DH-GRIPPER-OFF-'].update(button_color='white on red')

            DeActivateMotorControl(window, f'--DH--', devs_list[d_index].c_gui)
            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.go_fwrd_on), sType = pm.RunType.single)
            devs_list[d_index].dev_mDC._mDev_pressed = True


        elif f'-DH-GRIPPER-OFF-' in event:
            window[f'-{devs_list[d_index].c_gui}-DH-GRIPPER-OFF-'].update(button_color='tomato on red')
            window[f'-{devs_list[d_index].c_gui}-DH-GRIPPER-ON-'].update(button_color='white on green')

            DeActivateMotorControl(window, f'--DH--', devs_list[d_index].c_gui)
            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index], cmd=pm.OpType.go_bcwrd_off), sType = pm.RunType.single)
            devs_list[d_index].dev_mDC._mDev_pressed = True



        elif  f'-DH_GRIPPER_TARGET-' in event:
            new_val = formFillProc(event, values, window, realNum = False, positiveNum = False, defaultValue = str(devs_list[d_index].dev_mDC.mDev_get_cur_pos()))
            # devs_list[d_index].dev_mDC._mDev_pos = new_val
            continue


        elif '-DH_GRIPPER_POS_SET-' in event:
            i = event[1]
            go_pos = values[f'-{i}-DH_GRIPPER_TARGET-']

            print_log(f'Move DH GRIPPER {devs_list[d_index].c_gui}/{i} to {go_pos} position')

            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index], cmd=pm.OpType.go_to_dest, args=pm.argsType(position=go_pos)), sType = pm.RunType.single)
            # devs_list[d_index].dev_mDC.go2pos(go_pos)

            devs_list[d_index].dev_mDC._mDev_pressed = True
            DeActivateMotorControl(window, f'--DH--', devs_list[d_index].c_gui)
        

        
        elif '-DH_GRIPPER_STOP-' in event:

            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index], cmd=pm.OpType.stop), sType = pm.RunType.single)
            
            ActivateMotorControl(window, devs_list[d_index].dev_mDC._mDev_type, devs_list[d_index].c_gui)
            
        elif '-DH_GRIPPER_TARGET_RESET-' in event:
            i = event[1]

            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.home), sType = pm.RunType.single)

            window[f'-{i}-DH_GRIPPER_POSSITION-'].update(value = 0)
            window[f'-{i}-DH_GRIPPER_TARGET-'].update(value = 0) 
        



    # GRIPPER
        elif '-GRIPPER-RPM-' in event:
            
            if devs_list[d_index].dev_mDC.mDev_type == '--GRIPPERv3--': 
                update_val = str(devs_list[d_index].dev_mDC.DevOpSPEED)
            elif devs_list[d_index].dev_mDC.mDev_type == '--GRIPPER--': 
                update_val = str(devs_list[d_index].dev_mDC.DEAFULT_VELOCITY_EV_VOLTAGE)
            else:
                update_val = None

            new_val = formFillProc(event, values, window, realNum = False, positiveNum = True, defaultValue = update_val)

            
            if devs_list[d_index].dev_mDC.mDev_type == '--GRIPPERv3--':
                devs_list[d_index].dev_mDC.rpm = new_val
            elif devs_list[d_index].dev_mDC.mDev_type == '--GRIPPER--':
                devs_list[d_index].dev_mDC.el_voltage = new_val
            else:
                print_err(f'-ERROR-: Wrong device type {devs_list[d_index].dev_mDC.mDev_type} at event {event}')            

        elif '-GRIPPER-CURR-' in event:
            
            new_val = formFillProc(event, values, window, realNum = False, positiveNum = True, defaultValue = devs_list[d_index].dev_mDC.DEFAULT_CURRENT_LIMIT)
            devs_list[d_index].dev_mDC.el_current_limit = new_val


        elif f'-GRIPPER-ON-' in event:
            window[f'-{devs_list[d_index].c_gui}-GRIPPER-ON-'].update(button_color='dark green on green')
            window[f'-{devs_list[d_index].c_gui}-GRIPPER-OFF-'].update(button_color='white on red')

            DeActivateMotorControl(window, f'--GRIPPERv3--', devs_list[d_index].c_gui)
            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.go_fwrd_on), sType = pm.RunType.single)
            # devs_list[d_index].dev_mDC.gripper_on()
            devs_list[d_index].dev_mDC.mDev_pressed = True


        elif f'-GRIPPER-OFF-' in event:
            window[f'-{devs_list[d_index].c_gui}-GRIPPER-OFF-'].update(button_color='tomato on red')
            window[f'-{devs_list[d_index].c_gui}-GRIPPER-ON-'].update(button_color='white on green')

            DeActivateMotorControl(window, f'--GRIPPERv3--', devs_list[d_index].c_gui)
            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index], cmd=pm.OpType.go_bcwrd_off), sType = pm.RunType.single)
            # devs_list[d_index].dev_mDC.gripper_off()
            devs_list[d_index].dev_mDC.mDev_pressed = True




        elif  f'-GRIPPER_TARGET-' in event:
            new_val = formFillProc(event, values, window, realNum = False, positiveNum = False, defaultValue = str(devs_list[d_index].dev_mDC.mDev_get_cur_pos()))
            devs_list[d_index].dev_mDC.mDev_pos = new_val


        elif '-GRIPPER_POS_SET-' in event:
            i = event[1]
            go_pos = values[f'-{i}-GRIPPER_TARGET-']

            print_log(f'Move GRIPPER {devs_list[d_index].c_gui}/{i} to {go_pos} position')

            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index], cmd=pm.OpType.go_to_dest, args=pm.argsType(position=go_pos)), sType = pm.RunType.single)
            # devs_list[d_index].dev_mDC.go2pos(go_pos)

            devs_list[d_index].dev_mDC.mDev_pressed = True
            DeActivateMotorControl(window, f'--GRIPPERv3--', devs_list[d_index].c_gui)
        

        
        elif '-GRIPPER_STOP-' in event:

            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index], cmd=pm.OpType.stop), sType = pm.RunType.single)
            # devs_list[d_index].dev_mDC.mDev_stop()
            
            ActivateMotorControl(window, devs_list[d_index].dev_mDC.mDev_type, devs_list[d_index].c_gui)
            
        elif '-GRIPPER_TARGET_RESET-' in event:
            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.home), sType = pm.RunType.single)
            # devs_list[d_index].dev_mDC.mDev_reset_pos()

            window[f'-{i}-GRIPPER_POSSITION-'].update(value = 0)
            window[f'-{i}-GRIPPER_TARGET-'].update(value = 0) 
        

    # TROLLEY    
    #     

        elif  f'-TROLLEY_TARGET-' in event:
            new_val = formFillProc(event, values, window, realNum = False, positiveNum = False, defaultValue = str(devs_list[d_index].dev_mDC.mDev_get_cur_pos()))
            devs_list[d_index].dev_mDC.mDev_pos = new_val

                #
        elif '-TROLLEY_VELOCITY-' in event:
            new_val  = formFillProc(event, values, window, realNum = False, positiveNum = False, defaultValue = devs_list[d_index].dev_mDC.DevOpSPEED)
            devs_list[d_index].dev_mDC.rpm = new_val


        elif '-TROLLEY_POS_SET-' in event:
            i = event[1]

            t_target = values[f'-{i}-TROLLEY_TARGET-']
            if not int_num_validator(t_target):
                print_err (f'ERROR - Wrong target value:->{t_target}<-')
                continue
            go_pos = int(t_target)

            print_log(f'Move TROLLEY {devs_list[d_index].c_gui}/{i} to {go_pos} position')

            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index], cmd=pm.OpType.go_to_dest, args=pm.argsType(position=go_pos)), sType = pm.RunType.single)
            # devs_list[d_index].dev_mDC.go2pos(go_pos)

            devs_list[d_index].dev_mDC.mDev_pressed = True

            DeActivateMotorControl(window, f'--TROLLEY--', devs_list[d_index].c_gui)
        
        elif '-TROLLEY_RIGHT-' in event:
            DeActivateMotorControl(window, '--TROLLEY--', devs_list[d_index].c_gui)

            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index], cmd=pm.OpType.go_fwrd_on), sType = pm.RunType.single)
            # devs_list[d_index].dev_mDC.mDev_forward()

            devs_list[d_index].dev_mDC.mDev_pressed = True

        elif '-TROLLEY_LEFT-' in event:
            DeActivateMotorControl(window, '--TROLLEY--', devs_list[d_index].c_gui)

            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index], cmd=pm.OpType.go_bcwrd_off), sType = pm.RunType.single)
            # devs_list[d_index].dev_mDC.mDev_backwrd()

            devs_list[d_index].dev_mDC.mDev_pressed = True
        
        elif '-TROLLEY_STOP-' in event:
            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index], cmd=pm.OpType.stop), sType = pm.RunType.single)
            # devs_list[d_index].dev_mDC.mDev_stop()

            ActivateMotorControl(window, '--TROLLEY--', devs_list[d_index].c_gui)
            
        elif '-TROLLEY_TARGET_RESET-' in event:
            i = event[1]
            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.home), sType = pm.RunType.single)
            # devs_list[d_index].dev_mDC.mDev_reset_pos()

            window[f'-{i}-TROLLEY_POSSITION-'].update(value = 0)
            window[f'-{i}-TROLLEY_TARGET-'].update(value = 0) 
        
        elif '-TROLLEY-CURR-' in event:
            new_val = formFillProc(event, values, window, realNum = False, positiveNum = True, defaultValue = str(devs_list[d_index].dev_mDC.DEFAULT_CURRENT_LIMIT))
            devs_list[d_index].dev_mDC.el_current_limit = new_val

# DIST_ROTATOR
        elif  f'-DIST_ROTATOR_TARGET-' in event:
            new_val = formFillProc(event, values, window, realNum = False, positiveNum = False, defaultValue = str(devs_list[d_index].dev_mDC.mDev_get_cur_pos()))
            devs_list[d_index].dev_mDC.mDev_pos = new_val

                #
        elif '-DIST_ROTATOR_VELOCITY-' in event:
            new_val =  formFillProc(event, values, window, realNum = False, positiveNum = False, defaultValue = str(devs_list[d_index].dev_mDC.DevOpSPEED))
            devs_list[d_index].dev_mDC.rpm = new_val

        elif '-DIST_ROTATOR_POS_SET-' in event:
            i = event[1]

            t_target = values[f'-{i}-DIST_ROTATOR_TARGET-']
            if not int_num_validator(t_target):
                print_err (f'Wrong target value:->{t_target}<-')
                continue

            go_pos = int(t_target)

            print_log(f'Move DIST ROTATOR {devs_list[d_index].c_gui}/{i} to {go_pos} position')

            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index], cmd=pm.OpType.go_to_dest, args=pm.argsType(position=go_pos)), sType = pm.RunType.single)
            # devs_list[d_index].dev_mDC.go2pos(go_pos)

            devs_list[d_index].dev_mDC.mDev_pressed = True
            DeActivateMotorControl(window, f'--DIST_ROTATOR--', devs_list[d_index].c_gui)
        
        elif '-DIST_ROTATOR_RIGHT-' in event:
            
            DeActivateMotorControl(window, '--DIST_ROTATOR--', devs_list[d_index].c_gui)

            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index], cmd=pm.OpType.go_fwrd_on), sType = pm.RunType.single)
            # devs_list[d_index].dev_mDC.mDev_forward()

            devs_list[d_index].dev_mDC.mDev_pressed = True

        elif '-DIST_ROTATOR_LEFT-' in event:
            DeActivateMotorControl(window, '--DIST_ROTATOR--', devs_list[d_index].c_gui)

            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index], cmd=pm.OpType.go_bcwrd_off), sType = pm.RunType.single)
            # devs_list[d_index].dev_mDC.mDev_backwrd()

            devs_list[d_index].dev_mDC.mDev_pressed = True
        
        elif '-DIST_ROTATOR_STOP-' in event:
            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index], cmd=pm.OpType.stop), sType = pm.RunType.single)
            # devs_list[d_index].dev_mDC.mDev_stop()

            ActivateMotorControl(window, '--DIST_ROTATOR--', devs_list[d_index].c_gui)
            
        elif '-DIST_ROTATOR_TARGET_RESET-' in event:
            i = event[1]

            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.home), sType = pm.RunType.single)
            # devs_list[d_index].dev_mDC.mDev_reset_pos()

            window[f'-{i}-DIST_ROTATOR_POSSITION-'].update(value = 0)
            window[f'-{i}-DIST_ROTATOR_TARGET-'].update(value = 0) 
        
        elif '-DIST_ROTATOR-CURR-' in event:
            new_val = formFillProc(event, values, window, realNum = False, positiveNum = True, defaultValue = str(devs_list[d_index].dev_mDC.DEFAULT_CURRENT_LIMIT))
            devs_list[d_index].dev_mDC.el_current_limit = new_val

     
# TIME_ROTATOR_

        elif  f'-TIME_ROTATOR_TARGET-' in event:
            new_val = formFillProc(event, values, window, realNum = False, positiveNum = True, defaultValue = str(devs_list[d_index].dev_mDC.DEFAULT_ROTATION_TIME))
            devs_list[d_index].dev_mDC.rotationTime = int(new_val)
                #
        elif '-TIME_ROTATOR_VELOCITY-' in event:
            new_val = formFillProc(event, values, window, realNum = False, positiveNum = False, defaultValue = devs_list[d_index].dev_mDC.DEAFULT_VELOCITY_EV_VOLTAGE)
            devs_list[d_index].dev_mDC.el_voltage = new_val

        elif '-TIME_ROTATOR_RIGHT-' in event:
            DeActivateMotorControl(window, '--TIME_ROTATOR--', devs_list[d_index].c_gui)

            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index], cmd=pm.OpType.go_fwrd_on, args=pm.argsType(time=int(values[f'-TIME_ROTATOR_TARGET-']))), sType = pm.RunType.single)
            # devs_list[d_index].dev_mDC.timeRotaterFw(int(values[f'-TIME_ROTATOR_TARGET-']))

            devs_list[d_index].dev_mDC.mDev_pressed = True

        elif '-TIME_ROTATOR_LEFT-' in event:
            DeActivateMotorControl(window, '--TIME_ROTATOR--', devs_list[d_index].c_gui)

            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index], cmd=pm.OpType.go_bcwrd_off, args=pm.argsType(time=int(values[f'-TIME_ROTATOR_TARGET-']))), sType = pm.RunType.single)
            # devs_list[d_index].dev_mDC.timeRotaterBw(int(values[f'-TIME_ROTATOR_TARGET-']))

            devs_list[d_index].dev_mDC.mDev_pressed = True
        
        elif '-TIME_ROTATOR_STOP-' in event:
            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index], cmd=pm.OpType.stop), sType = pm.RunType.single)
            # devs_list[d_index].dev_mDC.mDev_stop()

            ActivateMotorControl(window, '--TIME_ROTATOR--', devs_list[d_index].c_gui)
            
        elif '-TIME_ROTATOR-CURR-' in event:
            new_val = formFillProc(event, values, window, realNum = False, positiveNum = True, defaultValue = devs_list[d_index].dev_mDC.DEFAULT_CURRENT_LIMIT)
            devs_list[d_index].dev_mDC.el_current_limit = new_val

        if wTask and wTask.is_single():
            log_str = wTask.singleTaskRepr()
            # window['-OUTLOG-'].print(f'[{cmd[0]}] {cmd[1]}->{cmd[2:]}')

            #NO MORE OUTLOG
            # window['-OUTLOG-'].print(f'[{log_str}]')
        
        runTask(wTask, gui_task_list, process_manager, eventQ)
        
    # end of loop
    
    print_log(f'Exiting main loop')

    for m_dev in devs_list:
        print_log(f'Destroying device {m_dev}')
        if m_dev.dev_mDC:                                
            print_log('Stoping watchdog')
            m_dev.dev_mDC.mDev_in_motion = False
        # del m_dev
        # devs_list.remove(m_dev)

    window.close()

# === end  working cycle 

def runTask(wTask:pm.WorkingTask, gui_task_list:pm.WorkingTasksList, process_manager:pm.ProcManager, eventQ:Queue):
    if wTask:
        # devList:List[CDev] = wTask.exploreDevs()
        
        print_log(f'New task will be proceeded: {wTask}')
        preTaskProceedure(window, wTask)
        gui_task_list.addTask(wTask=wTask)
        process_manager.load_task(wTask, eventQ)                # staring task at proccess manager
    else:
        print_err(f'-ERROR -  Task = {wTask}')

if __name__ == "__main__":
    print_log('Starting here')
    layout = InitGUI()
    window = StartGUI(layout)
    workingCycle(window)
    print_inf(f'Working cycle DONE')
    sys.exit() 
