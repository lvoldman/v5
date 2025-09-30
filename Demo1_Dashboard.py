
from operator import pos, truediv
from re import T
from weakref import finalize
from threading import Condition, Thread
import PySimpleGUI as sg

from bs1_config import port_scan, CDev, gif103, read_params
from bs1_faulhaber import FH_Motor
from bs1_zaber import Zaber_Motor
from bs1_script import Tbl, Tbl_event_proc, Log_tbl, groups
from bs1_ni6002 import NI6002
from bs1_cam_modbus import Cam_modbus
from bs1_FHv3 import FH_Motor_v3

import os, sys, time, re
import math

from bs1_utils import print_log, print_inf, print_err, print_BUGBUG, exptTrace, s16, s32, real_num_validator, int_num_validator, real_validator, int_validator, file_name_validator 


R1_GEAR = 64
R2_GEAR = 64
R1_PITCH = 1.27
R2_DIAMETER = 6


print_log("Starting now")

"""
    Motors' control Dashboard

    LV Copyright 2024
"""

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



zaber_motors = 2
gripper_motors = 2
distance_rotators = 2


emergency_stop_pressed = False
stop_script_flag = False
calibration_in_process = False
step_in = False                                 # used to sync non motion devices (DELAY, CALIBR, etc)
event_procedure_in_progress = False



CUSTOM_MENU_RIGHT_CLICK_VER_LOC_EXIT = ['', ['Version', 'File Location', 'Exit']]


#================  GUI utils start here =======================
def activationControl (type, disableInd = False, ledColor = 'green', index = None):
    if type == '--DIST_ROTATOR--':
        SetLED (window, f'_dist_rotator_{index}_', ledColor)
        window[f'-{index}-DIST_ROTATOR_POS_SET-'].update(disabled = disableInd)
        window[f'-{index}-DIST_ROTATOR_TARGET_RESET-'].update(disabled = disableInd)
        window[f'-{index}-DIST_ROTATOR_LEFT-'].update(disabled = disableInd)
        window[f'-{index}-DIST_ROTATOR_RIGHT-'].update(disabled = disableInd)
        window[f'-{index}-DIST_ROTATOR_TARGET-'].update(disabled = disableInd)
        window[f'-{index}-DIST_ROTATOR_VELOCITY-'].update(disabled = disableInd)
        window[f'-{index}-DIST_ROTATOR-CURR-'].update(disabled = disableInd)
    elif type == '--GRIPPER--':
        SetLED (window, f'_gripper_{index}_', ledColor)
        window[f'-{index}-GRIPPER-RPM-'].update(disabled = disableInd)
        window[f'-{index}-GRIPPER-CURR-'].update(disabled = disableInd)
        window[f'-{index}-GRIPPER-OFF-'].update(disabled = disableInd)
        window[f'-{index}-GRIPPER-ON-'].update(disabled = disableInd)
    elif type == '--GRIPPERv3--':
        SetLED (window, f'_gripper_{index}_', ledColor)
        window[f'-{index}-GRIPPER_POS_SET-'].update(disabled = disableInd)
        window[f'-{index}-GRIPPER_TARGET_RESET-'].update(disabled = disableInd)
        window[f'-{index}-GRIPPER_TARGET-'].update(disabled = disableInd)
        window[f'-{index}-GRIPPER-RPM-'].update(disabled = disableInd)
        window[f'-{index}-GRIPPER-CURR-'].update(disabled = disableInd)
        window[f'-{index}-GRIPPER-OFF-'].update(disabled = disableInd)
        window[f'-{index}-GRIPPER-ON-'].update(disabled = disableInd)
    elif type == '--ZABER--':
        SetLED (window, f'_zaber_{index}_', ledColor)
        window[f'-{index:02d}-ZABER-GTD-'].update(disabled = disableInd)
        window[f'-{index:02d}-ZABER-GH-'].update(disabled = disableInd)
        window[f'-{index:02d}-ZABER-DST-'].update(disabled = disableInd)
        window[f'-{index:02d}-ZABER-VELOCITY-'].update(disabled = disableInd)
    elif type == '--DAQ-NI--':
        SetLED (window, f'_hotair_', ledColor)
        window[f'HOTAIR-START'].update(disabled = disableInd)
        window[f'HOTAIR-STOP'].update(disabled = disableInd)
    elif type == '--CAM--':
        SetLED (window, f'_calibr_', ledColor)
        window[f'-CALIBRATE-'].update(disabled = disableInd)
        window[f'-CALIBR_PROFILE-'].update(disabled = disableInd)
    else:
        print_err(f'PANIC - wrong type {type}')



def ActivateMotorControl(type, index = None):
    activationControl(type,  False, 'green', index)

def DeActivateMotorControl(type, index = None):
    activationControl(type,  True, 'red', index)

  

def InitDevs(devs_list):
    for m_dev in devs_list:
        if m_dev.C_type == '--DIST_ROTATOR--':
            ActivateMotorControl('--DIST_ROTATOR--', m_dev.c_gui)
            window[f'-{m_dev.c_gui}-DIST_ROTATOR_POSSITION-'].update(value = m_dev.dev_fh.fh_pos)
            window[f'-{m_dev.c_gui}-DIST_ROTATOR_VELOCITY-'].update(value = m_dev.dev_fh.rpm)
            window[f'-{m_dev.c_gui}-DIST_ROTATOR-CURR-'].update(value = m_dev.dev_fh.el_current_limit)
        elif m_dev.C_type == '--GRIPPER--':
            ActivateMotorControl(f'--GRIPPER--', m_dev.c_gui)
            window[f'-{m_dev.c_gui}-GRIPPER-RPM-'].update(value = m_dev.dev_fh.el_voltage)
            window[f'-{m_dev.c_gui}-GRIPPER-CURR-'].update(value = m_dev.dev_fh.el_current_limit)
        elif m_dev.C_type == '--ZABER--':
            window[f'-{m_dev.c_gui:02d}-ZABER-POSSITION-'].update(value = m_dev.dev_zaber.current_pos)
            window[f'-{m_dev.c_gui:02d}-ZABER-VELOCITY-'].update(value = m_dev.dev_zaber.velocity_in_percents)
            ActivateMotorControl(f'--ZABER--', m_dev.c_gui)
        elif m_dev.C_type == '--DAQ-NI--':
            ActivateMotorControl(f'--DAQ-NI--')
        elif m_dev.C_type == '--CAM--':
            window[f'-CALIBR_PROFILE-'].update(value = 'base')
            ActivateMotorControl(f'--CAM--')
        else:
            print_err(f'ERROR PANIC - wrong device {m_dev.C_type} in the list at {m_dev.c_gui} position')

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
            [[sg.Text(f'Gripper {i} (G{i})', font='Any 10',  key = f'-{i}-GRIPPER-TITLE-'), LEDIndicator(f'_gripper_{i}_'), sg.Text('On/Off'), \
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
        [sg.Push(), sg.Text(f'Distance Rotator R{i}', font='Any 10',  key = f'-{i}-DIST_ROTATOR-TITLE-'), LEDIndicator(f'_dist_rotator_{i}_'), sg.Push()],
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

def zaber_block(i):
    return (
            [[sg.Text(f'Zaber Z{i}', font='Any 10', key = f'-{i}-ZABER-TITLE-'), LEDIndicator(f'_zaber_{i}_')],
            [ sg.T('Pos'), sg.Text("_", size=(8, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                border_width = 2, key=f'-{i:02d}-ZABER-POSSITION-'), 
            sg.Text(f'Dest'), sg.Input(size=(7, 1), enable_events=True, key=f'-{i:02d}-ZABER-DST-')],
            [sg.Button(button_text = 'Dest', key = f'-{i:02d}-ZABER-GTD-'), \
             sg.Button(button_text = 'Home', key = f'-{i:02d}-ZABER-GH-'),
             sg.T('Velocity'), sg.Input(size=(3, 1), enable_events=True, key=f'-{i:02d}-ZABER-VELOCITY-'), sg.T('%')]  
             ]

    )

def dist_rotators_frames(line_s, line_e):
    return (
            [[sg.Frame('', dist_rotators_block(i), border_width=3, \
                                    expand_x=True, expand_y=True, element_justification = "center")]  for i in range (line_s, line_e+1) ]
    )


def gripper_frames(line_s, line_e):
    return (
            [[sg.Frame('', gripper_block(i), border_width=3, \
                                    expand_x=True, expand_y=True, element_justification = "center")]  for i in range (line_s, line_e+1) ]
    )



# def zaber_frames(line_s, line_e):
#     return (
#             [[sg.Frame('', zaber_block(i), border_width=2, \
#                                     expand_x=True, expand_y=True, element_justification = "center")]  for i in range (line_s, line_e+1) ]
#     )


    
top_banner = [
               [sg.Text('Dashboard', font='Any 12', background_color=DARK_HEADER_COLOR, enable_events=True, \
                        grab=False), sg.Push(background_color=DARK_HEADER_COLOR),
               sg.Text('© 2024', font='Any 12', background_color=DARK_HEADER_COLOR)],
               ]



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
 

script_col=[]
script_col.append([sg.Frame('', script, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                    border_width=1, vertical_alignment='down', element_justification = "center")])

script_col.append([sg.Push(), sg.Button('Park', border_width = 3, button_color = 'red'), 
                sg.Button('Unpark', border_width = 3, button_color = 'green'), sg.Push()])


# gripper_list = gripper_frames(1,(int)(gripper_motors/1))

# dist_rotators_list = dist_rotators_frames(1, (int)(distance_rotators/1))

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




# sys.exit()


layout = [
          [sg.Frame('', top_banner,   pad=(0,0), background_color=DARK_HEADER_COLOR,  expand_x=True, \
                    border_width=0, grab=True)],
        
          [   
            sg.Column(script_col, vertical_alignment = 'top', expand_y = 'True', scrollable=False, vertical_scroll_only=True, \
                     key='COLUMN-SC', justification='center', pad=0),
            # sg.Column(zaber_frames(1,(int)(zaber_motors)), scrollable=False, vertical_scroll_only=True, \
            #          key='COLUMN-Z1', vertical_alignment='center', justification='center',pad=0, expand_y = 'True'), 
            sg.Column(zaber_list, scrollable=False, vertical_scroll_only=True, \
                     key='COLUMN-Z1', vertical_alignment='center', justification='center',pad=0, expand_y = 'True'), 
        #    sg.Column(gripper_list, scrollable=False, vertical_scroll_only=True, \
        #              key='COLUMN_GT', vertical_alignment='center', justification='center',pad=0, expand_y = 'True'), 
            sg.Column(dist_rotators_list, scrollable=False, vertical_scroll_only=True, \
                     key='COLUMN_GT', vertical_alignment='center', justification='center',pad=0, expand_y = 'True'), 
            
            ],
        [sg.Sizegrip(background_color=BORDER_COLOR)]
        ]





# ========================================================= 


def LocateDevice (ev_name) -> int:

    d_index = 0
    for m_dev in devs_list:
        if 'GRIPPER' in ev_name and (m_dev.C_type == '--GRIPPER--' or m_dev.C_type == '--GRIPPERv3--'):
            if   m_dev.c_gui == int (ev_name[1]):
                break
        elif 'DIST_ROTATOR' in ev_name and m_dev.C_type == '--DIST_ROTATOR--' :
            if   m_dev.c_gui == int (ev_name[1]):
                break

        elif  'ZABER' in ev_name and m_dev.C_type == '--ZABER--':
            if  m_dev.c_gui == int (ev_name[1:3]):
                print_log(f'Found device at d_index = {d_index} for event "{ev_name}"')
                break
        d_index += 1

    if d_index >= len(devs_list):                                   # active device not found
            print_err(f'Non device assosiated event {ev_name}')
            return -1
        
    # print_log(f'Device located at index {d_index}')
    return (d_index)

def Correct_ZB_possition_after_unpark():
    for m_dev in devs_list:
        if m_dev.C_type == '--ZABER--':
            new_pos = m_dev.dev_zaber.GetPos()
            window[f'-{m_dev.c_gui:02d}-ZABER-POSSITION-'].update(value = new_pos)


# def  proc_tuning(motor, dist):
#     device_n = (int)(motor[1:])
#     print_log(f'Zaber device = {device_n:02d} moving dist = {dist}')
#     d_index = LocateDevice(f'-{device_n:02d}-ZABER-POSSITION-')
#     if d_index < 0:                     # No device
#         print_err(f'No active Z device to execute cmd ')
#         return

#     move_pos = (float)(dist)
#     cur_pos = devs_list[d_index].dev_zaber.GetPos()
#     new_pos = cur_pos + move_pos

#     print_log(f'Curr pos = {cur_pos}, new pos = {new_pos}')
#     window.write_event_value(f'-{device_n:02d}-ZABER-DST-', str(new_pos))
#     window.write_event_value(f'-{device_n:02d}-ZABER-GTD-', None)


def  proc_tuning(motor, dist):
    if motor[0] == 'Z':
        device_n = (int)(motor[1:])
        print_log(f'Zaber device = {device_n:02d} moving dist = {dist}')
        d_index = LocateDevice(f'-{device_n:02d}-ZABER-POSSITION-')
        if d_index < 0:                     # No device
            print_err(f'No active Z device to execute cmd ')
            return

        move_pos = (float)(dist)
        cur_pos = devs_list[d_index].dev_zaber.GetPos()
        new_pos = cur_pos + move_pos

        print_log(f'Curr pos = {cur_pos}, new pos = {new_pos}')
        window.write_event_value(f'-{device_n:02d}-ZABER-DST-', str(new_pos))
        window.write_event_value(f'-{device_n:02d}-ZABER-GTD-', None)
    if motor[0] == 'R':
        device_n = (int)(motor[1:])
        print_log(f'Router device = {device_n} moving dist = {dist}')
        d_index = LocateDevice(f'-{device_n}-DIST_ROTATOR_TARGET-')
        if d_index < 0:                     # No device
            print_err(f'No active R device to execute cmd ')
            return
        if dist == 0:
            window.write_event_value(f'-{device_n}-DIST_ROTATOR_STOP-', None)
        else:
            if motor == 'R1':
                new_dist = (dist * 4 * 1024 * R1_GEAR) / R1_PITCH
            elif motor == 'R2':
                new_dist = (dist * 4 * 1024 * R2_GEAR) / (R2_DIAMETER * math.pi)
            else:
                print_err(f'Unsupported dev = {motor}')

            move_pos = (float)(new_dist)
            cur_pos = devs_list[d_index].dev_fh.fh_get_cur_pos()
            new_pos = round(cur_pos + move_pos)


            print_log(f'Curr pos = {cur_pos}, new pos = {new_pos}')
            window[f'-{device_n}-DIST_ROTATOR_TARGET-'].update(str(new_pos))
            window.write_event_value(f'-{device_n}-DIST_ROTATOR_POS_SET-', None)
        

def ThreadCloseProceedure():
    global emergency_stop_pressed
    global stop_script_flag

    for m_dev in devs_list:
        if  m_dev.C_type == '--DIST_ROTATOR--'  or m_dev.C_type == '--GRIPPER--' :
            
            
            new_pos = m_dev.dev_fh.fh_get_cur_pos()
            if m_dev.C_type == '--DIST_ROTATOR--':
                window[f'-{m_dev.c_gui}-DIST_ROTATOR_POSSITION-'].update(value = new_pos)
            
                

            
            if not m_dev.dev_fh.wd:                     # if no was running watch dog -> continue
                continue

            alive = m_dev.dev_fh.wd.is_alive()

            if m_dev.dev_fh.fh_pressed and not alive:
                                                        # watch dog is not active anymore
                print_log(f'Found completed WD for device {m_dev.C_type} on port {m_dev.C_port}, dev # ={m_dev.c_id} with GUI_ID={m_dev.c_gui}')
                if m_dev.C_type == '--DIST_ROTATOR--':
                    ActivateMotorControl('--DIST_ROTATOR--', m_dev.c_gui)
                    otf_cur = m_dev.dev_fh.el_current_on_the_fly
                    window[f'-{m_dev.c_gui}-DIST_ROTATOR_CUR_DISPLAY-'].update(value = otf_cur)

                              
                elif m_dev.C_type == '--GRIPPER--':
                    ActivateMotorControl(f'--GRIPPER--', m_dev.c_gui)
                    otf_cur = m_dev.dev_fh.el_current_on_the_fly
                    window[f'-{m_dev.c_gui}-GRIPPER_CUR_DISPLAY-'].update(value = otf_cur)
                    if not m_dev.dev_fh.success_flag:
                        window.write_event_value(f'Stop', None)
                        emergency_stop_pressed =  True
                        stop_script_flag = True
                        sg.popup_error('Attention.\nGRIPPER abnormal termination', background_color = 'orange')
                
                
                m_dev.dev_fh.fh_pressed = False
                
            elif m_dev.dev_fh.fh_pressed and alive:             # still running
                if emergency_stop_pressed:
                        m_dev.dev_fh.fh_stop()

                if m_dev.C_type == '--DIST_ROTATOR--':              # update position
                    new_pos = m_dev.dev_fh.fh_get_cur_pos()
                    window[f'-{m_dev.c_gui}-DIST_ROTATOR_POSSITION-'].update(value = new_pos)
                    otf_cur = m_dev.dev_fh.el_current_on_the_fly
                    window[f'-{m_dev.c_gui}-DIST_ROTATOR_CUR_DISPLAY-'].update(value = otf_cur)
                
                elif m_dev.C_type == '--GRIPPER--':              # update position
                    otf_cur = m_dev.dev_fh.el_current_on_the_fly
                    window[f'-{m_dev.c_gui}-GRIPPER_CUR_DISPLAY-'].update(value = otf_cur)


        elif m_dev.C_type == '--ZABER--':
            if m_dev.dev_zaber.wd and not m_dev.dev_zaber.wd.is_alive() and m_dev.dev_zaber.z_in_motion:
                ActivateMotorControl(f'--ZABER--', m_dev.c_gui)
                m_dev.dev_zaber.z_in_motion = False                 # Avoid GUI activation control while already active
                if not m_dev.dev_zaber.success_flag:
                    window.write_event_value(f'Stop', None)
                    emergency_stop_pressed = True
                    stop_script_flag = True
                    sg.popup_error('Attention.\nZABER abnormal termination', background_color = 'orange')
            else:
                if emergency_stop_pressed:
                        m_dev.dev_zaber.stop_motion()
                
            
            new_pos = m_dev.dev_zaber.GetPos()
            window[f'-{m_dev.c_gui:02d}-ZABER-POSSITION-'].update(value = new_pos)

        

def run_script_cont_thread(window, selected_group, ret_event, scriptCondition):
                                                    # Does not work properly with pySimpleGUI for some reason
    global emergency_stop_pressed
    global stop_script_flag

    stop_script_flag = False
    emergency_stop_pressed = False
    print_log (f'Selection: {selected_group}')

    if selected_group == []:
        print_err(f'Empty script, exiting script thread')
        window.write_event_value(ret_event, None)
        return
    
    if selected_group in groups:                 # probably block is not Script - run group selected correctly
        gr_index = groups.index(selected_group)
    else:
        for gr_index, gr in enumerate(groups):
            if selected_group[0] in gr:
                break
    print_log (f'Starting script at group # {gr_index}')
    while gr_index < len (groups) and not stop_script_flag:
        if(no_motion()):


            print_log(f'Script - run group # {gr_index}')
            window.write_event_value(f'Step', groups[gr_index])
            gr_index += 1
            print_log(f'Waiting (sleeping) for notification (wakeup)')
            with scriptCondition:
                print_log(f'lock() acquired')
                condRes = scriptCondition.wait(0.3)
                if not condRes:
                    print_log(f'scriptCondition released by TIMEOUT')
                else:
                    time.sleep(0.2)                                         # context swich need
                # scriptCondition.wait_for(lambda : no_motion())

            print_log(f'Waked up after notification')


        
        else:                                   # in motion
                time.sleep(0.5)

    print_log(f'emergency_stop_pressed = {emergency_stop_pressed}, group index = {gr_index}, groups = {len (groups)}')
    while not no_motion():
        time.sleep(0.5)                        #wait to complete op.

    window.write_event_value(ret_event, None)
    print_inf(f'Script completed, exiting script thread')





def run_script_cont(window, selected_group):
                                                    # need groups to simulate GUI
                                                    # unfortunately python does not provide 
                                                    # more convenience  method
    global emergency_stop_pressed
    global stop_script_flag

    stop_script_flag = False
    emergency_stop_pressed = False
    print_log (f'Selection: {selected_group}')

    if selected_group == []:
        print_err(f'Empty script')
        return
    
    if selected_group in groups:                 # probably block is not selected correctly
        gr_index = groups.index(selected_group)
    else:
        for gr_index, gr in enumerate(groups):
            if selected_group[0] in gr:
                break
    print_log (f'Starting script at group # {gr_index}')
    while gr_index < len (groups) and not stop_script_flag:
        if(no_motion()):
            print_log(f'Script - run group # {gr_index}')
            window.write_event_value(f'Step', groups[gr_index])
            gr_index += 1
            time.sleep(0.5)                     # contex swich need  0.1
        else:                                   # in motion
            time.sleep(0.5)

        
    print_BUGBUG(f'emergency_stop_pressed = {emergency_stop_pressed}, group index = {gr_index}, groups = {len (groups)}')
    while not no_motion():
        time.sleep(0.5)                        #wait to coplete op.


    

def script_proc(window, event, values):
    global step_in
    subscript = Tbl_event_proc(window, event, values)
    if subscript == []:
        print_log(f'Empty subscript')
        return subscript
    
    print_log(f'Subscript: {subscript}')
                                                            # duplicated devs are eliminated on early stage

    for cmd in subscript:
        devType = cmd[1] if cmd[1] == 'SYS' else cmd[1][0]
        print_log(f'Running now: {cmd}, dev = {cmd[1]}, type = {devType}, len(cmd) = {len(cmd)}')
        match devType:

            case 'R':                               # Distance rotator
                device_n = (int)(cmd[1][1])
                d_index = LocateDevice(f'-{device_n}-DIST_ROTATOR_TARGET_RESET-')
                                                    # Just a trick to locate device

                print_log(f'Distance rotator, ind = {d_index}, device = {device_n}')
                if d_index < 0:                     # No device
                    print_log(f'No active R device to execute cmd - {cmd}')
                    continue
                if len(cmd) == 5:                          # Update rpm
                    print_log(f'Update Rotator RPM =  {cmd[4]}')
                    window[f'-{device_n}-DIST_ROTATOR_VELOCITY-'].update(str(cmd[4]))
                    devs_list[d_index].dev_fh.rpm = cmd[4]
                
                if cmd[2] == 'HO':
                    window.write_event_value(f'-{device_n}-DIST_ROTATOR_TARGET_RESET-', None)
                    print_log(f'Reset instance rotator home pos')
                    continue

                elif cmd[2] == 'REL':
                    window.write_event_value(f'-{device_n}-DIST_ROTATOR_STOP-', None)
                    print_log(f'DIST_ROTATOR Stop/Release')
                    continue

                print_log(f'Curr pos = {devs_list[d_index].dev_fh.fh_pos}, move pos = {(int)(cmd[3])}')
                move_pos = (int)(cmd[3])
                cur_pos = devs_list[d_index].dev_fh.fh_pos
                if cmd[2] == 'MR':
                    new_pos = cur_pos - move_pos
                elif cmd[2] == 'ML':
                    new_pos = cur_pos + move_pos
                else:
                    print_err(f'Error R cmd = {cmd}')
                    continue
                print_log(f'Curr pos = {cur_pos}, new pos = {new_pos}')
                devs_list[d_index].dev_fh.fh_pos = new_pos
                window[f'-{device_n}-DIST_ROTATOR_TARGET-'].update(str(new_pos))
                window.write_event_value(f'-{device_n}-DIST_ROTATOR_POS_SET-', None)


            case 'Z':
                device_n = (int)(cmd[1][1:])
                print_log(f'Zaber device = {device_n:02d}')
                d_index = LocateDevice(f'-{device_n:02d}-ZABER-POSSITION-')
                if d_index < 0:                     # No device
                    print_err(f'No active Z device to execute cmd - {cmd}')
                    continue
                if len(cmd) == 5:                          # Update rpm
                    print_log(f'Update ZABER Velocity =  {cmd[4][:-1]}% / {cmd[4]}')
                    window[f'-{device_n:02d}-ZABER-VELOCITY-'].update(str(cmd[4][:-1]))  # remove % sign character
                    devs_list[d_index].dev_zaber.velocity_in_percents = int(cmd[4][:-1])
                else:
                    window[f'-{device_n:02d}-ZABER-VELOCITY-'].update(str(100))
                    devs_list[d_index].dev_zaber.velocity_in_percents = 100             # set to default 100%

                move_pos = (float)(cmd[3])
                cur_pos = devs_list[d_index].dev_zaber.GetPos()
                if cmd[2] == 'MA':
                    new_pos = move_pos
                elif cmd[2] == 'MR':
                        new_pos = cur_pos + move_pos
                else:
                    print_err(f'Error Zaber cmd = {cmd}')
                    continue
                print_log(f'Curr pos = {cur_pos}, new pos = {new_pos}')
                window.write_event_value(f'-{device_n:02d}-ZABER-DST-', str(new_pos))
                window.write_event_value(f'-{device_n:02d}-ZABER-GTD-', None)

            case 'G':
                device_n = (int)(cmd[1][1])
                d_index = LocateDevice(f'-{device_n}-GRIPPER-ON-')
                if d_index < 0:                     # No device
                    print_err(f'No active G device to execute cmd - {cmd}')
                    continue
                print_log(f'Gripper device = {device_n}, cmd = "{cmd[2]}", type={type(cmd[2])}, status = {devs_list[d_index].dev_fh.gripper_onof}, type = {type(devs_list[d_index].dev_fh.gripper_onof)}')

                if (cmd[2]):
                    window.write_event_value(f'-{device_n}-GRIPPER-ON-', None)
                else:
                    window.write_event_value(f'-{device_n}-GRIPPER-OFF-', None)
            case _:
                if cmd[1] == 'SYS':
                    if cmd[2] == 'HALT':
                        window.write_event_value(f'Stop', None)

                    elif cmd[2] == 'DELAY':
                        print_log(f'Delay = {cmd[3]}')
                        step_in = True
                        time.sleep(float(cmd[3]))
                        step_in = False

                    elif cmd[2] == 'HOTAIR':
                            op = 'START' if cmd[3] == '1' else 'STOP'
                            window.write_event_value(f'HOTAIR-{op}', None)
                    elif cmd[2] == 'CALIBR':
                            print_log(f'CALIBR script cmd on {cmd[3]} motor')
                            window['-Z-SELECTOR-'].update(f'{cmd[3]}')
                            if len(cmd) == 5:
                                profile = cmd[4]
                            else:
                                profile = 'base'
                            print_log(f'len(cmd) = {len(cmd)} profile = {profile}')
                            window[f'-CALIBR_PROFILE-'].update(str(profile))
                            # window.write_event_value(f'-CALIBR_PROFILE-', profile)
                            window.write_event_value(f'-CALIBRATE-', cmd[3])
                            pass

                    else:
                        print_err(f'Unrecognized script cmd = {cmd}')
        
                    

   
    return subscript

    

def no_motion():
    global step_in
    global event_procedure_in_progress
    
    if step_in or event_procedure_in_progress:
        return False

    for m_dev in devs_list:
        if m_dev.dev_zaber and m_dev.dev_zaber.z_in_motion  or   m_dev.dev_fh and m_dev.dev_fh.fn_in_motion:
            print_log(f'Device {m_dev.C_type} # {m_dev.c_gui} is still in motion. No script/log operation allowed')
            return False

    return True


#========================================= Utils end here =======================



#========================= GUI code starts here =============================


window = sg.Window('Dashboard', layout, margins=(0,0), background_color=BORDER_COLOR,  \
                # no_titlebar=False, resizable=False, right_click_menu=sg.MENU_RIGHT_CLICK_EDITME_VER_LOC_EXIT, \
                no_titlebar=False, resizable=True, right_click_menu=CUSTOM_MENU_RIGHT_CLICK_VER_LOC_EXIT, \
                element_justification='c', finalize = True)

window.read(timeout=0) 
window.Finalize()


for i in range (1, zaber_motors + 1):
    DeActivateMotorControl('--ZABER--', i)

for i in range (1, gripper_motors + 1):
    DeActivateMotorControl('--GRIPPERv3--', i)

for i in range (1, distance_rotators + 1):
    DeActivateMotorControl('--DIST_ROTATOR--', i)

DeActivateMotorControl('--TIME_ROTATOR--')

DeActivateMotorControl('--DAQ-NI--')

DeActivateMotorControl('--CAM--')



sg.PopupAnimated(image_source=gif103, text_color='blue', 
                 message='Loading...',background_color='grey', time_between_frames=100)    


# init devices and pereferials 
devs_list = port_scan()
parms_table = read_params()
ni_dev = None
calibration_cam = None
InitDevs(devs_list)

for dev in devs_list:
    if dev.C_type == '--GRIPPERv3--':
        window[f'-{dev.c_gui}-VOLT-RPM-'].update('Velocity (rpm) ->>')
        dev.dev_fh.set_parms(parms_table, f'G{dev.c_gui}')
    elif dev.C_type == '--GRIPPER--':
        window[f'-{dev.c_gui}-VOLT-RPM-'].update('Velocity DC (mV) ->>')
        dev.dev_fh.set_parms(parms_table, f'G{dev.c_gui}')
    elif dev.C_type == '--ZABER--':
        if dev.dev_zaber.is_parked():
            DeActivateMotorControl('--ZABER--', dev.c_gui)
            SetLED (window, f'_zaber_{dev.c_gui}_', 'blue')
        dev.dev_zaber.set_parms(parms_table, f'Z{dev.c_gui}')
    elif dev.C_type == '--DIST_ROTATOR--':
         dev.dev_fh.set_parms(parms_table, f'R{dev.c_gui}')
    elif dev.C_type == '--DAQ-NI--':
        ni_dev = dev.dev_ni
    elif dev.C_type == '--CAM--':
        calibration_cam = dev.dev_cam
        



sg.PopupAnimated(image_source=None)

print_log(f'{len(devs_list)} devices found')

script_condition = Condition()

last_subscript = []                         # workaround for pySimpleGUI bug / Table.update( failes to update)

while True:                                                             # Event Loop
    
    event, values = window.read(timeout=100) 
    if not event == '__TIMEOUT__':
        print_BUGBUG(f'Proceeding  {event} event')
    

    if event == sg.WIN_CLOSED or event == 'Exit':

        sg.PopupAnimated(image_source='Images/attention_image.gif',message='Wait for Zaber devices be parked',no_titlebar=False)
        Zaber_Motor.park_all()
        sg.PopupAnimated(image_source=None)
        break

    elif emergency_stop_pressed == True and no_motion():
        emergency_stop_pressed = False
        sg.popup_auto_close(f'Emergency stop\ndone', non_blocking = True)

    if event == '__TIMEOUT__':
        ThreadCloseProceedure()
        continue

    elif event == 'Park':
        if not no_motion():
            continue
        print_log(f'Parking all devices')
        Zaber_Motor.park_all()
        for i in range (1, zaber_motors + 1):
            DeActivateMotorControl('--ZABER--', i)
            SetLED (window, f'_zaber_{i}_', 'blue')
        continue
    elif event == 'Unpark':
        if not no_motion():
            continue
        print_log(f'Unparking all devices')
        Zaber_Motor.unpark_all()
        Correct_ZB_possition_after_unpark()
        for i in range (1, zaber_motors + 1):
            ActivateMotorControl('--ZABER--', i)
        continue

    elif event == 'Version':
        sg.popup_scrolled(sg.get_versions(), keep_on_top=True)
        continue
    elif event == 'File Location':
        sg.popup_scrolled('This Python file is:', __file__)   
        continue 
    elif event == 'Stop':
        emergency_stop_pressed = True
        stop_script_flag = True
        # window[f'Run'].update(disabled = False)

        continue
    elif event == 'Run':
        window[f'Run'].update(disabled = True)
        window.perform_long_operation(lambda :
            run_script_cont(window, values['-TABLE-']), '-SCRIPT-RUN-DONE-')
        continue
    
    

# SCRIPT    
#     
    elif event in ['Step', '-TABLE-', 'Open File']: 
        if no_motion():
            print_log(f'No motion & event = {event}')
            event_procedure_in_progress = True
            script_proc(window,event, values)
            time.sleep(0)                           # explisit context switch
            event_procedure_in_progress = False
        continue




    # elif event in ['Step', '-TABLE-', 'Open File', 'Clear Log', 'Save Log']: 
    #     if no_motion():

    #         subscript = script_proc(window,event, values)
            
    #                                                                         ########## workaround for pySimpleGUI bug / Table.update( failes to update)
    #         if subscript == last_subscript:
    #             print_BUGBUG(f'subscript = {subscript}, last_subscript = {last_subscript}')
    #             continue
    #         else:
    #             last_subscript = subscript
            
    #                                                                     ###############
    #         time.sleep(0)                           # explisit context switch
    #         if len(subscript):                       # non empty script
    #             with script_condition:
    #                 script_condition.notify()
    #             print_BUGBUG(f'Notification sent')
    #     continue

    elif event == '-SCRIPT-RUN-DONE-':
        window[f'Run'].update(disabled = False)
        sg.popup_auto_close('Scritp done')
        continue

    elif event == 'HOTAIR-START':
        if (ni_dev):
            DeActivateMotorControl('--DAQ-NI--')
            print_log(f'Operating  JTSE Hot Air Station')
            ni_dev.jtseStart()
            ActivateMotorControl('--DAQ-NI--')
        else:
            print_err(f'No NI device found to operate JTSE control')

        continue

    elif event == 'HOTAIR-STOP':
        if (ni_dev):
            DeActivateMotorControl('--DAQ-NI--')
            print_log(f'Stoping JTSE Hot Air Station')
            ni_dev.jtseStop()
            ActivateMotorControl('--DAQ-NI--')
        else:
            print_err(f'No NI device found to operate JTSE control')

        continue
    elif event == '-CALIBRATE_DONE-':
        dist = values["-CALIBRATE_DONE-"]
        proc_tuning(calibration_cam.motor, dist)
        ActivateMotorControl('--CAM--')
        calibration_in_process = False

    elif event == '-CALIBRATE-':
        if calibration_cam:
            print_log(f'Calibrate button pressed - {values["-Z-SELECTOR-"]} ')
            DeActivateMotorControl('--CAM--')
            SetLED (window, '_calibr_', 'DeepPink2')
            window.Finalize()
            
            calibration_in_process = True
            calibration_in_process = calibration_cam.start_cam_tuning_operation(values["-Z-SELECTOR-"], values[f'-CALIBR_PROFILE-'], window)
            if not calibration_in_process:
                print_err(f'Calibration on {values["-Z-SELECTOR-"]} failed to start')
                ActivateMotorControl('--CAM--')
                # SetLED (window, '_calibr_', 'green')
                sg.popup_auto_close('Calibration failed to start')
                step_in = False                         # script sync
                continue
                
            print_log(f'calibration will be operated on Motor # {int(values["-Z-SELECTOR-"][-1])} ({values["-Z-SELECTOR-"]})')
            continue
        else:
            print_err(f'No ModBus/Camera server connection established. Calibration could not be performed')
            
             
    elif event == '-CALIBR_PROFILE-':
        if not file_name_validator(values[f'-CALIBR_PROFILE-']):    
                window[f'-CALIBR_PROFILE-'].update(values[f'-CALIBR_PROFILE-'][:-1])
        else:
            pass
    
    d_index = LocateDevice(event)

    if d_index < 0:
        continue

    # print_log(f'event = {event}, d_index = {d_index}')

    
# ZABER
    if '-ZABER-DST-' in event  :
        i = (int)(event[1:3])
        if  event == f'-{i:02d}-ZABER-DST-':
            print_log(f'-ZABER-DST- event received. ev = {event}, values = {values[event]}')
            if not real_validator(values[f'-{i:02d}-ZABER-DST-'], positive=True):    
                window[f'-{i:02d}-ZABER-DST-'].update(values[f'-{i:02d}-ZABER-DST-'][:-1])

                if not real_num_validator(values[f'-{i:02d}-ZABER-DST-'][:-1], positive=True):
                                                                            #invalid edited value
                    window[f'-{i:02d}-ZABER-DST-'].update(str(devs_list[d_index].dev_zaber.GetPos()))   
                    continue
            else:
                window[f'-{i:02d}-ZABER-DST-'].update(values[f'-{i:02d}-ZABER-DST-'])

            # Unpark possition update (commented) Correct_ZB_possition_after_unpark() called instead  
            # new_pos = devs_list[d_index].dev_zaber.GetPos()
            # window[f'-{i:02d}-ZABER-POSSITION-'].update(value = new_pos)

    elif '-ZABER-VELOCITY-' in event  :
        i = (int)(event[1:3])
        if  event == f'-{i:02d}-ZABER-VELOCITY-':
            print_log(f'-ZABER-VELOCITY- event received. ev = {event}, values = {values[event]}')
            if not int_validator(values[f'-{i:02d}-ZABER-VELOCITY-'], positive=True):    
                window[f'-{i:02d}-ZABER-VELOCITY-'].update(values[f'-{i:02d}-ZABER-VELOCITY-'][:-1])
                
                if not int_num_validator(values[f'-{i:02d}-ZABER-VELOCITY-'], positive=True):
                                                                            #invalid edited value
                    window[f'-{i:02d}-ZABER-VELOCITY-'].update(str(devs_list[d_index].dev_zaber.velocity_in_percents))  
                    continue 
                
            
            print_log(f'Corrected value = {values[f"-{i:02d}-ZABER-VELOCITY-"]}')
            devs_list[d_index].dev_zaber.velocity_in_percents = values[f'-{i:02d}-ZABER-VELOCITY-']



    elif '-ZABER-GTD-' in event:
        i = (int)(event[1:3])
        if not real_num_validator(values[f'-{i:02d}-ZABER-DST-'], positive=True):
                                                                # the value is invalid
            window[f'-{i:02d}-ZABER-DST-'].update(str(devs_list[d_index].dev_zaber.GetPos())) 
        else:
            DeActivateMotorControl(f'--ZABER--', devs_list[d_index].c_gui)
            devs_list[d_index].dev_zaber.move_abs(values[f'-{i:02d}-ZABER-DST-'])

    elif 'ZABER-GH-' in event:
        DeActivateMotorControl(f'--ZABER--', devs_list[d_index].c_gui)
        devs_list[d_index].dev_zaber.move_home()
        pass


# GRIPPER
    elif '-GRIPPER-RPM-' in event  :
        i = event[1]
        if  event == f'-{i}-GRIPPER-RPM-':
            if not int_validator(values[f'-{i}-GRIPPER-RPM-'], positive=True):    
                window[f'-{i}-GRIPPER-RPM-'].update(values[f'-{i}-GRIPPER-RPM-'][:-1])
                if not int_num_validator(values[f'-{i}-GRIPPER-RPM-'][:-1], positive=True):
                                                                            #invalid edited value
                    update_val = 'unmeasured'
                    if devs_list[d_index].dev_fh.fh_type == '--GRIPPERv3--': 
                        update_val = str(devs_list[d_index].dev_fh.rpm)
                    if devs_list[d_index].dev_fh.fh_type == '--GRIPPERv3--': 
                        update_val = str(devs_list[d_index].dev_fh.el_voltage)
                    window[f'-{i}-GRIPPER-RPM-'].update(update_val)   
                    continue
            
            new_val = values[f'-{i}-GRIPPER-RPM-']
            if devs_list[d_index].dev_fh.fh_type == '--GRIPPERv3--':
                devs_list[d_index].dev_fh.rpm = new_val
            elif devs_list[d_index].dev_fh.fh_type == '--GRIPPER--':
                devs_list[d_index].dev_fh.el_voltage = new_val
            else:
                print_err(f'Worng device type {devs_list[d_index].dev_fh.fh_type} at event {event}')
    elif '-GRIPPER-CURR-' in event  :
        i = event[1]
        if  event == f'-{i}-GRIPPER-CURR-':
            if not int_validator(values[f'-{i}-GRIPPER-CURR-'], positive=True):    
                window[f'-{i}-GRIPPER-CURR-'].update(values[f'-{i}-GRIPPER-CURR-'][:-1])
                if not int_num_validator(values[f'-{i}-GRIPPER-CURR-'][:-1], positive=True):
                                                                            #invalid edited value
                    window[f'-{i}-GRIPPER-CURR-'].update(str(devs_list[d_index].dev_fh.el_current_limit))   
                    continue 

            devs_list[d_index].dev_fh.el_current_limit = values[f'-{i}-GRIPPER-CURR-']
    elif f'-GRIPPER-ON-' in event:
        window[f'-{devs_list[d_index].c_gui}-GRIPPER-ON-'].update(button_color='dark green on green')
        window[f'-{devs_list[d_index].c_gui}-GRIPPER-OFF-'].update(button_color='white on red')

        DeActivateMotorControl(f'--GRIPPERv3--', devs_list[d_index].c_gui)
        devs_list[d_index].dev_fh.gripper_on()
        devs_list[d_index].dev_fh.fh_pressed = True


    elif f'-GRIPPER-OFF-' in event:
        window[f'-{devs_list[d_index].c_gui}-GRIPPER-OFF-'].update(button_color='tomato on red')
        window[f'-{devs_list[d_index].c_gui}-GRIPPER-ON-'].update(button_color='white on green')

        DeActivateMotorControl(f'--GRIPPERv3--', devs_list[d_index].c_gui)
        devs_list[d_index].dev_fh.gripper_off()
        devs_list[d_index].dev_fh.fh_pressed = True




    elif  f'-GRIPPER_TARGET-' in event:
        i = event[1]
        set_pos = values[f'-{i}-GRIPPER_TARGET-']
        if  not int_validator(set_pos):

            window[f'-{i}-GRIPPER_TARGET-'].update(set_pos[:-1])
            if not int_num_validator(set_pos[:-1]):
                set_pos = str(devs_list[d_index].dev_fh.fh_get_cur_pos())
                window[f'-{i}-GRIPPER_TARGET-'].update(str(set_pos))

        else:
            devs_list[d_index].dev_fh.fh_pos = set_pos
            window[f'-{i}-GRIPPER_TARGET-'].update(set_pos)
        
            #


    elif '-GRIPPER_POS_SET-' in event:
        i = event[1]

        go_pos = values[f'-{i}-GRIPPER_TARGET-']


        print_log(f'Move GRIPPER {devs_list[d_index].c_gui}/{i} to {go_pos} position')
        devs_list[d_index].dev_fh.go2pos(go_pos)
        devs_list[d_index].dev_fh.fh_pressed = True
        DeActivateMotorControl(f'--GRIPPERv3--', devs_list[d_index].c_gui)
    

    
    elif '-GRIPPER_STOP-' in event:
        devs_list[d_index].dev_fh.fh_stop()
        ActivateMotorControl(devs_list[d_index].dev_fh.fh_type, devs_list[d_index].c_gui)
        
    elif '-GRIPPER_TARGET_RESET-' in event:
        devs_list[d_index].dev_fh.fh_reset_pos()
        window[f'-{i}-GRIPPER_POSSITION-'].update(value = 0)
        window[f'-{i}-GRIPPER_TARGET-'].update(value = 0) 
    


    elif  f'-DIST_ROTATOR_TARGET-' in event:
        i = event[1]
        set_pos = values[f'-{i}-DIST_ROTATOR_TARGET-']
        if  not int_validator(set_pos):

            window[f'-{i}-DIST_ROTATOR_TARGET-'].update(set_pos[:-1])
            if not int_num_validator(set_pos[:-1]):
                set_pos = str(devs_list[d_index].dev_fh.fh_get_cur_pos())
                window[f'-{i}-DIST_ROTATOR_TARGET-'].update(str(set_pos))

        else:
            devs_list[d_index].dev_fh.fh_pos = set_pos
            window[f'-{i}-DIST_ROTATOR_TARGET-'].update(set_pos)
        
            #
    elif '-DIST_ROTATOR_VELOCITY-' in event:
        i = event[1]
        if not int_validator(values[f'-{i}-DIST_ROTATOR_VELOCITY-'], positive=True):
            window[f'-{i}-DIST_ROTATOR_VELOCITY-'].update(values[f'-{i}-DIST_ROTATOR_VELOCITY-'][:-1])
            if not int_num_validator(values[f'-{i}-DIST_ROTATOR_VELOCITY-'][:-1]):
                window[f'-{i}-DIST_ROTATOR_VELOCITY-'].update(str(devs_list[d_index].dev_fh.rpm))
                continue

        devs_list[d_index].dev_fh.rpm = values[f'-{i}-DIST_ROTATOR_VELOCITY-']

    elif '-DIST_ROTATOR_POS_SET-' in event:
        i = event[1]
        go_pos = 0
        go_pos = round(float(values[f'-{i}-DIST_ROTATOR_TARGET-']))

        print_log(f'Move DIST ROTATOR {devs_list[d_index].c_gui}/{i} to {go_pos} position')
        devs_list[d_index].dev_fh.go2pos(go_pos)
        devs_list[d_index].dev_fh.fh_pressed = True
        DeActivateMotorControl(f'--DIST_ROTATOR--', devs_list[d_index].c_gui)
    
    elif '-DIST_ROTATOR_RIGHT-' in event:
        DeActivateMotorControl('--DIST_ROTATOR--', devs_list[d_index].c_gui)
        devs_list[d_index].dev_fh.fh_forward()
        devs_list[d_index].dev_fh.fh_pressed = True

    elif '-DIST_ROTATOR_LEFT-' in event:
        DeActivateMotorControl('--DIST_ROTATOR--', devs_list[d_index].c_gui)
        devs_list[d_index].dev_fh.fh_backwrd()
        devs_list[d_index].dev_fh.fh_pressed = True
    
    elif '-DIST_ROTATOR_STOP-' in event:
        devs_list[d_index].dev_fh.fh_stop()
        ActivateMotorControl('--DIST_ROTATOR--', devs_list[d_index].c_gui)
        
    elif '-DIST_ROTATOR_TARGET_RESET-' in event:
        i = event[1]
        devs_list[d_index].dev_fh.fh_reset_pos()
        window[f'-{i}-DIST_ROTATOR_POSSITION-'].update(value = 0)
        window[f'-{i}-DIST_ROTATOR_TARGET-'].update(value = 0) 
    
    elif '-DIST_ROTATOR-CURR-' in event:
        i = event[1]
        if not int_validator(values[f'-{i}-DIST_ROTATOR-CURR-'], positive=True):    
            window[f'-{i}-DIST_ROTATOR-CURR-'].update(values[f'-{i}-DIST_ROTATOR-CURR-'][:-1])
            if not int_num_validator(values[f'-{i}-DIST_ROTATOR-CURR-'][:-1], positive=True):
                                                                        #invalid edited value
                window[f'-{i}-DIST_ROTATOR-CURR-'].update(str(devs_list[d_index].dev_fh.el_current_limit))   
                continue 
            
        devs_list[d_index].dev_fh.el_current_limit = values[f'-{i}-DIST_ROTATOR-CURR-']

############## 


  
    

 
print_log(f'Exiting main loop')

for m_dev in devs_list:
    print_log(f'Destroying device {m_dev}')
    if m_dev.dev_fh:                                #BUGBUG
        print_log('Stoping watchdog')
        m_dev.dev_fh.fn_in_motion = False
    # del m_dev
    # devs_list.remove(m_dev)
    



window.close()
