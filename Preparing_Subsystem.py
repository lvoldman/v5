
# from operator import pos, truediv
# from weakref import finalize

from re import T
import PySimpleGUI as sg


from bs1_utils import port_scan, CDev, gif103
from bs1_faulhaber import FH_Motor
from bs1_zaber import Zaber_Motor
from bs1_script import Tbl, Tbl_event_proc, Log_tbl, groups
from bs1_ni6002 import NI6002
from bs1_cam_modbus import Cam_modbus

import os, sys, time,  logging, re

"""
    Motors' control Dashboard

    LV Copyright 2023
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


format = "%(asctime)s: %(filename)s--%(funcName)s/%(lineno)d -- %(thread)d [%(threadName)s] %(message)s" 
logging.basicConfig(format=format, level=logging.DEBUG, datefmt="%H:%M:%S")
print_log = logging.debug


zaber_motors = 2

emergency_stop_pressed = False
stop_script_flag = False


CUSTOM_MENU_RIGHT_CLICK_VER_LOC_EXIT = ['', ['Version', 'File Location', 'Exit']]


#================  GUI utils start here =======================

def ActivateMotorControl(type, index = None):
    if type == '--TROLLEY--':
        SetLED (window, '_trolley_', 'green')
        window['-TROLLEY_POS_SET-'].update(disabled = False)
        window['-TROLLEY_TARGET_RESET-'].update(disabled = False)
        window['-TROLLEY_LEFT-'].update(disabled = False)
        window['-TROLLEY_RIGHT-'].update(disabled = False)
        window['-TROLLEY_TARGET-'].update(disabled = False)
        window['-TROLLEY_VELOCITY-'].update(disabled = False)
        window[f'-TROLLEY-CURR-'].update(disabled = False)
    
    elif type == '--ZABER--':
        SetLED (window, f'_zaber_{index}_', 'green')
        window[f'-{index:02d}-ZABER-GTD-'].update(disabled = False)
        window[f'-{index:02d}-ZABER-GH-'].update(disabled = False)
        window[f'-{index:02d}-ZABER-DST-'].update(disabled = False)
        window[f'-{index:02d}-ZABER-VELOCITY-'].update(disabled = False)
    else:
        print_log(f'PANIC - wrong type {type}')


def DeActivateMotorControl(type, index = None):
    if type == '--TROLLEY--':
        SetLED (window, '_trolley_', 'red')
        window['-TROLLEY_POS_SET-'].update(disabled = True)
        window['-TROLLEY_TARGET_RESET-'].update(disabled = True)
        window['-TROLLEY_LEFT-'].update(disabled = True)
        window['-TROLLEY_RIGHT-'].update(disabled = True)
        window['-TROLLEY_TARGET-'].update(disabled = True)
        window['-TROLLEY_VELOCITY-'].update(disabled = True)
        window[f'-TROLLEY-CURR-'].update(disabled = True)
    
    elif type == '--ZABER--':
        SetLED (window, f'_zaber_{index}_', 'red')
        window[f'-{index:02d}-ZABER-GTD-'].update(disabled = True)
        window[f'-{index:02d}-ZABER-GH-'].update(disabled = True)
        window[f'-{index:02d}-ZABER-DST-'].update(disabled = True)
        window[f'-{index:02d}-ZABER-VELOCITY-'].update(disabled = True)
    else:
        print_log(f'PANIC - wrong type {type}')

def InitDevs(devs_list):
    for m_dev in devs_list:
        if m_dev.C_type == '--TROLLEY--':
            ActivateMotorControl('--TROLLEY--')
            window['-TROLLEY_POSSITION-'].update(value = m_dev.dev_fh.fh_pos)
            window['-TROLLEY_VELOCITY-'].update(value = m_dev.dev_fh.rpm)
            window['-TROLLEY-CURR-'].update(value = m_dev.dev_fh.el_current_limit)
        
        elif m_dev.C_type == '--ZABER--':
            window[f'-{m_dev.c_gui:02d}-ZABER-POSSITION-'].update(value = m_dev.dev_zaber.current_pos)
            window[f'-{m_dev.c_gui:02d}-ZABER-VELOCITY-'].update(value = m_dev.dev_zaber.velocity_in_percents)
            ActivateMotorControl(f'--ZABER--', m_dev.c_gui)
        else:
            print_log(f'PANIC - wrong device {m_dev.C_type} in the list at {m_dev.c_gui} position')

def LEDIndicator(key=None, radius=30):
    return sg.Graph(canvas_size=(radius, radius),
             graph_bottom_left=(-radius, -radius),
             graph_top_right=(radius, radius),
             pad=(0, 0), key=key)

def SetLED(window, key, color):
    graph = window[key]
    graph.erase()
    graph.draw_circle((0, 0), 12, fill_color=color, line_color=color)



def zaber_block(i):
    return (
            [[sg.Text(f'Zaber {i}', font='Any 10'), LEDIndicator(f'_zaber_{i}_')],
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

def real_num_validator (str, positive = False):

    if not positive:
        int_nu = re.compile(r'-?\d+(\.\d+)?$') 
    else:
        int_nu = re.compile(r'\d+(\.\d+)?$')  
    if not int_nu.match(str):
        return False
    
    return True

def int_num_validator (str, positive = False):
    
    if not positive:
        int_nu = re.compile(r'-?\d+$')
    else:
        int_nu = re.compile(r'\d+$')
    if not int_nu.match(str):
        return False
    return True


def real_validator (str, positive = False):
    if not positive:
        int_nu = re.compile(r'-?(\d+(\.\d*)?){0,1}$') 
    else:
        int_nu = re.compile(r'(\d+(\.\d*)?){0,1}$') 
    if not int_nu.match(str):
        return False
    
    return True
    
    

def int_validator (str, positive = False):
    if not positive:
        int_nu = re.compile(r'-?\d*$')
    else:
        int_nu = re.compile(r'\d*$')
    if not int_nu.match(str):
        return False
    return True

    

def LocateDevice (ev_name) -> int:

    d_index = 0
    for m_dev in devs_list:
        if 'TROLLEY' in ev_name and m_dev.C_type == '--TROLLEY--':
            break

        elif  'ZABER' in ev_name and m_dev.C_type == '--ZABER--':
            # print_log(f'm_dev.c_gui = {m_dev.c_gui}  type(ev_name) = {type(ev_name)} \
            #           ev_name = "{ev_name}" ev_name[1:3] = "{(int)(ev_name[1:3])}"')
            if  m_dev.c_gui == int (ev_name[1:3]):
                print_log(f'Found device at d_index = {d_index} for event "{ev_name}"')
                break
        d_index += 1

    if d_index >= len(devs_list):
            print_log(f'Non device assosiated event {ev_name}')
            return -1
        
    # print_log(f'Device located at index {d_index}')
    return (d_index)

def Correct_ZB_possition_after_unpark():
    for m_dev in devs_list:
        if m_dev.C_type == '--ZABER--':
            new_pos = m_dev.dev_zaber.GetPos()
            window[f'-{m_dev.c_gui:02d}-ZABER-POSSITION-'].update(value = new_pos)



def ThreadCloseProceedure():
    global emergency_stop_pressed
    global stop_script_flag

    for m_dev in devs_list:
        if m_dev.C_type == '--TROLLEY--':
            if not m_dev.dev_fh.wd:                     # if no running watch dog -> continue
                continue

            alive = m_dev.dev_fh.wd.is_alive()

            if m_dev.dev_fh.fh_pressed and not alive:
                                                        # watch dog is not active anymore
                print_log(f'Found completed WD for device {m_dev.C_type} on port {m_dev.C_port} with GUI_ID={m_dev.c_gui}')
                if m_dev.C_type == '--TROLLEY--':
                    ActivateMotorControl('--TROLLEY--')
                    new_pos = m_dev.dev_fh.fh_get_cur_pos()
                    window['-TROLLEY_POSSITION-'].update(value = new_pos)
                    otf_cur = m_dev.dev_fh.el_current_on_the_fly
                    window['-TROLLEY_CUR_DISPLAY-'].update(value = otf_cur)

                
                m_dev.dev_fh.fh_pressed = False
                
            elif m_dev.dev_fh.fh_pressed and alive:
                if emergency_stop_pressed:
                        m_dev.dev_fh.fh_stop()

                if m_dev.C_type == '--TROLLEY--':
                    new_pos = m_dev.dev_fh.fh_get_cur_pos()
                    window['-TROLLEY_POSSITION-'].update(value = new_pos)
                    otf_cur = m_dev.dev_fh.el_current_on_the_fly
                    window['-TROLLEY_CUR_DISPLAY-'].update(value = otf_cur)

        elif m_dev.C_type == '--ZABER--':
            # if not m_dev.dev_zaber.wd:                # comment out to display position even while moving manualy
            #     continue
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
        print_log(f'Empty script')
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
        time.sleep(1)
    print_log(f'emergency_stop_pressed = {emergency_stop_pressed}, group index = {gr_index}, groups = {len (groups)}')


def script_proc(window, event, values):
    subscript = Tbl_event_proc(window, event, values)
    if subscript == []:
        print_log(f'Empty subscript')
        return subscript
    
    print_log(f'Subscript: {subscript}')
                                                            # duplicated devs are eliminated on early stage
    

    for cmd in subscript:
        print_log(f'Running now: {cmd}, dev = {cmd[1]}, type = {cmd[1][0]}, len(cmd) = {len(cmd)}')
        match cmd[1][0]:
            case 'T':                               # Trolley
                d_index = LocateDevice('-TROLLEY_TARGET_RESET-')
                                                    # Just a trick to locate device
                print_log(f'Trolley, ind = {d_index}')
                if d_index < 0:                     # No device
                    print_log(f'No active T device to execute cmd - {cmd}')
                    continue
                if len(cmd) == 5:                          # Update rpm
                    print_log(f'Update Trolley RPM =  {cmd[4]}')
                    window['-TROLLEY_VELOCITY-'].update(str(cmd[4]))
                    devs_list[d_index].dev_fh.rpm = cmd[4]
                
                if cmd[2] == 'HO':
                    new_pos = 0
                    window.write_event_value('-TROLLEY_TARGET_RESET-', None)
                    print_log(f'Reset TROLLEY home pos')
                    break

                print_log(f'Curr pos = {devs_list[d_index].dev_fh.fh_pos}, move pos = {(int)(cmd[3])}')
                move_pos = (int)(cmd[3])
                cur_pos = devs_list[d_index].dev_fh.fh_pos
                if cmd[2] == 'MR':
                    new_pos = cur_pos - move_pos
                elif cmd[2] == 'ML':
                    new_pos = cur_pos + move_pos
                else:
                    print_log(f'Error T cmd = {cmd}')
                    continue
                print_log(f'Curr pos = {cur_pos}, new pos = {new_pos}')
                devs_list[d_index].dev_fh.fh_pos = new_pos
                window['-TROLLEY_TARGET-'].update(str(new_pos))
                window.write_event_value('-TROLLEY_POS_SET-', None)
            case 'Z':
                device_n = (int)(cmd[1][1:])
                print_log(f'Zaber device = {device_n:02d}')
                d_index = LocateDevice(f'-{device_n:02d}-ZABER-POSSITION-')
                if d_index < 0:                     # No device
                    print_log(f'No active Z device to execute cmd - {cmd}')
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
                    print_log(f'Error Zaber cmd = {cmd}')
                    continue
                print_log(f'Curr pos = {cur_pos}, new pos = {new_pos}')
                window.write_event_value(f'-{device_n:02d}-ZABER-DST-', str(new_pos))
                window.write_event_value(f'-{device_n:02d}-ZABER-GTD-', None)

            case _:
                if cmd[1] == 'SYS':
                    if cmd[2] == 'HALT':
                        window.write_event_value(f'Stop', None)
                    elif cmd[2] == 'CALIBR':
                        print_log(f'CALIBR script cmd on Z{cmd[3]} motor')
                        window['-Z-SELECTOR-'].update(f'Z{cmd[3]}')
                        window.write_event_value(f'-CALIBRATE-', cmd[3])
                        pass

                    elif cmd[2] == 'TRIGGER':
                        
                        if len(cmd) == 4:
                            print_log(f'TRIGGER script cmd on SC {cmd[3]} schedule')
                            window['-SELECTOR-'].update(f'SC {cmd[3]}')
                        else:
                            print_log(f'TRIGGER script cmd on *current* schedule')
                            
                        window.write_event_value(f'TRIGGER', None)

                    elif cmd[2] == 'DELAY':
                        print_log(f'Delay = {cmd[3]}')
                        time.sleep(float(cmd[3]))
                    else:
                        print_log(f'Unrecognized SYS cmd = {cmd}')
                else:
                    print_log(f'Unrecognized script cmd = {cmd}')
        
                    



        window['-OUTLOG-'].print(f'[{cmd[0]}] {cmd[1]}->{cmd[2:]}')
    
    return subscript

def  proc_tuning(motor, dist):
    device_n = (int)(motor[1:])
    print_log(f'Zaber device = {device_n:02d}')
    d_index = LocateDevice(f'-{device_n:02d}-ZABER-POSSITION-')
    if d_index < 0:                     # No device
        print_log(f'No active Z device to execute cmd - {cmd}')
        return

    move_pos = (float)(dist)
    cur_pos = devs_list[d_index].dev_zaber.GetPos()
    new_pos = cur_pos + move_pos

    print_log(f'Curr pos = {cur_pos}, new pos = {new_pos}')
    window.write_event_value(f'-{device_n:02d}-ZABER-DST-', str(new_pos))
    window.write_event_value(f'-{device_n:02d}-ZABER-GTD-', None)

    

def no_motion():
    global emergency_stop_pressed
    for m_dev in devs_list:
        if m_dev.dev_zaber and m_dev.dev_zaber.z_in_motion  or   m_dev.dev_fh and m_dev.dev_fh.fn_in_motion:
            print_log(f'Device {m_dev.C_type} # {m_dev.c_gui} is in motion. No script/log operation allowed')
            return False
    return True



#========================================= Utils end here =======================
top_banner = [
               [sg.Text('Welder Control', font='Any 12', background_color=DARK_HEADER_COLOR, enable_events=True, \
                        grab=False), sg.Push(background_color=DARK_HEADER_COLOR),
               sg.Text('VOLDMAN-TECH © 2023', font='Any 12', background_color=DARK_HEADER_COLOR)],
               ]

trolley  = [[sg.Push(), sg.Text('Trolley', font='Any 10'), LEDIndicator('_trolley_'), sg.Push()],
            [sg.T('Position'), sg.Text("_", size=(8, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                border_width = 2, key='-TROLLEY_POSSITION-'), sg.Button(button_text = "Reset", key='-TROLLEY_TARGET_RESET-') ],
            [sg.Text('Target'), sg.Input(size=(10, 1), enable_events=True, key='-TROLLEY_TARGET-', \
                font=('Arial Bold', 8), justification='left'), sg.Button(button_text = "Set & Go", key='-TROLLEY_POS_SET-')],
            [sg.Button( button_color=sg.TRANSPARENT_BUTTON, image_filename = image_left, image_size=(22, 24), \
                image_subsample=2, border_width=0, key='-TROLLEY_LEFT-'),
                sg.Frame('',[[sg.Text('Velocity (RPM)')], [sg.Input(size=(15, 1), enable_events=True, key='-TROLLEY_VELOCITY-', \
                    font=('Arial Bold', 8), justification='left')]], border_width=0),
            sg.Button( button_color=sg.TRANSPARENT_BUTTON, image_filename = image_right, image_size=(22, 24), \
                image_subsample=2, border_width=0, key='-TROLLEY_RIGHT-')], 
            [sg.Text("_", size=(6, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                border_width = 2, key='-TROLLEY_CUR_DISPLAY-'),
                sg.Text(f'Stop (mA) >\n< Curr. (mA)'), sg.Input(size=(8, 1), enable_events=True, key=f'-TROLLEY-CURR-')],
            [sg.Button(button_text = 'Stop',  key='-TROLLEY_STOP-')]]



script = [
            [sg.Text("Command script")],
            [sg.Button('Run'), sg.Button(button_text = 'Emergency Stop', button_color = 'white on firebrick4', border_width = 10, key='Stop')], 
            [Tbl],
            [sg.Button('Step'), sg.Button('Open File')
            # [sg.Button('Run'), sg.Button('Stop'), sg.Button('Step'), sg.Button('Open File'), 
            #  sg.Button('Clear Log'), sg.Button('Save Log') 
            ], 
            [sg.Text('Run = Run the whole script now')],
            [sg.Text('Stop = Stop runing the script')],
            [sg.Text('Step = Run selected command block')],
            [sg.Text('Open File = Load command script'), sg.Sizegrip()]
            ]
 



script_col = []
zaber_col = zaber_frames(1,zaber_motors)

script_col.append([sg.Frame('', script, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                    border_width=1, vertical_alignment='down', element_justification = "center")])


weld_trigger = [[sg.Push(), sg.Button('Trigger', border_width = 5, button_color = 'red', expand_x = True, expand_y = True, key  = 'TRIGGER',), 
                 sg.OptionMenu(values=('SC 0', 'SC 1', 'SC 2', 'SC 3'), default_value = 'SC 0',  key='-SELECTOR-'),
                 sg.Push()]]

zaber_col.append([sg.Frame('', trolley, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                    border_width=3, vertical_alignment='top', element_justification = "center")])

zaber_col.append([sg.Frame('Welder', weld_trigger, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                     border_width=3, vertical_alignment='top', element_justification = "center", expand_y = False)])




layout = [
          [sg.Frame('', top_banner,   pad=(0,0), background_color=DARK_HEADER_COLOR,  expand_x=True, \
                    border_width=0, grab=True)],
        
          [   
            sg.Column(script_col, vertical_alignment = 'top', expand_y = 'True', scrollable=False, vertical_scroll_only=True, \
                     key='COLUMN-SC', justification='center', pad=0),
            sg.Column(zaber_col, vertical_alignment = 'top', expand_y = 'True', scrollable=False, vertical_scroll_only=True, \
                     key='COLUMN-LOG', justification='center',pad=0),         
            ],
        [sg.Sizegrip(background_color=BORDER_COLOR)]
        ]




#========================= GUI code starts here =============================



if __name__ == "__main__":

    window = sg.Window('Dashboard', layout, margins=(0,0), background_color=BORDER_COLOR,  \
                    # no_titlebar=False, resizable=False, right_click_menu=sg.MENU_RIGHT_CLICK_EDITME_VER_LOC_EXIT, \
                    no_titlebar=False, resizable=True, right_click_menu=CUSTOM_MENU_RIGHT_CLICK_VER_LOC_EXIT, \
                    element_justification='c', finalize = True)

    window.read(timeout=0) 
    window.Finalize()

    DeActivateMotorControl('--TROLLEY--')

    for i in range (1, zaber_motors + 1):
        DeActivateMotorControl('--ZABER--', i)


    sg.PopupAnimated(image_source=gif103, text_color='blue', 
                    message='Loading...',background_color='grey', time_between_frames=100)    


    # init devices and pereferials 
    devs_list = port_scan('welder_serials.yml')
    InitDevs(devs_list)
    ni_dev = NI6002()


    sg.PopupAnimated(image_source=None)

    print_log(f'{len(devs_list)} devices found')


    while True:                                                             # Event Loop
        
        event, values = window.read(timeout=100) 
        

        if event == sg.WIN_CLOSED or event == 'Exit':
            break

        elif emergency_stop_pressed == True and no_motion():
            emergency_stop_pressed = False
            sg.popup_auto_close(f'Emergency stop\ndone', non_blocking = True)

        
            ThreadCloseProceedure()
            continue

        elif event == '__TIMEOUT__':
            ThreadCloseProceedure()
            continue

        
        
        elif event == 'Park':
            if not no_motion():
                continue
            print_log(f'Parking all devices')
            Zaber_Motor.park_all()
            continue
        elif event == 'Unpark':
            if not no_motion():
                continue
            print_log(f'Unparking all devices')
            Zaber_Motor.unpark_all()
            Correct_ZB_possition_after_unpark()
            continue
        elif event == 'TRIGGER':
            print_log(f'Trigger - {values["-SELECTOR-"]} ')
            ni_dev.selector(int(values["-SELECTOR-"][-1]))
            ni_dev.trigger()
            print_log(f'{int(values["-SELECTOR-"][-1])} schedule trigger sent')
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
            continue
        elif event == 'Run':
            window.perform_long_operation(lambda :
                run_script_cont(window, values['-TABLE-']), '-SCRIPT-RUN-DONE-')
            continue
        
        

    # SCRIPT    
    #     LocateDevice
        elif event in ['Step', '-TABLE-', 'Open File', 'Clear Log', 'Save Log']: 
            if no_motion():
                script_proc(window,event, values)
            continue

        elif event == '-SCRIPT-RUN-DONE-':
            sg.popup_auto_close('Scritp done')
            continue
        
        d_index = LocateDevice(event)

        if d_index < 0:
            continue



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


    # 

    # TROLLEY    
    #     

        elif event == '-TROLLEY_TARGET-':
            set_pos = values['-TROLLEY_TARGET-']
            if  not int_validator(set_pos):

                                                                            # update does not always work - known tk problem
                                                                            # use new event instead
                                                                            # workaround found!!!
                window['-TROLLEY_TARGET-'].update(set_pos[:-1])
                if not int_num_validator(set_pos[:-1]):
                    set_pos = str(devs_list[d_index].dev_fh.fh_get_cur_pos())
                    window['-TROLLEY_TARGET-'].update(str(set_pos))

            else:
                devs_list[d_index].dev_fh.fh_pos = set_pos
                window['-TROLLEY_TARGET-'].update(set_pos)
            
                #
        elif event == '-TROLLEY_VELOCITY-':
            if not int_validator(values['-TROLLEY_VELOCITY-'], positive=True):
                window['-TROLLEY_VELOCITY-'].update(values['-TROLLEY_VELOCITY-'][:-1])
                if not int_num_validator(values['-TROLLEY_VELOCITY-'][:-1]):
                    window['-TROLLEY_VELOCITY-'].update(str(devs_list[d_index].dev_fh.rpm))
                    continue

            devs_list[d_index].dev_fh.rpm = values['-TROLLEY_VELOCITY-']

        elif event == '-TROLLEY_POS_SET-':
            go_pos = devs_list[d_index].dev_fh.fh_pos

            print_log(f'Move to {go_pos} position')
            devs_list[d_index].dev_fh.go2pos(go_pos)
            devs_list[d_index].dev_fh.fh_pressed = True
            DeActivateMotorControl('--TROLLEY--')
        
        elif event == '-TROLLEY_RIGHT-':
            DeActivateMotorControl('--TROLLEY--')
            devs_list[d_index].dev_fh.fh_forward()
            devs_list[d_index].dev_fh.fh_pressed = True

        elif event == '-TROLLEY_LEFT-':
            DeActivateMotorControl('--TROLLEY--')
            devs_list[d_index].dev_fh.fh_backwrd()
            devs_list[d_index].dev_fh.fh_pressed = True
        
        elif event == '-TROLLEY_STOP-':
            devs_list[d_index].dev_fh.fh_stop()
            ActivateMotorControl('--TROLLEY--')
            
        elif event == '-TROLLEY_TARGET_RESET-':
            devs_list[d_index].dev_fh.fh_reset_pos()
            window['-TROLLEY_POSSITION-'].update(value = 0)
            window['-TROLLEY_TARGET-'].update(value = 0) 
        
        elif event == '-TROLLEY-CURR-':
            if not int_validator(values['-TROLLEY-CURR-'], positive=True):    
                window[f'-TROLLEY-CURR-'].update(values[f'-TROLLEY-CURR-'][:-1])
                if not int_num_validator(values[f'-TROLLEY-CURR-'][:-1], positive=True):
                                                                            #invalid edited value
                    window[f'-TROLLEY-CURR-'].update(str(devs_list[d_index].dev_fh.el_current_limit))   
                    continue 
                
            devs_list[d_index].dev_fh.el_current_limit = values[f'-TROLLEY-CURR-']


    
        

    
    print_log(f'Exiting main loop')

    for m_dev in devs_list:
        print_log(f'Destroying device {m_dev}')
        if m_dev.dev_fh:                                #BUGBUG
            print_log('Stoping watchdog')
            m_dev.dev_fh.fn_in_motion = False
        del m_dev
        

    window.close()

