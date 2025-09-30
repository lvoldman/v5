__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["Leonid Voldman","Sasha Schechtman"]
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
import random
import os
from pathlib import Path



import bs1_proc_manager as pm
import bs1_status_monitor as sm 
import bs1_mecademic as mc



from bs1_config import port_scan, CDev, gif103, read_params, get_dev, getDevsList, load_dev_config
from bs1_faulhaber import FH_Motor
from bs1_zaber import Zaber_Motor
from bs1_script import Tbl, Log_tbl, groups, GetSubScript, LoadScriptFile, ClearLog, SaveLog, GetSubScriptDict
from bs1_ni6002 import NI6002
from bs1_cam_modbus import Cam_modbus, camRes
from bs1_FHv3 import FH_Motor_v3
from bs1_maxon import MAXON_Motor
from bs1_anim_0MQ import anim_0MQ
from bs1_phidget import PhidgetRELAY
from bs1_marco_modbus import Marco_modbus, pulseData


import os, sys, time, re
import math

from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, real_num_validator, \
    int_num_validator, real_validator, int_validator, file_name_validator, void_f, clearQ, removeElementQ, event2GUI, \
    non_empty_string_validator

# print_DEBUG = void_f


DEFAULT_GEAR = 64
# R1_PITCH = 1.27
DEFAULT_DIAMETER = 6
# CPT = 1024              #  counts per turn
CPT = 512              #  counts per turn
QUAD = 4                # quadrature encoding



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
image_on = './Images/on.png'
image_off = './Images/off.png'

# ====  default GUI configuration
zaber_motors = 16
gripper_motors = 4
trolley_motors = 3
distance_rotators = 3
time_rotators = 1
dh_grippers = 2
cams = 0
daq = 0
hmp = 0
phg = 0
mcdmc = 0
marco = 0
intrlck = 0

activeGUIflag = False

zaber_motors_in_GUI = 6
rotator_in_GUI = 4


# ==== Global vars
# emergency_stop_pressed = False
calibration_in_process = False
step_in = False                                 # used to sync non motion devices (DELAY, CALIBR, etc)
event_procedure_in_progress = False


ZMQ_PORT = 6001


# OFF button diagram
toggle_btn_off = b'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'

# ON button diagram
toggle_btn_on = b'iVBORw0KGgoAAAANSUhEUgAAAGQAAAAoCAYAAAAIeF9DAAARfUlEQVRoge1bCZRVxZn+qure+/q91zuNNNKAtKC0LYhs3R1iZHSI64iQObNkMjJk1KiJyXjc0cQzZkRwGTPOmaAmxlGcmUQnbjEGUVGC2tggGDZFBTEN3ey9vvXeWzXnr7u893oBkjOBKKlDcW9X1a137//Vv9ZfbNmyZTjSwhiDEAKGYVSYpnmOZVkzTdM8zTTNU4UQxYyxMhpzHJYupVSvUmqr67pbbNteadv2a7Ztd2SzWTiOA9d1oZQ6LGWOCJAACMuyzisqKroqGo1eYFlWxDRN3c4512OCejwWInZQpZQEQMa27WXZbHZJKpVank6nFYFzOGAOCwgR2zTNplgs9m/FxcXTioqKEABxvBL/SAsRngCwbXtNOp3+zpSLJzf3ffS5Jc8X/G0cam7DMIqKioruLy4uvjoej7NIJBICcbDnIN78cBXW71qH7d3bsTvZjoRMwpE2wIirjg0RjlbRi1wBBjcR5zFUx4ajtrQWZ46YjC+Mm4Gq0ipNJ8MwiGbTTNN8a+PyTUsSicT1jXMa0oO95oAc4k80MhqNvlBWVjYpHo9rrqD2dZ+sw9I1j6Nl/2qoGCCiDMzgYBYD49BghGh8XlEJRA5d6Z8EVFZBORJuSgEJhYahTfj7afMweczkvMcUcct7iUTikvr6+ta+0xIWAwJimmZdLBZ7uby8fGQsFtMo7zq4C/e+cg9aupphlBngcQ5OIFAVXvXA6DPZ5wkUIr4rAenfEyDBvfTulaMgHQWVVHC6HTSUN+GGP78JNUNqvCmUIiXfmkwmz6urq3s/f/oBARFC1MTj8eaKigq6ajCW/eZXuKd5EbKlGRjlBngRAzO5xxG8z0v7AAyKw2cNH180wQEmV07B2dUzcWbVFIwqHY2ySJnu68p04dOuHVi/Zx3eaF2BtXvXQkFCOYDb48LqieDGxptxwaQLw2kdx9mZSCSa6urqdgZt/QDhnBfFYjECY1JxcbEWU4+8/jAe+/DHME8wYZSIkCMKgOgLwueFKRTAJMPsmjm4YvxVGFUyyvs2LbF8iRCIL7+dLjs6d+DhdUvw7LZnoBiJMQnnoIP5p1yOK//sG+H0JL56e3ub6uvrtU4hLEKlTvrBNM37iouLJwWc8ejKH+Oxjx+FVW1BlAgtosDzCJ4PxEAgfJa5RAEnWiNw39QHcPqQCfqltdXkSCSSCWTSaUgyYcn4IZegqAiaboJjVNloLDxnMf667qu47pVvY5e7E2aVicc+ehScMVw+80r9E4ZhEK3vA/At+BiEHGIYRmNJScnblZWVjPTGyxuW4Z9Xf0+DYZQKMLM/GP2AGOy+X+cfdyElPbVsKu6f/gNURCr0uyaTSXR2duqrOsTXEO3Ky8v1lQZ1JA/i2hevwbsH10K5gL3fxh1Nd+L8My7wcFdKJZPJGePGjWt+9dVXPcHDGGOWZT1YXFysTdu2g21Y3Hy3FlPEGQVgMNYfDNa35hpyDiM+E5Wo3VTRhIdm/AjlVrn2I3bv3o329nakUin9LZyR/mQFzjCtfMY50qkU2ne362dcx0V5tAI/mfMEmqq+qEkiKgwsfvtu7DqwCwHtI5HIA3RvWZYHiBDiy0VFRdrpIz/jnlcWwy7Nap1RIKYCwvJBwAhByBG/P1h/xBXA6Oho3DvtARgQsG0HbW3tSCZT4AQAzweDhyBQG3iwSD2Akqkk2tva4WQdGNzAgxf9O0Zbo8EFQzaWweLli0KuEkI0bNu2bRbRn/viisIhWom/t2N9aNqyPjpjUK5AHhfwvHb+2QKEKYbvT1iIGI/BcST27dsL13U8MBgPweB5HOFd6W+h+7kPEFXHdbBn7x44rouoGcXds+4FyzDwIo6Wjmas274u4BKi/TWEAeecVViWdWEkYsEwBJauecLzM6LeD/VV4H3VwoT4GVgw7nZsvPgDr17k1VtOuh315gQoV/lWCXDr2O9i44Uf6HrL6Nshs7k+Kj9r+LnuWzFzFWRKes8eraKAi4ddgtPK66GURGdXpw8GL6gBR/S9Emhhf95VShddHR06vjVh+ARcMma29llEXODJtY+HksQwBGFQwTkX51qWZZmmhY7eTryzvxk8xrWfEZq2g+iM2SfMxf+c8xS+Ov5r/aj2d/Vfw09nPY1LSudoR8nXYGH/nHFzUS8nQNoyN2fQTcrvgANlq6PHIS4wr3a+Jlw6nUY2kwFjwhNPeaAInzOED4B3ZXmgsQI9Q5yTzmaQTmf03P/YcCVUGtp1WL2nGQd7OnwJwwmDc7kQ4ktBsPDNraugogCPHMKCYjnOuKvh7sMu34VnL0K9mgDpFOCBmBXD9WfeCJlU2qop4EByetN57X/oCoZJpZNRUzQSUklPeXMGoQEQ+toXGOYT3yO8yOMUkQcU1zpDcKHnpLlHVYzE5KopmkukCaza+uvwswkLAuR00u4EyLq2dV5symT9uaMAGIYrx14VNm1u3YQrHr8ctYtH4eT7R+PKn16Bzbs2hf3fGH81ZMItEE9UGsY0YHblXMBWA0ZcjlalldJU+QVNMOlKuFLqlU2rmAt/pecTXARXGuMBE4BGY3QANtyW8MAjn4XmllLhi6PO0iEWbgJrW9eGlhphwTnnY4P9jO0d27yQiBjEys5rbhjeqK879u3AxUsvxBvdr8EabsIaYWEVW4mvvHYpNrdv1mOaxjRB9voxIL88t/ZZfXP9jBvg9rr6BY9ZkcDpJRM0sRzb8QnsrWweXj1OITA05wTcQhwkhC/GvH4CQfgACh8w4iLbsbXYmnjiRB1WodXwScf2vEXITua0yxdsMu1Ot4MZrD8gff6cEJ+ImBnT98RyIs5hVAkYFYY2CMiRNCoNvHdgvR4Ti8QwMXpGASBL1z+BfT37MLRkKG4bf4dW4seqkCitiY7UxCIuITHFfTACEcR9YueLKw2CyOkW4hjBcyB4QOXaaH7y9kdVjgZ8g6U92Z7zZTgvJ0BKg4akm/ydHeruTDd4lOtKYAY6hpsMWxKbw3G1JWMLAGECeHrTU/p+7sSvoJ5P7CfSjlqRCnEjpsGAvykXiqVAmefpDtGnzauij0Um+t0TaQiUkkiJJxGUQoponuOQUp7vbarfgyKlRaXa9xho97C+4vTwftuBjwq1Omd48KMHsK93n+ag6yffqEMLx6SQESHJiJDeShV9iRuII5EHggg5RlejcHzQJ/KAIVGmuZA4Rfr7KAqFHr9SqjvYC46J2BGt0o29G5C0PWTPn3CBP3nhg/RDM6pn6PtkJon1nev7+TLEUQ+sv1/fk4IfUznmGCHihdClv2C0qBKFYGjlzVjhqmf9uSGnW3JmsAZSeFYSgd6Z6PJ+VAExEQ3fgbDgfsaEbhgeG6FZqZ9DNgBIq3d628NDS4fi2Yt/gdkVcz02lApfKpuJn037X4wuPUmP2di60RNnffZOiLNe6HwOm/d6oo1M4WNSGNCa+K1nBSnlE1uEK531UeqBWat1hfBM2wAAFoq6PCNAr36hudBVEjv2f+J9pVSojg7PTw7p5FLKj4NMiNqyWij7EB5y0MyARz58KGyuP7EeC2cuwqa/2Ko97f9oWoLThtSH/YtXLNKbWgX6KdhGEMB/fbT02AARFM6wqWOj9tBdx4Eg38E3ebnvhwiWrz9EKNY8P0XkiTkRWmnM7w84xXFtSFdhQ+t7Hi2kwpiK2vA1lFLbSGRtIkBIrk0bNU3vCWsPWYajCkS/R0iFjakNWLDilsN+681P3YgNqfUQxQIQhX3eljTDCx3PoaX1nf59R6lSWX2wWfsfru8vhA5eYLaKfEXPwvAJ83WDNnEDMISvX4QIn9W6Qy98ibe2v6mlA+WDTB05NeQQKeVm4pBfU74QPXDWqWeBpQCZUWFWRSEQuS1NmvC5jmfxV8/8JZ58p/8KX7rqCcx9ZA5+3vY0jAqh9+ALOSRHbZrrX7fQPs0xQoQpbOrdgJ09rZoOyXRa6wvB8j10plc744Gz6HEN90MnIvTchecMEucwFoou7alLhU/3/xbv7f6N53DbDGefdnb4yVLKlez111+vKCkp2V1VVWXRtu21//1NtDirYZ5ggFs8t6oHimfBQ1mlXLgJ6QUEHS/+pL3cGIco5uAxoc1g6nO6XDhdju43hxge5zAvOYD2n50OFzIrdTv1kzn9By86VCMxK/ZlXFd/k/60srIyUDg897GqMN4WEkLljcj/P9eazqTR1ekp8oW//Be8tONFzTXTKxvx0PyHPQtXqWxvb281iSxKd3wpk8lodp3f+HVNMEmiS+ZFYwfJtiP3nxPxqgxY1SYiNRYiIyzttZtDDW/r1/T0Byl2USpgDaM+s4DYBBCNNYeZ+nkCQ4f/j0bx3+2VjuXYevB9zSVdXV36Gsas8i0nFlhcOasrNy4/5sW8uTq9ubbs2oKXPvylTpuSWRfzm+aH7oLruoRBh6aIbdsPEUvZto3JtVPQVDlDp7BQrlGQ5hJi0kd0wVfMRDweF7rS6qbwMnGYDuHniTwCh/pELC9Eo/JA0Vwl9J6BflbhqFT9LiZwz/t3I5FN6D2MvXv3Qfoh+HxdEYixcKcw3BPxrClPZHGd00tz0DWZSeDOl+4AIl4q0PQTGjH91Aafrjpf64eEAfdl1/JMJkPpjhrJW8+/DVZXBE6P6+1ZBKD4Cl7JAYBRuT9C8SyPDjH/XyotCJOhTe3CXevvhO1k4Dg2drfv0fvoHkegQKfkgocMHPkhFYZUKqm3cWmOrGvju8/fhtZUq168RXYRFlx0e5gFKqVsqampeYWkFPcRUplM5ju9vb10RU1VDRacdTvsvbYX+LMLQQktr4FACcaE4AT16Orp36eS+YsIx7r0u7ij5XtIZpOwaddvzx60tbUhlUoXcgXru63LtPJub2vTz5AKIKd4wTM3oWVPi97WIF1188xbcVL1SQF3UBL2dXRPtBfz5s0LOnYqpYYahjGd9kfqauqgeoCWT1v0ytHZibxvdiILdV2/GNihPP6jpBp+5xJs5XKgLdWGVTtWYnxxHYZEh2ix09Pdg67uLmRtG45taxFPFiqB0NXdjb1796K7u0uPpbK1/QPc9PwN+KDrfe2HkfX69UlX4LKZ8zR30EKl7PgRI0Y8TOMvu+yyXF6W33ljT0/PDMoXIna8etY1Or71oy0PDZwo5yt6FQDTxwIbFJRjGGk/XNGvbnBQFIkSyP9pzbdwbsUs/E3d32J46QhIx0F3VxfCXCDi/mBF6sWp0Na1E0+2PImXt70MFkHIGQTGtRd8W4MBL3uR8nxvCF6JMGArVqwoeEXDMMJUUjKDKWHuxXd/gbtWfR92Wdbbbz8OUkmVn6erUtIz6RMSddHTMH1YI+qH1uPE0hEoiRRrEHqyPWjrbMPm3ZvQ/Onb2LhvE5ihNI3IUo3YEdwycwFmN1yaD8ZOylqsra0NU0kJi36AwE+2jsfjOtk6yGJs3d+KRS8vRPOBt3LJ1hGWE2efx2RrnVztRS5kxvOzdE1LL9ud+tzCkJK3SJneoyfTtnFYE26+cAHGVI/RRkCQbJ1IJM6rra0tSLYeFJDgOEIsFguPI9A2L7Wv+XgN/vOdn6B591tAnB0fxxECYBy/ZqUHhJsLo8Pf3yBHGRmgYUQT/qFxPhrHN2ogkFMLJKYuHTt27Kd9f4awGPDAjm8XE4pNUsr7HccJD+xMPXkqpo2dhgM9B7Dy/TfwbutabOvchvYD7eh1e+HS3uTn+cCO9I+vSe+ew0CxiKM6Xo3ailpMrpmiwyHDKqpDp88/SUXW1JLe3t7rx48fP/iBnYE4JL8QupZl0ZG2H8Tj8emUs/qnI21HVvKOtLUkk8nrxo0b9/ahHhyUQ/ILOYqZTKbZcZyGTCYzK5lMfjMajZ4fiUT0oU8vIir+dOgz79CnHz3P2rb9q0wm88NTTjll+ZHOc1gOKRjsn8Y1TZOORVOC3dmWZdUbhqGPRXPOS49TQHqUUj1SSjoWvdlxnJXZbPa1bDbbQb4K1SM6Fg3g/wC58vyvEBd3YwAAAABJRU5ErkJggg=='

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
    # elif type == '--DAQ-NI--':  
    #     if f'_trigger_' in window.AllKeysDict:  
    #         SetLED (window, f'_trigger_', ledColor)
    #         window[f'TRIGGER'].update(disabled = disableInd)
    #     if f'_hotair_' in window.AllKeysDict:  
    #         SetLED (window, f'_hotair_', ledColor)
    #         window[f'HOTAIR-START'].update(disabled = disableInd)
    #         window[f'HOTAIR-STOP'].update(disabled = disableInd)

    # elif type == '--PHG--':  
    #     if f'_dispenser_' in window.AllKeysDict:  
    #         SetLED (window, f'_dispenser_', ledColor)
    #         window[f'DISP_TRIG'].update(disabled = disableInd)
    #     if f'_uv_' in window.AllKeysDict:  
    #         SetLED (window, f'_uv_', ledColor)
    #         window[f'UV_TRIG'].update(disabled = disableInd)
    #     if f'_front_light_' in window.AllKeysDict:  
    #         SetLED (window, f'_front_light_', ledColor)
    #         window[f'FRONT_LIGHT'].update(disabled = disableInd)
    #     if f'_back_light_' in window.AllKeysDict:  
    #         SetLED (window, f'_back_light_', ledColor)
    #         window[f'BACK_LIGHT'].update(disabled = disableInd) 
    #     if f'_hole_light_' in window.AllKeysDict:  
    #         SetLED (window, f'_hole_light_', ledColor)
    #         window[f'HOLE_LIGHT'].update(disabled = disableInd)
    #     if f'_trigger_' in window.AllKeysDict:  
    #         SetLED (window, f'_trigger_', ledColor)
    #         window[f'TRIGGER'].update(disabled = disableInd)
    #     if f'_hotair_' in window.AllKeysDict:  
    #         SetLED (window, f'_hotair_', ledColor)
    #         window[f'HOTAIR-START'].update(disabled = disableInd)
    #         window[f'HOTAIR-STOP'].update(disabled = disableInd)
    elif type == '--DAQ-NI--':
        pass                                                                # no GUI fot NI
    elif  type == '--PHG--':
        if f'_phg_{index}_' in window.AllKeysDict: 
            SetLED (window, f'_phg_{index}_', ledColor)
            if f'-{index}-PHG-ON-' in window.AllKeysDict: 
                window[f'-{index}-PHG-ON-'].update(disabled = disableInd)
            if f'-{index}-PHG-OFF-' in window.AllKeysDict: 
                window[f'-{index}-PHG-OFF-'].update(disabled = disableInd)
            if f'-{index}-PHG-BUTTON-' in window.AllKeysDict: 
                window[f'-{index}-PHG-BUTTON-'].update(disabled = disableInd)
            if f'-{index}--PHG-TRIGGER-OFF-' in window.AllKeysDict: 
                window[f'-{index}-PHG-TRIGGER-OFF-'].update(disabled = disableInd)
            
        else:
            pass                   # not present in GUI
    
    elif type == '--CAM--' and f'_calibr_{index}_' in window.AllKeysDict:
        SetLED (window, f'_calibr_{index}_', ledColor)
        window[f'-{index}-CALIBRATE-'].update(disabled = disableInd)
        window[f'-{index}-CALIBR_PROFILE-'].update(disabled = disableInd)

    elif type == '--MARCO--' and f'_marco_' in window.AllKeysDict:
        SetLED (window, f'_marco_', ledColor)
        window[f'-MARCO_SET_TERM-'].update(disabled = disableInd)
        window[f'-MARCO_UPDATE_TEMP-'].update(disabled = disableInd)
        window[f'-MARCO_PULSE_ON-'].update(disabled = disableInd)
        # window[f'-MARCO_SET_PULSE_ON-'].update(disabled = disableInd)
        window[f'-MARCO_PULSE_COUNT-'].update(disabled = disableInd)
        # window[f'-MARCO_SET_PULSE_COUNT-'].update(disabled = disableInd)
        window[f'-MARCO_PULSE_CYCLE-'].update(disabled = disableInd)
        # window[f'-MARCO_SET_PULSE_CYCLE-'].update(disabled = disableInd)
        
        window[f'-MARCO_SET_PULSE_DATA-'].update(disabled = disableInd)

        window[f'-MARCO_PURGE_TIME-'].update(disabled = disableInd)
        window[f'-MARCO_SET_PURGE_TIME-'].update(disabled = disableInd)
        window[f'-MARCO_RESET_PULSE_COUNT-'].update(disabled = disableInd)
        window[f'-MARCO_PURGE_ON-'].update(disabled = disableInd)
        window[f'-MARCO_PURGE_OFF-'].update(disabled = disableInd)
        window[f'-MARCO_SINGLE_SHOT_ON-'].update(disabled = disableInd)
        window[f'-MARCO_SINGLE_SHOT_OFF-'].update(disabled = disableInd)
        window[f'-MARCO_PROGRAM-'].update(disabled = disableInd)

        

# BUGBUG
    
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

    elif type == '--MCDMC--' and '_mcdmc_' in window.AllKeysDict:  
        SetLED (window, '_mcdmc_', ledColor)
        SetLED (window, '_mcdmc_active_', ledColor)
        SetLED (window, '_cognex_', ledColor)
        window['_MCDMC_SW_ON_'].update(disabled = disableInd)
        window['_MCDMC_SW_OFF_'].update(disabled = disableInd)
        window['_MCDMC_OPEN_ROBOT_CTL_'].update(disabled = disableInd)
        

    else:
        print_err(f'{"Activation" if disableInd == False else "DeActivation"} control - wrong type {type}, index = {index}')




def ActivateMotorControl(window, type, index = None):
    activationControl(window, type,  False, 'green', index)

def DeActivateMotorControl(window, type, index = None):
    activationControl(window, type,  True, 'red', index)
    pass    




def deactivateGUI(window):
    global activeGUIflag
    activeGUIflag = False

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

    for i in range (1, time_rotators + 1):
        DeActivateMotorControl(window, '--TIME_ROTATOR--')

    for i in range (1, daq + 1):
        DeActivateMotorControl(window, '--DAQ-NI--')

    for i in range (1, cams + 1):
        DeActivateMotorControl(window, '--CAM--', i)

    for i in range (1, hmp + 1):
        DeActivateMotorControl(window, '--HMP--')

    for i in range (1, phg + 1):
        DeActivateMotorControl(window, '--PHG--', i)

    for i in range (1, mcdmc + 1):
        DeActivateMotorControl(window, '--MCDMC--')

    for i in range (1, marco + 1):
        DeActivateMotorControl(window, '--MARCO--')
    pass


# def MarcoGUIUpdate(window, m_dev):
#     _pulse_data:pulseData = m_dev.dev_marco.get_pulse_data()
#     window[f'-MARCO_UPDATE_TEMP-'].update(value = m_dev.dev_marco.get_set_temp())
#     if _pulse_data:
#         window[f'-MARCO_PULSE_ON-'].update(value = _pulse_data.pulse_on)
#         window[f'-MARCO_PULSE_CYCLE-'].update(value = _pulse_data.cycle_rate)
#         window[f'-MARCO_PULSE_COUNT-'].update(value = _pulse_data.pulse_count)
#     window[f'-MARCO_PURGE_TIME-'].update(value = m_dev.dev_marco.get_purge_time())
#     window[f'-MARCO_PROGRAM-'].update(value = 'Prog '+ str(m_dev.dev_marco.program_control()))

#     if m_dev.dev_marco.get_purge_status() == 1:
#         window[f'-MARCO_PURGE_ON-'].update(button_color='dark green on green')
#         window[f'-MARCO_PURGE_OFF-'].update(button_color='white on red')
        
#     else:
#         window[f'-MARCO_PURGE_OFF-'].update(button_color='tomato on red')
#         window[f'-MARCO_PURGE_ON-'].update(button_color='white on green')

#     if m_dev.dev_marco.get_shot_status() == 1:
#         window[f'-MARCO_SINGLE_SHOT_ON-'].update(button_color='dark green on green')
#         window[f'-MARCO_SINGLE_SHOT_OFF-'].update(button_color='white on red')
        
#     else:
#         window[f'-MARCO_SINGLE_SHOT_OFF-'].update(button_color='tomato on red')
#         window[f'-MARCO_SINGLE_SHOT_ON-'].update(button_color='white on green')


#     ActivateMotorControl(window, f'--MARCO--')    

def initGUIDevs(window, devs_list:List):
    global activeGUIflag
    activeGUIflag = True

    print_log(f'Activating GUI for devices: {devs_list}')
    for m_dev in devs_list:
        if m_dev.C_type == '--TROLLEY--':
            ActivateMotorControl(window, '--TROLLEY--', m_dev.c_gui)
            window[f'-{m_dev.c_gui}-TROLLEY_POSSITION-'].update(value = m_dev.dev_mDC.mDev_pos)
            window[f'-{m_dev.c_gui}-TROLLEY_VELOCITY-'].update(value = m_dev.dev_mDC.rpm)
            window[f'-{m_dev.c_gui}-TROLLEY-CURR-'].update(value = m_dev.dev_mDC.el_current_limit)

            _title = m_dev.get_device().getTitle()
            if _title:
                window[f'-{m_dev.c_gui}-TROLLEY-TITLE-'].update(_title)

        elif m_dev.C_type == '--DIST_ROTATOR--':
            ActivateMotorControl(window, '--DIST_ROTATOR--', m_dev.c_gui)
            window[f'-{m_dev.c_gui}-DIST_ROTATOR_POSSITION-'].update(value = m_dev.dev_mDC.mDev_pos)
            window[f'-{m_dev.c_gui}-DIST_ROTATOR_VELOCITY-'].update(value = m_dev.dev_mDC.rpm)
            window[f'-{m_dev.c_gui}-DIST_ROTATOR-CURR-'].update(value = m_dev.dev_mDC.el_current_limit)

            _title = m_dev.get_device().getTitle()
            if _title:
                window[f'-{m_dev.c_gui}-DIST_ROTATOR-TITLE-'].update(_title)
            
        elif m_dev.C_type == '--TIME_ROTATOR--':
            ActivateMotorControl(window, '--TIME_ROTATOR--', m_dev.c_gui)
            window[f'-TIME_ROTATOR_TARGET-'].update(value = m_dev.dev_mDC.rotationTime)
            window[f'-TIME_ROTATOR_VELOCITY-'].update(value = m_dev.dev_mDC.el_voltage)
            window[f'-TIME_ROTATOR-CURR-'].update(value = m_dev.dev_mDC.el_current_limit)

            _title = m_dev.get_device().getTitle()
            if _title:
                # window[f'-{m_dev.c_gui}-TIME_ROTATOR-TITLE-'].update(_title)
                window[f'-TIME_ROTATOR-TITLE-'].update(_title)

        elif m_dev.C_type == '--GRIPPER--':
            ActivateMotorControl(window, f'--GRIPPER--', m_dev.c_gui)
            window[f'-{m_dev.c_gui}-GRIPPER-RPM-'].update(value = m_dev.dev_mDC.el_voltage)
            window[f'-{m_dev.c_gui}-GRIPPER-CURR-'].update(value = m_dev.dev_mDC.el_current_limit)

            _title = m_dev.get_device().getTitle()
            if _title:
                window[f'-{m_dev.c_gui}-GRIPPER-TITLE-'].update(_title)

        elif m_dev.C_type == '--GRIPPERv3--':
            ActivateMotorControl(window, f'--GRIPPERv3--', m_dev.c_gui)
            if isinstance(m_dev.dev_mDC, MAXON_Motor):
                window[f'-{m_dev.c_gui}-GRIPPER-RPM-'].update(value = m_dev.dev_mDC.el_voltage)
            else:
                window[f'-{m_dev.c_gui}-GRIPPER-RPM-'].update(value = m_dev.dev_mDC.rpm)
            window[f'-{m_dev.c_gui}-GRIPPER-CURR-'].update(value = m_dev.dev_mDC.el_current_limit)

            _title = m_dev.get_device().getTitle()
            if _title:
                window[f'-{m_dev.c_gui}-GRIPPER-TITLE-'].update(_title)

        elif m_dev.C_type == '--ZABER--':
            window[f'-{m_dev.c_gui:02d}-ZABER-POSSITION-'].update(value = m_dev.dev_zaber.current_pos)
            window[f'-{m_dev.c_gui:02d}-ZABER-VELOCITY-'].update(value = m_dev.dev_zaber.velocity_in_percents)
            ActivateMotorControl(window, f'--ZABER--', m_dev.c_gui)
            if m_dev.dev_zaber.is_parked():
                print_log(f'ZABER {m_dev.dev_zaber.devName} is parked')
                DeActivateMotorControl(window, '--ZABER--', m_dev.c_gui)
                SetLED (window, f'_zaber_{m_dev.c_gui}_', 'blue')
            else:
                # print_log(f'ZABER {m_dev.dev_zaber.devName} is NOT parked')
                pass

            _title = m_dev.get_device().getTitle()
            print_log(f'ZABER GUI TITLE for {m_dev.dev_zaber.devName}, title = {_title}')
            if _title:
                window[f'-{m_dev.c_gui:02d}-ZABER-TITLE-'].update(_title)

        elif m_dev.C_type == '--DAQ-NI--':
            ActivateMotorControl(window, f'--DAQ-NI--')
        elif m_dev.C_type == '--PHG--':
            ActivateMotorControl(window, f'--PHG--', m_dev.c_gui)
            _title = m_dev.get_device().getTitle()
            # print_log(f'Activating PHG GUI for {m_dev.dev_phg}, title = {_title}')
            if _title and f'_phg_{m_dev.c_gui}_' in window.AllKeysDict:
                if m_dev.dev_phg.dType() == PhidgetRELAY.devType.onoff:
                    window[f'-{m_dev.c_gui}-PHG-'].update(_title)
                else:
                    window[f'-{m_dev.c_gui}-PHG-BUTTON-'].update(_title)


        elif m_dev.C_type == '--CAM--':
            window[f'-{m_dev.c_gui}-CALIBR_PROFILE-'].update(value = 'base')
            if not len(m_dev.get_device().MOTORS) == 0:
                window[f'-{m_dev.c_gui}-Z-SELECTOR-'].update(values = m_dev.get_device().MOTORS, value = m_dev.get_device().MOTORS[0])
            ActivateMotorControl(window, f'--CAM--', m_dev.c_gui)
        elif m_dev.C_type == '--HMP--':
            ActivateMotorControl(window, f'--HMP--' )
        elif m_dev.C_type == '--DH--':
            ActivateMotorControl(window, f'--DH--', m_dev.c_gui)
            window[f'-{m_dev.c_gui}-DH-GRIPPER-RPM-'].update(value = m_dev.dev_mDC.DevOpSPEED)
        elif m_dev.C_type == '--MCDMC--':
            ActivateMotorControl(window, f'--MCDMC--')
        elif m_dev.C_type == '--INTERLOCK--':
            ActivateMotorControl(window, f'--INTER_LOCK_ENABLE--')
        elif m_dev.C_type == '--MARCO--':
            _pulse_data:pulseData = m_dev.dev_marco.get_pulse_data()
            window[f'-MARCO_UPDATE_TEMP-'].update(value = m_dev.dev_marco.get_set_temp())
            if _pulse_data:
                window[f'-MARCO_PULSE_ON-'].update(value = _pulse_data.pulse_on)
                window[f'-MARCO_PULSE_CYCLE-'].update(value = _pulse_data.cycle_rate)
                window[f'-MARCO_PULSE_COUNT-'].update(value = _pulse_data.pulse_count)
            window[f'-MARCO_PURGE_TIME-'].update(value = m_dev.dev_marco.get_purge_time())
            m_dev.dev_marco.program_control()
            window[f'-MARCO_PROGRAM-'].update(value = 'Prog '+ str(m_dev.dev_marco.progNum))
            

            if m_dev.dev_marco.get_purge_status() == 1:
                window[f'-MARCO_PURGE_ON-'].update(button_color='dark green on green')
                window[f'-MARCO_PURGE_OFF-'].update(button_color='white on red')
                
            else:
                window[f'-MARCO_PURGE_OFF-'].update(button_color='tomato on red')
                window[f'-MARCO_PURGE_ON-'].update(button_color='white on green')

            if m_dev.dev_marco.get_shot_status() == 1:
                window[f'-MARCO_SINGLE_SHOT_ON-'].update(button_color='dark green on green')
                window[f'-MARCO_SINGLE_SHOT_OFF-'].update(button_color='white on red')
                
            else:
                window[f'-MARCO_SINGLE_SHOT_OFF-'].update(button_color='tomato on red')
                window[f'-MARCO_SINGLE_SHOT_ON-'].update(button_color='white on green')


            ActivateMotorControl(window, f'--MARCO--')
        elif m_dev.C_type == '--IOCONTROL--':
            pass
        else:
            print_err(f'ERROR PANIC - wrong device {m_dev.C_type} in the list at {m_dev.c_gui} position')


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
            [[sg.Text(f'G{i}', font='Any 10'), \
              sg.Text(f'', font='Any 10', key = f'-{i}-GRIPPER-TITLE-'), LEDIndicator(f'_gripper_{i}_'), sg.Text('On/Off'), \
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
        [sg.Push(), sg.Text(f'R{i}', font='Any 10'), \
         sg.Text(f'', font='Any 10', key = f'-{i}-DIST_ROTATOR-TITLE-'), LEDIndicator(f'_dist_rotator_{i}_'), sg.Push()],
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
        [sg.Text(f'DH - (D{i}) ', font='Any 10'),  
            sg.Text(f'', font='Any 10', key = f'-{i}-DH-GRIPPER-TITLE-'), LEDIndicator(f'_dh_gripper_{i}_'), sg.Text('On/Off'), 
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

def cam_block(i):
    return ( [[sg.Push(), sg.Button('Calibrate', border_width = 5, button_color = 'dodger blue', expand_x = True, expand_y = True, key  = f'-{i}-CALIBRATE-',), 
                    sg.OptionMenu(values=('--'), default_value = None,  key=f'-{i}-Z-SELECTOR-'),
                    LEDIndicator(f'_calibr_{i}_'),
                    sg.Push()],
                    [sg.Text('Profile/Filter:')], [sg.Input(size=(15, 1), enable_events=True, key=f'-{i}-CALIBR_PROFILE-', \
                        font=('Arial Bold', 8), justification='left')]
                    ]
            )


def trolley_block(i):
    return ([
        [sg.Push(), sg.Text(f'T{i} ', font='Any 10'), 
         sg.Text(f'', font='Any 10', key = f'-{i}-TROLLEY-TITLE-'), LEDIndicator(f'_trolley_{i}_'), sg.Push()],
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

def cam_frames(line_s, line_e):
    return (
            [[sg.Frame(f'CAM{i}', cam_block(i), border_width=3, \
                                    expand_x=True, expand_y=True, element_justification = "center")]  for i in range (line_s, line_e+1) ]
    )


def gripper_frames(line_s, line_e):
    return (
            [[sg.Frame('', gripper_block(i), border_width=3, \
                                    expand_x=True, expand_y=True, element_justification = "center")]  for i in range (line_s, line_e+1) ]
    )


def zaber_block(i):
    return (
            [[sg.Text(f'Z{i} ', font='Any 10'), \
              sg.Text(f'', font='Any 10', key = f'-{i:02d}-ZABER-TITLE-'), LEDIndicator(f'_zaber_{i}_')],
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
    dev_ind = None
    for m_dev in devs_list:
        if 'TROLLEY' in ev_name and m_dev.C_type == '--TROLLEY--':
            if   m_dev.c_gui == int (ev_name[1]):
                dev_ind = d_index
                break
        if 'TIME_ROTATOR' in ev_name and m_dev.C_type == '--TIME_ROTATOR--':
            dev_ind = d_index
            break
        elif 'GRIPPER' in ev_name and (m_dev.C_type == '--GRIPPER--' or m_dev.C_type == '--GRIPPERv3--'):
            if   m_dev.c_gui == int (ev_name[1]):
                dev_ind = d_index
                break
        elif 'DIST_ROTATOR' in ev_name and m_dev.C_type == '--DIST_ROTATOR--' :
            if   m_dev.c_gui == int (ev_name[1]):
                dev_ind = d_index
                break

        elif '-CALIBR' in ev_name and m_dev.C_type == '--CAM--' :
            if   m_dev.c_gui == int (ev_name[1]):
                dev_ind = d_index
                break

        # elif ('HOTAIR' in ev_name or 'TRIGGER' in  ev_name) and m_dev.C_type == '--DAQ-NI--':
        #     break

        elif 'DAQ' in  ev_name and m_dev.C_type == '--DAQ-NI--' and m_dev.get_device().devName == ev_name:
            dev_ind = d_index
            break

        elif ('HMP' in ev_name) and m_dev.C_type == '--HMP--':
            dev_ind = d_index
            break

        elif  'ZABER' in ev_name and m_dev.C_type == '--ZABER--':
            if  m_dev.c_gui == int (ev_name[1:3]):
                print_DEBUG(f'Found device at d_index = {d_index} for event "{ev_name}"')
                dev_ind = d_index
                break
        
        elif '-DH' in ev_name and (m_dev.C_type == '--DH--'):
            if   m_dev.c_gui == int (ev_name[1]):
                dev_ind = d_index
                break

        if 'PHG' in ev_name and m_dev.C_type == '--PHG--':
            if   m_dev.c_gui == int (ev_name[1]):
                dev_ind = d_index
                break
        # if ''

        # elif 'DISP_TRIG' in ev_name:
        #     # print_DEBUG(f'Event = {ev_name}, m_dev = {m_dev}, m_dev.dev_phg = {m_dev.dev_phg}')
        #     if m_dev.dev_phg and m_dev.dev_phg.dType() == PhidgetRELAY.devType.dispenser:
        #         break
            
        # elif 'UV_TRIG' in ev_name:
        #     # print_DEBUG(f'Event = {ev_name}, m_dev = {m_dev}, m_dev.dev_phg = {m_dev.dev_phg}')
        #     if m_dev.dev_phg and m_dev.dev_phg.dType() == PhidgetRELAY.devType.uv:
        #         break
        
        # elif 'FRONT_LIGHT' in ev_name:
        #     # print_DEBUG(f'Event = {ev_name}, m_dev = {m_dev}, m_dev.dev_phg = {m_dev.dev_phg}')
        #     if m_dev.dev_phg and m_dev.dev_phg.dType() == PhidgetRELAY.devType.front_light:
        #         break

        # elif 'BACK_LIGHT' in ev_name:
        #     # print_DEBUG(f'Event = {ev_name}, m_dev = {m_dev}, m_dev.dev_phg = {m_dev.dev_phg}')
        #     if m_dev.dev_phg and m_dev.dev_phg.dType() == PhidgetRELAY.devType.back_light:
        #         break
        # elif 'HOLE_LIGHT' in ev_name:
        #     # print_DEBUG(f'Event = {ev_name}, m_dev = {m_dev}, m_dev.dev_phg = {m_dev.dev_phg}')
        #     if m_dev.dev_phg and m_dev.dev_phg.dType() == PhidgetRELAY.devType.hole_light:
        #         break

        # elif ('TRIGGER' in ev_name):
        #     if m_dev.dev_phg and m_dev.dev_phg.dType() == PhidgetRELAY.devType.welder:
        #         break

        # elif 'HOTAIR' in ev_name:
        #     if m_dev.dev_phg and m_dev.dev_phg.dType() == PhidgetRELAY.devType.jtse_hotair:
        #         break
        
        elif 'MCDMC' in ev_name and m_dev.C_type == '--MCDMC--' :
            dev_ind = d_index
            break

        elif 'COGNEX' in ev_name and m_dev.C_type == '--MCDMC--' :
            dev_ind = d_index
            break

        elif 'CGNX_DB' in ev_name and m_dev.C_type == '--DB--' :
            dev_ind = d_index
            break

        elif 'MARCO' in ev_name and m_dev.C_type == '--MARCO--' :
            dev_ind = d_index
            break

        # elif ('INTER_LOCK_ENABLE'  in ev_name or 'LCK1' in ev_name) and m_dev.C_type == '--INTERLOCK--' :
        #     # print_log(f'BUGBUG ev_name = {ev_name},  dev = {m_dev}')
        #     break

        # elif 'LCK2' in ev_name and m_dev.C_type == '--INTERLOCK--' and m_dev.get_device().devName == 'LCK2':
        #     # print_log(f'BUGBUG ev_name = {ev_name},  dev = {m_dev}')
        #     break

        elif m_dev.get_device().devName == ev_name[1:-1] and m_dev.C_type == '--IOCONTROL--':
            dev_ind = d_index
            break

        d_index += 1

    if d_index >= len(devs_list):                                   # active device not found
            print_err(f'Non device assosiated event -> {ev_name}/{ev_name[1:-1]}, type = {m_dev.C_type}')
            return None
        
    # print_log(f'BUGBUG - Found device located at index {d_index}: ev_name = {ev_name},  dev = {devs_list[d_index]}')
    return (dev_ind)

def Correct_ZB_possition_after_unpark(window, devs_list):
    for m_dev in devs_list:
        if m_dev.C_type == '--ZABER--':
            new_pos = m_dev.dev_zaber.GetPos()
            window[f'-{m_dev.c_gui:02d}-ZABER-POSSITION-'].update(value = new_pos)


def isThereParkedZaber(window, devs_list):
    for m_dev in devs_list:
        if m_dev.C_type == '--ZABER--':
            if m_dev.get_device().is_parked():
                return True
            
    return False
    

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
            print_log(f'Non controlled device associated cmd. dev =  {dev}')

# 
ScrRunLock = Lock()
def ScriptRunner(window:sg.Window, devs_list:List[CDev], selected_group:List[str], scriptQ:Queue, retEvent:str, eventQ:Queue, cyclic:int = 1):

    if ScrRunLock.locked():
        print_err(f'-WARNING another script is still in progress. Trying to kill it')
        scriptQ.put('-STOP-')
        time.sleep(0.5)
        if ScrRunLock.locked():
            print_err(f"-ERROR: failed eliminate another script that is still in progress. ")
            return
    
    ScrRunLock.acquire()

    clearQ(scriptQ)
    gr_index = -1

    print_log (f'Selection: {selected_group}')

    if selected_group == []:
        print_err(f'Empty script')
        ScrRunLock.release()
        window.write_event_value(retEvent, None)
        return
    
    

    if selected_group in groups:                 # probably block is not selected correctly, find correct group index
        gr_index = groups.index(selected_group)
    else:
        for gr_index, gr in enumerate(groups):
            if selected_group[0] in gr:
                break
            
    print_log (f'Starting script at group # {gr_index} of {len (groups)} groups')   

    clearQ(scriptQ)
    while gr_index < len (groups):
        print_log(f'Running step # {gr_index} ')
        print_log(f'Group -> {groups[gr_index]}')

        eventQ.put(event2GUI(event='-SCRIPT_STEP-', value = groups[gr_index]))
        
        print_log(f'Step # {gr_index} is waiting for completion')
        msg:str = scriptQ.get()
        # scriptQ.task_done()
        print_log(f'Script runner get msg = {msg}, Step # {gr_index}')

        if msg == '-STOP-':
            removeElementQ(eventQ, '-SCRIPT_STEP-')         # remove -SCRIPT_STEP- if still not proceeded 
            print_log(f'Stop button pressed. Group = {gr_index}')
            break
        elif msg == '-STEP_DONE-':
            print_log(f'Step # {gr_index} done')
            gr_index += 1
        else:
            print_err(f'Unexpected msg = {msg}')
        
        if cyclic > 1 and gr_index >= len (groups):
            cyclic -= 1
            eventQ.put(event2GUI(event='-DECREMENT_CYCLIC-', value = cyclic))
            gr_index = 0

    window.write_event_value(retEvent, None)
    print_log(f'Script done')
    ScrRunLock.release()
    return

def get_calibration_pos(devs_list, motor) -> int:
    if motor[0] == 'Z':
        device_n = (int)(motor[1:])  
        d_index = LocateDevice(f'-{device_n:02d}-ZABER-POSSITION-', devs_list)
        if d_index is None:                     # No device
            print_err(f'No active Z device ({motor}) to execute cmd ')
            return 0  
        _pos = devs_list[d_index].dev_zaber.GetPos()
        print_log(f'Calibrater position = {_pos}, motor = {motor}')
        return _pos
    
    elif motor[0] == 'D':
        _D_motors = motor.split('/')
        if _D_motors[0][1] == _D_motors[1][1]:
            print_err(f'ERROR: Identical DH Robotics Gripper devices selected for calibration = {motor}/{_D_motors}')
            return 0

        device_n1 = (int)(_D_motors[0][1:]) 

        d_index_1 = LocateDevice(f'-{device_n1}-DH_GRIPPER_POSSITION-', devs_list)

        if d_index_1 is None:                     # No device
            print_err(f'No active DH devices ({motor}) to execute cmd ')
            return 0  
        
        _pos =  devs_list[d_index_1].dev_mDC.mDev_get_cur_pos()
        print_log(f'Calibrater position = {_pos}, motor = {motor}')

        return _pos
    else:
        print_err('Calibration pos return. Only ZABER dev are supported')
        return 0


def  proc_tuning(window, devs_list, motor, dist, _abs_moving = False)-> pm.WorkingTask:               # calibration
    
    wTask:pm.WorkingTask = None

    if motor[0] == 'Z':
        device_n = (int)(motor[1:])
        print_log(f'Zaber device = {device_n:02d} moving dist = {dist}')
        d_index = LocateDevice(f'-{device_n:02d}-ZABER-POSSITION-', devs_list)
        if d_index is None:                     # No device
            print_err(f'No active Z device ({motor}) to execute cmd ')
            return wTask


        if not _abs_moving:
            if float(dist) == 0:
                wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.stop), sType = pm.RunType.single)
            else:
                move_pos = (float)(dist)
                cur_pos = devs_list[d_index].dev_zaber.GetPos()
                new_pos = cur_pos + move_pos
                print_log(f'Calibration in process. Curr pos = {cur_pos}, new pos = {new_pos}, Absolute movin = {_abs_moving}')
                wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.go_to_dest, args=pm.argsType(position=new_pos)), sType = pm.RunType.single)

        else:
            new_pos = (float)(dist)
            print_log(f'Calibration in process.New pos = {new_pos}, Absolute movin = {_abs_moving}')
            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.go_to_dest, args=pm.argsType(position=new_pos)), sType = pm.RunType.single)
    
    elif motor[0] == 'R':
        device_n = (int)(motor[1:])
        print_log(f'Router device = {device_n} moving dist = {dist}')
        d_index = LocateDevice(f'-{device_n}-DIST_ROTATOR_TARGET-', devs_list)
        if d_index is None:                     # No device
            print_err(f'No active R device to execute cmd ')
            return wTask

        
        if dist == 0:
            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.stop), sType = pm.RunType.single)
        else:
                                        # counts = dist × Gear ratio × (Counts per turn × 4 [quadrature encoding]) / (π × diameter)
                                        # An encoder with 1,024 CPT (counts or pulses per turn) gives 4,096 states per turn, or a nominal resolution of 360°/4096 = 0.088°.



            # if motor == 'R1':
            #     # new_dist = (dist * QUAD * CPT * R1_GEAR) / R1_PITCH 
            #     new_dist = (dist * QUAD * CPT * R1_GEAR) / (R1_DIAMETER * math.pi)
            # elif motor == 'R2':
            #     new_dist = (dist * QUAD * CPT * R2_GEAR) / (R2_DIAMETER * math.pi)   
            # else:
            #     print_err(f'Unsupported dev = {motor}')
            #     new_dist = 0

            # _DIAMETER = devs_list[d_index].get_device().diameter if  devs_list[d_index].get_device().diameter is not None else DEFAULT_DIAMETER
            # _GEAR = devs_list[d_index].get_device().gear if  devs_list[d_index].get_device().gear is not None else DEFAULT_GEAR


            # _DIAMETER = (__diam := devs_list[d_index].get_device().diameter)  if  __diam is not None else DEFAULT_DIAMETER
            # _GEAR = (__gear := devs_list[d_index].get_device().gear) if  __gear is not None else DEFAULT_DIAMETER


            __diam = devs_list[d_index].get_device().diameter
            __gear = devs_list[d_index].get_device().gear
            _DIAMETER = __diam  if  __diam is not None else DEFAULT_DIAMETER
            _GEAR = __gear if  __gear is not None else DEFAULT_DIAMETER
            new_dist = (dist * QUAD * CPT * _GEAR) / (_DIAMETER * math.pi)   


            move_pos = (float)(new_dist)
            cur_pos = devs_list[d_index].dev_mDC.mDev_get_cur_pos()
            new_pos = round(cur_pos + move_pos)


            print_log(f'Curr pos = {cur_pos}, new pos = {new_pos}, DIAMETER={_DIAMETER}, GEAR= {_GEAR}, dist = {dist}mm/{new_dist}/{move_pos} ')
            window[f'-{device_n}-DIST_ROTATOR_TARGET-'].update(str(new_pos))

            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.go_to_dest, args=pm.argsType(position=new_pos)), sType = pm.RunType.single)
    elif motor[0] == 'D':
        # dist *= 1000                            # correct for DH robotics (dist is in degrees, not )
        _int_dist:int = int(round(dist))
        _D_motors = motor.split('/')
        if _D_motors[0][1] == _D_motors[1][1]:
            print_err(f'ERROR: Identical DH Robotics Gripper devices selected for calibration = {motor}/{_D_motors}')

        device_n1 = (int)(_D_motors[0][1:])
        device_n2 = (int)(_D_motors[1][1:])
        print_log(f'DH Robotics grippers = {device_n1}/{device_n2}  moving dist = {_int_dist}')
        d_index_1 = LocateDevice(f'-{device_n1}-DH_GRIPPER_POSSITION-', devs_list)
        d_index_2 = LocateDevice(f'-{device_n2}-DH_GRIPPER_POSSITION-', devs_list)
        if d_index_1 is None or d_index_2 is None:                     # No device
            print_err(f'No active PH Robotics (D) devices ({motor}/{_D_motors}) to execute cmd ')
            return wTask

        wTask_1 = None
        wTask_2 = None
    
        new_pos_1 =  0
        new_pos_2 = 0
        cur_pos_1= devs_list[d_index_1].dev_mDC.mDev_get_cur_pos()
        cur_pos_2= devs_list[d_index_2].dev_mDC.mDev_get_cur_pos()

        if _int_dist == 0 and not _abs_moving:
            print_log(f'dist = 0. Stopping DH Robotics {device_n1}/{device_n2} ')
            wTask_1 = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index_1],cmd=pm.OpType.stop), sType = pm.RunType.single)
            wTask_2 = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index_2],cmd=pm.OpType.stop), sType = pm.RunType.single)
        else:
            if _abs_moving:
                new_pos_1 =  _int_dist
                new_pos_2 = (-1) * _int_dist
            else:
                new_pos_1 = cur_pos_1 + _int_dist
                new_pos_2 = cur_pos_2 - _int_dist

            print_log(f'Calibration in process. Curr pos = {cur_pos_1}/{cur_pos_2}, new pos = {new_pos_1}/{new_pos_2}')
            wTask_1 = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index_1],cmd=pm.OpType.go_to_dest, args=pm.argsType(position=new_pos_1)), sType = pm.RunType.single)
            wTask_2 = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index_2],cmd=pm.OpType.go_to_dest, args=pm.argsType(position=new_pos_2)), sType = pm.RunType.single)

        tempTaskList:List[pm.WorkingTask] = list()
        tempTaskList.append(wTask_1)
        tempTaskList.append(wTask_2)
        wTask = pm.WorkingTask(tempTaskList, sType=pm.RunType.parallel)


    else:
         print_err(f'Unsupported dev = {motor}')
    
    return wTask

        




#========================================= Utils end here =======================


# ===  Initiation GUI topology, etc
def InitGUI(parms:dict):
    load_dev_config()
    confDevs = getDevsList()
    print_log (f'DevList = \n{confDevs}')

    global zaber_motors 
    global gripper_motors 
    global distance_rotators 
    global time_rotators 
    global dh_grippers 
    global cams 
    global daq 
    global hmp 
    global phg 
    global mcdmc 
    global marco 
    global intrlck 
    global trolley_motors


    zaber_motors = len(confDevs['--ZABER--'])
    gripper_motors = len(confDevs['--GRIPPERv3--'])
    distance_rotators = len(confDevs['--DIST_ROTATOR--'])
    time_rotators = len(confDevs['--TIME_ROTATOR--'])
    dh_grippers = len(confDevs['--DH--'])
    cams = len(confDevs['--CAM--'])
    daq = len(confDevs['--DAQ-NI--'])
    hmp = len(confDevs['--HMP--'])
    phg_dict:dict = confDevs['--PHG--']
    trolley_motors = len(confDevs['--TROLLEY--'])
    phg = len(phg_dict)
    mcdmc = len(confDevs['--MCDMC--'])
    marco = len(confDevs['--MARCO--'])
    intrlck = len(confDevs['--INTERLOCK--'])
    jbc = len(confDevs['--JTSE--'])
    
    welder =  hotair = lights = dispencer_uv = False

    phg_keys = list(phg_dict.keys())

    # print_log(f'PHG devs (keys) = {phg_keys}')

    # if phg > 0:
    #     if 'WELDER' in phg_keys:
    #         welder = True
    #         print_log(f"WELDER: {phg_dict['WELDER']}")
    #     if 'JTSE_HOTAIR' in phg_keys:
    #         hotair = True
    #         print_log(f"WELDER: {phg_dict['JTSE_HOTAIR']}")
    #     if 'FRONTLIGHT' in phg_keys or 'BACKLIGHT' in phg_keys or 'HOLELIGHT' in phg_keys:
    #         lights = True
    #         print_log(f"LIGHTS: {phg_dict['FRONTLIGHT'] if 'FRONTLIGHT' in phg_keys else 'NO FRONT LIGHT'}, {phg_dict['BACKLIGHT'] if 'BACKLIGHT' in phg_keys else 'NO BACK LIGHT' }, {phg_dict['HOLELIGHT'] if 'HOLELIGHT' in phg_keys else 'NO HOLE LIGHT' }")
         
    #     if 'DISP' in phg_keys or 'UV' in phg_keys:
    #         dispencer_uv = True
    #         print_log(f"DISP/UV: {phg_dict['DISP'] if 'DISP' in phg_keys else 'NO DISPENCER'}, {phg_dict['UV'] if 'UV' in phg_keys else 'NO UV' }")
    


    # if zaber_motors % 2 > 0:    
    #     zaber_motors += 1

    print_log(f'Activating devs: zaber_motors= {zaber_motors}, gripper_motors={gripper_motors}, trolley_motors={trolley_motors},  distance_rotators= {distance_rotators}')
    print_log(f'Activating devs(cont): time_rotators= {time_rotators}, dh_grippers={dh_grippers}, cams={cams},  daq= {daq}, hmp = {hmp}, phg={phg}, marco dispenser = {marco}')
    _version = os.path.abspath(__name__).split( '\\')[-3]
    _ver =re.compile(r'^v\d+(\.\d+)?$')
    print_log(f'Version: {_version}')
    if not _ver.match(_version):
        print_log (f'Wrong version format')
        _version = ''

    top_banner = [
                [sg.Text('Dashboard', font='Any 12', background_color=DARK_HEADER_COLOR, enable_events=True, \
                            grab=False), sg.Push(background_color=DARK_HEADER_COLOR),
                sg.Text(f'{_version}  © 2023', font='Any 12', background_color=DARK_HEADER_COLOR)],
                ]


    timeRotator = [[sg.Push(), sg.Text('Time Rotator (Spinner)', font='Any 10'), \
                  sg.Text('', font='Any 10', key = f'-TIME_ROTATOR-TITLE-'), LEDIndicator('_time_rotator_'), sg.Push()],
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
                [sg.Text(text = "Command script", key = '-SCRIPT_NAME-')],
                [sg.Button('Run'), sg.Button(button_text = 'Stop Script', button_color = 'white on firebrick4', border_width = 10, key='Stop'), \
                    sg.Checkbox('Interlock', default=False, enable_events = True, key='-INTER_LOCK_ENABLE-'),
                    # sg.Checkbox('Cyclic Runs', default=False, enable_events = False, key='-CYCLIC_RUN-')],
                    sg.Text(text = "Cyclic Runs"), sg.Input(default_text='1', size=(3, 1), enable_events=True, key='-CYCLIC_RUN-')],
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
    
    calibration = [[sg.Push(), sg.Button('Calibrate', border_width = 5, button_color = 'dodger blue', expand_x = True, expand_y = True, key  = '-CALIBRATE-',), 
                    sg.OptionMenu(values=('--'), default_value = None,  key='-Z-SELECTOR-'),
                    LEDIndicator('_calibr_'),
                    sg.Push()],
                    [sg.Text('Profile/Filter:')], [sg.Input(size=(15, 1), enable_events=True, key='-CALIBR_PROFILE-', \
                        font=('Arial Bold', 8), justification='left')]
                    ]

    # if daq > 0:
    #     weld_trigger = [[sg.Push(), sg.Button('Trigger', border_width = 5, button_color = 'red', expand_x = True, expand_y = True, key  = 'TRIGGER',), 
    #                     sg.OptionMenu(values=('SC 0', 'SC 1', 'SC 2', 'SC 3'), default_value = 'SC 0',  key='-SELECTOR-'), LEDIndicator('_trigger_'),
    #                     sg.Push()]]
    # else:
    if 'PHG' in parms.keys():
        print_log(f'PHG devices in params: {parms["PHG"]} (len = {len(parms["PHG"])}) <<>> in config: {phg_dict} (len={len(phg_dict)}) ')
    
    phg_toggle = list()
    phg_trigger = list()
    phg_onoff =  list()
    _index = 0
    for _key, _value in phg_dict.items():
        _dev_name = _key
        _parsed_phg = _value.split(',')
        # if len(_parsed_phg) > 2 and (_parsed_phg[2].strip().upper() == 'FALSE' or _parsed_phg[2] == False):
        #     continue
        print_log(f'Parsed phg ({_parsed_phg}): len = {len(_parsed_phg)}')
        _type = _parsed_phg[1].strip()
        _index += 1

        print_log(f'Setup GUI for PHG: {_key}:{_value} / type = {_type} _phg_{_index}_')

        if  _type == 'TRIGGER':
            _phg_element = [sg.Push(), sg.Button(f'{_dev_name}', border_width = 5, button_color = 'red', \
                    expand_x = True, expand_y = True, key  = f'-{_index}-PHG-BUTTON-',), \
                    LEDIndicator( f'_phg_{_index}_'), sg.Button('Off', border_width = 5, button_color = 'green', expand_x = True, \
                    expand_y = True, key  =f'-{_index}-PHG-TRIGGER-OFF-'),
                    sg.Push()]
            if not (len(_parsed_phg) > 2 and (_parsed_phg[2].strip().upper() == 'FALSE' or _parsed_phg[2] == False)):
                phg_trigger.append(_phg_element)
        elif  _type == 'TOGGLE':
            _phg_element = [sg.Push(), sg.Button(f'{_dev_name}', border_width = 5, button_color = 'gray', \
                    expand_x = True, expand_y = True, key  = f'-{_index}-PHG-BUTTON-',), \
                    LEDIndicator( f'_phg_{_index}_'),
                    sg.Push()]
            if not (len(_parsed_phg) > 2 and (_parsed_phg[2].strip().upper() == 'FALSE' or _parsed_phg[2] == False)):
                phg_toggle.append(_phg_element)
        elif  _type == 'ONOFF':
            _phg_element = [sg.Push(), sg.Text(f'{_dev_name}', key = f'-{_index}-PHG-'), \
                sg.Button('On', border_width = 5, button_color = 'orange', expand_x = True, \
                expand_y = True, key  = f'-{_index}-PHG-ON-'), 
                sg.Button('Off', border_width = 5, button_color = 'green', expand_x = True, \
                expand_y = True, key  =f'-{_index}-PHG-OFF-'), LEDIndicator(f'_phg_{_index}_'),
                sg.Push()]
            if not (len(_parsed_phg) > 2 and (_parsed_phg[2].strip().upper() == 'FALSE' or _parsed_phg[2] == False)):
                phg_onoff.append(_phg_element)    
        else:
            print_err(f'ERROR - Unknown type: [{_type}]')



    # weld_trigger = [[sg.Push(), sg.Button('Trigger', border_width = 5, button_color = 'red', \
    #                 expand_x = True, expand_y = True, key  = 'TRIGGER',), 
    #                 LEDIndicator('_trigger_'),
    #                 sg.Push()]]

    # hot_air = [[sg.Push(), sg.Button('Start', border_width = 5, button_color = 'orange', expand_x = True, \
    #             expand_y = True, key  = 'HOTAIR-START'), 
    #             sg.Button('Stop', border_width = 5, button_color = 'green', expand_x = True, \
    #             expand_y = True, key  = 'HOTAIR-STOP'), LEDIndicator('_hotair_'),
    #             sg.Push()]]


                
    # dispenser_uvDev = [[sg.Push(), sg.Button('Dispenser', border_width = 5, button_color = 'red', expand_x = False, expand_y = True, key  = 'DISP_TRIG',), 
    #                 LEDIndicator('_dispenser_'), sg.Push(),
    #                 sg.Push(), sg.Button('UV', border_width = 5, button_color = 'red', expand_x = False, expand_y = True, key  = 'UV_TRIG',), 
    #                 LEDIndicator('_uv_'), sg.Push()]]
    
    # cam_light = [[sg.Push(), sg.Button('Front', border_width = 5, button_color = 'gray', expand_x = False, expand_y = True, key  = 'FRONT_LIGHT',), 
    #                 LEDIndicator('_front_light_'), sg.Push(),
    #                 sg.Push(), sg.Button('Back', border_width = 5, button_color = 'gray', expand_x = False, expand_y = True, key  = 'BACK_LIGHT',), 
    #                 LEDIndicator('_back_light_'), sg.Push(),
    #                 sg.Push(), sg.Button('Back', border_width = 5, button_color = 'gray', expand_x = False, expand_y = True, key  = 'HOLE_LIGHT',), 
    #                 LEDIndicator('_hole_light_'), sg.Push()]
    #             ]


    mcdmc_asyril =  [[sg.Push(), LEDIndicator('_mcdmc_'), sg.Text('Mecademic  ', font='Any 10'), sg.Push()],
                [
                    sg.Button('Deactivate', border_width = 5, button_color = 'red', expand_x = True, \
                    expand_y = True, key  = f'_MCDMC_SW_OFF_'),  \
                    sg.Button('Activate', border_width = 5, button_color = 'green', expand_x = True, \
                    expand_y = True, key  =f'_MCDMC_SW_ON_'), LEDIndicator(f'_mcdmc_active_'), \
                    sg.Push()
                ],
                [
                    sg.Button('Open Robot Control', border_width = 5, button_color = 'white on violet', expand_x = True, \
                    expand_y = True, key  = f'_MCDMC_OPEN_ROBOT_CTL_')
                ],
                [  sg.Text('X:'),sg.Text("_", size=(10, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                    border_width = 2, key='-MCDMC_POS_X-'),
                    sg.Text('Y:'),sg.Text("_", size=(10, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                    border_width = 2, key='-MCDMC_POS_Y-')
                ],
                [ sg.Text('Z:'),sg.Text("_", size=(10, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                    border_width = 2, key='-MCDMC_POS_Z-'),
                    sg.Text('γ:'),sg.Text("_", size=(10, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                    border_width = 2, key='-MCDMC_POS_GAMMA-')
                ],   
                [
                    sg.Button('Triger COGNEX', border_width = 5, button_color = 'white on blue', expand_x = True, \
                    expand_y = True, key  = f'_COGNEX_TRIGGER_')
                ],
                [
                    sg.Push(), LEDIndicator('_cognex_'), sg.Text('Cognex Online Status', font='Any 10'), sg.Push()
                ],
                
                # [sg.T('PickUp'),  sg.Text('X:'),sg.Text("_", size=(6, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                #     border_width = 2, key='-MCDMC_PICKUP_X-'),
                #     sg.Text('Y:'),sg.Text("_", size=(6, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                #     border_width = 2, key='-MCDMC_PICKUP_Y-'),
                # ],  
                # [sg.Text('Put'), sg.Text('X:'), 
                #     sg.Input(size=(10, 1), enable_events=True, key='-MCDMC_PUT_X-', font=('Arial Bold', 8), justification='left'),
                #     sg.Text('Y:'), 
                #     sg.Input(size=(10, 1), enable_events=True, key='-MCDMC_PUT_Y-', font=('Arial Bold', 8), justification='left')],
                # [
                #     sg.Button(button_text = "Pick & Place", key='-MCDMC_PICK_AND_PUT-'), sg.Checkbox('Automatic', default=True, enable_events = True, key='-MCDMC_AUTOMATIC-')
                # ]
            ]                       
    statistic = [[sg.Text('Valid FPCB'), sg.Text('Proceeded Products')],
                [sg.Text("_", size=(10, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                    border_width = 2, key='-CGNX_VALID_PRODUCT-'),
                sg.Text("_", size=(10, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                    border_width = 2, key='-CGNX_PROCEEDED_PRODUCT-'), sg.Button(button_text = "Reset", key='-CGNX_DB_RESET_COUNTER-')],
                [sg.Text('Wrong FPCB'), sg.Text('Up-Side Down'), sg.Text('Out of Range')],
                [sg.Text("_", size=(10, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                    border_width = 2, key='-CGNX_WRONG_PRODUCT-'), 
                sg.Text("_", size=(10, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                    border_width = 2, key='-CGNX_UP_SIDE_DOWN-'),
                sg.Text("_", size=(10, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                    border_width = 2, key='-CGNX_OUT_OF_RANGE-')],
            ]
    _blower = [[sg.Text('Blower Temperature'), 
                sg.Text("_", size=(4, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', border_width = 2, key='-JBC_TEMP-'),
                sg.Text('°C'),]

    ]
    # pulse_on_layout = [[sg.Input(size=(10, 1), enable_events=True, key='-MARCO_PULSE_ON-', font=('Arial Bold', 8), justification='left'),
    #                 sg.Button(button_text = "Set", key='-MARCO_SET_PULSE_ON-')]]    
    
    # pulse_cycle_layout = [[sg.Input(size=(10, 1), enable_events=True, key='-MARCO_PULSE_CYCLE-', font=('Arial Bold', 8), justification='left'),
    #                 sg.Button(button_text = "Set", key='-MARCO_SET_PULSE_CYCLE-')]] 
    
    # pulse_count_layout = [[sg.Input(size=(10, 1), enable_events=True, key='-MARCO_PULSE_COUNT-', font=('Arial Bold', 8), justification='left'),
    #                 sg.Button(button_text = "Set", key='-MARCO_SET_PULSE_COUNT-')]] 
    
    
    pulse_data_layout = [[sg.Text('Opening time'), sg.Text('Repetition rate'), sg.Text('# per trigger')],
                         [sg.Input(size=(13, 1), enable_events=True, key='-MARCO_PULSE_ON-', font=('Arial Bold', 8), justification='left'),  
                          sg.Input(size=(13, 1), enable_events=True, key='-MARCO_PULSE_CYCLE-', font=('Arial Bold', 8), justification='left'),
                            sg.Input(size=(13, 1), enable_events=True, key='-MARCO_PULSE_COUNT-', font=('Arial Bold', 8), justification='left'),
                        ],
                         [sg.Button(button_text = "Set Pulse data", key='-MARCO_SET_PULSE_DATA-')]
                        ] 

    
    purge_time_layout = [[sg.Input(size=(10, 1), enable_events=True, key='-MARCO_PURGE_TIME-', font=('Arial Bold', 8), justification='left'),
                    sg.Button(button_text = "Set", key='-MARCO_SET_PURGE_TIME-')]] 

    temperature_layout = [[sg.Text("_", size=(10, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                    border_width = 2, key='-MARCO_ACTUAL_TEMP-'),
                    sg.Input(size=(10, 1), enable_events=True, key='-MARCO_UPDATE_TEMP-', font=('Arial Bold', 8), justification='left'),
                    sg.Button(button_text = "Set", key='-MARCO_SET_TERM-')]]
    
    pulse_reset_counter_layout = [[ sg.Text("_", size=(10, 1), relief = sg.RELIEF_SUNKEN, justification = 'center', \
                                     border_width = 2, key='-MARCO_PULSE_COUNT_AFTER_RESET-'),
                                    sg.Button(button_text = "Reset", key='-MARCO_RESET_PULSE_COUNT-')
                                ]] 
    sinle_shot_layout = [[
                        sg.Button(button_text ='Start', size=(5, 1), button_color='white on green', key='-MARCO_SINGLE_SHOT_ON-', 
                                disabled_button_color = 'light slate gray on navajo white'),
                         sg.Button(button_text ='Stop', size=(5, 1), button_color='white on red', key='-MARCO_SINGLE_SHOT_OFF-', 
                                disabled_button_color = 'light slate gray on navajo white')

                        ]]
    _prog_names = ['Prog 1', 'Prog 2', 'Prog 3', 'Prog 4', 'Prog 5', 'Prog 6', 'Prog 7', 'Prog 8', 'Prog 9', 'Prog 10']
    program_layout = [[sg.Combo(values=_prog_names, default_value='Prog 1', enable_events = True, 
                                size=(10, 1), key='-MARCO_PROGRAM-', font=('Arial Bold', 8))]] 

    marco_dispenser =   [[sg.Push(), sg.Text('Dispenser status', font='Any 10'), LEDIndicator('_marco_'),  sg.Push()],
                        [sg.Frame('Temperature', temperature_layout)],
                        [sg.Frame('Pulse data', pulse_data_layout, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                            border_width=1, vertical_alignment='top', element_justification = "center", expand_y = False)],
                        # [sg.Frame('Pulse opening time', pulse_on_layout), sg.Frame('Pulse repetition rate', pulse_cycle_layout)], 
                        # [sg.Frame('Pulses per trigger', pulse_count_layout), sg.Frame('Max purge time', purge_time_layout)],  
                        [sg.Frame('Max purge time', purge_time_layout)],
                        [sg.Frame('Pulses since last reset', pulse_reset_counter_layout), 
                         sg.Button(button_text ='On', size=(2, 1), button_color='white on green', key='-MARCO_PURGE_ON-', 
                                disabled_button_color = 'light slate gray on navajo white'),
                         sg.Button(button_text ='Off', size=(2, 1), button_color='white on red', key='-MARCO_PURGE_OFF-', 
                                disabled_button_color = 'light slate gray on navajo white')
                        ],
                        [sg.Frame('Program to trigger', program_layout, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                            border_width=1, vertical_alignment='top', element_justification = "center", expand_y = False)],
                        [sg.Frame('Single Shot / start pulse sequence manually ', sinle_shot_layout, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                            border_width=1, vertical_alignment='top', element_justification = "center", expand_y = False)],

                 
                 ]

                          
               
    

    script_col=[]
    log_col=[]

    dh_grippers_list = dh_grippers_frames(1, (int)(dh_grippers/1))

    cam_list = cam_frames(1, (int)(cams/1))



    script_col.append([sg.Frame('', script, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                        border_width=1, vertical_alignment='down', element_justification = "center")])
    
    script_col.append([sg.Push(), sg.Button('Park', border_width = 3, button_color = 'red'), 
                    sg.Button('Unpark', border_width = 3, button_color = 'green'), sg.Push()])
    
    script_col.append([sg.Push(), sg.Button('Reload parms', border_width = 3, button_color = 'orange', key='RELOAD'), 
                     sg.Push()])

    if dh_grippers > 0:
        log_col.append([sg.Frame('', dh_grippers_list, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                            border_width=1, vertical_alignment='down', element_justification = "center")])

    if hmp > 0:
        log_col.append([sg.Frame('', measurement_block, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                            border_width=1, vertical_alignment='down', element_justification = "center")])
 

    if cams > 0:
        log_col.append([sg.Frame('', cam_list, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                            border_width=1, vertical_alignment='down', element_justification = "center")])
        # log_col.append([sg.Frame('Camera Calibration', calibration, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
        #                     border_width=3, vertical_alignment='top', element_justification = "center", expand_y = False)])


    if phg > 0:
        if len(phg_toggle) > 0:
            log_col.append([sg.Frame('Toogle Control', phg_toggle, pad=(0,0),  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                            border_width=1, vertical_alignment='top', element_justification = "center", expand_y = False)])

        if len(phg_trigger) > 0:
            log_col.append([sg.Frame('Triggers Control', phg_trigger, pad=(0,0),  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                            border_width=1, vertical_alignment='top', element_justification = "center", expand_y = False)])

        if len(phg_onoff) > 0:
            log_col.append([sg.Frame('ON/OFF Control', phg_onoff, pad=(0,0),  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                            border_width=1, vertical_alignment='top', element_justification = "center", expand_y = False)])


    # if phg > 0:
    #     if dispencer_uv:
    #         log_col.append([sg.Frame('Dispenser & UV Control', dispenser_uvDev, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
    #                         border_width=3, vertical_alignment='top', element_justification = "center", expand_y = False)])
    #     if lights:
    #         log_col.append([sg.Frame('Camera Light', cam_light, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
    #                         border_width=3, vertical_alignment='top', element_justification = "center", expand_y = False)])

    #     if welder:
    #         log_col.append([sg.Frame('Welder', weld_trigger, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
    #                         border_width=3, vertical_alignment='top', element_justification = "center", expand_y = False)])
        
    #     if hotair:
    #         log_col.append([sg.Frame('JTSE Hot Air Station', hot_air, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
    #                         border_width=3, vertical_alignment='top', element_justification = "center", expand_y = False)])

    if mcdmc > 0:
        log_col.append([sg.Frame('Robot', mcdmc_asyril, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                            border_width=3, vertical_alignment='top', element_justification = "center", expand_y = False)]) 
        log_col.append([sg.Frame('Statistic', statistic, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                            border_width=3, vertical_alignment='top', element_justification = "center", expand_y = False)]) 
    if jbc > 0:
        log_col.append([sg.Frame('JBC Control', _blower , pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                            border_width=3, vertical_alignment='top', element_justification = "center", expand_y = False)]) 
    
    if marco > 0:
        log_col.append([sg.Frame('Marco Dispenser', marco_dispenser, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                            border_width=3, vertical_alignment='top', element_justification = "center", expand_y = False)]) 
    


    # if daq > 0:
    #     log_col.append([sg.Frame('Welder', weld_trigger, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
    #                         border_width=3, vertical_alignment='top', element_justification = "center", expand_y = False)])
        
    #     log_col.append([sg.Frame('JTSE Hot Air Station', hot_air, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
    #                         border_width=3, vertical_alignment='top', element_justification = "center", expand_y = False)])



    gripper_list = gripper_frames(1,(int)(gripper_motors/1))



    trolley_list = trolley_frames(1, (int)(trolley_motors/1))

    # dist_rotators_list = dist_rotators_frames(1, (int)(distance_rotators/1))

    # if time_rotators > 0:
    #     dist_rotators_list.append([sg.Frame('', timeRotator, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
    #                         border_width=3, vertical_alignment='top', element_justification = "center")])


    dev_layout = list()
    dev_layout.append(sg.Column(script_col, vertical_alignment = 'top', expand_y = 'True', scrollable=False, vertical_scroll_only=True, \
                            key='COLUMN-SC', justification='center', pad=0))    
    
    dev_layout.append(sg.Column(log_col, vertical_alignment = 'top', expand_y = 'True', scrollable=False, vertical_scroll_only=True, \
                            key='COLUMN-LOG', justification='center',pad=0))
    
    print_log(f'zabers = {zaber_motors}, colums = {zaber_motors // zaber_motors_in_GUI}, remainder= {zaber_motors % zaber_motors_in_GUI}/{zaber_motors - (zaber_motors // zaber_motors_in_GUI) * zaber_motors_in_GUI}')
    for _it in range (0, zaber_motors // zaber_motors_in_GUI ):
        dev_layout.append(sg.Column(zaber_frames(_it*zaber_motors_in_GUI + 1, _it*zaber_motors_in_GUI + zaber_motors_in_GUI ), scrollable=False, vertical_scroll_only=True, \
                            key='COLUMN-Z1', vertical_alignment='center', justification='center',pad=0, expand_y = 'True'))
        print_log(f'Adding zabers {_it*zaber_motors_in_GUI + 1} - {_it*zaber_motors_in_GUI + zaber_motors_in_GUI}')

    if (zaber_motors % zaber_motors_in_GUI) > 0:
        print_log(f'Adding zabers {(zaber_motors // zaber_motors_in_GUI) * zaber_motors_in_GUI  + 1} - {zaber_motors}')
        dev_layout.append(sg.Column(zaber_frames((zaber_motors // zaber_motors_in_GUI) * zaber_motors_in_GUI  + 1, \
                zaber_motors ), \
                    scrollable=False, vertical_scroll_only=True, \
                key='COLUMN-Z1', vertical_alignment='center', justification='center',pad=0, expand_y = 'True'))


    dev_layout.append(sg.Column(gripper_list, scrollable=False, vertical_scroll_only=True, \
                            key='COLUMN_GT', vertical_alignment='center', justification='center',pad=0, expand_y = 'True'))

    dev_layout.append(sg.Column(trolley_list, scrollable=False, vertical_scroll_only=True, \
                            key='COLUMN_GT', vertical_alignment='center', justification='center',pad=0, expand_y = 'True'))


    _rotators = distance_rotators + time_rotators

    print_log(f'rotators = {_rotators}, colums = {_rotators // rotator_in_GUI}, remainder= {_rotators % rotator_in_GUI}/{_rotators - (_rotators // rotator_in_GUI) * rotator_in_GUI}')
    for _it in range (0, distance_rotators // rotator_in_GUI ):
        dev_layout.append(sg.Column(dist_rotators_frames(_it + 1, _it + rotator_in_GUI ), scrollable=False, vertical_scroll_only=True, \
                            key='COLUMN-Z1', vertical_alignment='center', justification='center',pad=0, expand_y = 'True'))
    last_rotator_column_list = list()
    if (distance_rotators % rotator_in_GUI) > 0:
        print_log(f'Adding rotators: {(distance_rotators // rotator_in_GUI) * rotator_in_GUI  + 1} - {distance_rotators} + {time_rotators} spinners')
        last_rotator_column_list = dist_rotators_frames((distance_rotators // rotator_in_GUI) * rotator_in_GUI  + 1, \
                distance_rotators)
        # last_rotator_column = sg.Column(dist_rotators_frames((distance_rotators // rotator_in_GUI) * rotator_in_GUI  + 1, \
        #         distance_rotators), \
        #         scrollable=False, vertical_scroll_only=True, \
        #         key='COLUMN-Z1', vertical_alignment='center', justification='center',pad=0, expand_y = 'True')
        pass
    if time_rotators > 0:
        last_rotator_column_list.append([sg.Frame('', timeRotator, pad=BPAD_TOP,  expand_x=True,  relief=sg.RELIEF_GROOVE, \
                            border_width=3, vertical_alignment='top', element_justification = "center")])

    dev_layout.append(sg.Column(last_rotator_column_list, \
                scrollable=False, vertical_scroll_only=True, \
                key='COLUMN-Z1', vertical_alignment='center', justification='center',pad=0, expand_y = 'True'))

    layout = [
                [sg.Frame('', top_banner,   pad=(0,0), background_color=DARK_HEADER_COLOR,  expand_x=True, \
                            border_width=0, grab=True)],
                dev_layout,
                
                [sg.Sizegrip(background_color=BORDER_COLOR)]
                ]
    return layout

    # if zaber_motors > 6:

    #     layout = [
    #             [sg.Frame('', top_banner,   pad=(0,0), background_color=DARK_HEADER_COLOR,  expand_x=True, \
    #                         border_width=0, grab=True)],
                
    #             [   
    #                 sg.Column(script_col, vertical_alignment = 'top', expand_y = 'True', scrollable=False, vertical_scroll_only=True, \
    #                         key='COLUMN-SC', justification='center', pad=0),
    #                 sg.Column(log_col, vertical_alignment = 'top', expand_y = 'True', scrollable=False, vertical_scroll_only=True, \
    #                         key='COLUMN-LOG', justification='center',pad=0),         
    #                 sg.Column(zaber_frames(1,(int)(zaber_motors/2)), scrollable=False, vertical_scroll_only=True, \
    #                         key='COLUMN-Z1', vertical_alignment='center', justification='center',pad=0, expand_y = 'True'), 
    #                 sg.Column(zaber_frames((int)(zaber_motors/2)+1, 2*(int)(zaber_motors/2)), scrollable=False, vertical_scroll_only=True, \
    #                         key='COLUMN-Z2', vertical_alignment='center', justification='center',pad=0, expand_y = 'True'),

    #             sg.Column(gripper_list, scrollable=False, vertical_scroll_only=True, \
    #                         key='COLUMN_GT', vertical_alignment='center', justification='center',pad=0, expand_y = 'True'), 

    #                 sg.Column(trolley_list, scrollable=False, vertical_scroll_only=True, \
    #                         key='COLUMN_GT', vertical_alignment='center', justification='center',pad=0, expand_y = 'True'), 
    #                 sg.Column(dist_rotators_list, scrollable=False, vertical_scroll_only=True, \
    #                         key='COLUMN_GT', vertical_alignment='center', justification='center',pad=0, expand_y = 'True'), 
                    
    #                 ],
    #             [sg.Sizegrip(background_color=BORDER_COLOR)]
    #             ]
    #     return layout
        
    # else:
    #     layout = [
    #             [sg.Frame('', top_banner,   pad=(0,0), background_color=DARK_HEADER_COLOR,  expand_x=True, \
    #                         border_width=0, grab=True)],
                
    #             [   
    #                 sg.Column(script_col, vertical_alignment = 'top', expand_y = 'True', scrollable=False, vertical_scroll_only=True, \
    #                         key='COLUMN-SC', justification='center', pad=0),
    #                 sg.Column(log_col, vertical_alignment = 'top', expand_y = 'True', scrollable=False, vertical_scroll_only=True, \
    #                         key='COLUMN-LOG', justification='center',pad=0),         
    #                 sg.Column(zaber_frames(1,(int)(zaber_motors/1)), scrollable=False, vertical_scroll_only=True, \
    #                         key='COLUMN-Z1', vertical_alignment='center', justification='center',pad=0, expand_y = 'True'), 

    #             sg.Column(gripper_list, scrollable=False, vertical_scroll_only=True, \
    #                         key='COLUMN_GT', vertical_alignment='center', justification='center',pad=0, expand_y = 'True'), 

    #                 sg.Column(trolley_list, scrollable=False, vertical_scroll_only=True, \
    #                         key='COLUMN_GT', vertical_alignment='center', justification='center',pad=0, expand_y = 'True'), 
    #                 sg.Column(dist_rotators_list, scrollable=False, vertical_scroll_only=True, \
    #                         key='COLUMN_GT', vertical_alignment='center', justification='center',pad=0, expand_y = 'True'), 
                    
    #                 ],
    #             [sg.Sizegrip(background_color=BORDER_COLOR)]
    #             ]
        
    #     return layout

# === end InitGUI settings


collors_LST = ['RED', 'GREEN', 'BLUE', 'BUZZ']
def activateAlarm(color, devsLst, state = True)->bool:
    for _dev in devsLst:
        if _dev.C_type == '--PHG--' :
            if _dev.get_device().devName in collors_LST:
                if color is None:
                    _dev.get_device().devOperate(state)
                elif _dev.get_device().devName == color:
                    _dev.get_device().devOperate(state)
                    return True
        
    return False





def formFillProc(event, values, window, realNum = False, positiveNum = True, defaultValue = 0, max_size = 0):
    print_DEBUG(f'DEBUG: Event received =  {event}, values = {values[event]}, positiveNum = {positiveNum}, default value = {defaultValue}')
    if realNum:
        valValidator = real_validator
        numValidator = real_num_validator
        cast = float
    else:
        valValidator = int_validator
        numValidator = int_num_validator
        cast = int

    if max_size > 0 and len(str(values[event])) > max_size:
        print_DEBUG(f'Oversize - {values[event]}= {len(str(values[event]))}')
        window[event].update(values[event][:-1])
        return values[event][:-1]


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
                    # element_padding=(0,0), element_justification='c', finalize = True)
    

    window.read(timeout=0) 
    window.Finalize()
    print_log(f'Starting GUI. trolley = {trolley_motors}, zaber ={zaber_motors}, gripper = {gripper_motors}')

    deactivateGUI(window)

    sg.PopupAnimated(image_source=gif103, text_color='blue', 
                    message='Loading...',background_color='grey', time_between_frames=100)    

    return window

def startIO(window:sg.Window,devs_list:List):
    _IOs = list()
    ind = 0
    for ind, _dev in enumerate(devs_list):
        if _dev.C_type == '--IOCONTROL--':
            _op_device_ind = LocateDevice (_dev.get_device().getIODev(), devs_list)
            if _op_device_ind is not None:
                _op_dev = devs_list[_op_device_ind]
                _dev.get_device().startOp(window, _op_dev.get_device())
                print_log(f'Configuring IO control {_dev.get_device().devName} be operated by {_op_dev.get_device().devName}')
                _IOs.append(_dev)
            else:
                print_err(f'No IO device found for {_dev.get_device().devName} IO control')

    print_log(f'Found {ind} configured IO controls: {_IOs}')


# def armInterlockDevs(window:sg.Window,devs_list:List)->List:
#     _lcks = list()
#     ind = 0
#     for ind, _dev in enumerate(devs_list):
#         if _dev.C_type == '--INTERLOCK--':
#             ni_interlock_ind = LocateDevice (_dev.get_device().getIODev(), devs_list)
#             if ni_interlock_ind > 0:
#                 ni_interlock = devs_list[ni_interlock_ind]
#                 _dev.get_device().startOp(window, ni_interlock.get_device())
#                 print_log(f'Arming Interlock {_dev.get_device().devName} at {ni_interlock.get_device().devName}')
#                 _lcks.append(_dev)
#             else:
#                 print_err(f'No DAQ device found for {_dev.get_device().devName} device')
#     print_log(f'Armend {ind} intelock devices: {_lcks}')

#     return _lcks

def _stopProc(scriptQ:Queue, eventQ:Queue, gui_task_list:list):
    scriptQ.put('-STOP-')
    removeElementQ(eventQ, '-SCRIPT_STEP-')
    
    for tsk in gui_task_list.getAllTasks():
        tsk.EmergencyStop()


def workingCycle (window, parms:dict):

    # global emergency_stop_pressed
                                            # init devices and pereferials 
    devs_list = port_scan()
    parms_table:dict = parms
    initGUIDevs(window, devs_list)

    scriptRunning:bool = False
    # _emergency_status:bool = False

    statusPub = anim_0MQ()
    statusPub._publisher(ZMQ_PORT)
    # ni_dev = None
    # calibration_cam = None

    sg.PopupAnimated(image_source=None)

    print_log(f'{len(devs_list)} devices found')
    scriptQ:Queue = Queue()

    startIO(window,devs_list)
    # Arming Interlock devs
    # LCKs_List = armInterlockDevs(window,devs_list)

    # interlock2_index = LocateDevice('-LCK2-', devs_list)
    # if interlock2_index > 0:                                            # LCK2 found
    #     print_log(f'Found LCK = {devs_list[interlock2_index]}, ind = {interlock2_index} ')
    #     ni_interlock_ind = LocateDevice (devs_list[interlock2_index].get_device().getIODev(), devs_list)
    #     print_log(f'Found ni interlock index = {ni_interlock_ind}/{devs_list[ni_interlock_ind]}')
    #     ni_interlock = devs_list[ni_interlock_ind]
    #     print_log(f'Found NI = {devs_list[ni_interlock_ind]}')

    #     devs_list[interlock2_index].get_device().startOp(window, ni_interlock.get_device())
    # else:
    #     print_log(f'No EMO (LCK2) was defined')

    control_monitor = sm.StatusMonitor(window = window, devs_list = devs_list, statusPub = statusPub, timeout = 1)
    # control_monitor = sm.StatusMonitor(window = window, devs_list = devs_list, statusPub = statusPub, timeout = 60)  #BUGBUGBUG
    process_manager = pm.ProcManager(window=window)
    gui_task_list = pm.WorkingTasksList()

    eventQ:Queue = Queue()
    # lockState:True = False

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
                        
                    scriptQ.put('-STOP-')    
                    # _emergency_status = True
                    activateAlarm(None, devs_list, False)                   # switch off all
                    activateAlarm('RED', devs_list, True)                   
                    sg.popup_error(f'Attention.\nAbnormal termination of {recvEvent.device}', background_color = 'orange')
                    
                    
                    continue

                elif recvEvent.event == '-DECREMENT_CYCLIC-':
                    _cyclic_iter = recvEvent.value
                    window[f'-CYCLIC_RUN-'].update(_cyclic_iter)
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
                            if not _DEBUG:
                                window.write_event_value('Stop', None)  
                                print_err(f'Cant perform STEP, since one of listed commands operate device that is not active. Stop script')

                            print_err(f'Cant perform STEP, since one of listed commands operate device that is not active. Go next')
                            sg.popup_error(f'Cant perform STEP, since one of \nlisted commands operate device that is not active.\n End Script', background_color = 'orange')
                            activateScriptControl(window)
                            if steptask:                                            # if it's part of script 
                                scriptQ.put('-STEP_DONE-')       

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



        elif event == sg.WIN_CLOSED or event == 'Exit':

            print_log(f'Exit received. Stop all activity')
            for tsk in gui_task_list.getAllTasks():
                tsk.EmergencyStop()

            if control_monitor:
                control_monitor.StopMonitor()

            # sg.PopupAnimated(image_source='Images/attention_image.gif',message='Wait for Zaber devices be parked',no_titlebar=False)
            # Zaber_Motor.park_all()

            sg.PopupAnimated(image_source=None)
            break

        # elif emergency_stop_pressed == True and no_motion(devs_list):
        #     emergency_stop_pressed = False
        #     sg.popup_auto_close(f'Emergency stop\ndone', non_blocking = True)

                      

            
        # if event == '__TIMEOUT__':
        #     ThreadCloseProceedure(window, devs_list, statusPub)
        #     continue

        # elif event == '-TASK_DONE-':
        #     taskID = values['-TASK_DONE-']
        #     completedTask = gui_task_list.getTask(taskID)
        #     print_log(f'-TASK_DONE- event received (task id = {taskID}). Task = {completedTask} ')
        #     if completedTask:
        #         TaskCloseProceedure(window, completedTask)
        #         if completedTask.isStep():
        #             print_log(f'STEP task DONE')
        #             scriptQ.put('-STEP_DONE-')
        #         else:
        #             print_log(f'Single task DONE')
        #         res = gui_task_list.delTask(taskID)
        #     else:
        #         print_err(f'-ERROR- No task # {taskID} found in the tasks list')
        #         continue

        #     activateScriptControl(window)

        #     # window.write_event_value(f'-UPDATE_STATUS-', control_monitor)  #BUGBUGBUG 
        #     control_monitor.monitorUpdate(realTime = True)

        #     continue
        
        # elif event == '-TASK_ERROR-':
        #     taskID = values['-TASK_ERROR-']
        #     completedTask = gui_task_list.getTask(taskID)
        #     if completedTask:
        #         completedTask.EmergencyStop()
        #         TaskCloseProceedure(window, completedTask)
        #         res = gui_task_list.delTask(taskID)
        #         # sg.popup_auto_close(f'Emergency stop \noperated', non_blocking = True)
        #     else:
        #         print_err(f'-ERROR- No task # {taskID} found in the tasks list')
        #         continue
            
        #     if completedTask.isStep():
        #         # scriptQ.put('-STOP-')
        #         activateScriptControl(window)


        #     # window.write_event_value(f'-UPDATE_STATUS-', control_monitor)  #BUGBUGBUG 
        #     control_monitor.monitorUpdate(realTime = True)

        #     continue
        
        elif event == 'Park':
            # if not no_motion(devs_list):
            #     continue
            print_log(f'Parking all devices')
            Zaber_Motor.park_all()

            for m_dev in devs_list:
                if m_dev.C_type == '--ZABER--':
                    i = m_dev.c_gui
                    DeActivateMotorControl(window, '--ZABER--', i)
                    SetLED (window, f'_zaber_{i}_', 'blue')


            # for i in range (1, zaber_motors + 1):
            #     DeActivateMotorControl(window, '--ZABER--', i)
            #     SetLED (window, f'_zaber_{i}_', 'blue')


            continue
        elif event == 'Unpark':
            # if not no_motion(devs_list):
            #     continue
            print_log(f'Unparking all devices')
            Zaber_Motor.unpark_all()
            Correct_ZB_possition_after_unpark(window, devs_list)


            for m_dev in devs_list:
                if m_dev.C_type == '--ZABER--':
                    i = m_dev.c_gui
                    ActivateMotorControl(window, '--ZABER--', i)


            # for i in range (1, zaber_motors + 1):
            #     ActivateMotorControl(window, '--ZABER--', i)

            continue

        elif event == 'RELOAD':
            parms_table = read_params()
            for m_dev in devs_list:
                m_dev.get_device().set_parms(parms_table)
            initGUIDevs(window, devs_list)
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
            activateAlarm(None, devs_list, False)                   # switch off all

            # if  _emergency_status:
            #     activateAlarm('RED', devs_list, True)                   
            #     _emergency_status = False

            print_log(f'Stop button was pressed')
            _stopProc(scriptQ, eventQ, gui_task_list)
            # scriptQ.put('-EMERGENCY_STOP-')
            # removeElementQ(eventQ, '-SCRIPT_STEP-')
            
            # for tsk in gui_task_list.getAllTasks():
            #     tsk.EmergencyStop()

            
            # emergency_stop_pressed = True
            sg.popup_auto_close(f'Stop done', non_blocking = True)
            # window[f'Run'].update(disabled = False)

           
            continue

        elif event == '-CYCLIC_RUN-':
            new_val = formFillProc(event, values, window, realNum = False, positiveNum = True, defaultValue = 1, max_size=2)
            continue

        elif event == 'Run':
            if not (window['-SCRIPT_NAME-'].get()[-3:].lower() == 'yml'):
                print_log(f'No script loaded. Default ={window["-SCRIPT_NAME-"].get()}')
                continue
            activateAlarm("GREEN", devs_list, True)                   # 

            if len(gui_task_list.getAllTasks()) > 0:
                sg.popup_auto_close("The new script can't run\nuntill previous operation completed", non_blocking = True)
                continue

            if isThereParkedZaber(window, devs_list):
                _ch = sg.popup_yes_no("There are parked Zaber devices.\nDo you want to cancel automatic runnning?",  title="Zaber parked")
                if _ch ==  "Yes" :
                    continue


            # window[f'Run'].update(disabled = True)

            deActivateScriptControl(window)

            # script_thread = Thread(target=run_script_cont_thread, args=(window, devs_list, values['-TABLE-'], '-SCRIPT-RUN-DONE-', script_condition, ))
            # 
            # window.perform_long_operation(lambda :
            #     run_script_cont(window, devs_list, values['-TABLE-']), '-SCRIPT-RUN-DONE-')
            cyclic = int(values['-CYCLIC_RUN-'])
            print_log(f'Running new script thread!')

            script_thread = Thread(target=ScriptRunner, args=(window, devs_list, values['-TABLE-'], \
                                                                        scriptQ, '-SCRIPT_DONE-', eventQ, cyclic,  ))
            script_thread.start()
            print_log(f'Running new script thread = {script_thread}/{script_thread.ident}')

            scriptRunning = True
            continue
        
        elif event == '-SCRIPT_DONE-':

            activateAlarm(None, devs_list, False)                   # switch off all
            # if  _emergency_status:
            #     activateAlarm('RED', devs_list, True)                   
            #     _emergency_status = False

            print_log(f'Script completed. Activating controls')
            activateScriptControl(window)
            # sg.popup_auto_close('Scritp done')
            scriptRunning = False
            continue



        elif event in ['Step', '-TABLE-']:
        # elif event in ['Step', '-TABLE-', '-SCRIPT_STEP-']:
        # elif event in ['Step',  '-SCRIPT_STEP-']:
            if isThereParkedZaber(window, devs_list):
                _ch = sg.popup_yes_no("There are parked Zaber devices.\nDo you want to cancel the runnning?",  title="Zaber parked")
                if _ch ==  "Yes" :
                    continue
                
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
                if wTask == None :
                    if not _DEBUG:
                        print_err(f'Cant perform STEP, since one of listed commands operate device that is not active. Stop script')
                        window.write_event_value('Stop', None)

                    print_err(f'Cant perform STEP, since one of listed commands operate device that is not active. Go next')
                    activateScriptControl(window)

                    if steptask:                                            # if it's part of script 
                        scriptQ.put('-STEP_DONE-')

                    continue

            else:
                print_err(f'Script validation failed for script: {scriptStep} ')

            # if no_motion(devs_list):
            #     event_procedure_in_progress = True
            #     script_proc(window, devs_list, event, values)
            #     event_procedure_in_progress = False
            
        
        elif event == 'Open File':
            # _script_file_name = LoadScriptFile(parms)
            
            _script_file_name = LoadScriptFile(parms_table)

            if _script_file_name is not None:
                window[f'-SCRIPT_NAME-'].update(_script_file_name) 
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


        if (not ((event == 'Step') or (event == '-SCRIPT_STEP-') or (event == '-TABLE-') or (event == '-INTER_LOCK_ENABLE-'))) and d_index is None:
        # if (not ((event == 'Step') or (event == '-SCRIPT_STEP-'))) and d_index < 0:
            continue


# Peripheral devices section
#

        # ni_dev = get_dev('--DAQ-NI--', devs_list)
        # calibration_cam = get_dev('--CAM--', devs_list)
        hmp_dev = get_dev('--HMP--', devs_list)
        # phg_dev = get_dev('--PHG--', devs_list)
        # marco_dev = get_dev('--MARCO--', devs_list)

        if  '-PHG-' in event:
            DeActivateMotorControl(window, '--PHG--', devs_list[d_index].c_gui)
            if '-BUTTON-' in event:                     # i.e. -1-PHG-BUTTON-
                if devs_list[d_index].get_device().dType() == PhidgetRELAY.devType.trigger:
                    wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.phg_operate), sType = pm.RunType.single)
                elif devs_list[d_index].get_device().dType() == PhidgetRELAY.devType.toggle:
                    wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.phg_operate), sType = pm.RunType.single)
                    if devs_list[d_index].get_device().isOpen():
                        window[event].update(button_color='gray')
                    else:
                        window[event].update(button_color='yellow')     
                else:
                    print_err(f'ERROR - Unrecognized event: {event} for device {devs_list[d_index]}')
            elif  devs_list[d_index].get_device().dType() == PhidgetRELAY.devType.onoff:                
                if  '-ON-' in event:                 # i.e. -2-PHG-ON-  
                    wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.phg_operate, args=pm.argsType(light_on = True)), \
                                           sType = pm.RunType.single)
                    window[event].update(button_color='dark green on green')
                    window[f'-{devs_list[d_index].c_gui}-PHG-OFF-'].update(button_color='white on red')
                elif  '-OFF-' in event:                  # -2-PHG-OFF-
                    wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.phg_operate, args=pm.argsType(light_on = False)),\
                                            sType = pm.RunType.single)
                    window[event].update(button_color='tomato on red')
                    window[f'-{devs_list[d_index].c_gui}-PHG-ON-'].update(button_color='white on green')

                else:
                    print_err(f'ERROR - Unrecognized event: {event} for device {devs_list[d_index]}')
                    continue
            elif devs_list[d_index].get_device().dType() == PhidgetRELAY.devType.trigger and ('-OFF-' in event):
                wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.phg_operate, args=pm.argsType(light_on = False)),\
                                        sType = pm.RunType.single)
                # window[event].update(button_color='tomato on red')
            else:
                print_err(f'ERROR - Unrecognized event: {event} for device {devs_list[d_index]}')
                continue
            
            ActivateMotorControl(window, '--PHG--', devs_list[d_index].c_gui)

##########################                        

        # if event == 'TRIGGER':
        #     if ni_dev and devs_list[d_index].C_type == '--DAQ-NI--':
        #         DeActivateMotorControl(window, '--DAQ-NI--')
        #         print_log(f'Trigger - {values["-SELECTOR-"]} ')

        #         wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.trigger, args=pm.argsType(trigger_selector=int(values["-SELECTOR-"][-1]))), sType = pm.RunType.single)


        #         print_log(f'{int(values["-SELECTOR-"][-1])} schedule trigger will be sent')
        #         ActivateMotorControl(window, '--DAQ-NI--')
        #     elif phg_dev  and devs_list[d_index].C_type == '--PHG--':
        #         DeActivateMotorControl(window, '--PHG--', devs_list[d_index].c_gui)

        #         wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.phg_trigger), sType = pm.RunType.single)

        #         print_log(f'Welder trigger will be sent. dev = {devs_list[d_index]}/ {devs_list[d_index].get_device()}')
        #         ActivateMotorControl(window, '--PHG--', devs_list[d_index].c_gui)
        #     else:
        #         print_err(f'No NI/PHG device found to operate TRIGGER control')
            

        # elif event == 'DISP_TRIG':
            
        #     if phg_dev:
        #         DeActivateMotorControl(window, '--PHG--')

        #         wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.phg_trigger), sType = pm.RunType.single)

        #         print_log(f'Dispenser trigger will be sent. dev = {devs_list[d_index]}/ {devs_list[d_index].get_device()}')
        #         ActivateMotorControl(window, '--PHG--')
        #     else:
        #         print_err(f'No PGH device found to operate TRIGGER control')

        # elif event == 'UV_TRIG':
            
        #     if phg_dev:
        #         DeActivateMotorControl(window, '--PHG--')

        #         wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.phg_trigger), sType = pm.RunType.single)

        #         print_log(f'UV trigger will be sent. dev = {devs_list[d_index]}/ {devs_list[d_index].get_device()}')
        #         ActivateMotorControl(window, '--PHG--')
        #     else:
        #         print_err(f'No PGH device found to operate TRIGGER control')

        # elif event == 'FRONT_LIGHT':
            
        #     if phg_dev:
        #         DeActivateMotorControl(window, '--PHG--')
                
        #         if devs_list[d_index].get_device().isOpen():
        #             window[f'FRONT_LIGHT'].update(button_color='gray')
        #         else:
        #             window[f'FRONT_LIGHT'].update(button_color='yellow')

        #         wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.phg_toggle), sType = pm.RunType.single)
                

        #         print_log(f'FRONT LIGHT  will be sent. dev = {devs_list[d_index]}/ {devs_list[d_index].get_device()}')
        #         ActivateMotorControl(window, '--PHG--')
        #     else:
        #         print_err(f'No PGH device found to operate TRIGGER control')

        # elif event == 'BACK_LIGHT':
            
        #     if phg_dev:
        #         DeActivateMotorControl(window, '--PHG--')

        #         if devs_list[d_index].get_device().isOpen():
        #             window[f'BACK_LIGHT'].update(button_color='gray')
        #         else:
        #             window[f'BACK_LIGHT'].update(button_color='yellow')

        #         wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.phg_toggle), sType = pm.RunType.single)



        #         print_log(f'BACK LIGHT trigger will be sent. dev = {devs_list[d_index]}/ {devs_list[d_index].get_device()}')
        #         ActivateMotorControl(window, '--PHG--')
        #     else:
        #         print_err(f'No PGH device found to operate TRIGGER control')

        # elif event == 'HOLE_LIGHT':
            
        #     if phg_dev:
        #         DeActivateMotorControl(window, '--PHG--')

        #         if devs_list[d_index].get_device().isOpen():
        #             window[f'HOLE_LIGHT'].update(button_color='gray')
        #         else:
        #             window[f'HOLE_LIGHT'].update(button_color='yellow')

        #         wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.phg_toggle), sType = pm.RunType.single)



        #         print_log(f'HOLE LIGHT trigger will be sent. dev = {devs_list[d_index]}/ {devs_list[d_index].get_device()}')
        #         ActivateMotorControl(window, '--PHG--')
        #     else:
        #         print_err(f'No PGH device found to operate TRIGGER control')

        # elif event == 'HOTAIR-STOP':
        #     if (ni_dev):
        #         DeActivateMotorControl(window, '--DAQ-NI--')
        #         print_log(f'Stoping JTSE Hot Air Station')

        #         wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.hotair, args=pm.argsType(start_stop=False)), sType = pm.RunType.single)
        #         # ni_dev.jtseStop()

        #         ActivateMotorControl(window, '--DAQ-NI--')
        #     elif phg_dev  and devs_list[d_index].C_type == '--PHG--':
        #         DeActivateMotorControl(window, '--PHG--')


        #         wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.phg_toggle, args=pm.argsType(light_on = False)), sType = pm.RunType.single)

        #         print_log(f'JTSE Hotair OFF/STOP will be sent. dev = {devs_list[d_index]}/ {devs_list[d_index].get_device()}')
        #         ActivateMotorControl(window, '--PHG--')
        #     else:
        #         print_err(f'No NI device found to operate JTSE control')

            

        # elif event == 'HOTAIR-START':
        #     if (ni_dev):
        #         DeActivateMotorControl(window, '--DAQ-NI--')
        #         print_log(f'Operating  JTSE Hot Air Station')

        #         wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.hotair, args=pm.argsType(start_stop=True)), sType = pm.RunType.single)
        #         # ni_dev.jtseStart()
                
        #         ActivateMotorControl(window, '--DAQ-NI--')
        #     elif phg_dev  and devs_list[d_index].C_type == '--PHG--':
        #         DeActivateMotorControl(window, '--PHG--')

        #         wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.phg_toggle, args=pm.argsType(light_on = True)), sType = pm.RunType.single)

        #         print_log(f'JTSE Hotair ON/START will be sent. dev = {devs_list[d_index]}/ {devs_list[d_index].get_device()}')
        #         ActivateMotorControl(window, '--PHG--')
        #     else:
        #         print_err(f'No NI/PHG device found to operate JTSE control')


        # ni = LocateDevice (devs_list[d_index].get_device().getIODev(), devs_list)

        # elif '-LCK' in event or event == '-INTER_LOCK_ENABLE-': 
        #     if values['-INTER_LOCK_ENABLE-']:
        #         _alarmed = False
        #         for _intl_dev in LCKs_List:
        #             if _intl_dev.get_device().getStatus():          # interlock in alarm status
        #                 window.write_event_value('Stop', None)
        #                 activateAlarm(None, devs_list, False)
        #                 activateAlarm('RED', devs_list)
        #                 deactivateGUI(window)
        #                 print_log(f'{ _intl_dev} interlock device is alarmed')
        #                 _alarmed = True
        #                 break
        #         if not _alarmed:
        #             print_log(f'No alarmed devices were found')
        #             activateAlarm(None, devs_list, False)
        #             activateAlarm('GREEN', devs_list)
        #             initGUIDevs(window, devs_list)
        #     else:
        #         print_log(f'Interlock V-box disarmed')
        #         activateAlarm(None, devs_list, False)
        #         activateAlarm('GREEN', devs_list)
        #         initGUIDevs(window, devs_list)

        #     continue

        # elif event == '-DOOR-' or event == '-EMERG-':
        #     print_log(f'Interlock event happnes. enable = {values['-INTER_LOCK_ENABLE-']}')
        #     if values['-INTER_LOCK_ENABLE-']:                               # inrelock selected
        #         if devs_list[d_index].get_device().getStatus():
        #             window.write_event_value('Stop', None)                  # send Stop 
        #             activateAlarm(None, devs_list, False)
        #             activateAlarm('RED', devs_list)
        #             deactivateGUI(window)
        #             print_log(f'{event[1:-2]}  is alarmed')
        #             window.write_event_value('-INTER_LOCK_ENABLE-', None) 

        elif event == '-INTER_LOCK_ENABLE-' or  event == '-DOOR-' or event == '-EMERG-':
            _door = LocateDevice('-DOOR-', devs_list)
            _emer = LocateDevice('-EMERG-', devs_list)
            if _door is None or _emer is None:
                print_err(f'The DOOR and/or EMO device are no present in the system. Interlock will be ignored')
                sg.popup_auto_close(F'The DOOR and/or EMO device are no present in the system.\nInterlock will be ignored')
                continue
            _door_status = devs_list[_door].get_device().getStatus()
            _emer_status = devs_list[_emer].get_device().getStatus()
            print_log(f'Interlock enale checket up. Door status = {_door_status}, Emergency button status = {_emer_status}')
            if values['-INTER_LOCK_ENABLE-'] and (_door_status or _emer_status):   # door opened or emergesy pressed while inrelock selected
                _stopProc(scriptQ, eventQ, gui_task_list)
                # window.write_event_value('Stop', None)                  # send Stop 
                activateAlarm(None, devs_list, False)
                activateAlarm('RED', devs_list)
                deactivateGUI(window)
                print_log(f'event {event} of dev {event[1:-1]} is alarmed. Stop activated')
            else:
                activateAlarm('RED', devs_list, False)
                initGUIDevs(window, devs_list)
            
            continue
        
        elif event == '-ON_OFF_BUTTON-':
            _on_off_ind = LocateDevice('-ON_OFF_BUTTON-', devs_list)
            if devs_list[d_index].get_device().getStatus():                     # on (GREEN) pressed 
                print_log(f'Run (GREEN) pressed. Stop Script')
                _door = LocateDevice('-DOOR-', devs_list)
                _emer = LocateDevice('-EMERG-', devs_list)
                if _door is not None and _emer is not None:
                    _door_status = devs_list[_door].get_device().getStatus()
                    _emer_status = devs_list[_emer].get_device().getStatus()
                    print_log(f'ON (GREEN) pressed. Door status = {_door_status}, Emergency button status = {_emer_status}, Interlock enable = {values["-INTER_LOCK_ENABLE-"]}')
                    if not (values['-INTER_LOCK_ENABLE-'] and (_door_status or _emer_status)):   # can be operated
                        window.write_event_value('Run', None) 
                    else:
                        print_log(f"Can't run Script. door = {_door_status}, emerg = {_emer_status}")
                else:
                    print_err(f'ERROR: There is no DOOR [{_door}] or EMERG device [{_emer}] present in the system')
            else:
                print_log(f'Stop (RED) pressed. Stop Script')
                _stopProc(scriptQ, eventQ, gui_task_list)
                # window.write_event_value('Stop', None)                  # send Stop 
            continue

        elif event == '-D1_D2_GRIPERS_TOGGLE-':
            print_log(f'{event} received')
            _d1_ind = LocateDevice('-1-DH-GRIPPER-OFF-', devs_list)
            _d2_ind = LocateDevice('-2-DH-GRIPPER-OFF-', devs_list)
            _d1_d2 = LocateDevice('-D1_D2_GRIPERS_TOGGLE-', devs_list)
            if _d1_ind is not None and _d2_ind is not None and _d1_d2 is not None:
                _d1_d2_status = devs_list[_d1_d2].get_device().getStatus()
                _d1_stat = devs_list[_d1_ind].get_device().OnOff
                _d2_stat = devs_list[_d2_ind].get_device().OnOff
                if not _d1_d2_status:
                    continue
                if _d1_stat  and _d2_stat:
                    print_log('Closing both DH Grippers')
                    devs_list[_d1_ind].get_device().gripper_off()
                    devs_list[_d2_ind].get_device().gripper_off()
                elif not _d1_stat  and not _d2_stat:
                    print_log('Opening both DH Grippers')
                    devs_list[_d1_ind].get_device().gripper_on()
                    devs_list[_d2_ind].get_device().gripper_on()
                elif _d1_stat:
                    print_log('Opening DH Gripper - 1')
                    devs_list[_d1_ind].get_device().gripper_off()
                else:
                    print_log('Closing DH Gripper - 1')
                    devs_list[_d1_ind].get_device().gripper_on()
            else:
                print_err(f'ERROR: D1 [{_d1_ind}] or D2 [{_d2_ind}] or both [{_d1_d2}] are not present in the system ')
            
            continue

                
                



        # elif event == '-LCK1-' or event == '-INTER_LOCK_ENABLE-':                     # interlock DAQ
        #     ni_interlock_ind = LocateDevice (devs_list[d_index].get_device().getIODev(), devs_list)

        #     ni_interlock = devs_list[ni_interlock_ind]

        #     print_log(f'event = {event}, interlock = {values["-INTER_LOCK_ENABLE-"]}, lock status = {devs_list[d_index].get_device().getStatus()} ')

        #     if event == '-INTER_LOCK_ENABLE-':
        #         if values['-INTER_LOCK_ENABLE-']:
        #             devs_list[d_index].get_device().startOp(window, ni_interlock.get_device())
        #             print_log(f'Activating operation for {devs_list[d_index].get_device()} on {ni_interlock}')
        #         else:
        #             devs_list[d_index].get_device().stopOp()
        #             print_log(f'DEactivating operation for {devs_list[d_index].get_device()} on {ni_interlock}')


        #     if devs_list[d_index].get_device().getStatus() == True:        # door is open (PIZZATO blue/black pair)
        #         if values['-INTER_LOCK_ENABLE-']:
        #             print_log(f'Door is open, Interlock is armed -> deactivete controls //  {devs_list[d_index].get_device()} on {ni_interlock}')
        #             window.write_event_value('Stop', None)
        #             deactivateGUI(window)
        #         else:
        #             devs_list[d_index].get_device().stopOp()
        #             print_log(f'Door is open, Interlock is NOT armed -> DO NOT deactivete controls //  {devs_list[d_index].get_device()} on {ni_interlock}')
        #     else:                                                       # door is closed
        #         if activeGUIflag == False:
        #             print_log(f'Door is closed, GUI is NOT inactive -> activate GUI // {devs_list[d_index].get_device()} on {ni_interlock}')
        #             initGUIDevs(window, devs_list)
        #         else:
        #             print_log(f'Door is closed, GUI is ALREADY active -> DO NOT activate GUI // {devs_list[d_index].get_device()} on {ni_interlock}')
            

        #     continue

        # elif event == '-LCK2-':
        #     ni_interlock_ind = LocateDevice (devs_list[d_index].get_device().getIODev(), devs_list)

        #     ni_interlock = devs_list[ni_interlock_ind]

        #     print_log(f'event = {event}, lock status = {devs_list[d_index].get_device().getStatus()} ')
        #     if devs_list[d_index].get_device().getStatus() == False:        # EMO is pressed
        #         print_log(f'EMO is pressed -> deactivete controls //  {devs_list[d_index].get_device()} on {ni_interlock}')
        #         window.write_event_value('Stop', None)
        #         deactivateGUI(window)
        #         activateAlarm(None, devs_list, False)
        #         activateAlarm('RED', devs_list)
        #         pass

        #     else:                                                           # True - EMO is unpressed
        #         print_log(f'EMO is released -> activate GUI // {devs_list[d_index].get_device()} on {ni_interlock}')
        #         initGUIDevs(window, devs_list)
        #         activateAlarm(None, devs_list, False)
        #         activateAlarm('GREEN', devs_list)
        #         pass




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

        elif event == '-MCDMC_PUT_X-' or event == '-MCDMC_PUT_Y-':
            if values[event] == '':
                continue
            
            new_val = formFillProc(event, values, window, realNum = True, positiveNum = False, defaultValue = 0)
            continue

        elif event == '-MCDMC_PICK_AND_PUT-':
            if values["-MCDMC_PUT_X-"] == '' or values["-MCDMC_PUT_Y-"] == '':
                continue
            
            devs_list[d_index].get_device().robot_put_pos = mc.partPos(values["-MCDMC_PUT_X-"], values["-MCDMC_PUT_Y-"])
            
            print_log(f'Pick Up the part and put at X = {values["-MCDMC_PUT_X-"]}, Y = {values["-MCDMC_PUT_Y-"]}')
                
            wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.pick_n_put, \
                                             args=pm.argsType(x_coord = values["-MCDMC_PUT_X-"], y_coord = values["-MCDMC_PUT_Y-"])), \
                                             sType = pm.RunType.single)

        elif event == '-MCDMC_AUTOMATIC-':
            disableInd= True
            if window['-MCDMC_AUTOMATIC-'].get():
                disableInd =  True
            else:
                disableInd =  False


            window['-MCDMC_PUT_X-'].update(disabled = disableInd)
            window['-MCDMC_PUT_Y-'].update(disabled = disableInd)

        
        elif  event == '_MCDMC_SW_ON_' :               
            devs_list[d_index].get_device().setActivate()
            window[event].update(button_color='dark green on green')
            window[f'_MCDMC_SW_OFF_'].update(button_color='white on red')
            continue
        elif event == '_MCDMC_SW_OFF_' : 
            devs_list[d_index].get_device().setUnActive()
            window[event].update(button_color='tomato on red')
            window[f'_MCDMC_SW_ON_'].update(button_color='white on green')
            continue
      
        elif event == '_MCDMC_OPEN_ROBOT_CTL_':
            devs_list[d_index].get_device().openRobotURL()
            continue

        elif event == '_COGNEX_TRIGGER_':
            devs_list[d_index].get_device().triggerCognex()
            continue


        elif event == '-CGNX_DB_RESET_COUNTER-':
            devs_list[d_index].get_device().resetCounters()
            continue


        elif  '-CALIBRATE_CONTINUE-' in event:
            i = (int)(event[1])
            # calibration_cam = devs_list[d_index].get_device()
            # camRes(res=True, dist=dist, repQ=reportQ)
            calibrRes:camRes = values[f"-{i}-CALIBRATE_CONTINUE-"]
            dist = calibrRes.dist
            if not calibrRes.res:                               # Operation failed
                # emergency_stop_pressed = True
                print_err(f'Calibration error. Stoping.')
                sg.popup_error('Calibration failed.\n Vision Error', font=("Arial Bold", 60))
            else:
                wTask = proc_tuning(window, devs_list, devs_list[d_index].get_device().motor, dist, calibrRes.abs)
                # runTask(wTask, gui_task_list, process_manager, calibrRes.repQ)
                if wTask:
                    process_manager.load_task(wTask, calibrRes.repQ)
                else:                   # wTask = None
                    print_err(f'Error calibration tunning')
                    calibrRes.repQ.put(event2GUI(event='-TASK_ERROR-'))
                continue


        elif '-CALIBRATE_POSITION-' in event:
            i = (int)(event[1])
            # calibration_cam = devs_list[d_index].get_device()
            _repQ =  values[f"-{i}-CALIBRATE_POSITION-"]
            _postion = get_calibration_pos(devs_list, devs_list[d_index].get_device().motor)
            _repQ.put(_postion)
            continue  

        elif '-CALIBRATE_DONE-'  in event:
            # calibration_cam = devs_list[d_index].get_device()
            i = (int)(event[1])
            dist = values[f"-{i}-CALIBRATE_DONE-"]

###############################################

            if dist == sys.maxsize:
                # emergency_stop_pressed =  True
                print_err(f'Calibration error. Stoping.')
                sg.popup_error('Calibration failed.\n Vision Error', font=("Arial Bold", 60))
            #     pass
            # else:
            #     wTask = proc_tuning(window, devs_list, devs_list[d_index].get_device().motor, dist)
    ###############################################

            ActivateMotorControl(window, '--CAM--', i)
            calibration_in_process = False
            continue
        


        elif  '-CALIBRATE-'  in event:
            i = (int)(event[1])
            # calibration_cam = devs_list[d_index].get_device()
            if devs_list[d_index].get_device() is not None:
                if not values[f"-{i}-Z-SELECTOR-"] in devs_list[d_index].get_device().MOTORS:
                    print_log(f'Wrong CAM selection: {values[f"-{i}-Z-SELECTOR-"]}. Available CAMs are {devs_list[d_index].get_device().MOTORS}')
                    sg.popup_error(f'Wrong CAM selection: {values[f"-{i}-Z-SELECTOR-"]}. \nAvailable CAMs are {devs_list[d_index].get_device().MOTORS}', background_color = 'violet red')
                    continue

                print_log(f'Calibrate button pressed for {values[f"-{i}-Z-SELECTOR-"]} profile = {values[f"-{i}-CALIBR_PROFILE-"]}  ')
                DeActivateMotorControl(window, '--CAM--', i)
                SetLED (window, f'_calibr_{i}_', 'DeepPink2')
                window.Finalize()
                
                calibration_in_process = True

                if devs_list[d_index].get_device().cam_status():
                    wTask = pm.WorkingTask(pm.CmdObj(device=devs_list[d_index],cmd=pm.OpType.calibrate, \
                                                    args=pm.argsType(calibr_selector = values[f"-{i}-Z-SELECTOR-"], profile = values[f'-{i}-CALIBR_PROFILE-'])), \
                                                    sType = pm.RunType.single)
                else:
                    calibration_in_process = False

                # calibration_in_process = devs_list[d_index].get_device().start_cam_tuning_operation(values[f"-{i}-Z-SELECTOR-"], values[f'-{i}-CALIBR_PROFILE-'], window)

                
                if not calibration_in_process:
                    print_err(f'Calibration on {values[f"-{i}-Z-SELECTOR-"]} failed to start/no modbus client found')
                    ActivateMotorControl(window, '--CAM--', i)
                    sg.popup_auto_close('Calibration failed to start')
                    continue
                    
                print_log(f'calibration will be operated on ZABER Motor {int(values[f"-{i}-Z-SELECTOR-"][-1])}')
                
            else:
                print_err(f'No ModBus/Camera server connection established. Calibration could not be performed')
                
            
                
        elif  '-CALIBR_PROFILE-'  in event:
            i = (int)(event[1])
            # calibration_cam = devs_list[d_index].get_device()
            # if not file_name_validator(values[f'-{i}-CALIBR_PROFILE-']):    
            if not non_empty_string_validator(values[f'-{i}-CALIBR_PROFILE-']):    
                window[f'-{i}-CALIBR_PROFILE-'].update(values[f'-{i}-CALIBR_PROFILE-'][:-1])
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
        

    # Dispenser

        elif '-MARCO_UPDATE_TEMP-' in event:
            _def = devs_list[d_index].dev_marco.set_temp_
            new_val = formFillProc(event, values, window, realNum = False, positiveNum = True, defaultValue = _def)
            continue

        elif '-MARCO_SET_TERM-' in event:
            devs_list[d_index].dev_marco.set_temp(values['-MARCO_UPDATE_TEMP-'])
            continue

        elif '-MARCO_PROGRAM-' in event:
            devs_list[d_index].dev_marco.program_control(values['-MARCO_PROGRAM-'][-1])
            # MarcoGUIUpdate(window, devs_list[d_index])
            continue
        
        
        elif '-MARCO_PULSE_ON-' in event:
            _def = devs_list[d_index].dev_marco.pulse_on_
            new_val = formFillProc(event, values, window, realNum = False, positiveNum = True, defaultValue = _def)
            continue

        # elif '-MARCO_SET_PULSE_ON-' in event:
        #     # devs_list[d_index].dev_marco.set_pulse_on(values['-MARCO_PULSE_ON-'])
        #     devs_list[d_index].dev_marco.set_pulse_data(_on_time = values['-MARCO_PULSE_ON-'], _cycl_rate= , _pulse_count = )
        #     continue

        elif '-MARCO_PULSE_COUNT-' in event:
            _def = devs_list[d_index].dev_marco.pulse_count_
            new_val = formFillProc(event, values, window, realNum = False, positiveNum = True, defaultValue = _def)
            continue

        # elif '-MARCO_SET_PULSE_COUNT-' in event:
        #     devs_list[d_index].dev_marco.set_pulse_count(values['-MARCO_PULSE_COUNT-'])
        #     continue

        elif '-MARCO_PULSE_CYCLE-' in event:
            _def = devs_list[d_index].dev_marco.cycle_rate_
            new_val = formFillProc(event, values, window, realNum = False, positiveNum = True, defaultValue = _def)
            continue

        elif '-MARCO_SET_PULSE_DATA-' in event:
            devs_list[d_index].dev_marco.set_pulse_data(_on_time = values['-MARCO_PULSE_ON-'], _cycl_rate=  values['-MARCO_PULSE_CYCLE-'], _pulse_count =  values['-MARCO_PULSE_COUNT-'])
            continue



        elif '-MARCO_PURGE_TIME-' in event:
            _def = devs_list[d_index].dev_marco.get_purge_time_
            new_val = formFillProc(event, values, window, realNum = False, positiveNum = True, defaultValue = _def)
            continue

        elif '-MARCO_SET_PURGE_TIME-' in event:
            devs_list[d_index].dev_marco.set_purge_time(values['-MARCO_PURGE_TIME-'])
            continue


        elif '-MARCO_RESET_PULSE_COUNT-' in event:
            devs_list[d_index].dev_marco.reset_pulse_count()
            continue



        elif f'-MARCO_PURGE_ON-' in event:
            window[f'-MARCO_PURGE_ON-'].update(button_color='dark green on green')
            window[f'-MARCO_PURGE_OFF-'].update(button_color='white on red')
            devs_list[d_index].dev_marco.set_purge(1)
            continue
        
        elif f'-MARCO_PURGE_OFF-' in event:
            window[f'-MARCO_PURGE_OFF-'].update(button_color='tomato on red')
            window[f'-MARCO_PURGE_ON-'].update(button_color='white on green')
            devs_list[d_index].dev_marco.set_purge(0)
            continue

        
        elif f'-MARCO_SINGLE_SHOT_ON-' in event:
            window[f'-MARCO_SINGLE_SHOT_ON-'].update(button_color='dark green on green')
            window[f'-MARCO_SINGLE_SHOT_OFF-'].update(button_color='white on red')
            devs_list[d_index].dev_marco.single_shot(1)
            continue
        
        elif f'-MARCO_SINGLE_SHOT_OFF-' in event:
            window[f'-MARCO_SINGLE_SHOT_OFF-'].update(button_color='tomato on red')
            window[f'-MARCO_SINGLE_SHOT_ON-'].update(button_color='white on green')
            devs_list[d_index].dev_marco.single_shot(0)
            
            continue





    # GRIPPER
        elif '-GRIPPER-RPM-' in event:
            
            if devs_list[d_index].dev_mDC.mDev_type == '--GRIPPERv3--':
                if isinstance(devs_list[d_index].dev_mDC, MAXON_Motor):
                    update_val = str(devs_list[d_index].dev_mDC.DEAFULT_VELOCITY_EV_VOLTAGE)
                else:
                    update_val = str(devs_list[d_index].dev_mDC.DevOpSPEED)
            elif devs_list[d_index].dev_mDC.mDev_type == '--GRIPPER--': 
                update_val = str(devs_list[d_index].dev_mDC.DEAFULT_VELOCITY_EV_VOLTAGE)
            else:
                update_val = None

            new_val = formFillProc(event, values, window, realNum = False, positiveNum = True, defaultValue = update_val)

            
            if devs_list[d_index].dev_mDC.mDev_type == '--GRIPPERv3--':
                if isinstance(devs_list[d_index].dev_mDC, MAXON_Motor):
                    devs_list[d_index].dev_mDC.el_voltage = new_val
                else:
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
        elif m_dev.dev_interlock:  
            print_log('Stoping interlock thread')
            m_dev.dev_interlock.mDev_stop()
        elif m_dev.dev_iocontrol:  
            print_log('Stoping interlock thread')
            m_dev.dev_iocontrol.mDev_stop()

        del m_dev
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

_DEBUG = False

if __name__ == "__main__":
    if len(sys.argv) == 2:
        if sys.argv[1].strip().upper() == 'DEBUG':
            _DEBUG = True
            print_DEBUG = print_log

    print_log('Starting here')
    parms:dict =  read_params()
    layout = InitGUI(parms)
    window = StartGUI(layout)
    workingCycle(window,  parms)
    print_inf(f'Working cycle DONE')
    sys.exit()