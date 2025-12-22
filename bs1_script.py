#!/usr/bin/env python
__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "1.0.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Tool"

import PySimpleGUI as sg
import string, io
import yaml, sys, re
import datetime, time
from typing import List
import os


from bs2_config import  DevType, systemDevices
from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, void_f
# print_DEBUG = void_f


comment_color = 'blue'
main_color = 'white'
main_bg_collor = 'grey'
alt_color = 'white'
alt_bg_collor = 'green'
cmt_collor = 'blue'
cmt_bg_collor = 'orange'
sel_color = 'red'
sel_bg_collor = 'yellow'
legal_cmds_set = ['MA', 'MR', 'ON', 'OFF', 'HO', 'HALT', 'TRIGGER', 'DELAY', 'CALIBR', 'DISP', 'UV', 'FRONTLIGHT', 'BACKLIGHT', 'HOLELIGHT', 'DB']
legal_dev_set = ['T', 'Z', 'G', 'CMNT', 'SYS', 'SP', 'RO', 'D']


# data = [[["_"], ["_"],["__"],["_______"],["_"]]]    # cmd line in the table format
# headings = [['#'],['Dev'], ['Cmd'], ['Arg'], ['RPM'] ]  # colums titels

data = [[["_"], ["_"],["_______________________________"]]]    # cmd line in the table format
headings = [['#'],['Dev'], ['Cmd']]  # colums titels
#BUGBUGBUG

def validate_dev_cmd(dev:str, cmd:str, sysDevs: systemDevices) -> bool:
    from bs1_base_motor import Command
    try:
        _cmd:Command = Command.parse_cmd(f'{dev}.{cmd}')
    except Exception as ex:
        print_log (f' Invalid device/command - dev: {dev}, cmd: {cmd}. Exception: {ex}')
        return False

def validate_cmd(dev, cmd, sysDevs: systemDevices):
    parms:dict = sysDevs.getParams()
    _devList:dict = sysDevs.getConfDevs()  # get configured devices (in config file, not really appeared in the system)
                                 # validate command to be run
    print_log (f'dev = {dev}, cmd = {cmd}')
    try:

        re_dev = re.compile(r'\bS[1-3]\b|\bR[1-9]\b|\bT[1-3]\b|\bZ[1][0-6]\b|\bZ[1-9]\b|G[1-4]|D[1-2]|\bCMNT\b|\bMCDMC\b|\bSYS\b|\bNOP\b|\bPHG\b|\bDISP[1-9]\b|\bCAM[1-3]\b|\bDB\b')
        dev = dev.strip()
        if not re_dev.match(dev): 
            print_log (f'Invalid device - *{dev}*')
            return False
    
        elif 'CAM' == dev[0:3]:
            _cmd_re = r''
            _cmd_re += r'|(\bCALIBR\b\s+(\bZ[1][0-6]\b|\bZ[1-9]\b|\bR[1-9]\b|\bD[1-9]\s*/\s*D[1-9]\b)\s*(\s+\S+)(\s+(ABS|REL))?\s*$)'
            _cmd_re += r'|(\s*TERM\s*$)'
            _cmd_re += r'|(\s*CHECK\s+\w\s*$)'
            cmd_re = re.compile(_cmd_re)
            # cmd_re = re.compile(r'(\bCALIBR\b\s+(\bZ[1][0-6]\b|\bZ[1-9]\b|\bR[1-9]\b|\bD[1-9]\s*/\s*D[1-9]\b)\s*(\s+\S+)(\s+(ABS|REL))?\s*$)|(\s*TERM\s*$)')
            if not cmd_re.match(cmd):
                print_log (f' Invalid CAM ({dev} cmd - {cmd}')
                return False
            _cam_profiles = None
            # if not (cmd.split(' ')[0].strip() == 'CHECK' and parms[dev] is not None and \
            #         'PROFILES' in parms[dev].keys() and  \
            #         (_cam_profiles := parms[dev]['PROFILES']) is not None and\
            #         cmd.split(' ')[1].strip() in _cam_profiles):
            if cmd.split(' ')[0].strip() == 'CHECK':
                if not (parms[dev] is not None and \
                    'PROFILES' in parms[dev].keys() and  \
                    (_cam_profiles := parms[dev]['PROFILES']) is not None and\
                    cmd.split(' ')[1].strip() in _cam_profiles):

                    print_log (f" Invalid CAM ({dev} cmd - {cmd}. Wrong or not existing profile name ({cmd.split(' ')[1].strip()}). Profiles = {_cam_profiles}")
                    return False
                
                

        elif 'DISP' in dev :
            cmd_re = re.compile(r'(\s*((\bSET PROGRAM\b)\s+\b([0-9]|10)\b\s*)|(\bSINGLE_SHOT\b)$)')   # 
            if not cmd_re.match(cmd):
                print_log (f', Invalid MARCO({dev}) command - "{cmd}"')
                return False  

        elif dev[0] == 'T':
            cmd_re = re.compile(r'(\s*(\bMR\b|\bML\b|\bMLSTALL\b|\bMLSTALL\b)\s+\b[0-9][0-9]{0,8}\b(\s+\b[0-9][0-9]{0,3}\b\s*)?$)|\bHO\b|\bREL\b|\bSTALL\b')
            if not cmd_re.match(cmd):
                print_log (f', Invalid T ({dev}) com - "{cmd}"')
                return False
            

        elif dev[0] == 'Z':
            # cmd_re = re.compile(r'\s*(\bMA\b\s+\d+(\.\d+)?(\s+\b([1-9]|[1-9][0-9]|100)\b%\s*)*|\bMR\b\s+-?\d+(\.\d+)?(\s+\b([1-9]|[1-9][0-9]|100)\b%\s*)*)|\bRND\b\s+\d+(\.\d+)?\s+\d+(\.\d+)?\s*|\bHO\b$')
            cmd_re = re.compile(r'\s*(MA\s+\d+(\.\d+)?(\s+([1-9]|[1-9][0-9]|100)%\s*)?|MR\s+-?\d+(\.\d+)?(\s+([1-9]|[1-9][0-9]|100)%\s*)?|\bRND\b\s+\d+(\.\d+)?\s+\d+(\.\d+)?\s*|\bHO\b$\s*|\bVIBRATE\b$\s*)$')
            if not cmd_re.match(cmd):
                print_log (f', Invalid Z ({dev}) com - "{cmd}"')
                return False

        elif dev[0] == 'G':
            cmd_re = re.compile(r'\s*(\bTrue\b|\bFalse\b)$')
            if not cmd_re.match(cmd):
                print_log (f', Invalid G ({dev}) com - "{cmd}"')
                return False
                
        elif dev == 'DB':
            cmd_re = re.compile(r'\s*ADD\s+(\bTrue\b|\bFalse\b)\s*$')
            if not cmd_re.match(cmd):
                print_log (f', Invalid DB ({dev}) cmd: "{cmd}"')
                return False
            
        elif dev[0] == 'D':
            # cmd_re = re.compile(r'\s*(\bTrue\b|\bFalse\b)|(\s*(\bMA\b)\s+-?[0-9][0-9]{0,8}(\s+\b[0-9][0-9]{0,5}\b\s*)?$)|\bHO\b$')
            cmd_re = re.compile(r'\s*(\bTrue\b|\bFalse\b)|(\s*(\bMA|MR\b)\s+-?[0-9][0-9]{0,8}(\s+\b[0-9][0-9]{0,5}\b\s*)?$)|\bHO\b$')
            if not cmd_re.match(cmd):
                print_log (f', Invalid DH ({dev}) com - "{cmd}"')
                return False
                
        elif dev == 'CMNT':
            cmd_re = re.compile(r'.*')
            if not cmd_re.match(cmd):
                print_log (f', Invalid Comment line - {cmd}')
                return False 
        
        
                

        elif dev == 'SYS':
            # cmd_re = re.compile(r'\s*(\bHALT\b|\bHOTAIR\b(\s+[0,1])\s*$|\bCALIBR\b\s+(Z(6|11)|R[1,2])\s*(\s+[a-zA-Z0-9-_]+\.{0,1}[A-Za-z]{0,4})\s*$|\bTRIGGER\b(\s+[0-3])?|\bDELAY\b\s+\d+(\.\d+){0,1})|\bDISP\b|\bUV\b|\bFRONTLIGHT\b|\bBACKLIGHT\b$')
            # cmd_re = re.compile(r'\s*(\bHALT\b|\bHOTAIR\b(\s+[0,1])\s*$|\bCALIBR\b\s+(Z(6|11)|R[1,2])\s*(\s+\S+)\s*$|\bTRIGGER\b(\s+[0-3])?|\bDELAY\b\s+\d+(\.\d+){0,1})|\bDISP\b|\bUV\b|\bFRONTLIGHT\s*(\s+(ON|OFF)){0,1}\b|\bBACKLIGHT\s*(\s+(ON|OFF)){0,1}\b$')
            # cmd_re = re.compile(r'\s*(\bHALT\b|\bHOTAIR\b(\s+[0,1])\s*$|\bCALIBR\b\s+(Z(6|11)|R[1,2])\s*(\s+\S+)\s*$|\bTRIGGER\b(\s+[0-3])?|\bDELAY\b\s+\d+(\.\d+){0,1})|\bDISP\b|\bUV\b|\b(FRONTLIGHT|BACKLIGHT)\s*(\s+(ON|OFF)){0,1}\b$')
            # cmd_re = re.compile(r'\s*(\bHALT\b|\bHOTAIR\b(\s+[0,1])\s*$|\bCALIBR\b\s+(\bZ[1][0-6]\b|\bZ[1-9]\b|\bR[1-9]\b|\bD[1-9]\s*/\s*D[1-9]\b)\s*(\s+\S+)\s*$|\bTRIGGER\b(\s+[0-3])?|\bDELAY\b\s+\d+(\.\d+){0,1})|\bDISP\b|\bUV\b|\b(FRONTLIGHT|BACKLIGHT)\s*(\s+(ON|OFF)){0,1}\b$')
            # cmd_re = re.compile(r'PLAY\s+([a-zA-Z]:\\)?((?:[^<>:"/\\|?*]+\\)*)([\w\s-]+)\.[\w]+$|\s*(\bHALT\b|\bHOTAIR\b(\s+[0,1])\s*$|\bCALIBR\b\s+(\bZ[1][0-6]\b|\bZ[1-9]\b|\bR[1-9]\b|\bD[1-9]\s*/\s*D[1-9]\b)\s*(\s+\S+)\s*$|\bTRIGGER\b(\s+[0-3])?|\bDELAY\b\s+\d+(\.\d+){0,1})|\bDISP\b|\bUV\b|\b(FRONTLIGHT|BACKLIGHT)\s*(\s+(ON|OFF)){0,1}\b$')
            # cmd_re = re.compile(r'PLAY\s+([a-zA-Z]:\\)?((?:[^<>:"/\\|?*]+\\)*)([\w\s-]+)\.mp3$|\s*(\bHALT\b|\bHOTAIR\b(\s+[0,1])\s*$|\bCALIBR\b\s+(\bZ[1][0-6]\b|\bZ[1-9]\b|\bR[1-9]\b|\bD[1-9]\s*/\s*D[1-9]\b)\s*(\s+\S+)\s*$|\bTRIGGER\b(\s+[0-3])?|\bDELAY\b\s+\d+(\.\d+){0,1})|\bDISP\b|\bUV\b|\b(FRONTLIGHT|BACKLIGHT|HOLELIGHT)\s*(\s+(ON|OFF)){0,1}\b$')
            # cmd_re = re.compile(r'PLAY\s+([a-zA-Z]:\\)?((?:[^<>:"/\\|?*]+\\)*)([\w\s-]+)\.mp3$|\s*(\bHALT\b|\bHOTAIR\b(\s+[0,1])\s*$|\bCALIBR\b\s+(\bZ[1][0-6]\b|\bZ[1-9]\b|\bR[1-9]\b|\bD[1-9]\s*/\s*D[1-9]\b)\s*(\s+\S+)(\s+(ABS|REL))?\s*$|\bTRIGGER\b(\s+[0-3])?|\bDELAY\b\s+\d+(\.\d+){0,1})|\bDISP\b|\bUV\b|\b(FRONTLIGHT|BACKLIGHT|HOLELIGHT)\s*(\s+(ON|OFF)){0,1}\b$')
            # cmd_re = re.compile(r'PLAY\s+([a-zA-Z]:\\)?((?:[^<>:"/\\|?*]+\\)*)([\w\s-]+)\.mp3$|\s*(\bHALT\b|\bCALIBR\b\s+(\bZ[1][0-6]\b|\bZ[1-9]\b|\bR[1-9]\b|\bD[1-9]\s*/\s*D[1-9]\b)\s*(\s+\S+)(\s+(ABS|REL))?\s*$|\bDELAY\b\s+\d+(\.\d+){0,1})$')
            cmd_re = re.compile(r'PLAY\s+([a-zA-Z]:\\)?((?:[^<>:"/\\|?*]+\\)*)([\w\s-]+)\.mp3$|\s*(\bHALT\b|\bDELAY\b\s+\d+(\.\d+){0,1})$')
            
            # _cmd = cmd.split()
            # if _cmd[0] == 'CALIBR':
            #     if not _cmd[1] in MOTORS:
            #         print_log (f'Undefined motor device {_cmd[1]} in SYSTEM cmd - {cmd}')
            #         return False

            
            if not cmd_re.match(cmd):
                print_log (f' Invalid SYSTEM cmd - {cmd}')
                return False
            
        elif dev =='PHG':  
            cmd_re = re.compile(r'\w+\s*(\s+(\b(?:ON|OFF)\b)){0,1}\b$')

            if not cmd_re.match(cmd):
                print_err (f' Invalid PHG cmd - {cmd}')
                return False
            
            _phg_devs_list = _devList[DevType.PHG.value]
            _phg_dev = cmd.split(' ')[0].strip()

            if not _phg_dev in _phg_devs_list.keys():
                print_err (f'ERROR Invalid PHG device - {_phg_dev} is not listed in {_phg_devs_list}. Cmd - {cmd}')
                return False
            else:
                print_log (f' Cmd - {cmd}. PHG device - {_phg_dev} , list - {_phg_devs_list.keys()}. ')

        elif dev[:4] =='PHG_':  
            cmd_re = re.compile(r'\w+\s*(\s+(\b(?:ON|OFF)\b)){0,1}\b$')
            cmd = dev[4:] + ' ' + cmd
            if not cmd_re.match(cmd):
                print_err (f' Invalid PHG_ cmd - {cmd}')
                return False
            
            _phg_devs_list = _devList[DevType.PHG.value]
            _phg_dev = cmd.split(' ')[0].strip()

            if not _phg_dev in _phg_devs_list.keys():
                print_err (f'ERROR Invalid PHG device - {_phg_dev} is not listed in {_phg_devs_list}. Cmd - {cmd}')
                return False
            else:
                print_log (f' Cmd - {cmd}. PHG device - {_phg_dev} , list - {_phg_devs_list.keys()}. ')

        elif dev[0] == 'R':
            # cmd_re = re.compile(r'(\s*(\bMR\b|\bML\b)\s+\b[0-9][0-9]{0,8}\b(\s+\b[0-9][0-9]{0,5}\b\s*)?$)|(\s*(\bMA\b)\s+\b-?[0-9][0-9]{0,8}\b(\s+\b[0-9][0-9]{0,5}\b\s*)?$)|\bHO\b|\bREL\b')
            # cmd_re = re.compile(r'(\s*(\bMRC\b|\bMLC\b)(\s+\b[0-9][0-9]{0,5}\b\s*)?$)|(\s*(\bMR\b|\bML\b)\s+\b[0-9][0-9]{0,8}\b(\s+\b[0-9][0-9]{0,5}\b\s*)?$)|(\s*(\bMA\b)\s+\b-?[0-9][0-9]{0,8}\b(\s+\b[0-9][0-9]{0,5}\b\s*)?$)|\bHO\b|\bREL\b')
            # cmd_re = re.compile(r'(\s*(\bMRC\b|\bMLC\b)(\s+\b[0-9][0-9]{0,8}\b\s+\b[0-9][0-9]{0,5}\b\s*)?$)|(\s*(\bMRC\b|\bMLC\b)(\s+\b[0-9][0-9]{0,8}\b\s*)?$)|(\s*(\bMR\b|\bML\b)\s+\b[0-9][0-9]{0,8}\b(\s+\b[0-9][0-9]{0,5}\b\s*)?$)|(\s*(\bMA\b)\s+\b-?[0-9][0-9]{0,8}\b(\s+\b[0-9][0-9]{0,5}\b\s*)?$)|\bHO\b|\bREL\b')
            # cmd_re = re.compile(r'(\s*(\bMRC\b|\bMLC\b)(\s+\b[0-9][0-9]{0,8}\b\s+\b[0-9][0-9]{0,5}\b\s*)?$)|(\s*(\bMRC\b|\bMLC\b)(\s+\b[0-9][0-9]{0,8}\b\s*)?$)|(\s*(\bMR\b|\bML\b)\s+\b[0-9][0-9]{0,8}\b(\s+\b[0-9][0-9]{0,5}\b\s*)?$)|(\s*(\bMA\b)\s+-?\b[0-9][0-9]{0,8}\b(\s+\b[0-9][0-9]{0,5}\b\s*)?$)|\bHO\b|\bREL\b')
            cmd_re = re.compile(r'(\s*(\bMRC\b|\bMLC\b)(\s+\b[0-9][0-9]{0,8}\b\s+\b[0-9][0-9]{0,5}\b(\s+\b(HIGH|LOW)\b){0,1}\s*)?$)|(\s*(\bMR\b|\bML\b)\s+\b[0-9][0-9]{0,8}\b(\s+\b[0-9][0-9]{0,5}\b\s*)?$)|(\s*(\bMA\b)\s+-?\b[0-9][0-9]{0,8}\b(\s+\b[0-9][0-9]{0,5}\b\s*)?$)|\bHO\b|\bREL\b')

            if not cmd_re.match(cmd):
                print_log (f', Invalid R({dev}) command - "{cmd}"')

                return False
            
        elif dev[0] == 'S':
            cmd_re = re.compile(r'(\s*(\bMR\b|\bML\b)\s+\b[0-9][0-9]{0,8}\b(\s+\b[0-9][0-9]{0,3}\b\s*)?$)')
            if not cmd_re.match(cmd):
                print_log (f', Invalid S({dev}) command - "{cmd}"')
                return False         
            
        elif dev == 'MCDMC':

            # cmd_re = re.compile(r'(\s*(\bPUT|PUTPCB\b)\s+-?\d+(\.\d+)?\s+-?\d+(\.\d+)?\s*$)|(\s*\bINSERTPCB\b\s+-?\d+(\.\d+)?\s+-?\d+(\.\d+)?\s+-?\d+(\.\d+)?\s*$)')   # PUT/INSERT 52.45097 -159.15698

            # cmd_re = re.compile(r'\
            #                     (\s*\bMOVEABS\b\s+\w+(\s+[1-9]|[1-9][0-9]|100)?\s*$) |\
            #                     (\s*(\bMOVESAFE|PUT\b)\s+\w+(\s+[1-9]|[1-9][0-9]|100)?\s*$) |\
            #                     (\s*\bMOVEREL\b(\s+-?\d+(\.\d+))(\s+-?\d+(\.\d+))(\s+-?\d+(\.\d+))(\s+-?\d+(\.\d+))(\s+[1-9]|[1-9][0-9]|100)?\s*$) |\
            #                     (\s*\bMOVEUP\b\s*$) |\
            #                     (\s*\bVACUUM\b\s+(ON|OFF)\s*$) |\
            #                     (\s*\bHOME\b\s*$) |\
            #                     (\s*\bPICKUP\b\s+(ASYRIL|COGNEX|w+)\s*$) \
            #                     ')
            __mcdm_re = ''
            __mcdm_re += r'|(\s*\bMOVEABS\b\s+\w+(\s+(?:[1-9]|[1-9][0-9]|100))?\s*$)'
            __mcdm_re += r'|(\s*(\bMOVESAFE\b)\s+\w+(\s+(?:[1-9]|[1-9][0-9]|100))?\s*$)'
            # __mcdm_re += r'|(\s*\bMOVEREL\b\s+((-?\d+(\.\d+)?\s+-?\d+(\.\d+)?\s+-?\d+(\.\d+)?\s+-?\d+(\.\d+)?)|(\w+))(\s+(?:[1-9]|[1-9][0-9]|100))?\s*$)'
            __mcdm_re += r'|(\s*\bMOVEREL\b((\s+(\b(-?\d+(\.\d+)?)\b\s*){4})|(\w+))(\s+(?:[1-9]|[1-9][0-9]|100))?\s*$)'
            __mcdm_re += r'|(\s*\bMOVEUP\b\s*$)'
            __mcdm_re += r'|(\s*\bVACUUM\b\s+(ON|(OFF(\s+\d+(\.\d+)?\s*)?)\s*$))'
            # __mcdm_re += r'|(\s*\bHOME\b\s*$)'
            __mcdm_re += r'|(\s*\bPICKUP\b\s+(ASYRIL|COGNEX|\w+)\s*$)'
            # __mcdm_re += r'|(\s*(\bMOVESAFE|PUT\b)\s+\w+(\s+[1-9]|[1-9][0-9]|100)?\s*$)'


    ################### For back compatability ############################
            __mcdm_re += r'|(\s*(\bPUT|PUTPCB\b)\s+(-?\d+(\.\d+)?\s+-?\d+(\.\d+)?)|(\w+)\s*$)'
            __mcdm_re += r'|(\s*\bINSERTPCB\b\s+-?\d+(\.\d+)?\s+-?\d+(\.\d+)?\s+-?\d+(\.\d+)?\s*$)'


            cmd_re = re.compile(__mcdm_re)
            if not cmd_re.match(cmd):
                print_log (f', Invalid MCDMC({dev}) command - "{cmd}"')
                return False  
            
            _put = r'\s*(\bPUT|PUTPCB\b)\s+-?\d+(\.\d+)?\s+-?\d+(\.\d+)?\s*$'
            _put_re = re.compile(_put)
            __moverel = r'(\s*\bMOVEREL\b\s+(-?\d+(\.\d+)?\s+-?\d+(\.\d+)?\s+-?\d+(\.\d+)?\s+-?\d+(\.\d+)?)(\s+(?:[1-9]|[1-9][0-9]|100))?\s*$)'
            __moverel_re = re.compile(__moverel)

            
            if parms['MCDMC'] is not None:
                if '3DPOSITIONS' in parms['MCDMC'].keys() and (_coord3D := parms['MCDMC']['3DPOSITIONS']) is not None:
                # _coord3D = parms['MCDMC']['3DPOSITIONS']
                # if _coord3D is not None:
                    _subcmd = cmd.split(' ')[0].strip()
                    print_log(f'cmd = {cmd}, subcmd = {_subcmd}')
                    if  (_subcmd == 'MOVEABS' or _subcmd == 'MOVESAFE') and not cmd.split(' ')[1].strip() in _coord3D.keys():
                        print_err (f"ERROR: Invalid POSITION in MCDMC MOVEABS/MOVESAFE/PUT  - {cmd.split(' ')[1]} is not listed in {_coord3D}. Cmd = {cmd}")
                        return False
                    elif  _subcmd == 'PICKUP' and not (cmd.split(' ')[1].strip() in _coord3D.keys() or cmd.split(' ')[1].strip() == 'ASYRIL' or cmd.split(' ')[1].strip() == 'COGNEX') :
                        print_err (f"ERROR: Invalid POSITION in PICKUP  - {cmd.split(' ')[1]} is not listed in {_coord3D}. Cmd = {cmd}")
                        return False
                    elif (_subcmd == 'PUTPCB' or _subcmd == 'PUT') and not cmd.split(' ')[1].strip() in _coord3D.keys() and not _put_re.match(cmd):
                        print_err (f"ERROR: Invalid POSITION in PUTPCB/PUT  - {cmd.split(' ')[1]} is not listed in {_coord3D}. Cmd = {cmd}")
                        return False
                    else:
                        print_log (f" Cmd - {cmd}. Subcmd - {cmd.split(' ')[0]} , list - {_coord3D.keys()}. ")

                elif '3DOFFSET' in parms['MCDMC'].keys() and (_offset3D := parms['MCDMC']['3DOFFSET']) is not None:
                    if (_subcmd == 'NOVEREL') and not cmd.split(' ')[1].strip() in _offset3D.keys() and not __moverel_re.match(cmd):
                        print_err (f"ERROR: Invalid OFFSET POSITION in NOVEREL  - {cmd.split(' ')[1]} is not listed in {_offset3D}. Cmd = {cmd}")
                        return False

        

        elif dev == 'NOP':
            pass
        else:
            print_log (f' Inrecognized command line - {cmd}')
            return False

    except Exception as ex:
        e_type, e_filename, e_line_number, e_message = exptTrace(ex)
        return False

    return True

new_colors:List = list()                         # Colors per line (in fact one of two main and alt colors for adjacent command group)
new_script:List = list()                         # The loaded  script in text format will be dispalyes in the table (using pweudo graphic)
LoadedScript:dict = dict()                       # The script in dictionary format (as it loaded from YAML file)
groups= list()                                   # list of lists of commands's group 

def collectDevices(vScript):
    colectD:set= set()
    for _key, _val in vScript.items():
        if  isinstance(_val, dict):
            colectD.update(collectDevices(_val))
        else:
            colectD.add(_key)

    return colectD


def scriptParallelValidator(vScript:dict, op = 'S'):
    parSet:List= list()

    try:
        for _key, _val in vScript.items():
            if  isinstance(_val, dict): 
                scriptParallelValidator(_val, str(_key)[-1])
                if op == 'P':
                    parSet.append(collectDevices(_val))
                
            else:
                if op == 'P':
                    tmpSet = set()
                    tmpSet.add(_key)
                    parSet.append(tmpSet)

    except Exception as ex:
        exptTrace(ex)
        raise(ex)

    tmpSet = set()
    
    for _set in parSet:
        _set.discard('SYS')                      # exlude SYS
        _set.discard('PHG')                      # exlude PHG relay
        _set.discard('CAM1')                      # exlude CAM1 
        _set.discard('CAM2')                      # exlude CAM2
        if not tmpSet.isdisjoint(_set):
            raise Exception (f'The same devices {list(tmpSet.intersection(_set))} in paralel proceeding')
        tmpSet.update(_set)
    
    return parSet

def scriptValidator(vScript:dict)->bool:
    try:
        numSet:set = set()
        for _key, _val in vScript.items():
            if isinstance(_val, dict):
                print_log(f'key = {str(_key)}, val = {_val}')
                if str(_key)[:-1] in numSet:
                    raise Exception (f'The same OP number appears more than once: {str(_key)[:-1]}')
                else:
                    print_DEBUG(f'Adding new key {str(_key)[:-1]} <{_val}> to the set {numSet}')
                    numSet.add(str(_key)[:-1])
                
                scriptValidator(_val)
            else:
                pass
    except Exception as ex:
        # print_log(f'Exception -> {ex} of type {type(ex)}')
        exptTrace(ex=ex)
        raise(ex)
    
    return True



# def load_script(filename):
#     global LoadedScript

#     new_script.clear()
#     new_colors.clear()
#     groups.clear()

#     group_n:int = 0
#     group_index = 0
    
#     try:
#         with open(filename) as script_file:

#             doc = yaml.safe_load(script_file)

#             print_log(f'Script doc = {doc}\n\n')
#             LoadedScript = doc

#             line = 0
#             group_index = 0
#             for group_n, commands in doc.items():       # command is a dictionary 
#                 group = []
#                 _shift = 0
#                 print_log(f'group_n={group_n} -> commands = {commands}')
#                 for _count, device in enumerate(commands):
#                     row = list()
#                     if _count == 0:
#                         row.append(str(group_n))
#                     elif _count > 0:
#                         if _count == (len(commands) - 1):           # last command in group
#                             row.append(f'╚')
#                         else:
#                             row.append(f'╠')

#                     # row.append(group_n)

#                     row.append(device)
#                     single_cmd = commands[device]

#                     if not validate_cmd(device, single_cmd if type(single_cmd) == str else str(single_cmd)):
#                         raise Exception("Script error")
                    
#                     if isinstance(single_cmd, str):
#                         com_arg = re.split('\s+', single_cmd)
#                         for args in com_arg:

#                             row.append(args)
#                     else:
#                         row.append(commands[device])

#                     print_log (f'\n---- {row} type of row = {type(row)} ---')

#                     new_script.append(row)
#                     group.append(line)

#                     if group_index % 2:
#                         row_color = (line, main_color, main_bg_collor)
#                     else:
#                         row_color = (line, alt_color, alt_bg_collor)
#                     if device == 'CMNT':
#                         row_color = (line, cmt_collor, cmt_bg_collor)
#                     new_colors.append(row_color)
#                     line += 1   
#                 group_index += 1
#                 groups.append(group)
#             print_log(f'new_script = {new_script}')
#             print_log(f'new_colors = {new_colors}')
#             print_log(f'groups = {groups}')
#     except Exception as ex:
#         # print_log(f'Exception -> {ex} of type {type(ex)}')
#         exptTrace(ex=ex)
#         sg.popup(f'Wrong command format in group: {group_n} [serial: {group_index+1}] \n {ex}', keep_on_top=True)
#         # sys.exit()
#         return False
#     else:
#         pass

#     Tbl.update(values = new_script, row_colors = new_colors, select_rows=groups[0])
#     Tbl.SetFocus(force = True )
#     return True

def scriptFormater(subScript) -> dict:
    new_dict = dict()
    for _key, _val in subScript.items():
        if isinstance(_val, dict):
            tmpDict = dict(NOP = 'No operation')
            tmpDict.update(_val)
            print_log(f'new_dict = {tmpDict}')
            subDict = scriptFormater(tmpDict)

            # subDict = scriptFormater(_val)
            new_dict[_key] = subDict
            print_log(f'*{_key} -> {subDict}')
        else:
            new_dict[_key] = _val
            print_log(f' {_key} -> {_val}')

    return new_dict 


def ymlPars(CmdScript:dict, sysDevs:systemDevices, group:List=None, line:int = 0, group_index:int = 0, cmdStr:str = str()):

    global new_colors, new_script, groups
    try:

        # if group == None:
        #     topLevel = True
        # else:
        #     topLevel = False
            
        # group_index = 0

        for group_n, commands in CmdScript.items():       # command is a dictionary 
            
            if group == None:
                group = list()

            print_DEBUG(f'group_n={group_n} -> commands = {commands}')
            for _count, device in enumerate(commands):

                row = list()
                delimiter = '.' if isinstance(commands[device], dict) else ''

                print_DEBUG(f'group = {group}, group_n={group_n}, group_index = {group_index},, cmdStr = {cmdStr}, _count = {_count}, device = {device}, commands = {commands}, commands[{device}]= {commands[device]}')
                leadColumn = cmdStr
                group_re = re.compile(r'(\b[1-9][0-9]*(P|S)\b)|(\bPROC_[a-zA-Z0-9]+(P|S)\b)')
                if not group_re.match(group_n):
                    raise Exception(f"Script error. Invalid group [{group_n}] format ")
                if _count == 0:
                    leadColumn  = leadColumn + (str(group_n[:-1])) + delimiter
                elif _count > 0:
                    con_char = "╚" if group_n[-1] == "P" else "└"
                    if _count == (len(commands) - 1):           # last command in group
                        leadColumn  = (f'{len(leadColumn)*" "}{"╚" if group_n[-1] == "P" else "└"}')
                    else:
                        leadColumn  = (f'{len(leadColumn)*" "}{"╠" if group_n[-1] == "P" else "├"}')


                # row = list()
                row.append(leadColumn)


                # print_DEBUG(f'row = {row}, leadColumn= {leadColumn}, _count = {_count}, device = {device}, commands[device] = {commands[device]}, type(commands[device])= {type(commands[device])}')

                if isinstance(commands[device], dict):
                    print_DEBUG(f'Going next level with {dict([(device, commands[device])])} ')
                    ymlPars(CmdScript=dict([(device, commands[device])]), sysDevs=sysDevs, group=group, line=line, \
                            group_index=group_index, cmdStr=leadColumn)
                    continue
                
                row.append(device)
                single_cmd = commands[device]

                if not validate_cmd(device, single_cmd if type(single_cmd) == str else str(single_cmd), sysDevs):
                    raise Exception(f'Script error. Group = {group_n}, Device= {device} ')
                
                if isinstance(single_cmd, str):
                    com_arg = re.split('\s+', single_cmd)
                    for args in com_arg:

                        row.append(args)
                else:
                    row.append(commands[device])

                print_log (f'---- {row} type of row = {type(row)} ---')



#BUGBUGBUG      # testing mode!
                strRow = ''.join(str(x)+' ' for x in row[2:])
                rw = row[0]
                # dv = row[1]
                dv = ' ' if row[1] == 'NOP' else row[1]
                strRow = ' ' if row[1] == 'NOP' else strRow
                row = list()
                row.append(rw)
                row.append(dv)
                row.append(strRow)
                
#BUGBUGBUG

        

                new_script.append(row)

            #     group.append(line)

            #     print_DEBUG(f'--line = {line}, group_index = {group_index}, row = {row}')

            #     if group_index % 2:
            #         row_color = (line, main_color, main_bg_collor)
            #     else:
            #         row_color = (line, alt_color, alt_bg_collor)
            #     if device == 'CMNT':
            #         row_color = (line, cmt_collor, cmt_bg_collor)
            #     new_colors.append(row_color)

            #     print_DEBUG(f'**line = {line}, group_index = {group_index}, row = {row}, row_color = {row_color}')

            #     line += 1   

            # groups.append(group)
            # print_DEBUG(f'group = {group}, groups = {groups}, group_index = {group_index}, group_n = {group_n}, new_colors={new_colors}')
            # group = None
            # if topLevel:
            #     group_index += 1

    except Exception as ex:
        exptTrace(ex=ex)
        raise ex


# Note the variable in Python are references 
def countLines(CmdScript:dict, lines = 0):
    for group_n, commands in CmdScript.items():
        if isinstance(commands, dict):
            print_DEBUG(f'Going next level with lines = {lines},  commands = {commands}')
            lines = countLines(CmdScript=commands, lines=lines)
            print_DEBUG(f'Sublevel counted {lines} lines ')
            continue
        print_DEBUG(f' group_n = {group_n} Counting line {lines}: {commands}')
        lines += 1
    return lines

def buildScriptTable(CmdScript:dict):
    global new_colors, new_script, groups

    groupIndex = 0
    LineIndex = 0
    for group_n, commands in CmdScript.items():       # command is a dictionary 
        lines = countLines(CmdScript=commands)
        print_DEBUG(f'Lines in group = {group_n} -> {lines}')
        tmp_group = list()
        for _count in range (lines):
            tmp_group.append(LineIndex)
            
            if groupIndex % 2:
                row_color = (LineIndex, main_color, main_bg_collor)
            else:
                row_color = (LineIndex, alt_color, alt_bg_collor)
            # if device == 'CMNT':
            #     row_color = (LineIndex, cmt_collor, cmt_bg_collor)
            LineIndex += 1
            new_colors.append(row_color)
            print_DEBUG(f'group_n = {group_n}, line =  {LineIndex},  groupIndex = {groupIndex} -> row_color = {row_color}')
        groups.append(tmp_group)
        groupIndex += 1

        print_DEBUG(f'Added group {group_n} # {groupIndex}')


    pass

def load_embeded_script(filename, sysDevs:systemDevices)->bool:
    global LoadedScript
    global new_colors, new_script, groups

    new_script.clear()          # reset script list
    new_colors.clear()          # reset colors list
    groups.clear()

    group_n:int = 0
    group_index = 0
    
    try:
        with open(filename) as script_file:

            doc = yaml.safe_load(script_file)

            print_log(f'Script doc = {doc}\n\n')

            scriptValidator(doc)

            scriptParallelValidator(doc)

            LoadedScript = scriptFormater(doc)

            print_DEBUG(f'LoadedScript == doc = {LoadedScript == doc}')

            ymlPars(LoadedScript, sysDevs=sysDevs)

            buildScriptTable(LoadedScript)
 
            print_log(f'new_script = {new_script}')
            print_log(f'new_colors = {new_colors}')
            print_log(f'groups = {groups}')
    except Exception as ex:
        # print_log(f'Exception -> {ex} of type {type(ex)}')
        exptTrace(ex=ex)
        sg.popup(f'Wrong command format \n {ex}', keep_on_top=True)
        # sys.exit()
        return False
    else:
        pass

    Tbl.update(values = new_script, row_colors = new_colors, select_rows=groups[0])
    Tbl.SetFocus(force = True )
    return True


def get_subscript(script, sub_script_lines)->List[str]:
    devs = set()
    subscript:List[str] = list()
    for _count, line in enumerate(sub_script_lines):
        print_DEBUG(f'count = {_count}, line = {line}')
        cmd = script[line]
        if cmd[1] in devs:              # chek for devs duplication
            print_err(f'-WARNING -Duplicated dev {cmd[1]} in {cmd}')
            continue        
        subscript.append(cmd)
        devs.add(cmd[1])
        print_log(f'adding  command = {cmd}')
        
    return subscript


def next_group(row):

    if row+1 >= len(new_script):
        return groups[0]

    for gr in groups:
        if (int)(gr[0]) > row:
            return gr
    return groups[0]    

# def Tbl_event_proc(window, event, values):
#     # global Tbl
#     subscript =[]
    

#     if event == 'Step' or event == '-TABLE-':
#         if groups == []:
#             return []
#         print_log (f'values[-TABLE-] = {values["-TABLE-"]}, type = {type(values["-TABLE-"])}')
#         subscript = get_subscript(new_script, values["-TABLE-"])
#         row = values['-TABLE-'][0] 
#         gr = next_group(row)
#         print_DEBUG(f'--BUGBUG-- subscript = {subscript}, current row = {row}, next group = {gr}')

#         if values['-TABLE-'][-1] + 1 >= len (new_script):
#             window['-TABLE-'].set_vscroll_position(0)
#             print_log(f'Moving scrool up')



#         if values['-TABLE-'][-1]  >= rows_in_table/2 and values['-TABLE-'][-1]  <= len(new_script) - rows_in_table/2:
#             diff = (values['-TABLE-'][-1]  - rows_in_table/2)/(len(new_script) )
#             print_log(f'ref row= {values["-TABLE-"][-1]  -  rows_in_table/2}, row_window = {len(new_script)}')
#             window['-TABLE-'].set_vscroll_position(diff)
#             print_log(f'set_vscroll_position({diff})')       
            

#             window.refresh()

#         window['-TABLE-'].update(select_rows = gr)
#         # gv = window['-TABLE-'].get()
       
            
            

#     elif event == "Open File":
#         print_log("[LOG] Clicked Open File!")
#         folder_or_file = sg.popup_get_file('Choose your file', keep_on_top=True, 
#                                 file_types = (('YAML Files', '*.yml'),('YAML Files', '*.yaml'),))
#         print_log("[LOG] User chose file: " + str(folder_or_file))
#         if folder_or_file:
#             load_script(folder_or_file)

#     elif event == 'Clear Log':
#         yesno = sg.popup_yes_no("You're about to delete all log data.\nDo you want to proceed",  title="YesNo")
#         if yesno == 'Yes':
#             Log_tbl.update(value='')

#     elif event == "Save Log":
#         folder_or_file = sg.popup_get_file('Select file to save log', keep_on_top=True, save_as='True')
#         if folder_or_file:
#             print(f'Writing to {str(folder_or_file)}')
#             with io.open(folder_or_file, "w", encoding="utf8") as log_f:
#                 log_f.write(Log_tbl.get())
#             log_f.close()


#     return subscript

def GetSubScriptDict(window, event, values, ind = 0)->dict:
    subscript:dict = dict()

    if groups == []:
        return []
    

    print_log (f'event =  {event}')
    print_DEBUG (f'values[-TABLE-] = {values["-TABLE-"]}, type = {type(values["-TABLE-"])}, ind = {ind}')
    # print_DEBUG (f'new_script  = {new_script}')
    
    if event == 'Step' or event == '-TABLE-':
        row = values['-TABLE-'][0] 
        rowsLst = values['-TABLE-']

    elif not ind == None:               # SCRIPT_STEP
        row = ind [0]
        rowsLst = ind

    else:
        print_err(f'-ERROR- unrecognized event combination ')

    if rowsLst in groups:
        print_log (f'Selected entire block - {rowsLst}')
        subscript =  list(LoadedScript.values())[groups.index(rowsLst)]
        group_n=  list(LoadedScript.keys())[groups.index(rowsLst)]
    else:
        print_log(f'Selected cmnd will be run in paralel mode - {rowsLst}')
        for rw in rowsLst:
            print_DEBUG(f'row in selected list = {rw}, new_script[{rw}] = {new_script[rw]}, new_script[{rw}][1] = {new_script[rw][1]}')
            # print_DEBUG(f'type of new_script[{rw}][1] = {type(new_script[rw][1])}, type of str =  {type(str([new_script[rw][1]]))}')
            # if len(subscript[new_script[rw][1]]) or subscript[new_script[rw][1]].isspace():
            # if len(new_script[rw][1]) == 0 or str(new_script[rw][1]).isspace():
            if not bool(new_script[rw][1].strip()):
                subscript['NOP'] = 'No Operation'
            else:
                subscript[new_script[rw][1]] = ''.join(str(x)+' ' for x in new_script[rw][2:])

        group_n = 'L' + new_script[rowsLst[0]][0]
        # have no idea whay I added 'L' prefix here, but it was like that before refactoring and is OBSOLETE now
        # removing it may break somthing, so let it be for now
    
    subscript = dict([(group_n, subscript)])

    print_log(f'row = {row}, group_n = {group_n}, Subscript = {subscript}')

    scroolTable(window=window, row=row, values=values)
    return subscript
    

def scroolTable(window:sg.Window, row:int, values):
    global new_script

    gr = next_group(row)
    print_DEBUG(f'Current row = {row}, values={values}, next group = {gr}')

    if values['-TABLE-'][-1] + 1 >= len (new_script):
        window['-TABLE-'].set_vscroll_position(0)
        print_log(f'Moving scrool up')



    if values['-TABLE-'][-1]  >= rows_in_table/2 and values['-TABLE-'][-1]  <= len(new_script) - rows_in_table/2:
        diff = (values['-TABLE-'][-1]  - rows_in_table/2)/(len(new_script) )
        print_log(f'ref row= {values["-TABLE-"][-1]  -  rows_in_table/2}, row_window = {len(new_script)}')
        window['-TABLE-'].set_vscroll_position(diff)
        print_log(f'set_vscroll_position({diff})')       
        

        window.refresh()

    window['-TABLE-'].update(select_rows = gr)

    

def GetSubScript(window, event, values, ind = None)->List[str]:

    subscript:List[str] = list()

    if groups == []:
        return []
    

    print_log (f'event =  {event}')
    print_DEBUG (f'values[-TABLE-] = {values["-TABLE-"]}, type = {type(values["-TABLE-"])}, ind = {ind}')
    print_DEBUG (f'new_script  = {new_script}')
    
    if event == 'Step' or event == '-TABLE-':
        row = values['-TABLE-'][0] 
        rowsLst = values['-TABLE-']

    elif not ind == None:               # SCRIPT_STEP
        row = ind [0]
        rowsLst = ind

    else:
        print_err(f'-ERROR- unrecognized event combination ')

    if rowsLst in groups:
        print_log (f'Selected entire block - {rowsLst}')
    else:
        print_log(f'Selected cmnd will be run in paralel mode - {rowsLst}')

    subscript = get_subscript(new_script, rowsLst)
    print_log(f'row = {row}, Subscript = {subscript}')



    print_DEBUG(f'Subscript from the Table = {subscript}, key = {subscript[0][0]}')
    gr = next_group(row)
    print_DEBUG(f'--BUGBUG-- subscript = {subscript}, current row = {row}, next group = {gr}')

    if values['-TABLE-'][-1] + 1 >= len (new_script):
        window['-TABLE-'].set_vscroll_position(0)
        print_log(f'Moving scrool up')



    if values['-TABLE-'][-1]  >= rows_in_table/2 and values['-TABLE-'][-1]  <= len(new_script) - rows_in_table/2:
        diff = (values['-TABLE-'][-1]  - rows_in_table/2)/(len(new_script) )
        print_log(f'ref row= {values["-TABLE-"][-1]  -  rows_in_table/2}, row_window = {len(new_script)}')
        window['-TABLE-'].set_vscroll_position(diff)
        print_log(f'set_vscroll_position({diff})')       
        

        window.refresh()

    window['-TABLE-'].update(select_rows = gr)
    # gv = window['-TABLE-'].get()
    return subscript

def LoadScriptFile(sysDevs:systemDevices):
     
    print_log("[LOG] Clicked Open File!")
    folder_or_file = sg.popup_get_file('Choose your file', keep_on_top=True, 
                            file_types = (('YAML Files', '*.yml'),('YAML Files', '*.yaml'),))
    print_log("[LOG] User chose file: " + str(folder_or_file))
    if folder_or_file:
        # load_script(folder_or_file)
        _res = load_embeded_script((folder_or_file), sysDevs)
        if _res:
            file_name = os.path.basename(folder_or_file)
            return  str(file_name)
        else:
            return None
    else:
        return None


def ClearLog():
    yesno = sg.popup_yes_no("You're about to delete all log data.\nDo you want to proceed",  title="YesNo")
    if yesno == 'Yes':
        Log_tbl.update(value='')

    
def SaveLog():
    folder_or_file = sg.popup_get_file('Select file to save log', keep_on_top=True, save_as='True')
    if folder_or_file:
        print(f'Writing to {str(folder_or_file)}')
        with io.open(folder_or_file, "w", encoding="utf8") as log_f:
            log_f.write(Log_tbl.get())
        log_f.close()



Log_tbl = sg.Multiline(size=(20,20), font='Courier 8', expand_x=True, expand_y=True, write_only=True,
                                   autoscroll=True, auto_refresh=True, key = '-OUTLOG-')



rows_in_table = 35
# rows_in_table = 15
first_displayed_row = 0




Tbl = sg.Table(values=data[0:][:], headings=headings, max_col_width=55,
                auto_size_columns=True,
                display_row_numbers=False,
                col_widths = 20,
                def_col_width = 20,
                justification='left',
                # cols_justification=('l','c','c','c', 'c'),
                # col_widths=[50, 1000, 50, 50, 50],
                # row_height=40,

                num_rows = rows_in_table,
                font = 'Courier 10',
                key='-TABLE-',
                text_color = main_color,
                background_color = main_bg_collor,
                selected_row_colors=(sel_color, sel_bg_collor),
                expand_x=True,
                expand_y=True,
                enable_click_events = False,

                vertical_scroll_only=True,
                select_mode = sg.TABLE_SELECT_MODE_EXTENDED, 
                bind_return_key = True,
                tooltip='Script')



#------------------------- U N I T E S T ----------------------------
#
# 
# 
# #
# 
# #
# 
# 
# 
#     
if __name__ == "__main__":

    from threading import  Thread
    from bs2_config import systemDevices
    sysDev = systemDevices()
    parms:dict =  sysDev.getParams()
    
    # ------ Window Layout ------
    layout = [[Tbl],
            [sg.Button('Run'), sg.Button('Stop'), sg.Button('Step'), sg.Button('Open File')],
            [sg.Text('Step = Run selected command block')],
            [sg.Text('Run = Run selected command block')],
            [sg.Text('Open File = Load command script'), sg.Sizegrip()]]

    # ------ Create Window ------
    window = sg.Window('Command Script', layout,
                    # ttk_theme='clam',
                    # font='Helvetica 25',
                    resizable=False
                    )

#----------  Util-----------
    def ScriptTestRunner(window:sg.Window, selected_group:List[str]):

        gr_index = -1

        print_log (f'Selection: {selected_group}')

        if selected_group == []:
            print_err(f'ERROR: Empty script')
            return
        
        

        if selected_group in groups:                 # probably block is not selected correctly, find correct group index
            gr_index = groups.index(selected_group)
        else:
            for gr_index, gr in enumerate(groups):
                if selected_group[0] in gr:
                    break
                
        print_log (f'Starting script at group # {gr_index} of {len (groups)} groups')   


        while gr_index < len (groups):
            print_log(f'Running step # {gr_index} -> {groups[gr_index]}')
            print_log(f'group # = {list(LoadedScript.keys())[gr_index]}')
            print_log(f'subscript = {LoadedScript[list(LoadedScript.keys())[gr_index]]}')
            ymlTest1(LoadedScript[list(LoadedScript.keys())[gr_index]])
            # window.write_event_value('Step', 'Step')
            window.write_event_value(' -SCRIPT_STEP-',  value = groups[gr_index])
           
            time.sleep(1)
            gr_index += 1

        print_log(f'Script done')


    
    def ymlTest1(subScript, cmdStr = str()):
        delimiter = str('-') if len(cmdStr) else ''
        for _key, _val in subScript.items():
            if isinstance(_val, dict):
                ymlTest1(_val, cmdStr=cmdStr + delimiter + str(_key))
            else:
                print_log(f'{cmdStr} {_key} -> {_val}')

        cmdStr = str() 


    def ymlTest2(subScript) -> dict:
        new_dict = dict()
        for _key, _val in subScript.items():
            if isinstance(_val, dict):
                tmpDict = dict(NOP = 'No operation')
                tmpDict.update(_val)
                print_log(f'new_dict = {tmpDict}')
                subDict = ymlTest2(tmpDict)

                # subDict = ymlTest2(_val)
                new_dict[_key] = subDict
                print_log(f'*{_key} -> {subDict}')
            else:
                new_dict[_key] = _val
                print_log(f' {_key} -> {_val}')

        return new_dict 

# ------ Event Loop ------
    
    

    while True:
        event, values = window.read()
        print_log(f'event = {event}, values = {values}')
        if event == sg.WIN_CLOSED:
            break
        elif event == 'Run':
            script_thread = Thread(target=ScriptTestRunner, args=(window, values['-TABLE-']))
            script_thread.start()
        elif event == 'Step' or event == '-TABLE-' :
            print_log(f'event =  {event}, values = {values}')
            # _sub = GetSubScript(window, event, values)
            _sub = GetSubScriptDict(window, event, values)
            print_log(f'Load operation: [event={event}] {_sub}')
        elif  event == ' -SCRIPT_STEP-':
            print_log(f'event =  {event}, values = {values}, values[event] = {values[event]}')
            # _sub = GetSubScript(window, event, values)
            _sub = GetSubScriptDict(window, event, values, values[event])
            print_log(f'Load operation: [event={event}] {_sub}')

        elif event == 'Open File':
            LoadScriptFile(sysDevs)
            window.refresh()
            print_log(f'Script = {LoadedScript}')
            print_log(f'Values = {LoadedScript.values()}')
            print_log(f'Keys = {LoadedScript.keys()}')
            ymlTest1(LoadedScript)
            print_log(f'New = {ymlTest2(LoadedScript)}')
        else:
            print_DEBUG(f'-BUGBUG - event = {event}, value = {values}')
            # print_DEBUG(f'-BUGBUG - event = {event}, value = {values[event]}')
            # Tbl_event_proc(window, event, values)
   
        


    window.close()

