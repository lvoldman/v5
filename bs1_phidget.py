__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["Leonid Voldman","Sasha Schechtman"]
__license__ = "SLA"
__version__ = "2.0.1"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Prototype"

from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Devices.DigitalOutput import *
import traceback
from typing import List


import sys, time
from collections import namedtuple
from queue import Queue 
from enum import Enum
import threading


TRIGGER_DELAY  = 0.5

from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, set_parm, get_parm, toInt 


class PhidgetRELAY:
    dev_state = Enum("dev_state", ["open", "close"])   # open means - ON, close means OFF

    # devType = Enum("devType", ["dispenser", "uv", "front_light", "back_light", "hole_light", "welder", "jtse_hotair"])
    devType = Enum("devType", ["onoff", "trigger", "toggle"])

    devInfoFields = ["devSN", "devChans", "channelClassName", "channelName"]
    devInfo = namedtuple("devInfo",  devInfoFields, defaults=[None,] * len(devInfoFields))
    devInfo.__annotations__={'devSN':int, 'devChans':int,  'channelClassName':str, 'channelName':str}         # specify type of elements

    attachedDev = namedtuple('attachedDev', ['type', 'obj'])

    # pdev = namedtuple('pdev', ['name', 'className', 'sn', 'channels'])

    __attachedDevsList:List[attachedDev] = list()               # devices defined in serials.yml and initiated by config util

    __existingDevs:List[devInfo] = None                          # list of devices were found in the system 


    @staticmethod
    def getPHGObj(tp):
        for _obj in PhidgetRELAY.__attachedDevsList:
            if _obj.type == tp:
                return _obj.obj
            
        print_err(f'Canot find GHP object of type {tp}')
        return None

    @staticmethod
    def test():
        PhidgetRELAY.__listDevs()

    @staticmethod
    def __listDevs():                                       # find all devices 
        if PhidgetRELAY.__existingDevs:                     # the finding process alredy done
            return
        
        PhidgetRELAY.__existingDevs =  list()
        dOlist = list()
        try:
            while True:
                dO = DigitalOutput()
                dO.openWaitForAttachment(1000)
                # channelClass = dO.getChannelClass()
                channelClassName = dO.getChannelClassName()
                channelName = dO.getChannelName()
                count = dO.getDeviceChannelCount(ChannelClass.PHIDCHCLASS_DIGITALOUTPUT)

                sn = dO.getDeviceSerialNumber()
                print_log (f'Found PHG device: SN = {sn}, channels = {count}, channelClassName = {channelClassName}, channelName = {channelName}')
                
                PhidgetRELAY.__existingDevs.append(PhidgetRELAY.devInfo(devSN=sn, devChans =count, channelName=channelName, channelClassName=channelClassName))
                dOlist.append(dO)                   # used to close Digital Output upon devices detection completed  
            
            
            
        except PhidgetException as ex:
            if ex.code == 3:        # timeout
                print_log(f'Timeout, no more devices')
            else:
                exptTrace(ex)
            
        except Exception as ex:
            exptTrace(ex)

        print_log(f'Found devices: {PhidgetRELAY.__existingDevs}')
        try:
            for dO in dOlist:
                dO.close()
        except Exception as ex:
            exptTrace(ex)


    @staticmethod
    def find_devs(sn, ch):
        if PhidgetRELAY.__existingDevs == None:                     # find devs
            PhidgetRELAY.__listDevs()

        try:
            for device in PhidgetRELAY.__existingDevs:
                if toInt(sn) == device.devSN and toInt(ch) < device.devChans:
                    print_log(f'Found device with sn = {sn}/{ch} -> {device}')
                    return device
            
            print_err(f'Error detecting PHG device with SN {sn} and channel {ch}')
            print_err(f'Available devices: {PhidgetRELAY.__existingDevs}')
            return None
            

        except Exception as ex:
            print_err(f'Exception while detecting PHG device with SN {sn} and channel {ch}')
            exptTrace(ex)
            return None
        

    def __init__(self, sn, chan, devT, dName, params_table):
        try:
            self.__devType:PhidgetRELAY.devType = devT
            self.devName = dName                                            # name as defined in serials.yml
            self.__sn = sn
            self.__chan = chan
            self.__trigger_delay:float = TRIGGER_DELAY
            self.__state = PhidgetRELAY.dev_state.close                    # close means OFF
            self.devNotificationQ:Queue = Queue()                          # for compotabolity and future prolonged op

            self.__digitalOutput = DigitalOutput()

            self.__digitalOutput.setDeviceSerialNumber(int(sn))

            self.__digitalOutput.setChannel(int(chan))

            self.__digitalOutput.openWaitForAttachment(5000)

            PhidgetRELAY.__attachedDevsList.append(PhidgetRELAY.attachedDev(type = devT, obj = self))
            print_log(f'Created object for Phidget: sn = {sn}/{chan}. Verification sn = {self.__digitalOutput.getDeviceSerialNumber()}')
            self.__digitalOutput.setDutyCycle(0)
            self.__title = dName
            self.__wd:threading.Thread = None

            self.set_parms(params_table)

        except Exception as ex:
            print_err(f'Exception while initiating PHG device with SN {sn} and channel {chan}')
            exptTrace(ex)
            self.__digitalOutput.close()


    def devOperate(self, onoff = None)->bool:
        print_log(f'Operating PHG: {self}, op={onoff}')
        if self.__devType == PhidgetRELAY.devType.trigger:
            
            if onoff is not None:
                if onoff:                                                   # nothing do for trigger
                    return True
                else:
                    return self.relayOnOff(onoff)

            
            return self.trigerDev()
        elif self.__devType == PhidgetRELAY.devType.toggle:

            if onoff is not None and not onoff:                                                   # nothing do for trigger
                return True
            
            return self.toggle()
        elif self.__devType == PhidgetRELAY.devType.onoff:
            if not (onoff is None):
                return self.relayOnOff(onoff)
            else:
                print_err(f'ERROR - no ON/OFF parameter for ONOFF device')
                return False
        else:
            print_err(f'ERROR - Unsupported dev type: {self.__devType}')
            return False
        
        return True



    def mDev_stop(self):
        print_log(f'Stoping {self.devName}')
        try:
            if self.__state == PhidgetRELAY.dev_state.open:        # ON -> OFF
                self.__digitalOutput.setDutyCycle(0)
        except Exception as ex:
                e_type, e_filename, e_line_number, e_message = exptTrace(ex)
                print_log(f'Triger op failed on dev {self.devName}')
                return False
        
        
        return True
    
    def __del__(self):
        self.mDev_stop()
        
        if PhidgetRELAY.attachedDev(type = self.__devType, obj = self) in PhidgetRELAY.__attachedDevsList:
            PhidgetRELAY.__attachedDevsList.remove(PhidgetRELAY.attachedDev(type = self.__devType, obj = self))
        else:
            print_err(f'-WARNING --no found PHG device to remove: type={self.__devType}, S/N = {self.__sn}, channel = {self.__chan}')

        self.__digitalOutput.close()

    def __repr__(self) -> str:
        return f'name = {self.devName}, type = {self.__devType}, sn/ch = {self.__sn}/{self.__chan}, delay = {self.__trigger_delay}'
    
    def dType(self):
        return self.__devType

    def __trigger_delay_Thread(self):
        try:
            time.sleep(self.__trigger_delay)
            self.__digitalOutput.setDutyCycle(0)
            print_log(f'End Trigger operation, delay = {self.__trigger_delay}')
        except Exception as e:
            print_log(f'Trigger delay thread stopped unexpectedly: {e}')        
        return

    def trigerDev (self)->bool:
        try:
            print_log(f'Triggering {self.devName} of type {self.__devType}, delay = {self.__trigger_delay}')
            self.__digitalOutput.setDutyCycle(1)

            # time.sleep(self.__trigger_delay)
            # self.__digitalOutput.setDutyCycle(0)

            if self.__wd is not None and  self.__wd.is_alive():
                print_log(f'The Trigger thread for {self.devName} device is running')
            else:
                self.__wd = threading.Thread(target=self.__trigger_delay_Thread)
                self.__wd.start()            
            
        except Exception as ex:
                e_type, e_filename, e_line_number, e_message = exptTrace(ex)
                print_log(f'Triger op failed on dev {self.devName}')
                return False
        
        return True

    def toggle (self)->bool:
        new_state = None
        try:
            if self.__state == PhidgetRELAY.dev_state.close:
                self.__digitalOutput.setDutyCycle(1)    
                new_state =  PhidgetRELAY.dev_state.open   
            elif self.__state == PhidgetRELAY.dev_state.open:
                self.__digitalOutput.setDutyCycle(0) 
                new_state =  PhidgetRELAY.dev_state.close
            else:
                print_err(f'ERROR: Unrecognized state for  {self.devName}: self.__state ')
                return False
            
            print_log(f'Toggle {self.devName} of type {self.__devType}: {self.__state} -> {new_state} ')
            self.__state = new_state        
        except Exception as ex:
                e_type, e_filename, e_line_number, e_message = exptTrace(ex)
                print_log(f'Toggle op failed on dev {self.devName}')
                return False
        
        return True

    def isOpen(self):
        return True if self.__state == PhidgetRELAY.dev_state.open else False
    
    def relayOnOff(self, flag:bool) -> bool:
        if flag:
            return self.__relayOn()
        else: 
            return self.__relayOff()

    def __relayOn(self):
        try:
            self.__digitalOutput.setDutyCycle(1)    
            print_log(f'Relay {self.devName} of type {self.__devType} has been switched ON ')
            self.__state = PhidgetRELAY.dev_state.open   
        except Exception as ex:
                e_type, e_filename, e_line_number, e_message = exptTrace(ex)
                print_log(f'Switch ON op failed on dev {self.devName}')
                return False
        
        return True


    def __relayOff(self):
        try:
            self.__digitalOutput.setDutyCycle(0)    
            print_log(f'Relay {self.devName} of type {self.__devType} has been switched OFF ')
            self.__state = PhidgetRELAY.dev_state.close   
        except Exception as ex:
                e_type, e_filename, e_line_number, e_message = exptTrace(ex)
                print_log(f'Switch Off op failed on dev {self.devName}')
                return False
        
        return True
    
    def set_parms(self, parms):
        _parms_str = get_parm('PHG', parms, self.devName)
        if _parms_str is None:
            return
        _parms =  _parms_str.split(',')    
        if len(_parms) > 0:
            self.__title = _parms[0]
        if len(_parms) > 1:
            if _parms[1].strip().isdigit():
                self.__trigger_delay = int(_parms[1].strip())/1000
            else:
                print_log(f'Wrong trigger delay format: >{_parms[1]}<')

        print_log(f'Reload partms: {self}')

    
    def getTitle(self)->str:
        return self.__title
    



################ UNITEST module ###############

if __name__ == "__main__":

    # event handlers 

    def onAttach(self):
        print("Attach [" + str(self.getChannel()) + "]!")

    def onDetach(self):
        print("Detach [" + str(self.getChannel()) + "]!")

    def onError(self, code, description):
        print("Code [" + str(self.getChannel()) + "]: " + ErrorEventCode.getName(code))
        print("Description [" + str(self.getChannel()) + "]: " + str(description))
        print("----------")


    def StupidExample():
        try:
            digitalOutput0 = DigitalOutput()

            digitalOutput0.openWaitForAttachment(5000)

            digitalOutput0.setDutyCycle(1)

            time.sleep(5)

            digitalOutput0.close()

        except PhidgetException as ex:
            traceback.print_exc()
            print("")
            print("PhidgetException " + str(ex.code) + " (" + ex.description + "): " + ex.details)



    def SimpleTest():

        PhidgetRELAY.test()
        # sys.exit()
        
        digOutPutLst = list()
        try:
            
            dO = DigitalOutput()
            print(f'Digital output = {dO}')

            dO.openWaitForAttachment(1000)

            cClass = dO.getChannelClass()
            count = dO.getDeviceChannelCount(ChannelClass.PHIDCHCLASS_DIGITALOUTPUT)

            sn = dO.getDeviceSerialNumber()
            print (f'SN = {sn}, chans = {count}')


            dO1 = DigitalOutput()
            print(f'Digital output = {dO1}')
            dO1.openWaitForAttachment(1000)
            # dID = dO1.getDeviceID()
            


            cClass = dO1.getChannelClass()
            count1 = dO1.getDeviceChannelCount(ChannelClass.PHIDCHCLASS_DIGITALOUTPUT)

            sn1 = dO1.getDeviceSerialNumber()
            print (f'SN1 = {sn1}, chans1 = {count1}')

            try:
    
                dO2 = DigitalOutput()
                print(f'Digital output = {dO2}')
                dO2.openWaitForAttachment(1000)

                cClass = dO2.getChannelClass()
                count2 = dO2.getDeviceChannelCount(ChannelClass.PHIDCHCLASS_DIGITALOUTPUT)

                sn2 = dO2.getDeviceSerialNumber()
                print (f'SN2 = {sn2}, chans1 = {count2}')

            # except Exception as ex:
            except PhidgetException as ex:
                    print(f'exception = {ex} / {ex.__dict__.keys()}')
                    print(f'exception = {ex} / {ex.code}')
                    exptTrace(ex)
            

            dO.close()
            dO1.close()
            # count += count1
            # dO2.close()

            print(f'Sum number of chans = {count}')

            for ch in range (int(count)):
                tmpDO = DigitalOutput()
                tmpDO.setDeviceSerialNumber(sn)
                digOutPutLst.append(tmpDO)

                digOutPutLst[ch].setChannel(ch)
                deviceSerialNumber = digOutPutLst[ch].getDeviceSerialNumber()
                print(f'DeviceSerialNumber {sn}/{ch} = {str(deviceSerialNumber)}')

                digOutPutLst[ch].openWaitForAttachment(5000)

                channelClass = digOutPutLst[ch].getChannelClass()
                print(f'ChannelClass: {sn}/{ch} = {str(channelClass)}')
                channelClassName = digOutPutLst[ch].getChannelClassName()
                print(f'ChannelClassName {sn}/{ch} = {str(channelClassName)}')
                channelName = digOutPutLst[ch].getChannelName()
                print(f'ChannelName: {sn}/{ch} = {str(channelName)}')

                # digOutPutLst[ch].setChannel(ch)
                # deviceSerialNumber = digOutPutLst[ch].getDeviceSerialNumber()
                # print(f'DeviceSerialNumber {ch} = {str(deviceSerialNumber)}')
                # channelClass = digOutPutLst[ch].getChannelClass()
                # print(f'ChannelClass: {ch} = {str(channelClass)}')
                # channelClassName = digOutPutLst[ch].getChannelClassName()
                # print(f'ChannelClassName {ch} = {str(channelClassName)}')
                # channelName = digOutPutLst[ch].getChannelName()
                # print(f'ChannelName: {ch} = {str(channelName)}')

            for __ch in range (int(count1)):
                ch = __ch + count
                tmpDO = DigitalOutput()
                tmpDO.setDeviceSerialNumber(sn1)
                digOutPutLst.append(tmpDO)

                digOutPutLst[ch].setChannel(__ch)
                deviceSerialNumber = digOutPutLst[ch].getDeviceSerialNumber()
                print(f'DeviceSerialNumber {sn1}/{__ch} = {str(deviceSerialNumber)}')

                digOutPutLst[ch].openWaitForAttachment(5000)

                channelClass = digOutPutLst[ch].getChannelClass()
                print(f'ChannelClass: {sn1}/{__ch} = {str(channelClass)}')
                channelClassName = digOutPutLst[ch].getChannelClassName()
                print(f'ChannelClassName {sn1}/{__ch} = {str(channelClassName)}')
                channelName = digOutPutLst[ch].getChannelName()
                print(f'ChannelName: {sn1}/{__ch} = {str(channelName)}')

            
            print(f'Initiated {len(digOutPutLst)} channels')
            
            # for ch, _digOut in enumerate(digOutPutLst):
            #     digOutPutLst[ch].setChannel(ch)
            #     deviceSerialNumber = digOutPutLst[ch].getDeviceSerialNumber()
            #     print(f'DeviceSerialNumber {ch} = {str(deviceSerialNumber)}')

            #     digOutPutLst[ch].openWaitForAttachment(5000)

            #     channelClass = digOutPutLst[ch].getChannelClass()
            #     print(f'ChannelClass: {ch} = {str(channelClass)}')
            #     channelClassName = digOutPutLst[ch].getChannelClassName()
            #     print(f'ChannelClassName {ch} = {str(channelClassName)}')
            #     channelName = digOutPutLst[ch].getChannelName()
            #     print(f'ChannelName: {ch} = {str(channelName)}')

                # #Create your Phidget channels
                # print('Step ------- 1 ---------------')
                # digitalOutput0 = DigitalOutput()
                # digitalOutput1 = DigitalOutput()
                # digitalOutput2 = DigitalOutput()
                # digitalOutput3 = DigitalOutput()



                # #Set addressing parameters to specify which channel to open (if any)
                # print('Step ------- 2 ---------------')
                # digitalOutput0.setChannel(0)
                # digitalOutput1.setChannel(1)
                # digitalOutput2.setChannel(2)
                # digitalOutput3.setChannel(3)

                # #Assign any event handlers you need before calling open so that no events are missed.
                # print('Step ------- 3 ---------------')
                # digitalOutput0.setOnAttachHandler(onAttach)
                # digitalOutput0.setOnDetachHandler(onDetach)
                # digitalOutput0.setOnErrorHandler(onError)

                # digitalOutput1.setOnAttachHandler(onAttach)
                # digitalOutput1.setOnDetachHandler(onDetach)
                # digitalOutput1.setOnErrorHandler(onError)

                # digitalOutput2.setOnAttachHandler(onAttach)
                # digitalOutput2.setOnDetachHandler(onDetach)
                # digitalOutput2.setOnErrorHandler(onError)

                # digitalOutput3.setOnAttachHandler(onAttach)
                # digitalOutput3.setOnDetachHandler(onDetach)
                # digitalOutput3.setOnErrorHandler(onError)

                # #Open your Phidgets and wait for attachment
                # print('Step ------- 4 ---------------')
                # digitalOutput0.openWaitForAttachment(5000)
                # digitalOutput1.openWaitForAttachment(5000)
                # digitalOutput2.openWaitForAttachment(5000)
                # digitalOutput3.openWaitForAttachment(5000)

                # # attached = digitalOutput0.getAttached()
                # # print("Attached: " + str(attached))
                # # attached = digitalOutput1.getAttached()
                # # print("Attached: " + str(attached))
                # # attached = digitalOutput2.getAttached()
                # # print("Attached: " + str(attached))
                # # attached = digitalOutput3.getAttached()
                # # print("Attached: " + str(attached))

                # deviceSerialNumber = digitalOutput0.getDeviceSerialNumber()
                # print("DeviceSerialNumber 0: " + str(deviceSerialNumber))
                # deviceSerialNumber = digitalOutput1.getDeviceSerialNumber()
                # print("DeviceSerialNumber 1: " + str(deviceSerialNumber))
                # deviceSerialNumber = digitalOutput2.getDeviceSerialNumber()
                # print("DeviceSerialNumber 2: " + str(deviceSerialNumber))
                # deviceSerialNumber = digitalOutput3.getDeviceSerialNumber()
                # print("DeviceSerialNumber 3: " + str(deviceSerialNumber))

                
                # channelClass = digitalOutput0.getChannelClass()
                # print("ChannelClass: " + str(channelClass))
                
                
                # channelClassName = digitalOutput0.getChannelClassName()
                # print("ChannelClassName: " + str(channelClassName))


                # channelName = digitalOutput0.getChannelName()
                # print("ChannelName: " + str(channelName))


                # count = digitalOutput0.getDeviceChannelCount(ChannelClass.PHIDCHCLASS_DIGITALOUTPUT)
                # print (f'Found {count} channels')


            while(True):
                try:
                    _val = input("Enter cmd: [0-3]/(0|1) ").split('/')

                    try:
                        if not len(_val) == 2:
                            print(f'wrong command format: {_val}')
                            continue
                        elif not (_val[0].isdigit() and int( _val[0])<=7 and int( _val[0])>=0 and _val[1].isdigit() and int( _val[1])<=1 and int( _val[1])>=0 ):
                            print(f'Error entering cmd parms: {_val}')
                            continue
                        else:
                            _parm_value = int(_val[1]) 
                            match _val[0]:
                                case '0':
                                    digOutPutLst[0].setDutyCycle(_parm_value)
                                    # digitalOutput0.setState(True if _parm_value == 1 else False)
                                    print(f'Channel {_val[0]}, setDutyCycle({_parm_value})')
                                case '1':
                                    digOutPutLst[1].setDutyCycle(_parm_value)
                                    # digitalOutput1.setState(True if _parm_value == 1 else False)
                                    print(f'Channel {_val[0]}, setDutyCycle({_parm_value})')
                                case '2':
                                    digOutPutLst[2].setDutyCycle(_parm_value)
                                    # digitalOutput2.setState(True if _parm_value == 1 else False)
                                    print(f'Channel {_val[0]}, setDutyCycle({_parm_value})')
                                case '3':
                                    digOutPutLst[3].setDutyCycle(_parm_value)
                                    # digitalOutput3.setState(True if _parm_value == 1 else False)
                                    print(f'Channel {_val[0]}, setDutyCycle({_parm_value})')
                                case '4':
                                    digOutPutLst[4].setDutyCycle(_parm_value)
                                    # digitalOutput0.setState(True if _parm_value == 1 else False)
                                    print(f'Channel {_val[0]}, setDutyCycle({_parm_value})')
                                case '5':
                                    digOutPutLst[5].setDutyCycle(_parm_value)
                                    # digitalOutput1.setState(True if _parm_value == 1 else False)
                                    print(f'Channel {_val[0]}, setDutyCycle({_parm_value})')
                                case '6':
                                    digOutPutLst[6].setDutyCycle(_parm_value)
                                    # digitalOutput2.setState(True if _parm_value == 1 else False)
                                    print(f'Channel {_val[0]}, setDutyCycle({_parm_value})')
                                case '7':
                                    digOutPutLst[7].setDutyCycle(_parm_value)
                                    # digitalOutput3.setState(True if _parm_value == 1 else False)
                                    print(f'Channel {_val[0]}, setDutyCycle({_parm_value})')
                                case _:
                                    print(f'something went wrong Channel = {_val[0]}, setDutyCycle({_parm_value})')    
                                

                            # print(f'Channel: {_val[0]}, setDutyCycle({_parm_value})/setState({True if _parm_value == 1 else False})')

                    except Exception as ex:
                        print(f'Something goes wrong. Exception = {ex}')
                        continue

                except KeyboardInterrupt as ex:
                    print(f'Exiting by ^C \n{ex}')
                    #Close your Phidgets once the program is done.
                    # for ch in range (int(count)):
                    for ch in range (len(digOutPutLst)):
                        digOutPutLst[ch].close()
                    print(f'{len(digOutPutLst)} channels closed')
                    # digitalOutput0.close()
                    # digitalOutput1.close()
                    # digitalOutput2.close()
                    # digitalOutput3.close()

                    sys.exit()
                except Exception as ex:
                    print(f'Exception operation error on {ex}')   
            


        except PhidgetException as ex:
            # will catch Phidget Exceptions here, and print the error informaiton.
            traceback.print_exc()
            print("")
            print("PhidgetException " + str(ex.code) + " (" + ex.description + "): " + ex.details)        



           

    print('Starting ....')
    SimpleTest()
    # StupidExample()
    

