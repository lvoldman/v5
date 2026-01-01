__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "1.0.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Tool"


import ctypes
import time



from ctypes import *
from ctypes import wintypes


'''
NCT6102D Super I/O chip GPIO access via direct port I/O runs on UM_RCO-3000 motherboard.
Ref: NUVOTON NCT6102D datasheet
The NCT6102D Super I/O chip provides 8 general-purpose input/output (GPIO) pins
that can be configured as either inputs or outputs. These GPIO pins can be controlled through the Extended Function
Index Register (EFIR) and Extended Function Data Register (EFDR).
Operation sequence:
1. Enter the Extended Function Mode by writing 0x87 twice to the Extended Function Enable Registers (EFERs) at port 0x2E or 0x4E.
2. Select the GPIO Logical Device by writing 0x07 to the EFIR and then writing 0x07 to the EFDR.
3. To configure the GPIO pins as outputs, write the desired output values to the GPIO Data          
Register (CR F1h) by writing 0xF1 to the EFIR and then writing the output values to the EFDR.
4. To read the GPIO pins configured as inputs, write 0xED to the EFIR and then read the input values from the EFDR.
5. Exit the Extended Function Mode by writing 0xAA to the EFER.

'''

'''
Extended Function Index Register (EFIR) and Extended Function Data Register (EFDR). The EFIR is located at the same address as the
EFER, and the EFDR is located at address (EFIR+1).
'''
AddrPort = 0x2e                     # Extended Function Enable Registers (EFER) - 0x2E or 0x4E / Extended Function Index Register (EFIR)
DataPort =  0x2f                    # Extended Function Data Register (EFDR)

SIO_UnLock_Value =  0x87
SIO_Lock_Value = 0xaa               # To exit the Extended Function Mode, writing 0xAA to the EFER is required.

LDN_SELECTOR =  0x07                # Logical Device Selector Function = 0x07
SIO_LDN_GPIO = 0x07                 # GPIO Logical Device Number = 0x07

GPIO_Port_OUT = 0xF1                # CR F1h. GPIO4 Data Register  / OUT // ref: NUVOTON NCT6102D 

GPIO_Port_IN = 0xED                 # CR EDh. GPIO3 Data Register  / IN // ref: NUVOTON NCT6102D


# path = '.\DLL\inpout32.dll' 


def WriteByte(port, _data:int) -> bool:
    _lst = [0] * 2

    _lst[0] = _data & 0xFF
    _bt = bytes(_lst)
    print (f'TRACE = Port= {port:04x}, Write Data: 0x{_bt[0]:04x}')

    # _bt = bytes(_data, 0)
    
    # inpout32.Out32(port, _data)
    inpout32.Out32(port, _bt[0])


def ReadByte(port) -> int:
    current_value = inpout32.Inp32(port)
    print (f'TRACE = Port= {port:04x},Read data: 0x{current_value:04x}')
    return current_value



def ReadIO() -> int:
    return ReadWriteIO()

def WriteIO (_writeData:int):
    ReadWriteIO(_writeData)    

def ReadWriteIO(_writeData:int = None) -> int:
    _readData:int = None
    #Enter_Config
    '''
    To place the chip into the Extended Function Mode, two successive writes of 0x87 (SIO_UnLock_Value) must be applied to Extended
    Function Enable Registers (EFERs, i.e. 0x2E or 0x4E)
    '''
    WriteByte (AddrPort, SIO_UnLock_Value)
    time.sleep(0.4)
    WriteByte (AddrPort, SIO_UnLock_Value)

    '''
    First, write the Logical Device Number (i.e. 0x07) to the EFIR and then write the number of the desired Logical
    Device to the EFDR. If accessing the Chip (Global) Control Registers, this step is not required.
    '''

    WriteByte (AddrPort, LDN_SELECTOR)
    WriteByte (DataPort, SIO_LDN_GPIO)          # Logical device = 7 

    '''
    Secondly, write the address of the desired configuration register within the Logical Device to the EFIR and then
    write (or read) the desired configuration register through the EFDR
    '''

    if _writeData is not None:                    # write operation
        #Set OUT1~OUT8Value
        WriteByte (AddrPort, GPIO_Port_OUT)
        _readData = ReadByte (DataPort);          # Read In1~In8 value form OUT port
        print(f'Writing {_writeData} / 0x{_writeData:02x} to 0x{GPIO_Port_OUT:02x} port //// Old values: {_readData} / 0x{_readData:02x}')
        WriteByte (DataPort, _writeData);         # _writeData: set OUT1~OUT8 value, OUT1=Bit0, OUT2=Bit1
        
    else:
        #Read In1~In8 value
        WriteByte (AddrPort, GPIO_Port_IN)
        _readData = ReadByte (DataPort);          # Read In1~In8 value from IN port
        print(f'Read {_readData} / 0x{_readData:02x} frokm port 0x{GPIO_Port_OUT:02x} port')
    
        
    
    # close config mode
    '''
    To exit the Extended Function Mode, writing 0xAA (SIO_Lock_Value) to the EFER is required.
    '''
    WriteByte (AddrPort, SIO_Lock_Value)

    return _readData


def simulReadWriteIO(_writeData:int) -> int:
    _readData:int = None
    #Enter_Config
    '''
    To place the chip into the Extended Function Mode, two successive writes of 0x87 (SIO_UnLock_Value) must be applied to Extended
    Function Enable Registers (EFERs, i.e. 0x2E or 0x4E)
    '''
    WriteByte (AddrPort, SIO_UnLock_Value)
    time.sleep(0.4)
    WriteByte (AddrPort, SIO_UnLock_Value)

    '''
    First, write the Logical Device Number (i.e. 0x07) to the EFIR and then write the number of the desired Logical
    Device to the EFDR. If accessing the Chip (Global) Control Registers, this step is not required.
    '''

    WriteByte (AddrPort, 0x07)
    WriteByte (DataPort, SIO_LDN_GPIO)          # Logical device = 7 ???

    '''
    Secondly, write the address of the desired configuration register within the Logical Device to the EFIR and then
    write (or read) the desired configuration register through the EFDR
    '''

    #Set OUT1~OUT8Value
    WriteByte (AddrPort, GPIO_Port_OUT)
    WriteByte (DataPort, _writeData);         # _writeData: set OUT1~OUT8 value, OUT1=Bit0, OUT2=Bit1

    #Read In1~In8 value
    WriteByte (AddrPort, GPIO_Port_IN)
    _readData = ReadByte (DataPort);          # Read In1~In8 value
    
    # close config mode
    '''
    To exit the Extended Function Mode, writing 0xAA (SIO_Lock_Value) to the EFER is required.
    '''
    WriteByte (AddrPort, SIO_Lock_Value)

    return _readData


import  sys, re

if __name__ == "__main__":

    inpout32 = ctypes.WinDLL("64" in sys.version and ".\\inpoutx64.dll" or ".\\inpout32.dll")


    # inpout32 = ctypes.WinDLL(".\inpoutx64.dll")

    # inpout32 = ctypes.WinDLL(".\inpout32.dll")

    if inpout32.IsInpOutDriverOpen():
        print(f'Succefuly connected to the Driver')
    else:
        print(f'Connection to the Diver was lost')

    while True:
        try:
            _val:str = input("Enter byte to write into the port: ")
            _valRE = re.compile(r'\s*[0-9A-Fa-f]{2}\s*$')
            if not (_val == '') and not _valRE.match(_val):
                print(f'Wrong byte HEX input. Try again.')
                continue
            if not (_val == ''):
                _hexSTR =  '0x' + (str(_val))
                _UintVal = int(_hexSTR, base=16)



                # _rValue = simulReadWriteIO(_UintVal)

                old_value = ReadWriteIO(_UintVal)
                print('----------------')
                _rValue = ReadWriteIO()


                print(f'Wrote {_UintVal:02x} / {_UintVal} to output port (old value = {old_value:02x}) // Read {_rValue:02x} from Input port')
                



        except KeyboardInterrupt as ex:
            print(f'Exiting by ^C \n{ex}')
            break


    sys.exit()

    




