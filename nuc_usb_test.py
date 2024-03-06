"""
# Python code transmits a byte to Arduino /Microcontroller
import serial
import time
path = '/dev/ttyACM0'
# path = '/dev/cu.usbmodem141301'
def ArduinoSent(left,up,width_x,width_y,serialObj):
    SendItem= '{0:0=3d}'.format(left)+'{0:0=3d}'.format(up)+'{0:0=3d}'.format(width_x) + '{0:0=3d}'.format(width_y)
    enc = SendItem.encode(encoding = "utf-8")
    ied =serialObj.write(enc)
    return ied

if __name__ == '__main__':
    # SerialObj = serial.Serial(path) # COMxx  format on Windows
    #                   # ttyUSBx format on Linux COM24 /dev/cu.usbmodem141301 /dev/ttyACM0

    # SerialObj.baudrate = 9600
    # SerialObj.parity  ='N'   # No parity
    # SerialObj.stopbits = 1   # Number of Stop bits = 1
    # time.sleep(3)
    # SerialObj.write(b'A')    #transmit 'A' (8bit) to micro/Arduino
    # print(SerialObj.read(size=1))
    # SerialObj.close()      # Close the port
    # 150070240160
    serialObj = serial.Serial(path,timeout=5)
    # serialObj.baudrate = 9600  # set Baud rate to 9600
    # SerialObj.bytesize = 8   # Number of data bits = 8
    left = 150
    up = 120 
    width_x = 240
    width_y = 160
    # SendItem="#"+str(left)+"&"+str(up)+"&"+str(width_x)+"&"+str(width_y)+"%"
    # "150070240160"
    time.sleep(3)
    # enc =  SendItem.encode('')
    # print(enc)
    # # Write data to the USB port
    # serialObj.write( b'Hello, World!')
    ArduinoSent(left,up,width_x,width_y,serialObj)
    # serialObj.flush()
    # print(ied)
    print(serialObj.read(size=12))
    # # x = 0
    # # while x< 12:
    
"""



"""import usb.core
import usb.util

# Find the USB device
dev = usb.core.find(idVendor=0x045e, idProduct=0x028e)

# If the device is not found, raise an error
if dev is None:
    raise ValueError('Device not found')

# Set the configuration of the USB device
dev.set_configuration()

# Write data to the USB port
dev.write(1, b'Hello, World!')

# Read data from the USB port
# data = dev.read(0x81, 1024)

# Print the data
# print(data)"""

"""
import usb.core
import usb.util

# Find the USB device
dev = usb.core.find(idVendor=0x045e, idProduct=0x028e)

# If the device is not found, raise an error
if dev is None:
    raise ValueError('Device not found')

# Set the configuration of the USB device
dev.set_configuration()

# Write data to the USB port
dev.write(1, b'Hello, World!')

# Read data from the USB port
data = dev.read(0x81, 1024)

# Print the data
print(data)
"""
from dronekit import connect , VehicleMode , LocationGlobalRelative , APIException
import time
import socket
try:
    import exceptions
except ImportError:
    pass
import math
import argparse
def connectMyCopter():
    parser = argparse.ArgumentParser(description="commands")
    parser.add_argument("--connect")
    args = parser.parse_args()
    connection_string = args.connect
    baud_rate = False
    vehicle = connect(connection_string,baud=baud_rate,wait_ready=True)
    return vehicle
def arm():
    while vehicle.is_armable == False:
        print("Waiting for vehicles to become armable")
        time.sleep(1)
    print("Vehicle is now armable")
    print("")
    vehicle.armed = True
    while vehicle.armed == False:
        print("Waiting for drone to become armed ")
        time.sleep(1)
    print("Vehicle is now armed")
    print("props are spinning, LOOK OUT!")
    return None
######
vehicle = connectMyCopter()
arm()
print("End of Script")
