"""
# Python code transmits a byte to Arduino /Microcontroller
import serial
import time
path = '/dev/ttyACM0'
# path = '/dev/cu.usbmodem141301'


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
import library
######
vehicle = library.connectMyCopter()
vehicle.mode = VehicleMode("GUIDED")


library.arm(vehicle)
library.send_ned_velocity(vehicle,1,0,0,10)
print("End of Script")
