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
# No way to Set the Copter to GUIDE mode in python code, plz do so using Mission planner
# python nuc_usb_test.py --connect localhost:14550
# from dronekit import connect , VehicleMode , LocationGlobalRelative , APIException
import library
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
vehicle = library.connectMyCopter()
# vehicle.mode = VehicleMode("GUIDED")
# modeUsed = ""
# modeUsed = "STABILIZE"
modeUsed = "GUIDED"
# modeUsed = "AUTO"
vehicle.mode = VehicleMode(modeUsed)


#
library.arm(vehicle)
offset = vehicle.messages['GLOBAL_POSITION_INT'].alt
# library.set_mode(vehicle,modeUsed)
# library.checklocation(vehicle)
library.takeoff(vehicle,10)

library.return_to_launch(vehicle)
# library.checklocation(vehicle)
library.disarm(vehicle)
# library.send_ned_velocity(vehicle,0,1,0,10)
#
# time.sleep(10)
#
# library.send_ned_velocity(vehicle,0,0,0,10)

# if vehicle.is_armable:
#     aTargetAltitude = 5
#     vehicle.simple_takeoff(aTargetAltitude)
#     while True:
#         print(" Altitude: ", vehicle.location.global_relative_frame.alt)
#         # Break and return from function just below target altitude.
#         if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
#             print("Reached target altitude")
#             break
#         time.sleep(1)
#     library.send_ned_velocity(vehicle,1,0,0,10)
#     print("End of Script")
# else:
#     vehicle = library.arm(vehicle)
# https://www.youtube.com/watch?v=XY2mnqYI9a0
# https://blog.csdn.net/laowangwin/article/details/130949007
# https://blog.csdn.net/qq_38264240/article/details/130977451
# https://www.google.com/search?q=mavproxy+%E7%9A%84%E5%AE%89%E8%A3%9D%E7%9B%AE%E9%8C%84&rlz=1C5CHFA_enHK924HK924&oq=mavproxy+%E7%9A%84%E5%AE%89%E8%A3%9D%E7%9B%AE%E9%8C%84&gs_lcrp=EgZjaHJvbWUyBggAEEUYOTIHCAEQIRigATIHCAIQIRigATIHCAMQIRigATIHCAQQIRigAdIBCTU0MDk5ajBqN6gCALACAA&sourceid=chrome&ie=UTF-8#ip=1
# https://ardupilot.org/planner/docs/mission-planner-installation.html#mission-planner-on-linux
# https://blog.csdn.net/qq_38264240/article/details/130977451
# https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html
# https://ardupilot.org/mavproxy/docs/getting_started/download_and_installation.html
# https://ardupilot.org/mavproxy/docs/getting_started/download_and_installation.html
# https://www.google.com/search?q=module+not+found+error+for+package+in+another+environment&rlz=1C5CHFA_enHK924HK924&oq=module+not+found+error+for+package+in+another+enviro&gs_lcrp=EgZjaHJvbWUqBwgBECEYoAEyBggAEEUYOTIHCAEQIRigAdIBCTE4Mzk1ajBqN6gCALACAA&sourceid=chrome&ie=UTF-8
# https://stackoverflow.com/questions/55367462/wxpython-error-on-setup-modulenotfound-no-module-named-wx
# https://zoomadmin.com/HowToInstall/UbuntuPackage/python-wxtools
# https://www.google.com/search?q=Obstacles+to+construction+automation&rlz=1C5CHFA_enHK924HK924&oq=Obstacles+to+construction+automation&gs_lcrp=EgZjaHJvbWUyBggAEEUYOTIKCAEQABiABBiiBDIKCAIQABiABBiiBDIKCAMQABiABBiiBDIKCAQQABiABBiiBNIBCTEzMzI3ajBqN6gCALACAA&sourceid=chrome&ie=UTF-8
# https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html
# https://ardupilot.org/dev/docs/sim-on-hardware.html
# https://ardupilot.org/copter/docs/initial-setup.html
# https://discuss.ardupilot.org/t/error-running-sim-vehicle-py/51649/9
# https://www.google.com/search?q=error+in+command+%5B%27load%27%2C+%27map%27%5D%3A+module+%27map%27+has+no+attribute+%27init%27&sca_esv=c1f14eea029e1e8c&rlz=1C5CHFA_enHK924HK924&sxsrf=ACQVn0_0H53sO8sWlyaXPu8Pb-B5V-r2nw%3A1711454177454&ei=4bcCZp6lG_-C2roP2o-7KA&oq=&gs_lp=Egxnd3Mtd2l6LXNlcnAiACoCCAQyBxAjGOoCGCcyBxAjGOoCGCcyBxAjGOoCGCcyBxAjGOoCGCcyBxAjGOoCGCcyBxAjGOoCGCcyBxAjGOoCGCcyBxAjGOoCGCcyBxAjGOoCGCcyBxAjGOoCGCdIvx9QAFikEHACeAGQAQCYAXqgAXqqAQMwLjG4AQPIAQD4AQGYAgOgApsBqAIKwgIQEAAYgAQYigUYQxixAxiDAcICCBAAGIAEGLEDwgIREC4YgAQYsQMYgwEYxwEY0QPCAgoQABiABBiKBRhDwgILEAAYgAQYsQMYgwHCAgsQLhiABBixAxiDAZgDEpIHAzIuMaAHpgc&sclient=gws-wiz-serp
# https://discuss.ardupilot.org/t/problem-in-running-sitl-in-linux/12075
# https://ardupilot.org/dev/docs/sitl-on-windows-wsl.html
# https://zh.wikipedia.org/wiki/%E9%80%82%E7%94%A8%E4%BA%8ELinux%E7%9A%84Windows%E5%AD%90%E7%B3%BB%E7%BB%9F
# https://pypi.org/project/map/
# https://github.com/THLO/map/blob/master/setup.py
# https://github.com/ArduPilot/MAVProxy/tree/master
# https://ardupilot.org/mavproxy/docs/getting_started/download_and_installation.html
# https://www.google.com/search?q=warning%3A+locale+not+supported+by+xlib&rlz=1C5CHFA_enHK924HK924&oq=warning%3A+locale+not+supported+by+xlib&gs_lcrp=EgZjaHJvbWUyBggAEEUYOTIGCAEQRRg60gEJMTY3MDlqMGo3qAIAsAIA&sourceid=chrome&ie=UTF-8
# https://github.com/dronekit/dronekit-python/issues/1058




