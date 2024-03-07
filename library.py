from dronekit import connect , VehicleMode , LocationGlobalRelative , APIException
import dronekit_sitl
import socket
# try:
#     import exceptions
# except ImportError:
#     pass
import cv2
import numpy as np
import torch
import math
import argparse
import pymavlink
import pymavlink.mavutil as mavutil
import time
def ArduinoSent(left,up,width_x,width_y,serialObj):
    SendItem= '{0:0=3d}'.format(left)+'{0:0=3d}'.format(up)+'{0:0=3d}'.format(width_x) + '{0:0=3d}'.format(width_y)
    enc = SendItem.encode(encoding = "utf-8")
    ied =serialObj.write(enc)
    return ied
def connectMyCopter():
    # parser = argparse.ArgumentParser(description="commands")
    # parser.add_argument("--connect")
    # args = parser.parse_args()
    # connection_string = args.connect
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

    baud_rate = False
    vehicle = connect(connection_string,baud=baud_rate,wait_ready=True)
    return vehicle
def arm(vehicle):
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
def compute_direction(vehicle):
    angle = vehicle.heading

def send_ned_velocity(vehicle,velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
def Box2Send(xyxy_best,serialObj,x,y):
    xmin = xyxy_best[0]
    ymin = xyxy_best[1]
    xmax = xyxy_best[2]
    ymax = xyxy_best[3]
    centre_point_x = (xmin+xmax)/2
    centre_point_y = (ymin+ymax)/2
    # print(type(xmin))
    width_x = int(torch.round((xmax-xmin)/ x))
    width_y = int(torch.round((ymax-ymin)/ y))
    
    left = int(torch.round(centre_point_x*100 / x))#centre_point_x - (x/2)
    up = int(torch.round(centre_point_y*100 / y))#centre_point_y - (y/2)

    # width_x,width_y
    # SendItem=str(left)+"&"+str(up)+"&"+str(width_x)+"&"+str(width_y)
    ArduinoSent(left,up,width_x,width_y,serialObj)
    # # Write data to the USB port
    # dev.write(1, b'Hello, World!')
    # serialObj.write(SendItem.encode('UTF-8')) 
