from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
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
import cmath
import argparse
import pymavlink
import pymavlink.mavutil as mavutil
import time


def ArduinoSent(left, up, width_x, width_y, serialObj):
    SendItem = '{0:0=3d}'.format(left) + '{0:0=3d}'.format(up) + '{0:0=3d}'.format(width_x) + '{0:0=3d}'.format(width_y)
    enc = SendItem.encode(encoding="utf-8")
    ied = serialObj.write(enc)
    return ied


def connectMyCopter():
    # parser = argparse.ArgumentParser(description="commands")
    # parser.add_argument("--connect")
    # args = parser.parse_args()
    # connection_string = args.connect
    # sitl = dronekit_sitl.start_default()
    # connection_string = sitl.connection_string()
    connection_string = "/dev/ttyUSB0"
    print("Connect On:", connection_string)

    # baud_rate = 57600
    # vehicle = connect(connection_string,baud=baud_rate,wait_ready=True)

    # Start a connection listening on a UDP port
    vehicle = mavutil.mavlink_connection(connection_string, baud=57600)
    # Wait for the first heartbeat
    #   This sets the system and component ID of remote system for the link
    vehicle.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (vehicle.target_system, vehicle.target_component))

    return vehicle


def arm(vehicle):
    # vehicle.mav.
    #  400

    message = vehicle.mav.command_long_encode(vehicle.target_system, vehicle.target_component,
                                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    # Send the COMMAND_LONG
    vehicle.mav.send(message)
    # Wait for a response (blocking) to the MAV_CMD_SET_MESSAGE_INTERVAL command and print result
    response = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
    print(response)
    print(response.command, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    print(response.result, mavutil.mavlink.MAV_RESULT_ACCEPTED)
    if response and response.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Command accepted")
    else:
        print("Command failed")
    # vehicle.mav.send()
    # Safety check UNCOMMENT BEFORE DEPLOYMENT
    # while vehicle.is_armable == False:
    #     print(f"Waiting for vehicles to become armable {vehicle.is_armable}")
    #     time.sleep(1)
    # print("Vehicle is now armable")
    # print("")
    # vehicle.armed = True
    # while vehicle.armed == False:
    #     print("Waiting for drone to become armed ")
    #     time.sleep(1)
    # print("Vehicle is now armed")
    # print("props are spinning, LOOK OUT!")
    # return vehicle


def compute_direction(vehicle, x, y, x_mid, y_mid):
    angle = math.radians(vehicle.heading)
    xa = x - x_mid
    ya = y - y_mid
    cmplx_coor = complex(xa, ya)
    length, box_angle = cmath.polar(cmplx_coor)
    correct_angle = (box_angle + angle) % (2 * np.pi)
    res = cmath.rect(np.log10(length), correct_angle)
    x_velo = res.real
    y_velo = res.imag
    send_ned_velocity(vehicle, x_velo, y_velo, 0, 1)


def printStatus(vehicle):
    # vehicle is an instance of the Vehicle class
    print("Autopilot Firmware version: %s" % vehicle.version)
    print("Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp)
    print("Global Location: %s" % vehicle.location.global_frame)
    print("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    print("Local Location: %s" % vehicle.location.local_frame)  # NED
    print("Attitude: %s" % vehicle.attitude)
    print("Velocity: %s" % vehicle.velocity)
    print("GPS: %s" % vehicle.gps_0)
    print("Groundspeed: %s" % vehicle.groundspeed)
    print("Airspeed: %s" % vehicle.airspeed)
    print("Gimbal status: %s" % vehicle.gimbal)
    print("Battery: %s" % vehicle.battery)
    print("EKF OK?: %s" % vehicle.ekf_ok)
    print("Last Heartbeat: %s" % vehicle.last_heartbeat)
    print("Rangefinder: %s" % vehicle.rangefinder)
    print("Rangefinder distance: %s" % vehicle.rangefinder.distance)
    print("Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
    print("Heading: %s" % vehicle.heading)
    print("Is Armable?: %s" % vehicle.is_armable)
    print("System status: %s" % vehicle.system_status.state)
    print("Mode: %s" % vehicle.mode.name)  # settable
    print("Armed: %s" % vehicle.armed)  # settable


def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # vehicle.
    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


def Box2Send(xyxy_best, serialObj, x, y):
    xmin = int(xyxy_best[0])
    ymin = int(xyxy_best[1])
    xmax = int(xyxy_best[2])
    ymax = int(xyxy_best[3])
    centre_point_x = (xmin + xmax) / 2
    centre_point_y = (ymin + ymax) / 2
    # print(type(xmin))
    width_x = int((xmax - xmin) / x)
    width_y = int((ymax - ymin) / y)
    # width_x = int(torch.round((xmax-xmin)/ x))
    # width_y = int(torch.round((ymax-ymin)/ y))

    left = int(centre_point_x * 100 / x)
    up = int(centre_point_y * 100 / y)
    # left = int(torch.round(centre_point_x*100 / x))#centre_point_x - (x/2)
    # up = int(torch.round(centre_point_y*100 / y))#centre_point_y - (y/2)

    # width_x,width_y
    # SendItem=str(left)+"&"+str(up)+"&"+str(width_x)+"&"+str(width_y)
    # ArduinoSent(left,up,width_x,width_y,serialObj)
    # # Write data to the USB port
    # dev.write(1, b'Hello, World!')
    # serialObj.write(SendItem.encode('UTF-8')) 
