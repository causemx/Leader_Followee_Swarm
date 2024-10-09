from inspect import isframe
import os
import socket
import fcntl
import struct
import time
import datetime
import numbers
from MissionUtil import MissionUtil
import numpy as np
#from pymavlink import mavutil
from v20 import ardupilotmega
import binascii

from numpy import radians, sin, cos, arcsin, sqrt

import json

modname = ardupilotmega.MAVLink(0,0,0,False)
mavlink = ardupilotmega

#from Status import *


host = "127.0.0.1"
port = 14553
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

global message,message_end,drone_status
message = ' '
message_end = ' '
drone_status = ""

statusKeyWithSubkeys = [
    'SYS_STATUS-voltage_battery'
    ,'GPS_RAW_INT-satellites_visible'
    ,'GPS_RAW_INT-fix_type'
    ,'GPS_RAW_INT-lat'
    ,'GPS_RAW_INT-lon'
    ,'EKF_STATUS_REPORT-compass_variance'           # 10
    ,'GLOBAL_POSITION_INT-alt'
    ,'GLOBAL_POSITION_INT-relative_alt'             # 20
    ,'SYS_STATUS-onboard_control_sensors_enabled'   # safety switch
    ,'HEARTBEAT-custom_mode'                        #5 loit, guide, stabilize etc.
    ,'EKF_STATUS_REPORT-flags'
    ,'GLOBAL_POSITION_INT-hdg'                      #yaw
]

class GetStatus(object):

    def readContent(self,absolute_path):
        """
        Set absolute path to status.txt and set raw content parameter of status.txt file.
        Attributes:
            absolute_path (str): The absolute path to the status.txt file.
        """
        rtn = ""
        while 1:
            try:
                fo = open(absolute_path, "r")
                rtn = fo.read()
                fo.close()
                if self.extractStatusValue('VIBRATION', 'clipping_2', rtn) == '':
                    time.sleep(0.2)
                    continue
                else:
                    return rtn
            except Exception as ex:
                return rtn

    def get_status(self,entry,subKey,status):
        entry += ' {'
        tempSubkey = subKey
        tempSubkey += ' : '
        beginIdx = status.find(entry) + len(entry) - 1
        endIdx = status.find('}',status.find(entry)) + 1
        jsonStr = status[beginIdx : endIdx]
        beginIdx = jsonStr.find(tempSubkey) + len(tempSubkey)
        endIdx = jsonStr.find(',', jsonStr.find(tempSubkey))
        value = ''
        if isinstance(endIdx, numbers.Number):
            value = jsonStr[beginIdx : endIdx]
        else:
            endIdx = jsonStr.find('}', jsonStr.find(tempSubkey))
            value = jsonStr[beginIdx : endIdx]
        return value

    def extract_entry_as_dict(self, entry, status_str):
        entry += ' {'
        begin_idx = status_str.find(entry) + len(entry)
        end_idx = status_str.find('}', status_str.find(entry))
        pre_json_str = status_str[begin_idx: end_idx]
        pre_json_str = pre_json_str.replace(" ", "")
        pre_json_arr = pre_json_str.split(",")
        for i, element in enumerate(pre_json_arr):
            if element.find('"') < 0 and element.find("'") < 0:
                pre_json_arr[i] = '"' + element[0:element.find(':')] + '"' + element[element.find(':'):]
        json_str = "{" + ','.join(pre_json_arr) + "}"
        # print datetime.datetime.now(), entry, '->', subKey, '=', value
        return json.loads(json_str)

def get_interface_ip(ifname):
    get_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        get_s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', bytes(os.path.basename(ifname[:15]).encode('utf-8')))
    )[20:24])

def get_lan_ip():
    ip = socket.gethostbyname(socket.gethostname())
    if ip.startswith('127.') and os.name != 'nt':
        interfaces = ["bat0", "wlan0"]
        for ifname in interfaces:
            try:
                ip = get_interface_ip(ifname)
                break
            except:
                pass
    return ip

def check_cmd(parm):
    if drone_status != "":
        if int(drone_status.split(',')[10]) == parm:
            return "success!"
        else:
            return "fail!"
    else:
        return "fail!"

def command_send(cmd_hex):
    #temp = str(bytearray.fromhex(cmd_hex))
    temp = bytearray.fromhex(cmd_hex)
    s.sendto(temp,(host,port))

def receive_from_main(mainstatus,mainstatus_end):
    global message,message_end
    message = mainstatus
    message_end = mainstatus_end

def responses(id):
    global message,message_end,drone_status
    cust_status = GetStatus()
    try:
        res_str = ""
        status_path = r"/home/pi/status.txt"
        if os.path.isfile(status_path):
            status = cust_status.readContent(status_path)
            if status != "":
                for i in range(len(statusKeyWithSubkeys)):
                    entry = statusKeyWithSubkeys[i].split('-')[0]
                    subKey = statusKeyWithSubkeys[i].split('-')[1]
                    if subKey == "compass_variance":
                        res_str += ',' + str(round(float(cust_status.get_status(entry,subKey,status)), 2))
                    else:
                        res_str += ',' + cust_status.get_status(entry,subKey,status)
                now = datetime.datetime.now()
                now_unix = time.mktime(now.timetuple())
                res_str += ',' + str(now_unix)
                #res_str += ',' + message_end
                #print (res_str)
                #print ('\n')
                #s.sendto(res_str,("192.168.199.1",36001))
                #s.sendto(res_str,("192.168.188.201",36001))
            drone_status = id + res_str
    except Exception as e:
        print(e)
    return res_str

def send_CMD(msg):
    bufStrWoSpaces = binascii.hexlify(bytearray(msg)).upper()
    print(bufStrWoSpaces)
    return s.sendto(msg,(host,port))

mode_mapping_acm = {
    0 : 'STABILIZE',
    1 : 'ACRO',
    2 : 'ALT_HOLD',
    3 : 'AUTO',
    4 : 'GUIDED',
    5 : 'LOITER',
    6 : 'RTL',
    7 : 'CIRCLE',
    8 : 'POSITION',
    9 : 'LAND',
    10 : 'OF_LOITER',
    11 : 'DRIFT',
    13 : 'SPORT',
    14 : 'FLIP',
    15 : 'AUTOTUNE',
    16 : 'POSHOLD',
    17 : 'BRAKE',
    18 : 'THROW',
    19 : 'AVOID_ADSB',
    20 : 'GUIDED_NOGPS',
    21 : 'SMART_RTL',
    22 : 'FLOWHOLD',
    23 : 'FOLLOW',
    24 : 'ZIGZAG',
    25 : 'SYSTEMID',
    26 : 'AUTOROTATE',
    27 : 'AUTO_RTL',
}

mode_mapping_sub = {
    0: 'STABILIZE',
    1: 'ACRO',
    2: 'ALT_HOLD',
    3: 'AUTO',
    4: 'GUIDED',
    7: 'CIRCLE',
    9: 'SURFACE',
    16: 'POSHOLD',
    19: 'MANUAL',
}

AP_MAV_TYPE_MODE_MAP = {
    # copter
    mavlink.MAV_TYPE_HELICOPTER:  mode_mapping_acm,
    mavlink.MAV_TYPE_TRICOPTER:   mode_mapping_acm,
    mavlink.MAV_TYPE_QUADROTOR:   mode_mapping_acm,
    mavlink.MAV_TYPE_HEXAROTOR:   mode_mapping_acm,
    mavlink.MAV_TYPE_OCTOROTOR:   mode_mapping_acm,
    mavlink.MAV_TYPE_DECAROTOR:   mode_mapping_acm,
    mavlink.MAV_TYPE_DODECAROTOR: mode_mapping_acm,
    mavlink.MAV_TYPE_COAXIAL:     mode_mapping_acm,
    # sub
    mavlink.MAV_TYPE_SUBMARINE: mode_mapping_sub,
}

def mode_mapping_byname(mav_type):
    '''return dictionary mapping mode names to numbers, or None if unknown'''
    mode_map = mode_mapping_bynumber(mav_type)
    if mode_map is None:
        return None
    inv_map = dict((a, b) for (b, a) in mode_map.items())
    return inv_map

def mode_mapping_bynumber(mav_type):
    '''return dictionary mapping mode numbers to name, or None if unknown'''
    return AP_MAV_TYPE_MODE_MAP[mav_type] if mav_type in AP_MAV_TYPE_MODE_MAP else None

def mode_mapping():
    mav_type = mavlink.MAV_TYPE_QUADROTOR
    return mode_mapping_byname(mav_type)

def set_mode_apm(mode, custom_mode = 0, custom_sub_mode = 0):
    '''enter arbitrary mode'''
    if isinstance(mode, str):
        mode_map = mode_mapping()
        if mode_map is None or mode not in mode_map:
            print("Unknown mode '%s'" % mode)
            return
        mode = mode_map[mode]
    # set mode by integer mode number for ArduPilot
    msg = modname.command_long_encode(1,
                                0,
                                mavlink.MAV_CMD_DO_SET_MODE,
                                0,
                                mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                mode,
                                0,
                                0,
                                0,
                                0,
                                0)
    buf = msg.pack(modname, False)
    send_CMD(buf)
    set_result = "fail!"
    for i in range(3):
        set_result = check_cmd(mode)
        if set_result == "success!":
            break
        elif set_result == "fail!":
            continue
        time.sleep(1)
    return "set mode " + set_result

def arducopter_arm():
    '''arm motors (arducopter only)'''
    msg = modname.command_long_encode(
        1,  # target_system
        0,
        mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
        0, # confirmation
        1, # param1 (1 to indicate arm)
        0, # param2 (all other params meaningless)
        0, # param3
        0, # param4
        0, # param5
        0, # param6
        0) # param7
    buf = msg.pack(modname, False)
    send_CMD(buf)
    return "sent arm success!"

def arducopter_disarm():
    '''disarm motors (arducopter only)'''
    msg = modname.command_long_encode(
        1, # target_system
        0,
        mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
        0, # confirmation
        0, # param1 (0 to indicate disarm)
        0, # param2 (all other params meaningless)
        0, # param3
        0, # param4
        0, # param5
        0, # param6
        0) # param7
    buf = msg.pack(modname, False)
    send_CMD(buf)
    return "sent disarm success!"

def take_off_CMD(alt):
    altitude = float(alt)
    msg = modname.command_long_encode(0, 0, mavlink.MAV_CMD_NAV_TAKEOFF,
                                    0, 0, 0, 0, 0, 0, 0, altitude)
    buf = msg.pack(modname, False)
    send_CMD(buf)
    return "sent take off success!"

def auto_take_off():
    set_mode_apm("GUIDED")
    arducopter_arm()
    time.sleep(1)
    arducopter_arm()
    time.sleep(3)
    take_off_CMD(5)
    return "sent take off success!"

import enum
class DRONE_YAW_DIRECTION(enum.IntEnum):
    COUNTER_CLOCKWISE = -1
    CLOCKWISE = 1

class DRONE_YAW_RELATIVE(enum.IntEnum):
    ABSOLUTE_ANGLE = 0
    RELATIVE_OFFECT = 1

def set_yaw(angle, speed, direction, relative):
    deg = float(angle)
    spd = float(speed)
    direct = float(direction)
    rela = float(relative)
    msg = modname.command_long_encode(
        1, # target_system
        0, 
        mavlink.MAV_CMD_CONDITION_YAW, #command
        0, # confirmation
        deg, # param1 (target angle, 0 is north)
        spd, # param2 (angular speed)
        direct, # param3 (direction: -1: counter clockwise, 1: clockwise)
        rela, # param4 (0: absolute angle, 1: relative offset)
        0, # param5
        0, # param6
        0) # param7
    buf = msg.pack(modname, False)
    send_CMD(buf)
    return "sent condition yaw success!"

class DRONE_HEADER(enum.IntEnum):
    TURNED = 1
    KEEP = 0

def set_position_target_CMD_encode(type_mask, lat, lon, altitude, yaw, yaw_rate):
    msg = modname.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        1, 0,    # target system, target component
        mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        type_mask, # type_mask (only speeds enabled)
        lat, lon,# x, y position in WGS84 frame [degE7] (type:int32_t)
        altitude,# altitude in m (type:float)
        0, 0, 0, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        yaw, yaw_rate)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    buf = msg.pack(modname, False)
    send_CMD(buf)
    return "sent target CMD success!"

def set_target_position_baseline(begin_lat, begin_lng, x_axis, y_axis, alt, leader_yaw, head_frame):
    mission_util = MissionUtil('udpout', '127.0.0.1', '14552')
    baseline_lat_lng = [999,999]
    baseline_lat_lng[0] = float(begin_lat)
    baseline_lat_lng[1] = float(begin_lng)
    altitude = float(alt)
    #shift_deg = int(drone_status.split(',')[12]) * 0.01
    shift_deg = float(leader_yaw)
    print("deg from leader = " + str(shift_deg))
    latlng = mission_util.cal_latlng_by_baseline(x_axis, y_axis, baseline_lat_lng, shift_deg)
    lat = int(latlng[0] * 1e7)
    lon = int(latlng[1] * 1e7)
    print("lat,lon=" + str(latlng[0]) + "," + str(latlng[1]))
    if head_frame == DRONE_HEADER.TURNED.value:
        type_mask = 0b0000111111111000
        yaw = 0
        yaw_rate = 0
    elif head_frame == DRONE_HEADER.KEEP.value:
        type_mask = 0b0000001111111000
        yaw = np.deg2rad(float(shift_deg))
        yaw_rate = 5
    return set_position_target_CMD_encode(type_mask, lat, lon, altitude, yaw, yaw_rate)

def dis_cal(d_lon,d_lat,k):
    aa = sin(d_lat / 2) ** 2 + k * sin(d_lon / 2) ** 2
    bb = sqrt(aa)
    c = 2 * arcsin(bb)
    return c

def checkDis(pos1, pos2):
    lat1 = float(pos1[0])
    lon1 = float(pos1[1])
    lat2 = float(pos2[0])
    lon2 = float(pos2[1])
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
    d_lon = lon2 - lon1
    d_lat = lat2 - lat1
    k = cos(lat1) * cos(lat2)
    c = dis_cal(d_lon,d_lat,k)  
    r = 6371
    return c * r * 1000

#reply = threading.Thread(target = responses, daemon=True)
#reply.start()
#time.sleep(2)


#test code
"""
set_mode_apm("GUIDED")
arducopter_arm()
arducopter_disarm()
take_off(5)
while True:
    data = input("please input [x,y] : ") #forward y   back -y   right x   left -x
    print(data.split(',')[0])
    print(data.split(',')[1])
    set_yaw(int(data.split(',')[0]),int(data.split(',')[1]),DRONE_YAW_DIRECTION.CLOCKWISE,DRONE_YAW_RELATIVE.ABSOLUTE_ANGLE)
    set_yaw(int(data.split(',')[0]),int(data.split(',')[1]),DRONE_YAW_DIRECTION.COUNTER_CLOCKWISE,DRONE_YAW_RELATIVE.ABSOLUTE_ANGLE)
    set_yaw(int(data.split(',')[0]),int(data.split(',')[1]),DRONE_YAW_DIRECTION.CLOCKWISE,DRONE_YAW_RELATIVE.RELATIVE_OFFECT)
    set_yaw(int(data.split(',')[0]),int(data.split(',')[1]),DRONE_YAW_DIRECTION.COUNTER_CLOCKWISE,DRONE_YAW_RELATIVE.RELATIVE_OFFECT)
    if data.split(',')[3] == '1':
        set_target_position(int(data.split(',')[0]),int(data.split(',')[1]),int(data.split(',')[2]),DRONE_HEADER.TURNED)
    elif data.split(',')[3] == '2':
        set_target_position(int(data.split(',')[0]),int(data.split(',')[1]),int(data.split(',')[2]),DRONE_HEADER.KEEP)
"""
#reply.join()