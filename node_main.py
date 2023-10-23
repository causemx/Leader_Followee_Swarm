import socket,time
import control_func
from enum import Enum
import threading

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

class DRONE_HEADER(Enum):
    TURNED = 1
    KEEP = 0

global FollowMode
FollowMode = 0
global drone_ID
drone_ID = -1
global mode
mode = "-1"


"""
a_formation[0] = LEADER
a_formation[1] = FOLLOWEE1
a_formation[2] = FOLLOWEE2
format: [x, y, z]
"""

class DroneFormationMapping(Enum):
    a_formation = ([0,0,0],[-300,-300,0],[300,-300,0])

def udp_listener(drone_ID):
    global FollowMode
    leader_lat_lng_alt_yaw = [" "," "," "," "] # [22.9949053, 120.148016, 5.41, 180.88]
    leader_last_lat_lng_alt_yaw = [" "," "," "," "]

    s.bind(("",24551))
    while True :
        try:
            data,addr=s.recvfrom(1024)
            data = data.decode()
            if data.split(' ')[0] == "takeoff":
                control_func.auto_take_off()
            if data.split(' ')[0] == "land":
                control_func.set_mode_apm("LAND")
            if data.split(' ')[0] == "line":
                if data.split(' ')[1] == "up":
                    if " " not in leader_lat_lng_alt_yaw and " " not in leader_last_lat_lng_alt_yaw:
                        if drone_ID != 1:
                            control_func.set_yaw(leader_lat_lng_alt_yaw[3],1,1,0)
                            if FollowMode == 1:
                                control_func.set_target_position_baseline(leader_lat_lng_alt_yaw[0], leader_lat_lng_alt_yaw[1] #leader lat,lng
                                                                    , DroneFormationMapping.a_formation.value[drone_ID - 1][0], DroneFormationMapping.a_formation.value[drone_ID - 1][1] #formation x,y
                                                                    , str(float(leader_lat_lng_alt_yaw[2]) + DroneFormationMapping.a_formation.value[drone_ID - 1][2]) #leader alt + fomation z
                                                                    , leader_lat_lng_alt_yaw[3] #leader yaw
                                                                    , DRONE_HEADER.TURNED.value) #header lock or not

            if data.split(' ')[0] == "mode":
                control_func.set_mode_apm(data.split(' ')[1])
            if data.split(' ')[0] == "follow":
                if data.split(' ')[1] == "on":
                    FollowMode = 1
                elif data.split(' ')[1] == "off":
                    FollowMode = 0
            if data.split(' ')[0] == "leaderPos":
                leader_lat_lng_alt_yaw[0] = data.split(' ')[1]
                leader_lat_lng_alt_yaw[1] = data.split(' ')[2]
                leader_lat_lng_alt_yaw[2] = data.split(' ')[3]
                leader_lat_lng_alt_yaw[3] = data.split(' ')[4]
                if " " in leader_last_lat_lng_alt_yaw:
                    leader_last_lat_lng_alt_yaw = leader_lat_lng_alt_yaw
                if " " not in leader_lat_lng_alt_yaw and " " not in leader_last_lat_lng_alt_yaw:
                    if drone_ID != 1:
                        if FollowMode == 1 and (control_func.checkDis(leader_lat_lng_alt_yaw, leader_last_lat_lng_alt_yaw) > 0.8 or abs(float(leader_lat_lng_alt_yaw[2]) - float(leader_last_lat_lng_alt_yaw[2])) > 0.8 or abs(float(leader_lat_lng_alt_yaw[3]) - float(leader_last_lat_lng_alt_yaw[3])) > 10):
                            control_func.set_target_position_baseline(leader_lat_lng_alt_yaw[0], leader_lat_lng_alt_yaw[1] #leader lat,lng
                                                                , DroneFormationMapping.a_formation.value[drone_ID - 1][0], DroneFormationMapping.a_formation.value[drone_ID - 1][1] #formation x,y
                                                                , str(float(leader_lat_lng_alt_yaw[2]) + DroneFormationMapping.a_formation.value[drone_ID - 1][2]) #leader alt + fomation z
                                                                , leader_lat_lng_alt_yaw[3] #leader yaw
                                                                , DRONE_HEADER.TURNED.value) #header lock or not
                leader_last_lat_lng_alt_yaw = leader_lat_lng_alt_yaw
        except :
            continue

def responser_loop():
    global FollowMode,drone_ID,mode
    while True :
        try :
            res_data = str(drone_ID) + "," + str(FollowMode)
            stat_data = control_func.responses(str(drone_ID))
            if stat_data != "":
                mode = stat_data.split(',')[10]
                res_data += stat_data
                print (res_data)
                s.sendto(res_data.encode('utf-8'),("192.168.199.1",24550))
            time.sleep(1)
        except :
            continue

while drone_ID == -1 or (myhost.split('.')[0] != '192' and myhost.split('.')[1] != '168'):
    myhost = control_func.get_lan_ip()
    print (myhost)
    fourth_str = myhost.split('.')[3]
    drone_ID = int(fourth_str)
    time.sleep(1)

udp_service = threading.Thread(target = udp_listener, args = (drone_ID,))
udp_service.start()
udp_service.join(2)
time.sleep(0.2)

responser = threading.Thread(target = responser_loop)
responser.start()
responser.join()