import socket,time,os
import threading
import sys
import control_func
from decimal import *


s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

statusKeyWithSubkeys = [
    'GPS_RAW_INT-lat'
    ,'GPS_RAW_INT-lon'
    ,'GLOBAL_POSITION_INT-relative_alt'             # 20
    ,'GLOBAL_POSITION_INT-hdg'                      #yaw
]

def udp_listener():
    s.bind(("",24550))
    while True :
        try :
            data,addr=s.recvfrom(1024)
            data = data.decode()
            print(data)
        except :
            continue

def cmd_sending(cmd):
    s.sendto(cmd.encode('utf-8'),("192.168.199.255",24551))

def boardcast_pos_loop():
    while True :
        pos_data = "leaderPos"
        leader_status = responses()
        if leader_status != "":
            pos_data += leader_status
            cmd_sending(pos_data)
        time.sleep(0.1)

def responses():
    cust_status = control_func.GetStatus()
    try:
        res_str = ""
        status_path = r"/home/pi/status.txt"
        if os.path.isfile(status_path):
            status = cust_status.readContent(status_path)
            if status != "":
                for i in range(len(statusKeyWithSubkeys)):
                    entry = statusKeyWithSubkeys[i].split('-')[0]
                    subKey = statusKeyWithSubkeys[i].split('-')[1]
                    if subKey == "lat" or subKey == "lon" :
                        temp = Decimal(cust_status.get_status(entry,subKey,status)) / Decimal(1e7)
                        res_str += ' ' + str(temp)
                    elif subKey == "relative_alt" :
                        temp = Decimal(cust_status.get_status(entry,subKey,status)) / Decimal(1000)
                        res_str += ' ' + str(temp)
                    elif subKey == subKey == "hdg" :
                        temp = Decimal(cust_status.get_status(entry,subKey,status)) / Decimal(100)
                        res_str += ' ' + str(temp)
    except Exception as e:
        print(e)
    return res_str

def cmd_moudle(args):
    elem = args.split(' ')
    try :
        if len(elem) == 1:
            if elem[0] == "takeoff":
                cmd_sending("takeoff")
            elif elem[0] == "land":
                cmd_sending("land")
        elif len(elem) > 1:
            if elem[0] == "mode":
                mode = elem[1]
                #cmd_sending("mode " + mode)
                if isinstance(mode, str):
                    mode_map = control_func.mode_mapping()
                    if mode_map is None or mode not in mode_map:
                        print("Unknown mode '%s'" % mode)
                    else:
                        cmd_sending("mode " + mode)
            elif elem[0] == "line" and elem[1] == "up":
                cmd_sending("line up")
            elif elem[0] == "follow":
                if elem[1] == "on":
                    cmd_sending("follow on")
                elif elem[1] == "off":
                    cmd_sending("follow off")
    except :
        pass

udp_service = threading.Thread(target = udp_listener)
udp_service.start()
udp_service.join(2)
time.sleep(0.2)

pos_service = threading.Thread(target = boardcast_pos_loop)
pos_service.start()
pos_service.join(2)
time.sleep(0.2)

while True:
    try:
        inputdata = input("GatewayCMD > ")
        cmd_moudle(inputdata)
    except KeyboardInterrupt:
        sys.exit(0)