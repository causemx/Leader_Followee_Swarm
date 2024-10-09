import socket
import time
import os
import threading
import sys
from decimal import Decimal
from typing import Dict, Callable

import control_func

# Constants
STATUS_PATH = r"/home/pi/status.txt"
HOST_GATEWAY = "192.168.199.255"
PORT_BINDING = 24550
PORT_SENDING = 24551
BUFFER_SIZE = 1024

# Socket setup
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

STATUS_KEYS = [
    'GPS_RAW_INT-lat',
    'GPS_RAW_INT-lon',
    'GLOBAL_POSITION_INT-relative_alt',
    'GLOBAL_POSITION_INT-hdg'
]

# Subkey handlers
SUBKEY_HANDLERS: Dict[str, Callable[[str], str]] = {
    "lat": lambda x: str(Decimal(x) / Decimal(1e7)),
    "lon": lambda x: str(Decimal(x) / Decimal(1e7)),
    "relative_alt": lambda x: str(Decimal(x) / Decimal(1000)),
    "hdg": lambda x: str(Decimal(x) / Decimal(100)),
}

def udp_listener():
    s.bind(("", PORT_BINDING))
    while True:
        try:
            data, addr = s.recvfrom(BUFFER_SIZE)
            print(data.decode())
        except Exception as e:
            print(f"Error in UDP listener: {e}")

def cmd_sending(cmd: str):
    print(f"Sending command: {cmd}")
    s.sendto(cmd.encode('utf-8'), (HOST_GATEWAY, PORT_SENDING))

def broadcast_pos_loop():
    while True:
        leader_status = responses()
        if leader_status != "":
            pos_data = f"leaderPos{leader_status}"
            cmd_sending(pos_data)
        time.sleep(0.1)

def responses() -> str:
    cust_status = control_func.GetStatus()
    
    if not os.path.isfile(STATUS_PATH):
        return "Status file not found"

    try:
        status = cust_status.readContent(STATUS_PATH)
        if not status:
            return "Status is empty"

        result = []
        for status_key in STATUS_KEYS:
            entry, subkey = status_key.split('-')
            value = cust_status.get_status(entry, subkey, status)
            handler = SUBKEY_HANDLERS.get(subkey, lambda x: x)
            result.append(handler(value))

        return ' '.join(result)

    except Exception as e:
        print(f"Error processing status: {e}")
        return "Error occurred while processing status"

def cmd_module(args: str) -> None:
    elements = args.split()
    
    if not elements:
        print("Error: No command provided")
        return

    command = elements[0]
    params = elements[1:]

    command_handlers: Dict[str, Callable] = {
        "takeoff": lambda: cmd_sending("takeoff"),
        "land": lambda: cmd_sending("land"),
        "mode": handle_mode,
        "line": handle_line,
        "follow": handle_follow
    }

    try:
        handler = command_handlers.get(command)
        if handler:
            handler(*params)
        else:
            print(f"Error: Unknown command '{command}'")
    except Exception as e:
        print(f"Error executing command '{command}': {str(e)}")

def handle_mode(mode: str) -> None:
    mode_map = control_func.mode_mapping()
    if not mode_map or mode not in mode_map:
        print(f"Error: Unknown mode '{mode}'")
    else:
        cmd_sending(f"mode {mode}")

def handle_line(direction: str) -> None:
    if direction == "up":
        cmd_sending("line up")
    else:
        print(f"Error: Unknown line direction '{direction}'")

def handle_follow(state: str) -> None:
    if state in ["on", "off"]:
        cmd_sending(f"follow {state}")
    else:
        print(f"Error: Invalid follow state '{state}'")

def main():
    udp_service = threading.Thread(target=udp_listener, daemon=True)
    pos_service = threading.Thread(target=broadcast_pos_loop, daemon=True)

    udp_service.start()
    pos_service.start()

    while True:
        try:
            inputdata = input("[*] GatewayCMD >> ")
            cmd_module(inputdata)
        except KeyboardInterrupt:
            print("\nExiting...")
            sys.exit(0)

if __name__ == "__main__":
    main()