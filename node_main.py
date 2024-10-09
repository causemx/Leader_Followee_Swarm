import socket
import time
import threading
from enum import Enum
from typing import List

import control_func


class DroneHeader(Enum):
    TURNED = 1
    KEEP = 0


class DroneFormationMapping(Enum):
    FORMATION_HORIZONTAL = ([0, 0, 0], [-500, 0, 0], [500, 0, 0])
    FORMATION_TRIANGLE = ([0, 0, 0], [-500, -500, 0], [500, -500, 0])
    FORMATION_VERTICAL = ([0, 0, 0], [0, -500, 0], [0, 500, 0])


HOST_GATEWAY = "192.168.199.1"
PORT_BINDING = 24551
PORT_SENDING = 24550
BUFFER_SIZE = 1024


class DroneNode:
    def __init__(self):
        self.socket = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP
        )
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.follow_mode = False
        self.drone_id = -1
        self.mode = "-1"
        self.leader_position = [" "] * 4
        self.leader_last_position = [" "] * 4
        self.current_formation = DroneFormationMapping.FORMATION_HORIZONTAL

    def udp_listener(self):
        self.socket.bind(("", PORT_BINDING))
        while True:
            try:
                data, _ = self.socket.recvfrom(BUFFER_SIZE)
                self.process_command(data.decode())
            except Exception as e:
                print(f"Error in UDP listener: {e}")

    def process_command(self, data: str):
        command = data.split()
        if not command:
            return

        cmd_type = command[0]
        if cmd_type == "takeoff":
            control_func.auto_take_off()
        elif cmd_type == "land":
            control_func.set_mode_apm("LAND")
        elif cmd_type == "line" and len(command) > 1 and command[1] == "up":
            self.handle_line_up()
        elif cmd_type == "mode" and len(command) > 1:
            control_func.set_mode_apm(command[1])
        elif cmd_type == "follow" and len(command) > 1:
            # self.follow_mode = (command[1] == "on")
            self.handle_follow_command(command)
        elif cmd_type == "leaderPos" and len(command) > 4:
            self.update_leader_position(command[1:5])

    def handle_follow_command(self, command):
        """
        Handle the "follow" command with an optional formation parameter:
        
        follow on [formation_type]
        
        formation_type:
        0 for FORMATION_HORIZONTAL (default)
        1 for FORMATION_TRIANGLE
        2 for FORMATION_VERTICAL 
        
        If no formation_type is provided, FORMATION_HORIZONTAL is used.
        """
        if len(command) > 1:
            self.follow_mode = command[1] == "on"
            
            if self.follow_mode:
                if len(command) > 2:
                    formation_type = int(command[2])
                    if formation_type == 0:
                        self.current_formation = DroneFormationMapping.FORMATION_HORIZONTAL
                    elif formation_type == 1:
                        self.current_formation = DroneFormationMapping.FORMATION_TRIANGLE
                    elif formation_type == 2:
                        self.current_formation = DroneFormationMapping.FORMATION_VERTICAL
                    else:
                        print(f"Invalid formation type: {formation_type}. Using default formation (HORIZONTAL).")
                        self.current_formation = DroneFormationMapping.FORMATION_HORIZONTAL
                else:
                    print("No formation type specified. Using default formation (HORIZONTAL).")
                    self.current_formation = DroneFormationMapping.FORMATION_HORIZONTAL
            else:
                print("Follow mode turned off.")
        else:
            print("Invalid follow command. Usage: follow [on|off] [formation_type]")

    def handle_line_up(self):
        if (
            all(pos != " " for pos in self.leader_position + self.leader_last_position)
            and self.drone_id != 1
        ):
            control_func.set_yaw(float(self.leader_position[3]), 1, 1, 0)
            if self.follow_mode:
                self.set_formation_position()

    def update_leader_position(self, new_position: List[str]):
        self.leader_position = new_position
        if " " in self.leader_last_position:
            self.leader_last_position = self.leader_position
        elif self.drone_id != 1 and self.follow_mode and self.should_update_position():
            self.set_formation_position()
        self.leader_last_position = self.leader_position

    def should_update_position(self) -> bool:
        return (
            control_func.checkDis(self.leader_position, self.leader_last_position) > 0.8
            or abs(float(self.leader_position[2]) - float(self.leader_last_position[2]))
            > 0.8
            or abs(float(self.leader_position[3]) - float(self.leader_last_position[3]))
            > 10
        )

    def set_formation_position(self):
        formation = DroneFormationMapping.FORMATION.value[self.drone_id - 1]
        control_func.set_target_position_baseline(
            self.leader_position[0],
            self.leader_position[1],
            formation[0],
            formation[1],
            str(float(self.leader_position[2]) + formation[2]),
            self.leader_position[3],
            DroneHeader.TURNED.value,
        )

    def responser_loop(self):
        while True:
            try:
                res_data = f"{self.drone_id},{int(self.follow_mode)}"
                stat_data = control_func.responses(str(self.drone_id))
                if stat_data:
                    self.mode = stat_data.split(",")[10]
                    res_data += stat_data
                    print(res_data)
                    self.socket.sendto(
                        res_data.encode("utf-8"), (HOST_GATEWAY, PORT_SENDING)
                    )
                time.sleep(1)
            except Exception as e:
                print(f"Error in responser loop: {e}")

    def initialize_drone_id(self):
        while self.drone_id == -1 or not self.is_valid_ip():
            myhost = control_func.get_lan_ip()
            print(myhost)
            self.drone_id = int(myhost.split(".")[3])
            time.sleep(1)

    def is_valid_ip(self) -> bool:
        ip = control_func.get_lan_ip().split(".")
        return ip[0] == "192" and ip[1] == "168"

    def run(self):
        self.initialize_drone_id()

        udp_service = threading.Thread(target=self.udp_listener, daemon=True)
        responser = threading.Thread(target=self.responser_loop, daemon=True)

        udp_service.start()
        responser.start()

        udp_service.join()
        responser.join()


if __name__ == "__main__":
    drone_node = DroneNode()
    drone_node.run()
