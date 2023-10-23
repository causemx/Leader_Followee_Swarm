#from pymavlink import mavutil
import binascii
import math
import numpy as np
#from deprecated import deprecated


class MissionUtil(object):
    def __init__(self, protocol, conn_ip, conn_port):
        #self.master = mavutil.mavlink_connection(':'.join([protocol, conn_ip, conn_port]))

        # set mode by integer mode number for ArduPilot
        self.mode_mapping_acm = {
            0: 'STABILIZE',
            1: 'ACRO',
            2: 'ALT_HOLD',
            3: 'AUTO',
            4: 'GUIDED',
            5: 'LOITER',
            6: 'RTL',
            7: 'CIRCLE',
            8: 'POSITION',
            9: 'LAND',
            10: 'OF_LOITER',
            11: 'DRIFT',
            13: 'SPORT',
            14: 'FLIP',
            15: 'AUTOTUNE',
            16: 'POSHOLD',
            17: 'BRAKE',
            18: 'THROW',
            19: 'AVOID_ADSB',
            20: 'GUIDED_NOGPS',
            21: 'SMART_RTL',
            22: 'FLOWHOLD',
            23: 'FOLLOW',
            24: 'ZIGZAG',
        }

    def __cal_quadrant(self, x_axis, y_axis):
        # type: (int, int) -> int
        if x_axis > 0 and y_axis > 0:
            return 1
        elif x_axis < 0 < y_axis:
            return 2
        elif x_axis < 0 and y_axis < 0:
            return 3
        elif x_axis > 0 > y_axis:
            return 4
        elif x_axis == 0 and y_axis > 0:
            return 5
        elif x_axis < 0 and y_axis == 0:
            return 6
        elif x_axis == 0 and y_axis < 0:
            return 7
        elif x_axis > 0 and y_axis == 0:
            return 8

    def cal_final_angle(self, x_axis, y_axis):
        quadrant = self.__cal_quadrant(x_axis, y_axis)
        final_angle = -1
        if quadrant == 1:
            angle = np.rad2deg(math.atan2(abs(y_axis), abs(x_axis)))
            final_angle = 90 - angle
        elif quadrant == 2:
            angle = np.rad2deg(math.atan2(abs(y_axis), abs(x_axis)))
            final_angle = 270 + angle
        elif quadrant == 3:
            angle = np.rad2deg(math.atan2(abs(y_axis), abs(x_axis)))
            final_angle = 270 - angle
        elif quadrant == 4:
            angle = np.rad2deg(math.atan2(abs(y_axis), abs(x_axis)))
            final_angle = 90 + angle
        elif quadrant == 5:
            final_angle = 0
        elif quadrant == 6:
            final_angle = 270
        elif quadrant == 7:
            final_angle = 180
        elif quadrant == 8:
            final_angle = 90
        return final_angle

    def location(self, lat, lng, brng, dist):
        a_1 = 6378137
        b = 6356752.3142
        f = 1 / 298.257223563
        alpha1 = np.deg2rad(brng)
        sin_alpha1 = math.sin(alpha1)
        cos_alpha1 = math.cos(alpha1)
        tan_u1 = (1 - f) * math.tan(np.deg2rad(lat))
        cos_u1 = 1 / math.sqrt((1 + tan_u1 * tan_u1))
        sin_u1 = tan_u1 * cos_u1
        sigma1 = math.atan2(tan_u1, cos_alpha1)
        sin_alpha = cos_u1 * sin_alpha1
        cos_sq_alpha = 1 - sin_alpha * sin_alpha
        u_sq = cos_sq_alpha * (a_1 * a_1 - b * b) / (b * b)
        a_1 = 1 + u_sq / 16384 * (4096 + u_sq * (-768 + u_sq * (320 - 175 * u_sq)))
        b_1 = u_sq / 1024 * (256 + u_sq * (-128 + u_sq * (74 - 47 * u_sq)))
        sigma = dist / (b * a_1)
        sigma_p = 2 * math.pi

        cos2_sigma_m = -1
        sin_sigma = -1
        cos_sigma = -1
        while abs(sigma - sigma_p) > 0.000000000001:
            cos2_sigma_m = math.cos(2 * sigma1 + sigma)
            sin_sigma = math.sin(sigma)
            cos_sigma = math.cos(sigma)
            delta_sigma = b_1 * sin_sigma * (cos2_sigma_m + b_1 / 4 * (
                    cos_sigma * (-1 + 2 * cos2_sigma_m * cos2_sigma_m) - b_1 / 6 * cos2_sigma_m * (
                    -3 + 4 * sin_sigma * sin_sigma) * (-3 + 4 * cos2_sigma_m * cos2_sigma_m)))
            sigma_p = sigma
            sigma = dist / (b * a_1) + delta_sigma

        tmp = sin_u1 * sin_sigma - cos_u1 * cos_sigma * cos_alpha1
        lat_end = math.atan2(sin_u1 * cos_sigma + cos_u1 * sin_sigma * cos_alpha1,
                             (1 - f) * math.sqrt(sin_alpha * sin_alpha + tmp * tmp))
        lamb_da = math.atan2(sin_sigma * sin_alpha1, cos_u1 * cos_sigma - sin_u1 * sin_sigma * cos_alpha1)
        c_1 = f / 16 * cos_sq_alpha * (4 + f * (4 - 3 * cos_sq_alpha))
        l_1 = lamb_da - (1 - c_1) * f * sin_alpha * (
                sigma + c_1 * sin_sigma * (cos2_sigma_m + c_1 * cos_sigma * (-1 + 2 * cos2_sigma_m * cos2_sigma_m)))
        lng_end = math.fmod((np.deg2rad(lng) + l_1 + 3 * math.pi), 2 * math.pi) - math.pi
        # return str(np.rad2deg(lat_end)) + "," + str(np.rad2deg(lng_end))
        return ['{:.7f}'.format(round(np.rad2deg(lat_end), 7)), '{:.7f}'.format(round(np.rad2deg(lng_end), 7))]

    #@deprecated(version='1.0', reason="You should use the cal_latlng_by_baseline function.")
    def cal_latlng_based_on_center(self, x_axis, y_axis, z_axis, center_lat, center_lng, shift_deg):
        latlng = None
        shift_deg = -(shift_deg)
        x_axis_temp = x_axis
        y_axis_temp = y_axis
        x_axis = math.cos(shift_deg * math.pi / 180) * x_axis_temp - math.sin(shift_deg * math.pi / 180) * y_axis_temp
        y_axis = math.sin(shift_deg * math.pi / 180) * x_axis_temp + math.cos(shift_deg * math.pi / 180) * y_axis_temp
        if x_axis == 0 and y_axis == 0:
            final_angle = 0
        else:
            final_angle = self.cal_final_angle(x_axis, y_axis)

        dist = np.sqrt(pow(x_axis, 2) + pow(y_axis, 2)) / 100
        new_point = self.location(center_lat, center_lng, final_angle, dist)
        new_lat = new_point[0]
        new_lng = new_point[1]
        latlng = [float(new_lat), float(new_lng)]
        return latlng

    def cal_latlng_by_baseline(self, x_axis, y_axis, baseline_lat_lng, shift_deg):
        """
        Calculate the lat and lng by given x, y, and base point.
        Args:
            x_axis: the drone's x axis. eg. -3
            y_axis: the drone's y axis. eg. 5
            baseline_lat_lng: base line drone's lat and lng in an array. eg. [22.9937223, 120.1503446]

        Returns: the drone's lat and lng in an array. eg. [22.9937228, 120.1503443]

        """
        return self.cal_latlng_based_on_center(x_axis, y_axis, 0, baseline_lat_lng[0], baseline_lat_lng[1], shift_deg)


    def inverse_map(self, map):
        inv_map = dict((a, b) for (b, a) in map.items())
        return inv_map

    def hex_of_set_global_relative_alt_int(self, target_system, target_component, latlng, alt):
        """

        Args:
            target_system: int
            target_component: int
            latlng: [float, float] 7 decimal places each.
            alt: int in meter.

        Returns:
            str
        """
        time_boot_ms = 0
        coordinate_frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT  # (3rd position in reverse order.
        # coordinate_frame = mavutil.mavlink.MAV_FRAME_GLOBAL_INT  # MSL
        type_mask = 511  # (5th and 6th position in reverse order.)

        # latlng = [22.9949053, 120.1480163]
        if latlng is None:
            print("set latlng to zeros")
            latlng = [0, 0]
        else:
            type_mask = type_mask & 504
            # print("found latlon", type_mask)
        vN = 0
        vE = 0
        vD = 0
        # if 1:
        #     vN = float(10.0)
        #     vE = float(10.0)
        #     vD = float(10.0)
        #     type_mask = type_mask & 455
        #     print("type_mask=", type_mask)

        msgObj = self.master.mav.set_position_target_global_int_encode(
            # msgObj = master.mav.set_position_target_global_int_send(
            time_boot_ms, target_system, target_component, coordinate_frame,
            type_mask,
            int(latlng[0] * 1e7),
            int(latlng[1] * 1e7),
            alt,
            vN, vE, vD,  # velocity
            0, 0, 0,  # accel x,y,z
            0, 0  # yaw, yaw rate
        )
        buf = msgObj.pack(self.master.mav, True)
        # print(buf)
        bufStrWoSpaces = binascii.hexlify(bytearray(buf)).upper()
        # print(bufStrWoSpaces)
        bufStrWiSpaces = ' '.join([i + j for i, j in zip(bufStrWoSpaces[::2], bufStrWoSpaces[1::2])])
        # print(bufStrWiSpaces)
        return bufStrWiSpaces
        # mavHex = bytearray.fromhex(bufStrWiSpaces)
        # print(mavHex)

        # relative alt: FE 35 00 FF 00 56 00 00 00 00 7D BE B4 0D E3 21 9D 47 00 00 C8 42 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 F8 01 01 01 06 63 3C
        # MSL:          FE 35 00 FF 00 56 00 00 00 00 7D BE B4 0D E3 21 9D 47 00 00 C8 42 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 F8 01 01 01 05 0B 16

    def checksum_custom(self, packet):
        # print packet
        a = 0x00
        b = 0x00

        for in_dat in packet:
            a += in_dat
            b += a

        ans = [bytes, bytes]
        ans[0] = a & 0xFF
        ans[1] = b & 0xFF

        return ans
