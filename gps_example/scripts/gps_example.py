#! /usr/bin/env python3
import numpy as np
import sys
import rclpy
from rclpy.node import Node


class Enu:
    A = 6378137.0
    E2 = 6.6943799014e-3

    @staticmethod
    def llh_to_ecef(lat_deg: float, lon_deg: float, alt: float) -> np.array:
        lat = np.radians(lat_deg)
        lon = np.radians(lon_deg)
        n_val = Enu.A / np.sqrt(1.0 - Enu.E2 * np.sin(lat) * np.sin(lat))
        return np.array([
            (n_val + alt) * np.cos(lat) * np.cos(lon),
            (n_val + alt) * np.cos(lat) * np.sin(lon),
            (n_val * (1 - Enu.E2) + alt) * np.sin(lat)
        ])

    @staticmethod
    def llh_to_rot_mat(lat_deg: float, lon_deg: float) -> np.array:
        lat = np.radians(lat_deg)
        lon = np.radians(lon_deg)
        return np.array([
            [-np.sin(lon), np.cos(lon), 0.0],
            [-np.sin(lat) * np.cos(lon), -np.sin(lat) * np.sin(lon), np.cos(lat)],
            [np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)]
        ])


class GpsExample(Node):
    def __init__(self):
        super().__init__('gps_example')
        ref_lat = self.declare_parameter('ref_lat', float('inf')).value
        ref_lon = self.declare_parameter('ref_lon', float('inf')).value
        ref_alt = self.declare_parameter('ref_alt', float('inf')).value
        if not np.isfinite(ref_lat) or not np.isfinite(ref_lon) or not np.isfinite(ref_alt):
            self.get_logger().error('Reference coordinate parameters not set!')
            rclpy.shutdown()
            sys.exit(0)

        # TODO: Subscribe to GPS position and advertise an ENU path

        # TODO: Calculate ECEF position and ECEF -> ENU rotation matrix at the reference coordinates

    def recv_fix(self, msg):
        pass

        # TODO: Convert incoming latitude/longitude/altitude data into ENU

        # TODO: Append position to the end of the path message and then publish it out


if __name__ == '__main__':
    rclpy.init()
    node_instance = GpsExample()
    rclpy.spin(node_instance)
