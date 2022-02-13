import json
import os
from http.server import BaseHTTPRequestHandler, HTTPServer

import lcm

from rover_msgs import Odometry, IMUData, GPS

PORT: int = 8000

__lcm: lcm.LCM

start = 0


class GPSServer(BaseHTTPRequestHandler):
    def do_HEAD(self):
        self.send_response(200)

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            with open("./jetson/gps_viewer/index.html", "rb") as html_file:
                self.wfile.write(html_file.read())
        elif self.path == '/data/':
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.end_headers()
            global start
            point = {
                "longitude": 42.29328577670418,
                "latitude": -83.71098762021397 + start
            }
            start += 0.001
            self.wfile.write(json.dumps(point).encode("utf-8"))
        else:
            self.send_response(404)


def odometry_callback(channel, msg):
    msg = Odometry.decode(msg)
    print(msg.val)

    long = str(msg.longitude_min)
    lat = str(msg.latitude_min)

    global __lcm
    __lcm.handle()


def imu_callback(channel, msg):
    msg = IMUData.decode(msg)
    print(msg.val)

    angle = str(msg.bearing_deg)

    global __lcm
    __lcm.handle()


def gps_callback(channel, msg):
    msg = GPS.decode(msg)
    print(msg.val)

    gps_long = str(msg.longitude_min)
    gps_lat = str(msg.latitude_min)


def main():
    global __lcm
    __lcm = lcm.LCM()
    __lcm.subscribe("/Odometry", odometry_callback)
    __lcm.subscribe("/IMUData", imu_callback)
    __lcm.subscribe("/GPS", imu_callback)
    with HTTPServer(("0.0.0.0", 8080), GPSServer) as web_server:
        try:
            web_server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
