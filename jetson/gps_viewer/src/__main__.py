import json
from http.server import BaseHTTPRequestHandler, HTTPServer

import lcm

from rover_msgs import Odometry, IMUData, GPS

PORT: int = 8000

__data: dict = {
    "gps": {
        "longitude": 42.29328577670418,
        "latitude": -83.71098762021397
    },
    "odo": {
        "longitude": 42.29328577670418,
        "latitude": -83.71098762021397
    },
    "heading": 0.0
}

__lcm: lcm.LCM


class GPSRequestHandler(BaseHTTPRequestHandler):
    def __init__(self, data, request, client_address, server):
        self.data = data
        super(GPSRequestHandler, self).__init__(request, client_address, server)

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
            self.wfile.write(json.dumps(self.data).encode("utf-8"))
        else:
            self.send_response(404)


def odometry_callback(channel, msg):
    global __lcm, __data
    msg = Odometry.decode(msg)
    __data["odo"]["longitude"] = msg.longitude_deg + msg.longitude_min / 60
    __data["odo"]["latitude"] = msg.latitude_deg + msg.latitude_min / 60


def imu_callback(channel, msg):
    global __lcm, __data
    msg = IMUData.decode(msg)
    __data["heading"] = msg.bearing_deg


def gps_callback(channel, msg):
    global __lcm, __data
    msg = GPS.decode(msg)
    __data["gps"]["longitude"] = msg.longitude_deg + msg.longitude_min / 60
    __data["gps"]["latitude"] = msg.latitude_deg + msg.latitude_min / 60


def make_handler(request, client_address, server):
    global __data, __lcm
    while __lcm.handle_timeout(1):
        pass
    return GPSRequestHandler(__data, request, client_address, server)


def main():
    global __lcm
    __lcm = lcm.LCM()
    __lcm.subscribe("/odometry", odometry_callback)
    __lcm.subscribe("/imu_data", imu_callback)
    __lcm.subscribe("/gps", gps_callback)
    # run_coroutines(__lcm.loop())
    with HTTPServer(("0.0.0.0", 8080), make_handler) as web_server:
        try:
            web_server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
