import Adafruit_BBIO.GPIO as GPIO
from rover_msgs import MosfetCmd
import lcm

lcm_ = lcm.LCM()
global out

pins = {
    MosfetCmd.HEAT_W_12V:    "P8_7",
    MosfetCmd.UV_LED_REP_5V: "P8_8",
    MosfetCmd.LED_24V:       "P8_9",
    MosfetCmd.HEAT_B_12V:    "P8_10",
    MosfetCmd.HEAT_Y_12V:    "P8_11",
    MosfetCmd.RA_LASER_3_3V: "P8_12"
}


def mosfet_callback(channel, msg):
    cmd = MosfetCmd.decode(msg)

    global out
    if cmd.enable:
        out = GPIO.HIGH
    else:
        out = GPIO.LOW

    GPIO.output(pins[cmd.device], out)


def main():
    for device in pins:
        GPIO.setup(pins[device], GPIO.OUT)

    lcm_.subscribe("/mosfet_cmd", mosfet_callback)

    while True:
        lcm_.handle()


if __name__ == "__main__":
    main()
