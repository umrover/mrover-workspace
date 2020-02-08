import Adafruit_BBIO.GPIO as GPIO
from rover_msgs import MosfetCmd
import lcm

lcm_ = lcm.LCM()

# TODO: Update device names accordng to LCM constants
pins = {
    MosfetCmd.DEV0: "P8_7",
    MosfetCmd.DEV1: "P8_8",
    MosfetCmd.DEV2: "P8_9",
    MosfetCmd.DEV3: "P8_10",
    MosfetCmd.DEV4: "P8_11",
    MosfetCmd.DEV5: "P8_12",
    MosfetCmd.DEV6: "P8_15",
    MosfetCmd.DEV7: "P8_16",
    MosfetCmd.DEV8: "P8_17",
    MosfetCmd.DEV9: "P8_18",
}


def mosfet_callback(channel, msg):
    cmd = MosfetCmd.decode(msg)

    if cmd.enable:
        out = GPIO.HIGH
    else:
        out = GPIO.LOW

    GPIO.output(pins[cmd.device], out)


def main():
    for device in pins:
        GPIO.setup(pins[device], GPIO.out)

    lcm_.subscribe("/mosfet_cmd", mosfet_callback)

    while True:
        lcm_.handle()


if __name__ == "__main__":
    main()
