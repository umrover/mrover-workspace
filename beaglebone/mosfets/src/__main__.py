import Adafruit_BBIO.GPIO as GPIO
from rover_msgs import Mosfet
import lcm

lcm_ = lcm.LCM()

# TODO: Update device names accordng to LCM constants
pins = {
    Mosfet.DEV0: "P8_7",
    Mosfet.DEV1: "P8_8",
    Mosfet.DEV2: "P8_9",
    Mosfet.DEV3: "P8_10",
    Mosfet.DEV4: "P8_11",
    Mosfet.DEV5: "P8_12",
    Mosfet.DEV6: "P8_15",
    Mosfet.DEV7: "P8_16",
    Mosfet.DEV8: "P8_17",
    Mosfet.DEV9: "P8_18",
}


def mosfet_callback(channel, msg):
    cmd = Mosfet.decode(msg)

    if cmd.enable:
        out = GPIO.HIGH
    else:
        out = GPIO.LOW

    GPIO.output(pins[cmd.device], out)


def main():
    for device in pins:
        GPIO.setup(pins[device], GPIO.out)

    lcm_.subscribe("/mosfet", mosfet_callback)

    while True:
        lcm_.handle()


if __name__ == "__main__":
    main()
