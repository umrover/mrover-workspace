import Adafruit_BBIO.GPIO as GPIO
from rover_msgs import Mosfet
import lcm

lcm_ = lcm.LCM()

mosfets = ["P8_21", "P8_23", "P8_25"]


def mosfet_callback(channel, msg):
    cmd = Mosfet.decode(msg)

    if cmd.enable:
        GPIO.output(mosfets[cmd.id], GPIO.HIGH)
    else:
        GPIO.output(mosfets[cmd.id], GPIO.LOW)


def main():
    for pin in mosfets:
        GPIO.setup(pin, GPIO.OUT)

    lcm_.subscribe("/mosfet", mosfet_callback)

    while True:
        lcm_.handle()


if __name__ == "__main__":
    main()
