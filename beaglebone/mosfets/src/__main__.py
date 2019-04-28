import Adafruit_BBIO.GPIO as GPIO
from rover_msgs import Mosfet
import lcm

lcm_ = lcm.LCM()

LED_BACKLIGHTS = "P8_11"
LED_UV = "P8_12"
SOLENOID_1 = "P8_14"
SOLENOID_2 = "P8_16"

mosfets = [LED_BACKLIGHTS, LED_UV, SOLENOID_1, SOLENOID_2]


def mosfet_callback(channel, msg):
    cmd = Mosfet.decode(msg)

    if cmd.enable:
        out = GPIO.HIGH
    else:
        out = GPIO.LOW

    if cmd.id == "backlights":
        GPIO.output(mosfets[0], out)
    elif cmd.id == "uv_leds":
        GPIO.output(mosfets[1], out)
    elif cmd.id == "solenoid_1":
        GPIO.output(mosfets[2], out)
    elif cmd.id == "solenoid_2":
        GPIO.output(mosfets[3], out)
    else:
        print("Invalid mosfet ID.")


def main():
    for pin in mosfets:
        GPIO.setup(pin, GPIO.OUT)

    lcm_.subscribe("/mosfet", mosfet_callback)

    while True:
        lcm_.handle()


if __name__ == "__main__":
    main()
