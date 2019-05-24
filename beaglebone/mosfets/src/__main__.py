import Adafruit_BBIO.GPIO as GPIO
from rover_msgs import Mosfet
from rover_msgs import ArmToggles
import lcm

lcm_ = lcm.LCM()

LED_BACKLIGHTS = "P8_14"
LED_UV = "P8_12"

ARM_SOLENOID = "P8_11"
ARM_MAGNET = "P8_14"
ARM_LASER = "P8_12"

sa_mosfets = [LED_BACKLIGHTS, LED_UV]
arm_mosfets = [ARM_SOLENOID, ARM_MAGNET, ARM_LASER]


def mosfet_callback(channel, msg):
    cmd = Mosfet.decode(msg)

    if cmd.enable:
        out = GPIO.HIGH
    else:
        out = GPIO.LOW

    if cmd.id == "backlights":
        GPIO.output(sa_mosfets[0], out)
    elif cmd.id == "uv_leds":
        GPIO.output(sa_mosfets[1], out)
    else:
        print("Invalid mosfet ID.")


def arm_callback(channel, msg):
    cmd = ArmToggles.decode(msg)

    if cmd.solenoid:
        GPIO.output(arm_mosfets[0], GPIO.HIGH)
    else:
        GPIO.output(arm_mosfets[0], GPIO.LOW)

    if cmd.electromagnet:
        GPIO.output(arm_mosfets[1], GPIO.HIGH)
    else:
        GPIO.output(arm_mosfets[1], GPIO.LOW)

    if cmd.laser:
        GPIO.output(arm_mosfets[2], GPIO.HIGH)
    else:
        GPIO.output(arm_mosfets[2], GPIO.LOW)


def main():
    GPIO.setup("P8_11", GPIO.OUT)
    GPIO.setup("P8_12", GPIO.OUT)
    GPIO.setup("P8_14", GPIO.OUT)

    lcm_.subscribe("/mosfet", mosfet_callback)
    lcm_.subscribe("/arm_toggles", arm_callback)

    while True:
        lcm_.handle()


if __name__ == "__main__":
    main()
