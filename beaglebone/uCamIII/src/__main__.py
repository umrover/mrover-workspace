from . import MicroCam3_BBB
import lcm


camera = None


def main():

    global camera
    camera = MicroCam3_BBB.Camera()

    camera.SERIAL_PORT = "/dev/ttyO4"
    camera.SERIAL_BAUD = 57600

    camera.UART = "UART4"
    camera.GPIO_RESET = "P8_7"

    camera.PACKAGE_SIZE = 512

    camera.CONTRAST = 2
    camera.BRIGHTNESS = 2
    camera.EXPOSURE = 2

    camera.startRoutine()

    synced = camera.setupRoutine()

    if not synced:
        print("Sync Failed")
        exit(1)

    print("Synced")

    lcm_ = lcm.LCM()

    lcm_.subscribe("/microcam", image_callback)

    while True:
        lcm_.handle()

    camera.stopRoutine()

    exit()


def image_callback(channel, msg):
    global camera

    print("Say cheese!")
    camera.imageRoutine()

    print("Done")


if __name__ == "__main__":
    main()
