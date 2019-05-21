from . import MicroCam3_BBB
from rover_msgs import MicroCam
import lcm


camera1 = None
camera2 = None


def image_callback(channel, msg):
    global camera1
    global camera2

    pic = MicroCam.decode(msg)

    if pic.id == "camera_1":
        camera1.imageRoutine()
    elif pic.id == "camera_2":
        camera2.imageRoutine()
    else:
        print("Invalid camera.")


def main():

    global camera1
    global camera2

    camera1 = MicroCam3_BBB.Camera()
    camera2 = MicroCam3_BBB.Camera()

    camera1.SERIAL_PORT = "/dev/ttyO1"
    camera1.SERIAL_BAUD = 57600
    camera2.SERIAL_PORT = "/dev/ttyO4"
    camera2.SERIAL_BAUD = 57600

    camera1.UART = "UART1"
    camera1.GPIO_RESET = "P8_7"
    camera2.UART = "UART4"
    camera2.GPIO_RESET = "P8_9"

    camera1.PACKAGE_SIZE = 512
    camera2.PACKAGE_SIZE = 512

    camera1.CONTRAST = 2
    camera1.BRIGHTNESS = 2
    camera1.EXPOSURE = 2
    camera2.CONTRAST = 2
    camera2.BRIGHTNESS = 2
    camera2.EXPOSURE = 2

    camera1.startRoutine()
    camera2.startRoutine()

    synced1 = camera1.setupRoutine()
    synced2 = camera2.setupRoutine()

    if not synced1 and not synced2:
        print("Sync failed with both cameras.", flush=True)
        exit(1)
    if not synced1:
        print("Sync Failed with Camera 1")
    if not synced2:
        print("Sync Failed with Camera 2")

    print("Synced", flush=True)

    lcm_ = lcm.LCM()

    lcm_.subscribe("/microcam", image_callback)

    while True:
        lcm_.handle()

    camera1.stopRoutine()
    camera2.stopRoutine()

    exit()


if __name__ == "__main__":
    main()
