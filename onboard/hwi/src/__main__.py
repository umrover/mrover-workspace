from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import Motors
import serial
import os

lcm_ = aiolcm.AsyncLCM()
ssc32_port = os.environ.get('MROVER_SSC32_PORT', '/dev/ttyUSB0')


def toTalonValue(amount):
    if amount > -0.1 and amount < 0.1:
        return 1513
    if amount > 0:
        return 1539 + ((2037 - 1539) * amount)
    if (amount < 0):
        return 1487 + ((1487 - 989) * amount)
    return 1513


def motor_callback(channel, msg):
    print("Recieving {} bytes from {}".format(len(msg), channel))
    # Message has left and right
    # Calculate talonValue for all left and all right Motors
    m = Motors.decode(msg)
    talonValueLeft = toTalonValue(m.left)
    talonValueRight = toTalonValue(m.right)
    # Channels 0-2 are left 3-5 are right
    sendMsg = ""
    for i in range(0, 3):
        sendMsg += "#{} P{} ".format(i, talonValueLeft)
    for i in range(3, 6):
        sendMsg += "#{} P{} ".format(i, talonValueRight)
    # send the talonValues to each motor with pySerial
    sendMsg += "T10\r"
    # Alternative is /dev/tty.usbserial-AI03GJKD
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    print(sendMsg)
    ser.write(sendMsg.encode("utf-8"))


def test():
    message = {"left": 1, "right": 1}
    print(motor_callback("test", message))


def main():
    lcm_.subscribe("/motor", motor_callback)
    run_coroutines(lcm_.loop())


if __name__ == "__main__":
    main()


'''
For serial messages

# <ch> P <pw> S <spd> ... # <ch> P <pw> S <spd> T <time> <cr>
<ch> = Channel number in decimal,0- 31.
<pw> = Pulse width in microseconds, 500 - 2500.
<spd> = Movement speed in uS per second for one channel. (Optional)
<time> = Time in mS for the entire move, affects all channels, 65535 max.
<cr> = Carriage return character, ASCII 13. (Required to initiate action)
<esc> = Cancel the current command, ASCII 27.
Servo Move Example: "#5 P1600 S750 <cr>"
'''
