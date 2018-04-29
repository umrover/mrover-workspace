import serial


class Status():
    WaitHeader = 0
    InMsg = 1


START = b'\x12'
END = b'\x13'


class Reader():

    def __init__(self):
        self.state = Status.WaitHeader
        self.buffer = []

    def feed(self, c):
        next_state = self.state
        if self.state == Status.WaitHeader:
            if c == START:
                next_state = Status.InMsg
                self.buffer = []
        elif self.state == Status.InMsg:
            if c == END:
                next_state = Status.WaitHeader
                self.state = next_state
                return True
            self.buffer.append(c)
        self.state = next_state
        return False


def main():
    ser = serial.Serial('/dev/ttyUSB0', timeout=1)
    ser.baudrate = 115200
    r = Reader()
    while True:
        c = ser.read()
        if r.feed(c):
            print(r.buffer)
