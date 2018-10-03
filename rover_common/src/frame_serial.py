from enum import Enum

START = b'\x12'
END = b'\x13'
ESC = b'\x7D'


class Status(Enum):
    WaitHeader = 0
    InMsg = 1
    AfterEsc = 2


class Reader:
    def __init__(self):
        self.state = Status.WaitHeader
        self.buffer = []

    def feed(self, c):
        next_state = self.state
        completed = False
        if self.state == Status.WaitHeader:
            if c == START:
                next_state = Status.InMsg
                self.buffer = []
        elif self.state == Status.InMsg:
            if c == ESC:
                next_state = Status.AfterEsc
            elif c == END:
                next_state = Status.WaitHeader
                completed = True
            else:
                self.buffer.append(c)
        elif self.state == Status.AfterEsc:
            self.buffer.append(c)
            next_state = Status.InMsg
        self.state = next_state
        return completed
