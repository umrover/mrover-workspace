import numpy as np


class Config():
    def __init__(self):
        # serial definitions
        self.port = '/dev/ttyACM2'
        self.baudrate = 115200

        # Data as the program handles
        self.SHperiod = np.uint32(200)
        self.ICGperiod = np.uint32(50000)
        self.AVGn = np.uint8([0, 6])
        self.MCLK = 2000000
        self.SHsent = np.uint32(200)
        self.ICGsent = np.uint32(50000)
        self.stopsignal = 0

        # Data arrays for received bytes
        self.rxData8 = np.zeros(7388, np.uint8)
        self.rxData16 = np.zeros(3694, np.uint16)
        self.pltData16 = np.zeros(3694, np.uint16)

        # Arrays for data to transmit
        self.txsh = np.uint8([0, 0, 0, 0])
        self.txicg = np.uint8([0, 0, 0, 0])
        self.txfull = np.uint8([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

        # Invert data
        self.datainvert = 1
        self.offset = 0
        self.balanced = 0
