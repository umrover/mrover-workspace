# Copyright (c) 2019 Esben Rossel
# All rights reserved.
#
# Author: Esben Rossel <esbenrossel@gmail.com>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.


# python imports
import serial

# application imports
import time


def rxtxonce(SerQueue, config):
    # open serial port
    try:
        ser = serial.Serial(config.port, config.baudrate)
        # share the serial handle with the stop-thread so
        # cancel_read may be called
        SerQueue.put(ser)
        # disable controls
        config.stopsignal = 0

        # wait to clear the input and output buffers,
        # if they're not empty data is corrupted
        while (ser.in_waiting > 0):
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.01)

        # Transmit key 'ER'
        config.txfull[0] = 69
        config.txfull[1] = 82
        # split 32-bit integers to be sent into 8-bit data
        config.txfull[2] = (config.SHperiod >> 24) & 0xff
        config.txfull[3] = (config.SHperiod >> 16) & 0xff
        config.txfull[4] = (config.SHperiod >> 8) & 0xff
        config.txfull[5] = config.SHperiod & 0xff
        config.txfull[6] = (config.ICGperiod >> 24) & 0xff
        config.txfull[7] = (config.ICGperiod >> 16) & 0xff
        config.txfull[8] = (config.ICGperiod >> 8) & 0xff
        config.txfull[9] = config.ICGperiod & 0xff
        # averages to perfom
        config.txfull[10] = config.AVGn[0]
        config.txfull[11] = config.AVGn[1]

        # transmit everything at once (the USB-firmware does not work if
        # all bytes are not transmitted in one go)
        ser.write(config.txfull)

        # wait for the firmware to return data
        config.rxData8 = ser.read(7388)

        # close serial port
        ser.close()

        if (config.stopsignal == 0):
            # combine received bytes into 16-bit data
            for rxi in range(3694):
                config.rxData16[rxi] = ((config.rxData8[2*rxi+1] << 8) +
                                        config.rxData8[2*rxi])

            # plot the new data
            # panel.bupdate.invoke()
            # hold values for saving data to file as the SHperiod
            # and ICGperiod may be updated after acquisition
            config.SHsent = config.SHperiod
            config.ICGsent = config.ICGperiod

        SerQueue.queue.clear()

    except serial.SerialException:
        print("By the great otter!",
              "There's a problem with the specified serial connection.")
