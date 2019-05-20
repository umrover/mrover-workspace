import serial
import time
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.UART as UART
from datetime import datetime
import os


class Camera:
    # DEFAULT SETTINGS, ACCESS TO MODIFY IN MAIN
    SERIAL_PORT = "/dev/ttyO4"
    SERIAL_BAUD = 57600

    UART = "UART4"
    GPIO_RESET = "P8_7"

    PACKAGE_SIZE = 512

    CONTRAST = 2
    BRIGHTNESS = 2
    EXPOSURE = 2

    # AVAILABLE ROUTINES, CALL AS NEEDED IN MAIN
    # each executes one or more commands
    # returns false if a critical process fails
    # camera should be restarted if a critical process fails

    def __init__(self, serial_port="/dev/tty04",
                 uart="UART4", gpio_reset="P8_7"):
        self.SERIAL_PORT = serial_port
        self.UART = uart
        self.GPIO_RESET = gpio_reset

    # CALL THIS ONCE. CALL THIS AGAIN IF CAMERA BECOMES STUCK.

    # initializes UART and serial port
    # pulls hardware reset low
    def startRoutine(self):
        self.__start()

        return True

    # CALL THIS TO SYNC WITH CAMERA. MULTIPLE ATTEMPTS MAY BE NECESSARY

    # confirms sync process with camera
    # sets the camera to never sleep
    # sets the camera's image size and type
    def setupRoutine(self):
        result = True

        result = result and self.__sync()

        if not result:
            return result

        result = result and self.__sleep()

        result = result and self.__initial()

        result = result and self.__setPackageSize()

        return result

    # camera takes an image and stores it in onboard memory
    def takeSnapshotRoutine(self):
        self.__snapshot()

        return True

    # camera outputs image from onboard memory
    def snapshotRoutine(self):
        result = self.__getPictureSnapshot()
        dataLength = result

        if (dataLength == -1):
            return False

        result = self.__dataDump(dataLength, self.PACKAGE_SIZE)

        if (result[0] == '\x00' and len(result) == 1):
            return False

        snapshotBuffer = result

        self.__filePicture("snapshot.jpeg", snapshotBuffer)

        self.__flush()

        return True

    # CALL THIS TO TAKE AND SAVE A JPEG IMMEDIATELY

    # camera takes an image and outputs it immediately
    def imageRoutine(self):
        result = self.__getPictureImage()
        dataLength = result

        if (dataLength == -1):
            return False

        result = self.__dataDump(dataLength, self.PACKAGE_SIZE)

        if (result[0] == '\x00' and len(result) == 1):
            return False

        imageBuffer = result

        self.__filePicture(imageBuffer)

        self.__flush()

        return True

    # CALL THIS AFTER MODIFYING THE ATTRIBUTES

    # configures contrast, brightness, and exposure
    def configRoutine(self):
        result = self.__config()

        return result

    # CALL THIS AT THE END OF THE PROGRAM

    # GPIO cleanup
    def stopRoutine(self):
        result = self.__stop()

        return result

    # END OF PUBLIC FUNCTIONS - THE REST ARE PRIVATE AND SHOULD NOT BE CALLED

    # SERIAL PORT USAGE
    def __send(self, command):
        commandRaw = self.__getCommand(command)
        # print("OUT =", end='')
        # print(command, end=' ')

        for x in range(len(commandRaw)):
            self.serialPort.write(commandRaw[x])
            # print(commandRaw[x], end='')
        # print(" ...WAIT")

    def __sendRaw(self, commandRaw):
        # command = self.__findCommand(commandRaw)
        # print("OUT =", end='')
        # print(command, end=' ')

        for x in range(len(commandRaw)):
            self.serialPort.write(commandRaw[x])
            # print(commandRaw[x], end='')
        # print(" ...WAIT")

    def __receive(self, length):
        # print("IN ", end='')

        timeBegin = time.time()
        response = []
        while(self.serialPort.in_waiting <= 0 and
              (time.time() - timeBegin) < 3):
            True

        if (time.time() - timeBegin > 3):
            response = [b'\xFF', b'\xFF']
        else:
            index = 0
            while(self.serialPort.in_waiting > 0 and index < length):
                response.append(self.serialPort.read())
                # print(response[index], end='')
                index += 1

        command = self.__findCommand(response)
        # print(" =", end='')
        # print(command)

        return command

    def __receiveRaw(self, length):
        # print("IN ", end='')

        timeBegin = time.time()
        response = []
        while(self.serialPort.in_waiting <= 0 and
              (time.time() - timeBegin) < 3):
            True

        if (time.time() - timeBegin > 3):
            response = [b'\xFF', b'\xFF']
        else:
            index = 0
            while(self.serialPort.in_waiting > 0 and index < length):
                response.append(self.serialPort.read())
                # print(response[index], end='')
                index += 1

        # command = self.__findCommand(response)
        # print(" =", end='')
        # print(command)

        return response

    def __flush(self):
        # print("OPERATION =FLUSH")
        empty = False
        while(empty):
            time.sleep(0.5)

            if (self.serialPort.in_waiting == 0):
                empty = True

            while(self.serialPort.in_waiting > 0):
                continue
                # print(hex(self.serialPort.read()), end='')

        # print("\nSUCCESS =FLUSH\n")

    # COMMAND ALIASING AND INTERPRETATION
    def __getCommand(self, command):
        if(command == "SYNC"):
            return [b'\xAA', b'\x0D', b'\x00', b'\x00', b'\x00', b'\x00']

        if(command == "ACK_SYNC"):
            return [b'\xAA', b'\x0E', b'\x0D', b'\x00', b'\x00', b'\x00']

        if(command == "SLEEP"):
            return [b'\xAA', b'\x15', b'\x00', b'\x00', b'\x00', b'\x00']

        if(command == "INITIAL"):
            return [b'\xAA', b'\x01', b'\x00', b'\x07', b'\x09', b'\x07']

        if(command == "SET_PACKAGE_SIZE"):
            return [b'\xAA', b'\x06', b'\x08', b'\x00', b'\x02', b'\x00']

        if(command == "SNAPSHOT"):
            return [b'\xAA', b'\x05', b'\x00', b'\x00', b'\x00', b'\x00']

        if(command == "GET_PICTURE_SNAPSHOT"):
            return [b'\xAA', b'\x04', b'\x01', b'\x00', b'\x00', b'\x00']

        if(command == "GET_PICTURE_IMAGE"):
            return [b'\xAA', b'\x04', b'\x05', b'\x00', b'\x00', b'\x00']

        if(command == "ACK_PACKAGE"):
            return [b'\xAA', b'\x0E', b'\x00', b'\x00', b'\x00', b'\x00']

        if(command == "CONFIG"):
            return [b'\xAA', b'\x14',
                    (self.CONTRAST).to_bytes(1, byteorder="big"),
                    (self.BRIGHTNESS).to_bytes(1, byteorder="big"),
                    (self.EXPOSURE).to_bytes(1, byteorder="big"), b'\x00']

    def __findCommand(self, commandRaw):
        # TIMEOUT
        if(commandRaw[0] == b'\xFF' and commandRaw[1] == b'\xFF'):
            return "!TIMEOUT!"
        # SYNC
        if(commandRaw[1] == b'\x0D'):
            return "SYNC"
        # ACK
        if(commandRaw[1] == b'\x0E'):
            if(commandRaw[2] == b'\x0D'):
                return "ACK_SYNC"
            if(commandRaw[2] == b'\x15'):
                return "ACK_SLEEP"
            if(commandRaw[2] == b'\x01'):
                return "ACK_INITIAL"
            if(commandRaw[2] == b'\x06'):
                return "ACK_SET_PACKAGE_SIZE"
            if(commandRaw[2] == b'\x05'):
                return "ACK_SNAPSHOT"
            if(commandRaw[2] == b'\x04'):
                return "ACK_GET_PICTURE"
            if(commandRaw[2] == b'\x00'):
                return "ACK_PACKAGE"
            if(commandRaw[2] == b'\x14'):
                return "ACK_CONFIG"

        # DATA
        if(commandRaw[1] == b'\x0A'):
            if(commandRaw[2] == b'\x01'):
                return "DATA_SNAPSHOT"
            if(commandRaw[2] == b'\x05'):
                return "DATA_IMAGE"

        # SPECIAL
        if(commandRaw[1] == b'\x0F'):
            return "!NON-ACKNOWLEDGE!"

        if(commandRaw[0] != b'\xAA'):
            return "!PACKAGE!"

        return "!UNKNOWN!"

    # MICROROUTINES - EACH CORRESPONDS TO ONE COMMAND
    def __start(self):
        # print("DELAY =START ...3")
        time.sleep(3)

        UART.setup(self.UART)

        # print("OPERATION =RESET_PULLDOWN\n")
        GPIO.setup(self.GPIO_RESET, GPIO.OUT, GPIO.PUD_DOWN)
        GPIO.output(self.GPIO_RESET, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(self.GPIO_RESET, GPIO.LOW)
        time.sleep(1)
        GPIO.output(self.GPIO_RESET, GPIO.HIGH)

        self.serialPort = serial.Serial(
            port=self.SERIAL_PORT, baudrate=self.SERIAL_BAUD)

    def __sync(self):
        # print("OPERATION =SYNC ...CRITICAL")
        syncAttempts = 0
        while (syncAttempts < 10):
            self.__send("SYNC")
            reply = self.__receive(6)
            if (reply == "ACK_SYNC"):
                reply = self.__receive(6)
                if (reply == "SYNC"):
                    self.__send("ACK_SYNC")

                    # print("SUCCESS =SYNC\n")

                    # print("DELAY =SYNC ...2")
                    time.sleep(2)

                    return True
            syncAttempts += 1

        # print("FAILURE =SYNC\n")
        return False

    def __sleep(self):
        # print("OPERATION =SLEEP")
        self.__send("SLEEP")
        reply = self.__receive(6)
        if (reply == "ACK_SLEEP"):
            # print("SUCCESS =SLEEP\n")
            return True

        # print("FAILURE =SLEEP\n")
        return True

    def __initial(self):
        initialAttempts = 0
        while(initialAttempts < 5):
            # print("OPERATION =INITIAL ...CRITICAL")
            self.__send("INITIAL")
            reply = self.__receive(6)
            if (reply == "ACK_INITIAL"):
                # print("SUCCESS =INITIAL\n")
                return True

            # print("FAILURE =INITIAL\n")

            initialAttempts += 1
        return False

    def __setPackageSize(self):
        setPackageSizeAttempts = 0
        while(setPackageSizeAttempts < 5):
            # print("OPERATION =SET_PACKAGE_SIZE ...CRITICAL")
            self.__send("SET_PACKAGE_SIZE")
            reply = self.__receive(6)
            if (reply == "ACK_SET_PACKAGE_SIZE"):
                # print("SUCCESS =SET_PICTURE_SIZE\n")
                return True

            # print("FAILURE =SET_PICTURE_SIZE\n")

            setPackageSizeAttempts += 1
        return False

    def __snapshot(self):
        # print("OPERATION =SNAPSHOT")
        self.__send("SNAPSHOT")
        reply = self.__receive(6)
        if (reply == "ACK_SNAPSHOT"):
            # print("SUCCESS =SNAPSHOT\n")

            # print("DELAY =SNAPSHOT ...3")
            time.sleep(3)

            return True

        # print("FAILURE =SNAPSHOT\n")
        return True

    def __getPictureSnapshot(self):
        # print("OPERATION =GET_PICTURE_SNAPSHOT ...CRITICAL")
        self.__send("GET_PICTURE_SNAPSHOT")
        reply = self.__receive(6)
        if (reply == "ACK_GET_PICTURE"):
            replyRaw = self.__receiveRaw(6)
            if (self.__findCommand(replyRaw) == "DATA_SNAPSHOT"):
                # print("SUCCESS =GET_PICTURE_SNAPSHOT\n")
                total = 0
                total = (int.from_bytes(
                    replyRaw[5], byteorder="big") | (total << 8))
                total = (int.from_bytes(
                    replyRaw[4], byteorder="big") | (total << 8))
                total = (int.from_bytes(
                    replyRaw[3], byteorder="big") | (total << 8))
                return total

        # print("FAILURE =GET_PICTURE_SNAPSHOT\n")
        self.__flush()
        return -1

    def __getPictureImage(self):
        # print("OPERATION =GET_PICTURE_IMAGE ...CRITICAL")
        self.__send("GET_PICTURE_IMAGE")
        reply = self.__receive(6)
        if (reply == "ACK_GET_PICTURE"):
            replyRaw = self.__receiveRaw(6)
            if (self.__findCommand(replyRaw) == "DATA_IMAGE"):
                # print("SUCCESS =GET_PICTURE_IMAGE\n")
                total = 0
                total = (int.from_bytes(
                    replyRaw[5], byteorder="big") | (total << 8))
                total = (int.from_bytes(
                    replyRaw[4], byteorder="big") | (total << 8))
                total = (int.from_bytes(
                    replyRaw[3], byteorder="big") | (total << 8))
                return total

        # print("FAILURE =GET_PICTURE_IMAGE\n")
        self.__flush()
        return -1

    def __dataDump(self, dataLength, packageSize):
        # print("OPERATION =DATA_DUMP ...CRITICAL")

        packageCount = int(dataLength / (packageSize - 6)) + 1

        index = 0
        buffer = []
        for packageNumber in range(packageCount):
            # print("PACKAGE " + str(packageNumber)+" OF " + str(packageCount))
            messageRaw = self.__getCommand("ACK_PACKAGE")
            messageRaw[5] = ((packageNumber) >> 8).to_bytes(
                1, byteorder='big')
            messageRaw[4] = ((packageNumber) & 255).to_bytes(
                1, byteorder='big')
            self.__sendRaw(messageRaw)

            reply = self.__receiveRaw(packageSize)
            dataAttempts = 0
            while(self.__findCommand(reply) == "!TIMEOUT!"):
                reply = self.__receiveRaw(packageSize)
                dataAttempts += 1
                if (dataAttempts > 3):
                    buffer = ['\x00']
                    return buffer

            for byteNumber in range(len(reply)):
                if (byteNumber > 3 and byteNumber < len(reply) - 2):
                    buffer.append(reply[byteNumber])
                    index += 1

        messageRaw = self.__getCommand("ACK_PACKAGE")
        messageRaw[5] = b'\xF0'
        messageRaw[4] = b'\xF0'
        self.__sendRaw(messageRaw)

        # print("SUCCESS =DATA_DUMP\n")
        return buffer

    def __config(self):
        # print("OPERATION =CONFIG ...CRITICAL")
        self.__send("CONFIG")
        reply = self.__receive(6)
        if (reply == "ACK_CONFIG"):
            # print("SUCCESS =CONFIG\n")
            return True

        # print("FAILURE =CONFIG\n")
        return False

    def __filePicture(self, data):
        # print("OPERATION =FILEPICTURE ..." + name + "\n")
        stamp = datetime.now()
        name = "/home/debian/mrover-workspace/beaglebone/uCamIII/" + \
               str(stamp.year) + "-" + str(stamp.month) + "-" + \
               str(stamp.day) + "+" + str(stamp.hour) + ":" + \
               str(stamp.minute) + ":" + str(stamp.second) + ".jpeg"

        file = open(name, "w+b")
        for i in range(len(data)):
            # print(data[i], end='')
            file.write(data[i])

        file.close()

        os.system('scp -l 2000 {} mrover@10.0.0.2:science-data/MicroCam/{}.jpg'
                  .format(name, round(time.time() * 1000)))

        # print("\nSUCCESS = FILEPICTURE ..." + name + "\n")

    def __stop(self):
        GPIO.cleanup()

        self.serialPort.close()
