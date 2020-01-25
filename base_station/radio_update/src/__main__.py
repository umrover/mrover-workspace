# Designed to make calls to /vagrant/scripts/signal_strength.tcl

import lcm
import time
import os
import re

from rover_msgs import RadioMessage

# Define constants
WAIT_SECONDS = 9
channel = "/radio"

# Initialize LCM object
lcm = lcm.LCM()


def extractSignalStrength(message):
    """ Extract signal strength from a line of grep output """

    message = message.splitlines()[-1]

    # Regex for consecutive digits 0-9 and - to extract the signal strength
    regex = re.search("[0-9-]+", message)

    message = regex.group(0)

    # message will be of fomat: <whitespace>"signal": <negative number>,<\n>

    return message


def main():
    """ Periodically call signal_strength.tcl & extract signal strength """

    while(True):

        # Read File contents to string

        msg_content = ''

        # Run expect script with arguments [radio username] [password] [ip]
        stream = os.popen("/vagrant/scripts/signal_strength.tcl "
                          + "mrover mrover 10.9.0.3")

        # Read the output from the scp command
        # The command runs grep for the line containing signal strength
        msg_content = stream.read()
        stream.close()

        msg_content = extractSignalStrength(msg_content)

        """
        with open('/vagrant/base_station/radio_update/rad_msg.txt', 'r') as f:
            msg_content = f.read()
        """

        # Create RadioMessage object and send over LCM
        message = RadioMessage()
        message.signal_strength = msg_content
        lcm.publish(channel, message.encode())

        # Print The message to the console
        print("published message:\n\tchannel: "
              + "{}\n\tcontent: {}\n".format(channel, msg_content))

        # Wait the amount of time define by WAIT_SECONDS
        time.sleep(WAIT_SECONDS)
