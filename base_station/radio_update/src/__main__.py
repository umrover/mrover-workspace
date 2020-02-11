# Designed to make calls to /vagrant/scripts/signal_strength.tcl

import asyncio
from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines
import os
import re

from rover_msgs import RadioMessage

# Define constants
WAIT_SECONDS = 9
UPDATE_CHANNEL = "/radio_update"
SETUP_CHANNEL = "/radio_setup"
SCRIPTS_DIR = "/vagrant/scripts/"
RADIO_USERNAME = "mrover"
RADIO_PWD = "mrover"
RADIO_IP = "10.9.0.3"

# Initialize LCM object
lcm_ = aiolcm.AsyncLCM()


def extractSignalStrength(message):
    """ Extract signal strength from a line of grep output """

    message = message.splitlines()[-1].strip()

    # If no radio connected, terminate this script
    if message == "Connection to radio timed out":
        print("Terminating signal strength script - no connection to radio")
        exit(1)

    # Regex for consecutive digits 0-9 and - to extract the signal strength
    regex = re.search("[0-9-]+", message)

    message = regex.group(0)

    # message will be of fomat: <whitespace>"signal": <negative number>,<\n>

    return message


def run_radio_setup_callback(channel, msg):
    """ Run radio_setup.tcl """

    args = "{} {} {} {} {} {} {}".format(RADIO_USERNAME,
                                         RADIO_PWD,
                                         RADIO_IP,
                                         5,
                                         907,
                                         -4,
                                         "station"
                                         )
    cmd = os.path.join(SCRIPTS_DIR, "radio_setup.tcl " + args)
    os.system(cmd)


async def send_sig_strength_loop():
    """ Periodically call signal_strength.tcl & extract signal strength """

    while(True):

        # Read File contents to string
        msg_content = ''

        print("Connecting to radio...")

        # Run expect script with arguments [radio username] [password] [ip]
        cmd = "signal_strength.tcl {} {} {}".format(RADIO_USERNAME,
                                                    RADIO_PWD,
                                                    RADIO_IP)
        stream = os.popen(os.path.join(SCRIPTS_DIR, cmd))

        # Read the output from the scp command
        # The command runs grep for the line containing signal strength
        msg_content = stream.read()
        stream.close()

        msg_content = extractSignalStrength(msg_content)

        # Create RadioMessage object and send over LCM
        message = RadioMessage()
        message.signal_strength = msg_content
        lcm_.publish(UPDATE_CHANNEL, message.encode())

        # Print The message to the console
        print("published message:\n\tchannel: "
              + "{}\n\tcontent: {}\n".format(UPDATE_CHANNEL, msg_content))

        # Wait the amount of time define by WAIT_SECONDS
        await asyncio.sleep(WAIT_SECONDS)


def main():
    lcm_.subscribe(SETUP_CHANNEL, run_radio_setup_callback)
    run_coroutines(send_sig_strength_loop(), lcm_.loop())
