import lcm
import sys
import time as t
import odrive as odv
# from ODriver_Req_State import *
# from ODriver_Pub_Encoders import *
# from ODriver_Req_Vel import *
# from ODriver_Pub_State import *
from rover_msgs import ODriver_Req_State, ODriver_Req_Vel, \
    ODriver_Pub_State, ODriver_Pub_Encoders
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, \
    CTRL_MODE_VELOCITY_CONTROL, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, \
    AXIS_STATE_IDLE
import modrive as Modrive


def main():
    global lcm_
    lcm_ = lcm.LCM()

    # insert lcm subscribe statements here

    # These have been mapped

    lcm_.subscribe("/odriver_req_state", odriver_req_state_callback)

    lcm_.subscribe("/odrive_req_vel", odriver_req_vel_callback)

    global modrive
    global legalAxis
    legalAxis = sys.argv[2]

    global msg
    global msg1
    msg = ODriver_Pub_Encoders()
    msg1 = ODriver_Pub_State()

    while True:
        lcm_.handle()
        nextState()

    exit()


states = ["BOOT", "DISARMED", "ARMED", "ERROR", "CALIBRATING", "EXIT"]
# Program states possible - BOOT, DISARMED, ARMED, ERROR, CALIBRATING, EXIT
# 							1		 2	      3	    4		 5         6
currentState = "BOOT"  # starting state
requestedState = "DISARMED"  # starting requested state
odrive = None  # starting odrive


def publish_state_msg(msg, state_number):
    global currentState
    currentState = states[state_number - 1]
    msg1.state = state_number
    msg1.serialid = sys.argv[1]
    lcm_.publish("/odriver_pub_state", msg1.encode())  # is lcm_ global?
    return t.time()


def publish_encoder_helper(msg, axis):
    msg.measuredCurrent = modrive.get_iq_measured(axis)
    msg.estVel = modrive.get_vel_estimate(axis)
    msg.serialid = sys.argv[1]
    if (axis == "RIGHT"):
        msg.axis = 'r'
    elif (axis == "LEFT"):
        msg.axis = 'l'
    lcm_.publish("/odriver_pub_encoders", msg)


def publish_encoder_msg(msg):
    if (legalAxis == "BOTH"):
        publish_encoder_helper(msg, "LEFT")
        publish_encoder_helper(msg, "RIGHT")
        return t.time()
    elif (legalAxis == "RIGHT"):
        publish_encoder_helper(msg, "RIGHT")
        return t.time()
    elif (legalAxis == "LEFT"):
        publish_encoder_helper(msg, "RIGHT")
        return t.time()


def nextState():
    # every time the state changes,
    # publish an odrive_state lcm message, with the new state
    global currentState
    global requestedState
    global odrive
    global encoderTime

    if (currentState == "BOOT"):
        # attempt to connect to odrive
        odrive = odv.find_any(serial_number=sys.argv[1])
        modrive = Modrive(odrive)  # arguments = odr

        # Block until odrive is connected
        # set current limit on odrive to 100
        modrive.set_current_lim(legalAxis, 100)
        # set controller's control mode to velocity control
        modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)
        # set currentState to DISARMED
        publish_state_msg(msg1, 2)

    elif (currentState == "DISARMED"):
        # if 100 ms have passed since last time data was published
        #   publish an odrive_data lcm message with measured
        #   current and estimated velocity

        # Unsure if using correct timestamp
        if (encoderTime - t.time() > 0.1):
            encoderTime = publish_encoder_msg(msg)
        # unsure if this is right
        errors = odv.dump_errors(odrive)
        if errors:
            # sets state to error
            publish_state_msg(msg1, 4)
        elif requestedState == "ARMED":
            modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)
            # sets state to armed
            publish_state_msg(msg1, 3)
        elif requestedState == "BOOT":
            odrive.reboot()
            # sets state to boot
            publish_state_msg(msg1, 1)
        elif requestedState == "CALIBRATING":
            modrive.requested_state(legalAxis,
                                    AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
            # sets state to calibrating
            publish_state_msg(msg1, 5)

    elif (currentState == "ARMED"):
        if (encoderTime - t.time() > 0.1):
            encoderTime = publish_encoder_msg(msg)
        errors = odv.dump_errors(odrive)
        if errors:
            # sets state to error
            publish_state_msg(msg1, 4)
        elif requestedState == "DISARMED":
            modrive.set_control_mode(legalAxis, AXIS_STATE_IDLE)
        # sets state to disarmed
            publish_state_msg(msg1, 2)
        elif requestedState == "BOOT":
            odrive.reboot()
        # sets state to boot
            publish_state_msg(msg1, 1)
        elif requestedState == "CALIBRATING":
            modrive.requested_state(legalAxis,
                                    AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
        # sets state to calibrating
            publish_state_msg(msg1, 5)
        elif (currentState == "ERROR"):
            if requestedState == "BOOT":
                odrive.reboot()
                # sets current state to boot
                publish_state_msg(msg1, 1)
            elif requestedState == "CALIBRATING":
                modrive.requested_state(legalAxis,
                                        AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
                # sets current state to calibrating
                publish_state_msg(msg1, 5)
    elif (currentState == "CALIBRATING"):
        # if odrive is done calibrating
        #   set current limit on odrive to 100
        #   set controller's control mode to velocity control
        #   set currentState to DISARMED
        # We don't know how to check if done calibrating
        # if odrive.

        # TODO: add in check for finish calibration(axis == idle)
        if legalAxis == "LEFT":
            if modrive.get_current_state("LEFT") == AXIS_STATE_IDLE:
                modrive.set_current_lim(legalAxis, 100)
                modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)
        elif legalAxis == "RIGHT":
            if modrive.get_current_state("RIGHT") == AXIS_STATE_IDLE:
                modrive.set_current_lim(legalAxis, 100)
                modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)
        elif legalAxis == "BOTH":
            if modrive.get_current_state("LEFT") == AXIS_STATE_IDLE \
                        and modrive.get_current_state("RIGHT") == AXIS_STATE_IDLE:
                modrive.set_current_lim(legalAxis, 100)
                modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)

    # sets state to disarmed
        publish_state_msg(msg1, 2)


def odriver_req_state_callback(channel, msg):
    message = ODriver_Req_State.decode(msg)
    if message.serialid == sys.argv[1]:
        requestedState = states[message.requestState - 1]
    # TODO: check which axis are legal
    if requestedState == "EXIT":
        if legalAxis == "LEFT":
             if modrive.get_vel_estimate("LEFT") == 0 and \
                    modrive.get_current_state("LEFT") == AXIS_STATE_IDLE
                sys.exit()
            else:
                modrive.set_vel(legalAxis, 0)
                modrive.requested_state(legalAxis, AXIS_STATE_IDLE)
                sys.exit()
        elif legalAxis == "RIGHT":
            if modrive.get_vel_estimate("RIGHT") == 0 and \
                    modrive.get_current_state("RIGHT") == AXIS_STATE_IDLE \
                sys.exit()
            else:
                modrive.set_vel(legalAxis, 0)
                modrive.requested_state(legalAxis, AXIS_STATE_IDLE)
                sys.exit()
        elif legalAxis == "BOTH":
            if modrive.get_vel_estimate("LEFT") == 0 and \
                    modrive.get_current_state("LEFT") == AXIS_STATE_IDLE \
                    and modrive.get_vel_estimate("RIGHT") == 0 \
                    and modrive.get_current_state("RIGHT") == AXIS_STATE_IDLE:
                sys.exit()
            else:
                modrive.set_vel(legalAxis, 0)
                modrive.requested_state(legalAxis, AXIS_STATE_IDLE)
                sys.exit()


def odriver_req_vel_callback(channel, msg):
    # if the program is in an ARMED state
    #   set the odrive's velocity to the float specified in the message
    # no state change
    message = ODriver_Req_Vel.decode(msg)
    if message.serialid == sys.argv[1]:
        if(currentState == "ARMED"):
            modrive.requested_state(legalAxis, AXIS_STATE_CLOSED_LOOP_CONTROL)
            modrive.set_vel(legalAxis, message.vel)


if __name__ == "__main__":
    main()
