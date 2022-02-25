import lcm
import sys
import time as t
import odrive as odv
import threading
import fibre
from rover_msgs import DriveVelCmd, \
    DriveStateData, DriveVelData
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, \
    CONTROL_MODE_VELOCITY_CONTROL, AXIS_STATE_IDLE

from odrive.utils import dump_errors
from enum import Enum


def main():
    global lcm_, modrive, left_speed, right_speed, legal_controller, \
        vel_msg, state_msg, usblock, speedlock, start_time, watchdog, odrive_bridge

    lcm_, left_speed, right_speed, start_time, legal_controller, \
        vel_msg, state_msg = lcm.LCM(), 0.0, 0.0, t.clock(), int(sys.argv[1]), DriveVelData(), DriveStateData()

    speedlock = threading.Lock()
    usblock = threading.Lock()

    threading._start_new_thread(lcmThreaderMan, ())
    odrive_bridge = OdriveBridge()
    # starting state is DisconnectedState()
    # start up sequence is called, disconnected-->disarm-->arm

    # flag for state when we have comms with base_station vs not
    prev_comms = False

    while True:
        watchdog = t.clock() - start_time
        if watchdog > 1.0:
            if prev_comms:
                print("loss of comms")
                prev_comms = False

            speedlock.acquire()
            left_speed = right_speed = 0
            speedlock.release()
        else:
            if not prev_comms:
                prev_comms = True
                print("regained comms")

        try:
            odrive_bridge.update()
        except (fibre.protocol.ChannelBrokenException, AttributeError):
            print("odrive has been unplugged")
            if usblock.locked():
                usblock.release()

            usblock.acquire()
            odrive_bridge.on_event(Event.DISCONNECTED_ODRIVE)
            usblock.release()

    exit()


def lcmThreaderMan():
    lcm_1 = lcm.LCM()
    lcm_1.subscribe("/drive_vel_cmd", drive_vel_cmd_callback)
    while True:
        lcm_1.handle()
        try:
            publish_encoder_msg()
        except (NameError, AttributeError, fibre.protocol.ChannelBrokenException):
            if usblock.locked():
                usblock.release()


states = ["DisconnectedState", "DisarmedState", "ArmedState", "ErrorState"]
# Program states possible - BOOT,  DISARMED, ARMED, ERROR


class Event(Enum):
    DISCONNECTED_ODRIVE = 1
    DISARM_CMD = 2
    ARM_CMD = 3
    ODRIVE_ERROR = 4


class State(object):
    """
    State object which provides some utility functions for the
    individual states within the state machine.
    """

    def __init__(self):
        print('Processing current state:', str(self))

    def on_event(self, event):
        """
        Handle events that are delegated to this State.
        """
        pass

    def __repr__(self):
        """
        Make it so __str__ method can describe the State.
        """
        return self.__str__()

    def __str__(self):
        """
        Returns the name of the State.
        State state
        str(state) = State
        """
        return self.__class__.__name__


class DisconnectedState(State):
    def on_event(self, event):
        """
        Handle events that are delegated to the Disconnected State.
        """
        global modrive
        if (event == Event.ARM_CMD):
            modrive.disarm()
            modrive.reset_watchdog()
            modrive.arm()
            return ArmedState()

        return self


class DisarmedState(State):
    def on_event(self, event):
        """
        Handle events that are delegated to the Disarmed State.
        """
        global modrive
        if (event == Event.DISCONNECTED_ODRIVE):
            return DisconnectedState()

        elif (event == Event.ARM_CMD):
            modrive.arm()
            return ArmedState()

        elif (event == Event.ODRIVE_ERROR):
            return ErrorState()

        return self


class ArmedState(State):
    def on_event(self, event):
        """
        Handle events that are delegated to the Armed State.
        """
        global modrive

        if (event == Event.DISARM_CMD):
            modrive.disarm()
            return DisarmedState()

        elif (event == Event.DISCONNECTED_ODRIVE):
            return DisconnectedState()

        elif (event == Event.ODRIVE_ERROR):
            return ErrorState()

        return self


class ErrorState(State):
    def on_event(self, event):
        """
        Handle events that are delegated to the Error State.
        """
        global modrive
        print(dump_errors(modrive.odrive, True))
        if (event == Event.ODRIVE_ERROR):
            try:
                modrive.reboot()  # only runs after initial pairing
            except:
                print('channel error caught')

            return DisconnectedState()

        return self


class OdriveBridge(object):

    def __init__(self):
        """
        Initialize the components.
        Start with a Default State
        """
        global modrive
        self.state = DisconnectedState()  # default is disarmed
        self.encoder_time = self.left_speed = self.right_speed = 0.0

    def connect(self):
        global modrive, legal_controller
        print("looking for odrive")

        # odrive 0 --> front motors
        # odrive 1 --> middle motors
        # odrive 2 --> back motors

        odrives = ["335D36623539", "335B36563539", "2066377F5753"]

        id = odrives[legal_controller]

        print(id)
        odrive = odv.find_any(serial_number=id)

        print("found odrive")
        usblock.acquire()
        modrive = Modrive(odrive)  # arguments = odr
        modrive.set_current_lim(modrive.CURRENT_LIM)
        usblock.release()
        self.encoder_time = t.time()

    def on_event(self, event):
        """
        Incoming events are
        delegated to the given states which then handle the event.
        The result is then assigned as the new state.
        The events we can send are disarm cmd, arm cmd, and calibrating cmd.
        """

        print("on event called, event:", event)

        self.state = self.state.on_event(event)
        publish_state_msg(state_msg, odrive_bridge.get_state())

    def update(self):
        if (str(self.state) == "ArmedState"):
            try:
                usblock.acquire()
                errors = modrive.check_errors()
                modrive.watchdog_feed()
                usblock.release()

            except (fibre.protocol.ChannelBrokenException, AttributeError):
                if usblock.locked():
                    usblock.release()
                errors = 0
                usblock.acquire()
                self.on_event(Event.DISCONNECTED_ODRIVE)
                usblock.release()
                print("unable to check errors of unplugged odrive")

            if errors:
                usblock.acquire()
                self.on_event(Event.ODRIVE_ERROR)
                usblock.release()
                return

            global speedlock, left_speed, right_speed

            speedlock.acquire()
            self.left_speed, self.right_speed = left_speed, right_speed
            speedlock.release()

            usblock.acquire()
            modrive.set_vel("LEFT", self.left_speed)
            modrive.set_vel("RIGHT", self.right_speed)
            usblock.release()

        elif (str(self.state) == "DisconnectedState"):
            self.connect()
            usblock.acquire()
            self.on_event(Event.ARM_CMD)
            usblock.release()

        elif (str(self.state) == "ErrorState"):
            usblock.acquire()
            self.on_event(Event.ODRIVE_ERROR)
            usblock.release()

    def get_state(self):
        return str(self.state)


"""
call backs
"""


def publish_state_msg(msg, state):
    global legal_controller
    msg.state = states.index(state)
    msg.controller = legal_controller
    lcm_.publish("/drive_state_data", msg.encode())
    print("changed state to " + state)


def publish_encoder_helper(axis):
    global modrive, legal_controller, usblock
    msg = DriveVelData()

    usblock.acquire()
    msg.current_amps = modrive.get_iq_measured(axis)
    msg.vel_percent = modrive.get_vel_estimate(axis)
    usblock.release()

    motor_map = {("LEFT", 0): 0, ("RIGHT", 0): 1,
                 ("LEFT", 1): 2, ("RIGHT", 1): 3,
                 ("LEFT", 2): 4, ("RIGHT", 2): 5}

    msg.axis = motor_map[(axis, legal_controller)]

    lcm_.publish("/drive_vel_data", msg.encode())


def publish_encoder_msg():
    publish_encoder_helper("LEFT")
    publish_encoder_helper("RIGHT")


def drive_vel_cmd_callback(channel, msg):
    # set the odrive's velocity to the float specified in the message
    # no state change

    global speedlock, odrive_bridge
    try:
        cmd = DriveVelCmd.decode(msg)
        if (odrive_bridge.get_state() == "ArmedState"):
            global left_speed, right_speed

            speedlock.acquire()
            left_speed, right_speed = cmd.left, cmd.right
            speedlock.release()
    except NameError:
        pass


if __name__ == "__main__":
    main()


class Modrive:
    CURRENT_LIM = 4

    def __init__(self, odr):
        self.odrive = odr
        self.front_axis = self.odrive.axis0
        self.back_axis = self.odrive.axis1
        self.set_current_lim(self.CURRENT_LIM)

    # viable to set initial state to idle?

    def __getattr__(self, attr):
        if attr in self.__dict__:
            return getattr(self, attr)
        return getattr(self.odrive, attr)

    def enable_watchdog(self):
        try:
            print("Enabling watchdog")
            self.front_axis.config.watchdog_timeout = 0.1
            self.back_axis.config.watchdog_timeout = 0.1
            self.watchdog_feed()
            self.front_axis.config.enable_watchdog = True
            self.back_axis.config.enable_watchdog = True
        except Exception as e:
            print("Failed in enable_watchdog. Error:")
            print(e)

    def disable_watchdog(self):
        try:
            print("Disabling watchdog")
            self.front_axis.config.watchdog_timeout = 0
            self.back_axis.config.watchdog_timeout = 0
            self.front_axis.config.enable_watchdog = False
            self.back_axis.config.enable_watchdog = False
        except fibre.protocol.ChannelBrokenException:
            print("Failed in disable_watchdog. Unplugged")

    def reset_watchdog(self):
        try:
            print("Resetting watchdog")
            self.disable_watchdog()
            # clears errors cleanly
            self.front_axis.error = self.back_axis.error = 0
            self.enable_watchdog()
        except fibre.protocol.ChannelBrokenException:
            print("Failed in disable_watchdog. Unplugged")

    def watchdog_feed(self):
        try:
            self.front_axis.watchdog_feed()
            self.back_axis.watchdog_feed()
        except fibre.protocol.ChannelBrokenException:
            print("Failed in watchdog_feed. Unplugged")

    def disarm(self):
        self.set_current_lim(self.CURRENT_LIM)
        self.closed_loop_ctrl()
        self.set_velocity_ctrl()

        self.set_vel("LEFT", 0)
        self.set_vel("RIGHT", 0)

        self.idle()

    def arm(self):
        self.closed_loop_ctrl()
        self.set_velocity_ctrl()

    def set_current_lim(self, lim):
        self.front_axis.motor.config.current_lim = lim
        self.back_axis.motor.config.current_lim = lim

    def _set_control_mode(self, mode):
        self.front_axis.controller.config.control_mode = mode
        self.back_axis.controller.config.control_mode = mode

    def set_velocity_ctrl(self):
        self._set_control_mode(CONTROL_MODE_VELOCITY_CONTROL)

    def get_iq_measured(self, axis):
        # measured current [Amps]
        if (axis == "LEFT"):
            return self.front_axis.motor.current_control.Iq_measured
        elif(axis == "RIGHT"):
            return self.back_axis.motor.current_control.Iq_measured

    def get_vel_estimate(self, axis):
        # divide by 1.5 to scale by percent
        if (axis == "LEFT"):
            return self.front_axis.encoder.vel_estimate
        elif(axis == "RIGHT"):
            return self.back_axis.encoder.vel_estimate

    def idle(self):
        self._requested_state(AXIS_STATE_IDLE)

    def closed_loop_ctrl(self):
        self._requested_state(AXIS_STATE_CLOSED_LOOP_CONTROL)

    def _requested_state(self, state):
        self.back_axis.requested_state = state
        self.front_axis.requested_state = state

    def set_vel(self, axis, vel):
        if (axis == "LEFT"):
            self.front_axis.controller.input_vel = -vel * 50
        elif axis == "RIGHT":
            self.back_axis.controller.input_vel = vel * 50

    def get_current_state(self):
        return (self.front_axis.current_state, self.back_axis.current_state)

    def check_errors(self):
        return self.front_axis.error + self.back_axis.error
