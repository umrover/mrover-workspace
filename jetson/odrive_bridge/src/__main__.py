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
    global lcm_, modrive, left_speed, right_speed, odrive_controller_index, \
        vel_msg, state_msg, usb_lock, speed_lock, start_time, watchdog, odrive_bridge

    lcm_, left_speed, right_speed, start_time, odrive_controller_index, \
        vel_msg, state_msg = lcm.LCM(), 0.0, 0.0, t.clock(), int(sys.argv[1]), DriveVelData(), DriveStateData()

    speed_lock = threading.Lock()
    usb_lock = threading.Lock()

    threading._start_new_thread(lcm_publisher_thread, ())
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

            speed_lock.acquire()
            left_speed = right_speed = 0
            speed_lock.release()
        else:
            if not prev_comms:
                prev_comms = True
                print("regained comms")

        try:
            odrive_bridge.update()
        except Exception:
            if usb_lock.locked():
                usb_lock.release()

            usb_lock.acquire()
            odrive_bridge.bridge_on_event(OdriveEvent.DISCONNECTED_ODRIVE)
            usb_lock.release()

    exit()


def lcm_publisher_thread():
    lcm_pub = lcm.LCM()
    lcm_pub.subscribe("/drive_vel_cmd", drive_vel_cmd_callback)
    while True:
        lcm_pub.handle()
        global start_time
        start_time = t.clock()
        try:
            publish_encoder_msg()
        except Exception:
            if usb_lock.locked():
                usb_lock.release()


states = ["DisconnectedState", "DisarmedState", "ArmedState", "ErrorState"]
# Program states possible - DISCONNECTED,  DISARMED, ARMED, ERROR


class OdriveEvent(Enum):
    DISCONNECTED_ODRIVE = 1
    DISARM_CMD = 2
    ARM_CMD = 3
    ODRIVE_ERROR = 4


class Axis(Enum):
    LEFT = 0
    RIGHT = 1


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
        if (event == OdriveEvent.ARM_CMD):
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
        if (event == OdriveEvent.DISCONNECTED_ODRIVE):
            return DisconnectedState()

        elif (event == OdriveEvent.ARM_CMD):
            modrive.arm()
            return ArmedState()

        elif (event == OdriveEvent.ODRIVE_ERROR):
            return ErrorState()

        return self


class ArmedState(State):
    def on_event(self, event):
        """
        Handle events that are delegated to the Armed State.
        """
        global modrive

        if (event == OdriveEvent.DISARM_CMD):
            modrive.disarm()
            return DisarmedState()

        elif (event == OdriveEvent.DISCONNECTED_ODRIVE):
            return DisconnectedState()

        elif (event == OdriveEvent.ODRIVE_ERROR):
            return ErrorState()

        return self


class ErrorState(State):
    def on_event(self, event):
        """
        Handle events that are delegated to the Error State.
        """
        global modrive
        print(dump_errors(modrive.odrive, True))
        if (event == OdriveEvent.ODRIVE_ERROR):
            try:
                modrive.reboot()  # only runs after initial pairing
            except Exception:
                pass

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
        self.left_speed = self.right_speed = 0.0

    def connect(self):
        global modrive, odrive_controller_index
        print("looking for odrive")

        # odrive 0 --> front motors
        # odrive 1 --> middle motors
        # odrive 2 --> back motors

        odrives = ["335D36623539", "335B36563539", "2066377F5753"]

        id = odrives[odrive_controller_index]

        print(id)
        odrive = odv.find_any(serial_number=id)

        print("found odrive")
        usb_lock.acquire()
        modrive = Modrive(odrive)  # arguments = odr
        modrive.set_current_lim(modrive.CURRENT_LIM)
        usb_lock.release()

    def bridge_on_event(self, event):
        """
        Incoming events are
        delegated to the given states which then handle the event.
        The result is then assigned as the new state.
        The events we can send are disarm cmd, arm cmd, and calibrating cmd.
        """

        print("on event called, odrive event:", event)

        self.state = self.state.on_event(event)
        publish_state_msg(state_msg, odrive_bridge.get_state_string())

    def update(self):
        if (str(self.state) == "ArmedState"):
            try:
                usb_lock.acquire()
                errors = modrive.check_errors()
                modrive.watchdog_feed()
                usb_lock.release()

            except Exception:
                if usb_lock.locked():
                    usb_lock.release()
                errors = 0
                usb_lock.acquire()
                self.bridge_on_event(OdriveEvent.DISCONNECTED_ODRIVE)
                usb_lock.release()

            if errors:
                usb_lock.acquire()
                self.bridge_on_event(OdriveEvent.ODRIVE_ERROR)
                usb_lock.release()
                return

            global speed_lock, left_speed, right_speed

            speed_lock.acquire()
            self.left_speed, self.right_speed = left_speed, right_speed
            speed_lock.release()

            usb_lock.acquire()
            modrive.set_vel(Axis.LEFT, self.left_speed)
            modrive.set_vel(Axis.RIGHT, self.right_speed)
            usb_lock.release()

        elif (str(self.state) == "DisconnectedState"):
            self.connect()
            usb_lock.acquire()
            self.bridge_on_event(OdriveEvent.ARM_CMD)
            usb_lock.release()

        elif (str(self.state) == "ErrorState"):
            usb_lock.acquire()
            self.bridge_on_event(OdriveEvent.ODRIVE_ERROR)
            usb_lock.release()

    def get_state_string(self):
        return str(self.state)


"""
call backs
"""


def publish_state_msg(msg, state_string):
    global odrive_controller_index
    # Shortens the state string which is of the form "[insert_odrive_state]State"
    # e.g. state_string is ErrorState, so short_state_string is Error
    msg.state = state_string[:len(state_string) - len("State")]
    msg.odrive_index = odrive_controller_index
    lcm_.publish("/drive_state_data", msg.encode())
    print("changed state to " + state_string)


def publish_encoder_helper(axis):
    global modrive, odrive_controller_index, usb_lock
    msg = DriveVelData()

    usb_lock.acquire()
    msg.current_amps = modrive.get_iq_measured(axis)
    msg.vel_m_s = modrive.get_vel_estimate(axis)
    usb_lock.release()

    motor_map = {(Axis.LEFT, 0): 0, (Axis.RIGHT, 0): 1,
                 (Axis.LEFT, 1): 2, (Axis.RIGHT, 1): 3,
                 (Axis.LEFT, 2): 4, (Axis.RIGHT, 2): 5}

    msg.axis = motor_map[(axis, odrive_controller_index)]

    lcm_.publish("/drive_vel_data", msg.encode())


def publish_encoder_msg():
    publish_encoder_helper(Axis.LEFT)
    publish_encoder_helper(Axis.RIGHT)


def drive_vel_cmd_callback(channel, msg):
    # set the odrive's velocity to the float specified in the message
    # no state change

    global speed_lock, odrive_bridge
    try:
        cmd = DriveVelCmd.decode(msg)
        if (odrive_bridge.get_state_string() == "ArmedState"):
            global left_speed, right_speed

            speed_lock.acquire()
            left_speed, right_speed = cmd.left, cmd.right
            speed_lock.release()
    except Exception:
        pass


if __name__ == "__main__":
    main()


class Modrive:
    CURRENT_LIM = 4
    # scales normalized inputs to max physical speed of rover in turn/s
    SPEED_MULTIPLIER = 50
    TURNS_TO_M_S_MULTIPLIER = 0.02513  # from turns/sec to m/s (for 2022 rover)

    def __init__(self, odr):
        self.odrive = odr
        self.left_axis = self.odrive.axis0
        self.right_axis = self.odrive.axis1
        self.set_current_lim(self.CURRENT_LIM)

    # viable to set initial state to idle?

    def __getattr__(self, attr):
        if attr in self.__dict__:
            return getattr(self, attr)
        return getattr(self.odrive, attr)

    def enable_watchdog(self):
        try:
            print("Enabling watchdog")
            self.left_axis.config.watchdog_timeout = 0.1
            self.right_axis.config.watchdog_timeout = 0.1
            self.watchdog_feed()
            self.left_axis.config.enable_watchdog = True
            self.right_axis.config.enable_watchdog = True
        except Exception as e:
            print(e)

    def disable_watchdog(self):
        try:
            print("Disabling watchdog")
            self.left_axis.config.watchdog_timeout = 0
            self.right_axis.config.watchdog_timeout = 0
            self.left_axis.config.enable_watchdog = False
            self.right_axis.config.enable_watchdog = False
        except fibre.protocol.ChannelBrokenException:
            print("Failed in disable_watchdog. Unplugged")

    def reset_watchdog(self):
        try:
            print("Resetting watchdog")
            self.disable_watchdog()
            # clears errors cleanly
            self.left_axis.error = self.right_axis.error = 0
            self.enable_watchdog()
        except fibre.protocol.ChannelBrokenException:
            print("Failed in disable_watchdog. Unplugged")

    def watchdog_feed(self):
        try:
            self.left_axis.watchdog_feed()
            self.right_axis.watchdog_feed()
        except fibre.protocol.ChannelBrokenException:
            print("Failed in watchdog_feed. Unplugged")

    def disarm(self):
        self.set_current_lim(self.CURRENT_LIM)
        self.closed_loop_ctrl()
        self.set_velocity_ctrl()

        self.set_vel(Axis.LEFT, 0)
        self.set_vel(Axis.RIGHT, 0)

        self.idle()

    def arm(self):
        self.closed_loop_ctrl()
        self.set_velocity_ctrl()

    def set_current_lim(self, lim):
        self.left_axis.motor.config.current_lim = lim
        self.right_axis.motor.config.current_lim = lim

    def _set_control_mode(self, mode):
        self.left_axis.controller.config.control_mode = mode
        self.right_axis.controller.config.control_mode = mode

    def set_velocity_ctrl(self):
        self._set_control_mode(CONTROL_MODE_VELOCITY_CONTROL)

    def get_iq_measured(self, axis):
        # measured current [Amps]
        if (axis == Axis.LEFT):
            return self.left_axis.motor.current_control.Iq_measured
        elif(axis == Axis.RIGHT):
            return self.right_axis.motor.current_control.Iq_measured

    def get_vel_estimate(self, axis):
        if (axis == Axis.LEFT):
            return self.left_axis.encoder.vel_estimate * self.TURNS_TO_M_S_MULTIPLIER
        elif (axis == Axis.RIGHT):
            return self.right_axis.encoder.vel_estimate * -self.TURNS_TO_M_S_MULTIPLIER

    def idle(self):
        self._requested_state(AXIS_STATE_IDLE)

    def closed_loop_ctrl(self):
        self._requested_state(AXIS_STATE_CLOSED_LOOP_CONTROL)

    def _requested_state(self, state):
        self.right_axis.requested_state = state
        self.left_axis.requested_state = state

    def set_vel(self, axis, vel):
        if (axis == Axis.LEFT):
            self.left_axis.controller.input_vel = -vel * self.SPEED_MULTIPLIER
        elif (axis == Axis.RIGHT):
            self.right_axis.controller.input_vel = vel * self.SPEED_MULTIPLIER

    def get_current_state(self):
        return (self.left_axis.current_state, self.right_axis.current_state)

    def check_errors(self):
        return self.left_axis.error + self.right_axis.error
