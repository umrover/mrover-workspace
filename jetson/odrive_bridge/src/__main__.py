import lcm
import sys
import time as t
import odrive as odv
import threading
from rover_msgs import DriveStateCmd, DriveVelCmd, \
    DriveStateData, DriveVelData
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, \
    CONTROL_MODE_VELOCITY_CONTROL, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, \
    AXIS_STATE_IDLE, ENCODER_MODE_HALL

from odrive.utils import dump_errors


def main():
    global lcm_
    lcm_ = lcm.LCM()

    global modrive
    global left_speed
    global right_speed

    left_speed = 0.0
    right_speed = 0.0

    global legal_controller
    global legal_axis

    global vel_msg
    global state_msg

    global lock
    global speedlock

    global start_time
    global watchdog

    start_time = t.clock()

    legal_controller = int(sys.argv[1])
    legal_axis = sys.argv[2]

    vel_msg = DriveVelData()
    state_msg = DriveStateData()

    speedlock = threading.Lock()
    lock = threading.Lock()

    threading._start_new_thread(lcmThreaderMan, ())
    global odrive_bridge
    odrive_bridge = OdriveBridge()
    # starting state is DisconnectedState()
    # start up sequence is called, disconnected-->disarm-->arm

    while True:
        watchdog = t.clock() - start_time
        if (watchdog > 1.0):
            print("loss of comms")
            left_speed = 0
            right_speed = 0

        try:
            odrive_bridge.update()
        except Exception:
            print("CRASH!")
            lock.acquire()
            odrive_bridge.on_event("disconnected odrive")
            lock.release()

    exit()


def lcmThreaderMan():
    lcm_1 = lcm.LCM()
    lcm_1.subscribe("/drive_state_cmd", drive_state_cmd_callback)
    lcm_1.subscribe("/drive_vel_cmd", drive_vel_cmd_callback)
    while True:
        lcm_1.handle()
        global start_time
        start_time = t.clock()
        try:
            publish_encoder_msg()
        except NameError:
            pass
        except AttributeError:
            pass
        except Exception:
            pass


events = ["disconnected odrive", "disarm cmd", "arm cmd", "calibrating cmd", "odrive error"]
states = ["DisconnectedState", "DisarmedState", "ArmedState", "CalibrateState", "ErrorState"]
# Program states possible - BOOT,  DISARMED, ARMED, CALIBRATE ERROR
# 							1		 2	      3	       4        5


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
        try:
            if (event == "arm cmd"):
                modrive.disarm()
                modrive.reset_watchdog()
                modrive.arm()
                modrive.set_velocity_ctrl()
                return ArmedState()
        except:
            print("trying to arm")

        return self


class DisarmedState(State):
    def on_event(self, event):
        """
        Handle events that are delegated to the Disarmed State.
        """
        global modrive
        if (event == "disconnected odrive"):
            return DisconnectedState()

        elif (event == "arm cmd"):
            modrive.arm()
            return ArmedState()

        elif (event == "calibrating cmd"):
            # sequence can be moved to armed ?
            return DisarmedState()

        elif (event == "odrive error"):
            return ErrorState()

        return self


class ArmedState(State):
    def __init__(self):
        global modrive
        modrive.watchdog_feed()

    def on_event(self, event):
        """
        Handle events that are delegated to the Armed State.
        """
        global modrive

        if (event == "disarm cmd"):
            modrive.disarm()
            return DisarmedState()

        elif (event == "disconnected odrive"):
            return DisconnectedState()

        elif (event == "odrive error"):
            return ErrorState()

        elif (event == "calibrating cmd"):
            modrive.reset()
            return CalibrateState()

        return self


class CalibrateState(State):
    def on_event(self, event):
        global modrive

        if (event == "arm cmd"):
            modrive.arm()
            return ArmedState()

        if(modrive.check_errors()):
            print("clearing calibration errors")
            dump_errors(modrive.odrive, True)

        return self


class ErrorState(State):
    def on_event(self, event):
        """
        Handle events that are delegated to the Error State.
        """
        global modrive
        dump_errors(modrive.odrive, True)
        modrive.watchdog_feed()
        if (event == "odrive error"):
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
        self.encoder_time = 0

    def connect(self):
        global modrive
        global legal_controller
        print("looking for odrive")

        # odrive 0 --> front motors
        # odrive 1 --> middle motors
        # odrive 2 --> back motors

        odrives = ["206E37635753", "2091358E524B", "2084399C4D4D"]
        id = odrives[legal_controller]

        print(id)
        odrive = odv.find_any(serial_number=id)

        print("found odrive")
        modrive = Modrive(odrive)  # arguments = odr
        modrive.set_current_lim(100)
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
        print(str(self.state))

        if (str(self.state) != "DisconnectedState"):
            modrive.watchdog_feed()
            modrive.print_debug()

        if (str(self.state) == "ArmedState"):

            global speedlock
            global left_speed
            global right_speed
            print("speed lock acquiring in armed state here")
            speedlock.acquire()
            print("speed lock acquired in armed state here")
            try:
                modrive.set_vel("LEFT", left_speed)
                modrive.set_vel("RIGHT", right_speed)
                modrive.watchdog_feed()
            except Exception:
                print("trying to arm unplugged odrive failed")
            speedlock.release()
            print("speed lock released armed state")

        elif (str(self.state) == "DisconnectedState"):
            self.connect()
            lock.acquire()
            self.on_event("arm cmd")
            lock.release()

        elif (str(self.state) == "CalibrateState"):
            self.connect()
            modrive.calibrate()
            self.connect()
            print("done calibrating")

            lock.acquire()
            self.on_event("arm cmd")
            lock.release()

        errors = modrive.check_errors()
        if errors:
            # if (errors == 0x800 or erros == 0x1000):

            lock.acquire()
            self.on_event("odrive error")
            lock.release()
            # first time will set to ErrorState
            # second time will reboot
            # because the error flag is still true

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
    global modrive
    global legal_controller
    msg = DriveVelData()
    msg.measuredCurrent = modrive.get_iq_measured(axis)
    msg.estimatedVel = modrive.get_vel_estimate(axis)

    motor_map = {("LEFT", 0): 0, ("RIGHT", 0): 1,
                 ("LEFT", 1): 2, ("RIGHT", 1): 3,
                 ("LEFT", 2): 4, ("RIGHT", 2): 5}

    msg.axis = motor_map[(axis, legal_controller)]

    lcm_.publish("/drive_vel_data", msg.encode())


def publish_encoder_msg():
    publish_encoder_helper("LEFT")
    publish_encoder_helper("RIGHT")


def drive_state_cmd_callback(channel, msg):
    print("requested state call back is being called")
    global odrive_bridge
    global legal_controller

    command_list = ["disarm cmd", "arm cmd", "calibrating cmd"]
    cmd = DriveStateCmd.decode(msg)
    if cmd.controller == legal_controller:  # Check which controller
        lock.acquire()
        odrive_bridge.on_event(command_list[cmd.state - 1])
        lock.release()


def drive_vel_cmd_callback(channel, msg):
    # set the odrive's velocity to the float specified in the message
    # no state change

    global speedlock
    global odrive_bridge
    try:
        cmd = DriveVelCmd.decode(msg)
        if (odrive_bridge.get_state() == "ArmedState"):
            global left_speed
            global right_speed

            print("speed lock acquiring callback")
            speedlock.acquire()
            print("speed lock acquired callback")
            left_speed = cmd.left
            right_speed = cmd.right
            speedlock.release()
            print("speed lock released callback")
    except NameError:
        pass


if __name__ == "__main__":
    main()


class Modrive:
    CURRENT_LIM = 30

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

    def reset(self):
        self._reset(self.front_axis)
        self._reset(self.back_axis)
        self.odrive.save_configuration()
        # the guide says to reboot here...

    def print_debug(self):
        try:
            print("Print control mode")
            print(self.front_axis.controller.config.control_mode)
            print(self.back_axis.controller.config.control_mode)
            print("Printing requested state")
            print(self.front_axis.current_state)
            print(self.back_axis.current_state)
        except Exception as e:
            print("Failed in print_debug. Error:")
            print(e)

    def enable_watchdog(self):
        try:
            print("Enabling watchdog")
            self.front_axis.config.watchdog_timeout = 1
            self.back_axis.config.watchdog_timeout = 1
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
        except Exception as e:
            print("Failed in disable_watchdog. Error:")
            print(e)

    def reset_watchdog(self):
        try:
            print("Resetting watchdog")
            self.disable_watchdog()
            dump_errors(self.odrive, True)
            self.enable_watchdog()
        except Exception as e:
            print("Failed in disable_watchdog. Error:")
            print(e)

    def watchdog_feed(self):
        try:
            self.front_axis.watchdog_feed()
            self.back_axis.watchdog_feed()
        except Exception as e:
            print("Failed in watchdog_feed. Error:")
            print(e)


    def calibrate(self):
        dump_errors(self.odrive, True)  # clears all odrive encoder errors
        self._requested_state(AXIS_STATE_FULL_CALIBRATION_SEQUENCE)

        front_state, back_state = self.get_current_state()

        # if both axes are idle it means its done calibrating
        while(front_state != AXIS_STATE_IDLE
                or back_state != AXIS_STATE_IDLE):
            front_state, back_state = self.get_current_state()
            pass

        self._pre_calibrate(self.front_axis)
        self._pre_calibrate(self.back_axis)
        self.odrive.save_configuration()
        # also says to reboot here...

    def disarm(self):
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
        # axis = self.odrive[axis_number]
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
        global legal_controller
        if (axis == "LEFT"):
            # TEMPORARY FIX FOR ROLLING ROVER SINCE
            # middle left odrive IS 2x more than the rest bc of the 48V maxon
            # TODO - fix when this is no longer the case!
            if (legal_controller == 1):
                self.front_axis.controller.input_vel = vel * 100
            else:
                self.front_axis.controller.input_vel = vel * 50
        elif axis == "RIGHT":
            self.back_axis.controller.input_vel = vel * -50

    def get_current_state(self):
        return (self.front_axis.current_state, self.back_axis.current_state)

    def _reset(self, m_axis):
        m_axis.motor.config.pole_pairs = 15
        m_axis.motor.config.resistance_calib_max_voltage = 4
        m_axis.motor.config.requested_current_range = 25
        m_axis.motor.config.current_control_bandwidth = 100

        m_axis.encoder.config.mode = ENCODER_MODE_HALL
        m_axis.encoder.config.cpr = 90
        m_axis.encoder.config.bandwidth = 100
        m_axis.controller.config.pos_gain = 1
        m_axis.controller.config.vel_gain = 0.02
        m_axis.controller.config.vel_integrator_gain = 0.1
        m_axis.controller.config.vel_limit = 1000
        m_axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

    def _pre_calibrate(self, m_axis):
        m_axis.motor.config.pre_calibrated = True
        m_axis.encoder.config.pre_calibrated = True

    def check_errors(self):
        front = self.front_axis.error
        back = self.back_axis.error
        return back + front


