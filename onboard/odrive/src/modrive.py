class Modrive:

    def __init__(self, odr):
        self.odrive = odr
        self.left_axis = self.odrive.axis0
        self.right_axis = self.odrive.axis1

    # viable to set initial state to idle?

    def __getattr__(self, attr):
        if attr in self.__dict__:
            return getattr(self, attr)
        return getattr(self.odrive, attr)

    def set_current_lim(self, axis, lim):
        if (axis == "LEFT"):
            self.left_axis.motor.config.current_lim = lim
        elif (axis == "RIGHT"):
            self.right_axis.motor.config.current_lim = lim
        else:
            self.left_axis.motor.config.current_lim = lim
            self.right_axis.motor.config.current_lim = lim

    # odrive.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

    def set_control_mode(self, axis, mode):
        if (axis == "LEFT"):
            self.left_axis.controller.config.control_mode = mode
        elif (axis == "RIGHT"):
            self.right_axis.controller.config.control_mode = mode
        elif (axis == "BOTH"):
            self.left_axis.controller.config.control_mode = mode
            self.right_axis.controller.config.control_mode = mode

    # odrive.axis0.motor.current_control.Iq_measured
    def get_iq_measured(self, axis):
        if (axis == "LEFT"):
            return self.left_axis.motor.current_control.Iq_measured
        elif(axis == "RIGHT"):
            return self.right_axis.motor.current_control.Iq_measured
        else:
            print("ERROR: cant get the measured iq for both motors at once")
            return 0

    # odrive.axis0.encoder.vel_estimate
    def get_vel_estimate(self, axis):
        if (axis == "LEFT"):
            return self.left_axis.encoder.vel_estimate
        elif(axis == "RIGHT"):
            return self.right_axis.encoder.vel_estimate
        else:
            print("ERROR: cant get the velocity estimate for both motors at \
                    once")
            return 0

    # odrive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    # odrive.axis0.requested_state = AXIS_STATE_IDLE
    # odrive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    def requested_state(self, axis, state):
        if (axis == "LEFT"):
            self.left_axis.requested_state = state
        elif (axis == "RIGHT"):
                self.right_axis.requested_state = state
        else:
            self.right_axis.requested_state = state
            self.left_axis.requested_state = state

    # odrive.axis0.encoder.vel_estimate == 0

    def est_vel(self, axis):
        if (axis == "LEFT"):
            return self.left_axis.encoder.vel_estimate
        elif (axis == "RIGHT"):
            return self.right_axis.encoder.vel_estimate

    def set_vel(self, axis, vel):
        if (axis == "LEFT"):
            self.left_axis.controller.vel_setpoint = vel
        elif axis == "RIGHT":
            self.right_axis.controller.vel_setpoint = vel
        elif axis == "BOTH":
            self.left_axis.controller.vel_setpoint = vel
            self.right_axis.controller.vel_setpoint = vel
        else:
            print("ERROR, unknown axis")

    def get_current_state(self, axis):
        if (axis == "LEFT"):
            return self.left_axis.current_state
        elif(axis == "RIGHT"):
            return self.right_axis.current_state
        else:
            print("cant get current state of both axes at once")
            return 0
