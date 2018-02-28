import asyncio
import random
import math
import time
from rover_common import heartbeatlib, aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import (Odometry, Joystick, DriveMotors, Sensors,
                        KillSwitch, Xbox, Encoder, Temperature, SAMotors)


class Toggle:
    def __init__(self, toggle):
        self.toggle = toggle
        self.previous = False
        self.input = False
        self.last_input = False

    def new_reading(self, reading):
        self.input = reading
        if self.input and not self.last_input:
            # just pushed
            self.last_input = True
            self.toggle = not self.toggle
        elif not self.input and self.last_input:
            # just released
            self.last_input = False

        self.previous = reading
        return self.toggle


lcm_ = aiolcm.AsyncLCM()
kill_motor = False
lock = asyncio.Lock()
enc_in = None
drill_on = Toggle(False)


def connection_state_changed(c, _):
    if c:
        print("Connection established.")
    else:
        print("Disconnected.")


def deadzone(magnitude, threshold):
    temp_mag = abs(magnitude)
    if temp_mag <= threshold:
        temp_mag = 0
    else:
        temp_mag = (temp_mag - threshold)/(1 - threshold)

    return math.copysign(temp_mag, magnitude)


def joystick_math(new_motor, magnitude, theta):
    new_motor.left = abs(magnitude)
    new_motor.right = new_motor.left

    if theta > 0:
        new_motor.right *= 1 - theta/2
    elif theta < 0:
        new_motor.left *= 1 + theta/2

    if magnitude < 0:
        new_motor.left *= -1
        new_motor.right *= -1
    elif magnitude == 0:
        new_motor.left += theta
        new_motor.right -= theta


def drive_control_callback(channel, msg):
    global kill_motor

    input_data = Joystick.decode(msg)
    input_data.forward_back = -math.copysign(input_data.forward_back ** 2,
                                             input_data.forward_back)

    new_kill_msg = KillSwitch()

    if kill_motor:
        if input_data.restart:
            kill_motor = False
            new_kill_msg.killed = False
            lcm_.publish('/kill_switch', new_kill_msg.encode())
        else:
            return

    damp = (input_data.dampen - 1)/(-2)

    new_motor = DriveMotors()

    if input_data.kill:
        new_motor.left = 0
        new_motor.right = 0
        kill_motor = True
        new_kill_msg.killed = True
        lcm_.publish('/kill_switch', new_kill_msg.encode())

    else:
        magnitude = deadzone(input_data.forward_back, 0.04)
        theta = deadzone(input_data.left_right, 0.1)

        joystick_math(new_motor, magnitude, theta)

        new_motor.left *= damp
        new_motor.right *= damp

    lcm_.publish('/motor', new_motor.encode())


def encoder_callback(channel, msg):
    global enc_in
    enc_in = Encoder.decode(msg)


def arm_control_callback(channel, msg):
    global enc_in
    xbox = Xbox.decode(msg)
    new_encoder = Encoder()
    if enc_in:
        new_encoder = enc_in
        new_encoder.joint_a = (enc_in.joint_a +
                               int(deadzone(xbox.left_js_x, 0.20)*5))
        new_encoder.joint_b = (enc_in.joint_b -
                               int(deadzone(xbox.left_js_y, 0.20)*5))
        new_encoder.joint_c = (enc_in.joint_c +
                               int((xbox.left_trigger -
                                    xbox.right_trigger)*2.5))
        new_encoder.joint_d = (enc_in.joint_d -
                               int(deadzone(xbox.right_js_y, 0.20)*5))
        new_encoder.joint_e = (enc_in.joint_e +
                               int(deadzone(xbox.right_js_x, 0.20)*5))
        new_encoder.joint_f = (enc_in.joint_f +
                               int((xbox.right_bumper -
                                    xbox.left_bumper)*5))
        enc_in = None
        lcm_.publish('/arm_demand', new_encoder.encode())


def sa_control_callback(channel, msg):
    global drill_on, door_open
    xbox = Xbox.decode(msg)
    new_sa_motors = SAMotors()

    val = drill_on.new_reading(xbox.right_bumper > 0.5)
    new_sa_motors.drill = -0.7 if val else 0.0
    new_sa_motors.lead_screw = deadzone(xbox.left_js_y, 0.1)
    new_sa_motors.lead_screw = math.copysign(
        new_sa_motors.lead_screw ** 2,
        new_sa_motors.lead_screw)
    if xbox.d_pad_up:
        new_sa_motors.door_actuator = 0.5
    elif xbox.d_pad_down:
        new_sa_motors.door_actuator = -0.5
    else:
        new_sa_motors.door_actuator = 0
    new_sa_motors.cache = deadzone(xbox.right_js_x, 0.2)

    print('drill? {} door: {} lead-screw: {}'.format(
        drill_on.toggle, new_sa_motors.door_actuator,
        new_sa_motors.lead_screw))

    lcm_.publish('/sa_motors', new_sa_motors.encode())


def autonomous_callback(channel, msg):
    input_data = Joystick.decode(msg)
    new_motor = DriveMotors()

    joystick_math(new_motor, input_data.forward_back, input_data.left_right)

    lcm_.publish('/motor', new_motor.encode())


async def transmit_fake_odometry():
    while True:
        new_odom = Odometry()
        new_odom.latitude_deg = random.randint(42, 43)
        new_odom.longitude_deg = random.randint(-84, -82)
        new_odom.latitude_min = random.uniform(0, 60)
        new_odom.longitude_min = random.uniform(0, 60)
        new_odom.bearing_deg = random.uniform(0, 359)

        with await lock:
            lcm_.publish('/odom', new_odom.encode())

        # print("Published new odometry")
        await asyncio.sleep(0.5)


async def transmit_fake_joystick():
    while True:
        new_joystick = Joystick()
        new_joystick.forward_back = random.uniform(-1, 1)
        new_joystick.left_right = random.uniform(-1, 1)
        new_joystick.dampen = random.uniform(-1, 1)
        new_joystick.kill = False
        new_joystick.restart = False

        with await lock:
            lcm_.publish('/drive_control', new_joystick.encode())

        print("Published new joystick\nfb: {}   lr: {}"
              .format(new_joystick.forward_back, new_joystick.left_right))
        await asyncio.sleep(0.5)


async def transmit_fake_sensors():
    while True:
        new_sensors = Sensors()
        new_sensors.timestamp = int(time.time())
        new_sensors.temperature = random.uniform(0, 100)
        new_sensors.moisture = random.uniform(0, 100)
        new_sensors.pH = random.uniform(0, 14)
        new_sensors.soil_conductivity = random.uniform(0, 100)
        new_sensors.uv = random.uniform(0, 100)

        with await lock:
            lcm_.publish('/sensors', new_sensors.encode())

        # print("Published new fake sensor data")
        # print("temp: {} moisture: {} ph: {} soil: {} uv: "
        #       .format(new_sensors.temperature, new_sensors.moisture,
        #               new_sensors.pH, new_sensors.soil_conductivity,
        #               new_sensors.uv))
        await asyncio.sleep(1)


async def transmit_temperature():
    while True:
        new_temps = Temperature()

        try:
            with open("/sys/class/hwmon/hwmon0/temp1_input", "r") as bcpu_file:
                new_temps.bcpu_temp = int(bcpu_file.read())
            with open("/sys/class/hwmon/hwmon2/temp1_input", "r") as gpu_file:
                new_temps.gpu_temp = int(gpu_file.read())
            with open("/sys/class/hwmon/hwmon4/temp1_input", "r") \
                    as tboard_file:
                new_temps.tboard_temp = int(tboard_file.read())
        except FileNotFoundError:
            print("Temperature files not found")
            return

        with await lock:
            lcm_.publish('/temperature', new_temps.encode())

        print("Published new tempertues")
        print("bcpu temp: {} gpu temp: {} tboard temp: {} ".format(
            new_temps.bcpu_temp/1000, new_temps.gpu_temp/1000,
            new_temps.tboard_temp/1000))
        await asyncio.sleep(1)


def motor_callback(channel, msg):
    input_data = DriveMotors.decode(msg)
    # This function is for testing the joystick-motor algorithm
    print("Left: {}  Right: {}\n".format(input_data.left, input_data.right))


def transmit_fake_encoders(channel, msg):
    lcm_.publish('/encoder', msg)


def enc_out_callback(channel, msg):
    enc = Encoder.decode(msg)
    print("Arm:\nA: {}\nB: {}\nC: {}\nD: {}\nE: {}\nF: {}\n"
          .format(enc.joint_a, enc.joint_b, enc.joint_c,
                  enc.joint_d, enc.joint_e, enc.joint_f))


def main():
    hb = heartbeatlib.OnboardHeartbeater(connection_state_changed, 0)
    # look LCMSubscription.queue_capacity if messages are discarded
    lcm_.subscribe("/drive_control", drive_control_callback)
    lcm_.subscribe("/autonomous", autonomous_callback)
    lcm_.subscribe('/motor', motor_callback)
    lcm_.subscribe('/arm_control', arm_control_callback)
    lcm_.subscribe('/sa_control', sa_control_callback)
    lcm_.subscribe('/encoder', encoder_callback)
    lcm_.subscribe('/arm_demand', enc_out_callback)
    # lcm_.subscribe('/arm_demand', transmit_fake_encoders)

    new_kill_msg = KillSwitch()
    new_kill_msg.killed = False
    lcm_.publish('/kill_switch', new_kill_msg.encode())

    new_encoder = Encoder()
    new_encoder.joint_a = 0
    new_encoder.joint_b = 0
    new_encoder.joint_c = 0
    new_encoder.joint_d = 0
    new_encoder.joint_e = 0
    new_encoder.joint_f = 0
    lcm_.publish('/encoder', new_encoder.encode())

    run_coroutines(hb.loop(), transmit_fake_odometry(),
                   transmit_fake_sensors(), lcm_.loop(),
                   transmit_temperature())
