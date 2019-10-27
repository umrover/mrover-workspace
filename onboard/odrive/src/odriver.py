import lcm	
import sys
import time as t
import odrive as odv
from rover_msgs import ODriver_Req_State, ODriver_Req_Vel, ODriver_Pub_State, ODriver_Pub_Encoders

def main():
	lcm_ = lcm.LCM()

	#insert lcm subscribe statements here
	
	#These have been mapped
	lcm_.subscribe("/odriver_req_state", odriver_req_state_callback)
	
	lcm_.subscribe("/odrive_req_vel", odriver_req_vel_callback)

	global legalAxis = sys.argv[2]
	
	while True:
		lcm_.handle()
		nextState()

	exit()

#Program states possible - BOOT, DISARMED, ARMED, ERROR, CALIBRATING, EXIT
#							1		2		3		4		5         6
currentState = "BOOT" #starting state
requestedState = "DISARMED" #starting requested state
odrive = None #starting odrive

def nextState():
	#every time the state changes, publish an odrive_state lcm message, with the new state
	global currentState
	global requestedState
	global odrive
	global encoderTime
	msg = odriver_pub_encoders()
	msg1 = odriver_pub_state()

	if (currentState == "BOOT"):
		#attempt to connect to odrive
		odrive = odv.find_any(serial_number=sys.argv[1])
		#Block until odrive is connected
		#set current limit on odrive to 50
		odrive.axis0.motor.config.current_lim = 50
		#set controller's control mode to velocity control
		odrive.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
		#set currentState to DISARMED
		currentState = "DISARMED"
		msg1.state = 2
		msg1.serialid = sys.argv[1]
		lcm_.publish("/odriver_pub_state", msg.encode())

	else if (currentState == "DISARMED"):
		#if 100 ms have passed since last time data was published
		#	publish an odrive_data lcm message with measured current and estimated velocity
		
		#Unsure if using correct timestamp
		if (encoderTime - t.time() > 0.1):
			msg.measuredCurrent = odrive.axis0.motor.current_control.Iq_measured
			msg.estVel = odrive.axis0.encoder.vel_estimate
			msg.serialid = sys.argv[1]
			lcm_.publish("/odriver_pub_encoders", msg)
			encoderTime = t.time()
		#check odrive's dump errors functionality

		#unsure if this is right
		
		#if there are errors
		#	set currentState to ERROR
		#else if requestedState is ARMED
		#	configure odrive for velocity control (starting with velocity 0)
		#	set currentState to ARMED
		#else if requested state is BOOT
		#	reboot the odrive
		#   set currentState to BOOT
		#else if requested state is CALIBRATING
		#	have the odrive recalibrate
		#	set currentState to CALIBRATING
		errors = odv.dump_errors(odrive)
		if not errors:
			currentState = "ERROR"
			msg1.state = 4
			msg1.serialid = sys.argv[1]
			lcm_.publish("/odriver_pub_state", msg1.encode())
			encoderTime = t.time()
		else if requestedState == "ARMED":
			odrive.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
			currentState = "ARMED"
			msg1.state = 3
			msg1.serialid = sys.argv[1]
			lcm_.publish("/odriver_pub_state", msg1.encode())
			encoderTime = t.time()
		else if requestedState == "BOOT":
			odrive.reboot()
			currentState = "BOOT"
			msg1.state = 1
			msg1.serialid = sys.argv[1]
			lcm_.publish("/odriver_pub_state", msg1.encode())
			encoderTime = t.time()
		else if requestedState == "CALIBRATING":
			odrive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
			currentState == "CALIBRATING"
			msg1.state = 5
			msg1.serialid = sys.argv[1]
			lcm_.publish("/odriver_pub_state", msg1.encode())
			encoderTime = t.time()



	else if (currentState == "ARMED"):
		#check odrive's dump errors functionality
		#if there are errors
		#	set currentState to ERROR
		#else if requestedState is DISARMED
		#	configure odrive for idling
		#	set currentState to DISARMED
		#else if requestedState is BOOT
		#	reboot the odrive
		#	set currentState to BOOT
		#else if requested state is CALIBRATING
		#	have the odrive recalibrate
		#	set currentState to CALIBRATING

		if (encoderTime - t.time() > 0.1):
			msg.measuredCurrent = odrive.axis0.motor.current_control.Iq_measured
			msg.estVel = odrive.axis0.encoder.vel_estimate
			msg.serialid = sys.argv[1]
			lcm_.publish("/odriver_pub_encoders", msg)
			encoderTime = t.time()
		errors = odv.dump_errors(odrive)
		if not errors:
			currentState = "ERROR"
			msg1.state = 4
			msg1.serialid = sys.argv[1]
			lcm_.publish("/odriver_pub_state", msg1.encode())
			encoderTime = t.time()
		else if requestedState == "DISARMED":
			odrive.axis0.requested_state = AXIS_STATE_IDLE
			currentState = "DISARMED"
			msg1.state = 2
			msg1.serialid = sys.argv[1]
			lcm_.publish("/odriver_pub_state", msg1.encode())
			encoderTime = t.time()
		else if requestedState == "BOOT":
			odrive.reboot()
			currentState = "BOOT"
			msg1.state = 1
			msg1.serialid = sys.argv[1]
			lcm_.publish("/odriver_pub_state", msg1.encode())
			encoderTime = t.time()
		else if requestedState == "CALIBRATING":
			odrive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
			currentState == "CALIBRATING"
			msg1.state = 5
			msg1.serialid = sys.argv[1]
			lcm_.publish("/odriver_pub_state", msg1.encode())
			encoderTime = t.time()
		else if (currentState == "ERROR"):
		#if requestedState is BOOT
		#	reboot the odrive
		#	set currentState to BOOT
		#else if requested state is CALIBRATING
		#	have the odrive recalibrate
		#	set currentState to CALIBRATING
		if requestedState == "BOOT":
			odrive.reboot()
			currentState = "BOOT"
			msg1.state = 1
			msg1.serialid = sys.argv[1]
			lcm_.publish("/odriver_pub_state", msg1.encode())
			encoderTime = t.time()
		else if requestedState == "CALIBRATING":
			odrive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
			currentState == "CALIBRATING"
			msg1.state = 5
			msg1.serialid = sys.argv[1]
			lcm_.publish("/odriver_pub_state", msg1.encode())
			encoderTime = t.time()
	else if (currentState == "CALIBRATING"):
		#if odrive is done calibrating
		#	set current limit on odrive to 50
		#	set controller's control mode to velocity control
		#	set currentState to DISARMED

		#We don't know how to check if done calibrating
		#if odrive.

		odrive.axis0.motor.config.current_lim = 50
		odrive.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
		currentState = "DISARMED"
		msg1.state = 2
		msg1.serialid = sys.argv[1]
		lcm_.publish("/odriver_pub_state", msg1.encode())
		encoderTime = t.time()




def odriver_req_state_callback(channel,msg):
	message = ODriver_Req_State.decode(msg)
	if message.serialid = sys.argv[1] :
		if message.requestState = 2:
			requestedState = "ARMED"
		else if message.requestState = 3:
			requestedState = "DISARMED"
		else if message.requestState = 5:
			requestedState = "CALIBRATING"
		else if message.requestState = 1:
			requestedState = "BOOT"
		else if message.requestState = 6:
			requestedState = "EXIT"

			if odrive.axis0.encoder.vel_estimate = 0:
				if axis0.current_state = AXIS_STATE_IDLE
					sys.exit()
				else:
					axis0.requested_state= AXIS_STATE_IDLE
					sys.exit()


	
def odriver_req_vel_callback(channel, msg):
	#if the program is in an ARMED state
	#	set the odrive's velocity to the float specified in the message
	#no state change
	message = ODriver_Req_Vel.decode(msg)
	if message.serialid = sys.argv[1] :
		if(currentState == "ARMED"):
			odrive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
			odrive.axis0.controller.vel_setpoint = message.vel
			

if __name__ == "__main__":
	main()