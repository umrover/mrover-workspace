import lcm	
import sys
import time as t
import odrive as odv
from rover_msgs import ODriver_Req_State, ODriver_Req_Vel, ODriver_Pub_State, ODriver_Pub_Encoders
from odrive.enums import *

import Modrive

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
#							               1		  2	      	3		  4		      5         6
currentState = "BOOT" #starting state
requestedState = "DISARMED" #starting requested state
odrive = None #starting odrive

def nextState():
	#every time the state changes, publish an odrive_state lcm message, with the new state
	global currentState
	global requestedState
	global odrive
	global encoderTime
  
  global states = ["BOOT", "DISARMED", "ARMED", "ERROR", "CALIBRATING", "EXIT"]
  
	msg = odriver_pub_encoders()
	msg1 = odriver_pub_state()

	if (currentState == "BOOT"):
		#attempt to connect to odrive
		odrive = odv.find_any(serial_number=sys.argv[1])
    modrive = Modrive(odrive) #arguments = odr
    
		#Block until odrive is connected
		#set current limit on odrive to 50
		modrive.set_current_lim(legalAxis, 50)
		#set controller's control mode to velocity control
		modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)
		#set currentState to DISARMED
		currentState = "DISARMED"
		msg1.state = 2
		msg1.serialid = sys.argv[1]
		lcm_.publish("/odriver_pub_state", msg.encode())

	elif (currentState == "DISARMED"):
		#if 100 ms have passed since last time data was published
		#	publish an odrive_data lcm message with measured current and estimated velocity
		
		#Unsure if using correct timestamp
		if (encoderTime - t.time() > 0.1):
      #change ODrive_Pub_Encoders.LCM
      #msg.measuredCurrentLeft = modrive.get_iq_measured(LEFT)
      #msg.measuredCurrentRight = modrive.get_iq_measured(RIGHT)

      #Same for vel, velRight and velLeft
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
		elif requestedState == "ARMED":
      modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)
			currentState = "ARMED"
			msg1.state = 3
			msg1.serialid = sys.argv[1]
			lcm_.publish("/odriver_pub_state", msg1.encode())
			encoderTime = t.time()
		elif requestedState == "BOOT":
			odrive.reboot()
			currentState = "BOOT"
			msg1.state = 1
			msg1.serialid = sys.argv[1]
			lcm_.publish("/odriver_pub_state", msg1.encode())
			encoderTime = t.time()
		elif requestedState == "CALIBRATING":
      modrive.requested_state(legalAxis, AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
			currentState == "CALIBRATING"
			msg1.state = 5
			msg1.serialid = sys.argv[1]
			lcm_.publish("/odriver_pub_state", msg1.encode())
			encoderTime = t.time()



	elif (currentState == "ARMED"):
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
			msg.estVel = modrive.axis0.encoder.vel_estimate
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
		elif requestedState == "DISARMED":
			modrive.set_control_mode(legalAxis,AXIS_STATE_IDLE)
			currentState = "DISARMED"
			msg1.state = 2
			msg1.serialid = sys.argv[1]
			lcm_.publish("/odriver_pub_state", msg1.encode())
			encoderTime = t.time()
		elif requestedState == "BOOT":
			odrive.reboot()
			currentState = "BOOT"
			msg1.state = 1
			msg1.serialid = sys.argv[1]
			lcm_.publish("/odriver_pub_state", msg1.encode())
			encoderTime = t.time()
		elif requestedState == "CALIBRATING":
			modrive.requested_state(legalAxis, AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
			currentState = "CALIBRATING"
			msg1.state = 5
			msg1.serialid = sys.argv[1]
			lcm_.publish("/odriver_pub_state", msg1.encode())
			encoderTime = t.time()
		elif (currentState == "ERROR"):
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
      elif requestedState == "CALIBRATING":
        modrive.requested_state(legalAxis, AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
        currentState = "CALIBRATING"
        msg1.state = 5
        msg1.serialid = sys.argv[1]
        lcm_.publish("/odriver_pub_state", msg1.encode())
        encoderTime = t.time()
	elif (currentState == "CALIBRATING"):
		#if odrive is done calibrating
		#	set current limit on odrive to 50
		#	set controller's control mode to velocity control
		#	set currentState to DISARMED

		#We don't know how to check if done calibrating
		#if odrive.

		modrive.set_current_lim(legalAxis, 50)
		modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)
		currentState = "DISARMED"
		msg1.state = 2
		msg1.serialid = sys.argv[1]
		lcm_.publish("/odriver_pub_state", msg1.encode())
		encoderTime = t.time()




def odriver_req_state_callback(channel,msg):
	message = ODriver_Req_State.decode(msg)
	if message.serialid == sys.argv[1]:
    requestedState = states[message.requestState - 1]
    if requestedState == "EXIT":
			if modrive.get_vel_estimate("LEFT") == 0 and modrive.get_current_state("LEFT") == AXIS_STATE_IDLE and 
      										modrive.get_vel_estimate("RIGHT") == 0 and modrive.get_current_state("RIGHT") == AXIS_STATE_IDLE:
					sys.exit()
			else:
          modrive.set_vel(legalAxis, 0)		
          modrive.requested_state(legalAxis, AXIS_STATE_IDLE)
					sys.exit()

    
	
def odriver_req_vel_callback(channel, msg):
	#if the program is in an ARMED state
	#	set the odrive's velocity to the float specified in the message
	#no state change
	message = ODriver_Req_Vel.decode(msg)
	if message.serialid == sys.argv[1] :
		if(currentState == "ARMED"):
      modrive.requested_state(legalAxis, AXIS_STATE_CLOSED_LOOP_CONTROL)
      modrive.set_vel(legalAxis, message.vel)			

if __name__ == "__main__":
	main()