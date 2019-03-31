class Toggle {
	constructor(init_state) {
		this.toggle = init_state
		this.previous = false
		this.input = false
		this.last_input = false
	}

	new_reading(reading) {
		this.input = reading
		if(this.input && !this.last_input) {
			//Just pushed
			this.last_input = true
			this.toggle = !this.toggle
		} else if(!this.input && this.last_input) {
			//Just released
			this.last_input = false;
		}

		this.previous = reading;
		return this.toggle;
	}
}

const quadratic = function(val) {
	return val*val * Math.sign(val)
}

const deadzone = function(magnitude, threshold) {
	let temp_mag = Math.abs(magnitude)
	if(temp_mag <= threshold) {
		temp_mag = 0
	} else {
		temp_mag = (temp_mag - threshold)/(1-threshold)
	}

	return temp_mag * Math.sign(magnitude)
}

const joystick_math =  function(new_motor, magnitude, theta) {
	new_motor.left = Math.abs(magnitude)
	new_motor.right = new_motor.left

	if(theta > 0) {
		new_motor.right *= 1 - (theta * 0.75)
	}else if(theta < 0){
		new_motor.left *= 1 + (theta * 0.75)
	}

	if(magnitude < 0){
		new_motor.left *= -1
        new_motor.right *= -1
	} else if(magnitude == 0) {
        new_motor.left += theta
        new_motor.right -= theta
	}
}

export {Toggle, quadratic, deadzone, joystick_math}