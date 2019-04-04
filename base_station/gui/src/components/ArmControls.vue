<template>
  <div class="wrap">
    <div class="control_buttons">
        <Checkbox ref="arm" v-bind:name="'Arm Controls'" v-on:toggle="updateControlMode('arm', $event)"/>
        <Checkbox ref="arm_ik" v-bind:name="'Inverse Kinematics'" v-on:toggle="updateControlMode('arm_ik', $event)"/>
    </div>
  </div>
</template>

<script>
import Checkbox from './Checkbox.vue'
import { mapGetters, mapMutations } from 'vuex'

import {Toggle, quadratic, deadzone, joystick_math} from '../utils.js'

let interval;

export default {

  computed: {

    ...mapGetters('controls', {
      controlMode: 'controlMode'
    }),
  },

  beforeDestroy: function () {
    window.clearInterval(interval);
  },


  created: function () {

    const XBOX_CONFIG = {
      'left_js_x': 0,
      'left_js_y': 1,
      'left_trigger': 6,
      'right_trigger': 7,
      'right_js_x': 2,
      'right_js_y': 3,
      'right_bumper': 5,
      'left_bumper': 4,
      'd_pad_up': 12,
      'd_pad_down': 13,
      'd_pad_right': 14,
      'd_pad_left': 15,
      'a': 0,
      'b': 1,
      'y': 3
    }

    const solenoid_toggle = new Toggle(false)
    const electromagnet_toggle = new Toggle(false)
    const laser_toggle = new Toggle(false)

    const updateRate = 0.05
    interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      for (let i = 0; i < 4; i++) {
        const gamepad = gamepads[i]
        if (gamepad) {
          if (gamepad.id.includes('Microsoft') || gamepad.id.includes('Xbox')) {
            const xboxData = {
              'type': 'Xbox',
              'left_js_x': gamepad.axes[XBOX_CONFIG['left_js_x']], // shoulder rotate
              'left_js_y': gamepad.axes[XBOX_CONFIG['left_js_y']], // shoulder tilt
              'left_trigger': gamepad.buttons[XBOX_CONFIG['left_trigger']]['value'], // elbow forward
              'right_trigger': gamepad.buttons[XBOX_CONFIG['right_trigger']]['value'], // elbow back
              'right_js_x': gamepad.axes[XBOX_CONFIG['right_js_x']], // hand rotate
              'right_js_y': gamepad.axes[XBOX_CONFIG['right_js_y']], // hand tilt
              'right_bumper': gamepad.buttons[XBOX_CONFIG['right_bumper']]['pressed'], // grip close
              'left_bumper': gamepad.buttons[XBOX_CONFIG['left_bumper']]['pressed'], // grip open
              'd_pad_up': gamepad.buttons[XBOX_CONFIG['d_pad_up']]['pressed'],
              'd_pad_down': gamepad.buttons[XBOX_CONFIG['d_pad_down']]['pressed'],
              'd_pad_right': gamepad.buttons[XBOX_CONFIG['d_pad_right']]['pressed'],
              'd_pad_left': gamepad.buttons[XBOX_CONFIG['d_pad_left']]['pressed']
            }

            const arm_toggles = {
              'type': 'ArmToggles',
              'solenoid': gamepad.buttons[XBOX_CONFIG['a']]['pressed'],
              'electromagnet': gamepad.buttons[XBOX_CONFIG['b']]['pressed'],
              'laser': gamepad.buttons[XBOX_CONFIG['y']]['pressed'],
            }

            let send_arm_toggles = false
            if (this.controlMode === 'arm') {
              send_arm_toggles = true
              let motor_speeds = [-deadzone(quadratic(xboxData.left_js_x), 0.09)*0.5,
                              -deadzone(quadratic(xboxData.left_js_y), 0.09)*0.5,
                              quadratic(xboxData.left_trigger-xboxData.right_trigger)*0.6,
                              deadzone(quadratic(xboxData.right_js_y), 0.09)*0.75,
                              deadzone(quadratic(xboxData.right_js_x), 0.09)*0.75,
                              (xboxData.d_pad_right-xboxData.d_pad_left)*0.6,
                              xboxData.right_bumper - xboxData.left_bumper]
              for(let i=0; i<7; i++) {
                const openloop_msg = {
                  'type': 'OpenLoopRAMotor',
                  'joint_id': i,
                  'speed': motor_speeds[i]
                }
                this.$parent.publish('/arm_motors', openloop_msg)
              }
            } else if(this.controlMode === 'arm_ik') {
              send_arm_toggles = true

              let speed = 0.25;
              const deltaPos = {
                'type': 'IkArmControl',
                'deltaX': deadzone(quadratic(xboxData['left_js_x']), 0.08)*speed*updateRate,
                'deltaZ': deadzone(quadratic(xboxData['left_js_y']), 0.08)*speed*updateRate,
                'deltaJointE': -deadzone(quadratic(xboxData['right_js_x']), 0.08)*0.4*updateRate,
                'deltaTilt': deadzone(quadratic(xboxData['right_js_y']), 0.08)*0.4*updateRate
              }

              deltaPos.deltaY = (xboxData['d_pad_up'] ? 1 : (xboxData['d_pad_down'] ? -1 : 0)) * speed * updateRate

              this.$parent.publish('/ik_arm_control', deltaPos);

              let openloop = {
                'type': 'OpenLoopRAMotor',
                'joint_id': 5,
                'speed': (xboxData['d_pad_right'] - xboxData['d_pad_left'])*0.60,
              }
              this.$parent.publish('/arm_motors', openloop)

              openloop.joint_id = 6
              openloop.speed = (xboxData['right_bumper'] - xboxData['left_bumper'])
              this.$parent.publish('/arm_motors', openloop)
            }

            if(send_arm_toggles) {
              arm_toggles.solenoid = solenoid_toggle.new_reading(arm_toggles.solenoid)
              arm_toggles.electromagnet = electromagnet_toggle.new_reading(arm_toggles.electromagnet)
              arm_toggles.laser = laser_toggle.new_reading(arm_toggles.laser)
              this.$parent.publish('/arm_toggles_toggle_data', arm_toggles)
            }
          }
        }
      }
      const talonConfig = {
        'type': 'TalonConfig',
        'enable_arm': (this.controlMode === 'arm' || this.controlMode === 'arm_ik'),
        'enable_sa': false
      }

      this.$parent.publish('/talon_config', talonConfig)
    }, updateRate*1000)
  },

  methods: {
    updateControlMode: function (mode, checked) {
      if (checked) {
        if (this.controlMode !== ''){
          this.$refs[this.controlMode].toggle()
        }

        this.setControlMode(mode)
      } else {
        this.setControlMode('')
      }
    },

    ...mapMutations('controls', {
      setControlMode: 'setControlMode'
    })
  },

  components: {
    Checkbox
  }
}
</script>

<style scoped>
.wrap {
  display: grid;
  grid-template-areas: "control_buttons";
  grid-template-columns: 1fr;
  align-items: center;
  height: 100%;
  padding: 0px 0px 0px 20px;
}

.control_buttons {
  grid-area: control_buttons;
  grid-template-areas: none;
  display: flex;
  padding: 0px 0px 0px 0px;
  height: 100%;
}

.led {
  width: 16px;
  height: 16px;
  border-radius: 8px;
  border: 1px solid;
}

.lightgreen {
  background-color: lightgreen;
}

.red {
  background-color: red;
}

.orange {
  background-color: orange;
}

.name {
  margin-left: 5px;
}
</style>
