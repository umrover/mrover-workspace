<template>
  <div class="wrap">
      <Checkbox ref="arm" v-bind:name="'Arm Controls'" v-on:toggle="updateControlMode('arm', $event)"/>
      <Checkbox ref="arm_ik" v-bind:name="'Inverse Kinematics'" v-on:toggle="updateControlMode('arm_ik', $event)"/>
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
      'x': 2,
      'y': 3
    }

    const electromagnet_toggle = new Toggle(false)
    const laser_toggle = new Toggle(false)

    const updateRate = 0.1
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
              'd_pad_left': gamepad.buttons[XBOX_CONFIG['d_pad_left']]['pressed'],
              'a': gamepad.buttons[XBOX_CONFIG['a']]['pressed'],
              'b': gamepad.buttons[XBOX_CONFIG['b']]['pressed'],
              'x': gamepad.buttons[XBOX_CONFIG['x']]['pressed'],
              'y': gamepad.buttons[XBOX_CONFIG['y']]['pressed']
            }

            const arm_toggles = {
              'type': 'ArmToggles',
              'solenoid': xboxData['b'],
              'electromagnet': electromagnet_toggle.new_reading(xboxData['a']),
              'laser': laser_toggle.new_reading(xboxData['x'])
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

              let speed = 0.05;
              const deltaPos = {
                'type': 'IkArmControl',
                'deltaX': -deadzone(quadratic(xboxData.left_js_y), 0.08)*speed*updateRate,
                'deltaY': -deadzone(quadratic(xboxData.left_js_x), 0.08)*speed*updateRate,
                'deltaZ': -deadzone(quadratic(xboxData.right_js_y), 0.08)*speed*updateRate
              }

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
              this.$parent.publish('/arm_toggles', arm_toggles)
            }
          }
        }
      }
      const talonConfig = {
        'type': 'TalonConfig',
        'enable_arm': true,
        'enable_sa': false
      }

      this.$parent.publish('/talon_config', talonConfig)
    }, updateRate*1000)
  },

  methods: {
    updateControlMode: function (mode, checked) {
      let ikEnabled = false
      if (checked) {
        if (this.controlMode !== ''){
          this.$refs[this.controlMode].toggle()
        }

        ikEnabled = (mode === 'arm_ik')
        this.setControlMode(mode)
      } else {
        this.setControlMode('')
      }

      const ikEnabledMsg = {
        'type': 'IkEnabled',
        'enabled': ikEnabled
      }

      this.$parent.publish('/ik_enabled', ikEnabledMsg)
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
  display: flex;
  align-items: center;
}

</style>
