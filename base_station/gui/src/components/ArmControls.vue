<template>
  <div class="wrap">
      <Checkbox ref="open-loop" v-bind:name="'Open Loop'" v-on:toggle="updateControlMode('open-loop', $event)"/>
      <Checkbox ref="closed-loop" v-bind:name="'Closed Loop'" v-on:toggle="updateControlMode('closed-loop', $event)"/>
      <div class="keyboard">
        <GimbalControls/>
      </div>
  </div>
</template>

<script>
import Checkbox from './Checkbox.vue'
import { mapGetters, mapMutations } from 'vuex'

import {Toggle, quadratic, deadzone, joystick_math} from '../utils.js'
import GimbalControls from './GimbalControls.vue'

let interval;

export default {
  data() {
    return {
      xboxControlEpsilon: 0.15
    }
  },

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
            if (this.controlMode !== 'open-loop' && checkXboxInput(xboxData)) {
              updateControlMode('open-loop', true)
            }
            if (this.controlMode === 'open-loop') {
              this.$parent.publish('/ra_control', xboxData)
            }
          }
        }
      }
    }, updateRate*1000)
  },

  methods: {
    updateControlMode: function (mode, checked) {

      if (checked) {
        // control mode can either be open loop or control mode, not both
        if (this.controlMode !== '' && this.controlMode !== 'off'){
          this.$refs[this.controlMode].toggle()
        }

        this.setControlMode(mode);
      }
      else {
        this.setControlMode('off');
      }

      const armStateMsg = {
        'type': 'ArmControlState',
        'state': this.controlMode
      }

      this.$parent.publish('/arm_control_state', armStateMsg);
    },

    checkXboxInput: function (xboxData) {
      if (abs(xboxData[left_js_x]) > this.xboxControlEpsilon) {
        return true
      }
      if (abs(xboxData[left_js_y]) > this.xboxControlEpsilon) {
        return true
      }
      if (abs(xboxData[left_trigger]) > this.xboxControlEpsilon) {
        return true
      }
      if (abs(xboxData[right_trigger]) > this.xboxControlEpsilon) {
        return true
      }
      if (abs(xboxData[right_js_x]) > this.xboxControlEpsilon) {
        return true
      }
      if (abs(xboxData[right_js_y] - 0) > this.xboxControlEpsilon) {
        return true
      }
      if (xboxData[left_bumper] !== 0) {
        return true
      }
      if (xboxData[right_bumper] !== 0) {
        return true
      }
      if (xboxData[a] !== 0) {
        return true
      }
      if (xboxData[b] !== 0) {
        return true
      }
      if (xboxData[x] !== 0) {
        return true
      }
      if (xboxData[y] !== 0) {
        return true
      }
      if (xboxData[d_pad_up] !== 0) {
        return true
      }
      if (xboxData[d_pad_down] !== 0) {
        return true
      }
      if (xboxData[d_pad_right] !== 0) {
        return true
      }
      if (xboxData[d_pad_left] !== 0) {
        return true
      }
      return false
    },

    ...mapMutations('controls', {
      setControlMode: 'setControlMode'
    })
  },

  components: {
    Checkbox,
    GimbalControls
  }
}
</script>

<style scoped>

.wrap {
  display: flex;
  align-items: center;
}

</style>
