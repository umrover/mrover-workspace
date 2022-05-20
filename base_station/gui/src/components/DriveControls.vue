<template>
  <div class="wrap">
    <h3> Drive </h3>
    <div class="controls">
      <span>Speed Limiter: {{ dampenDisplay }}%</span>
      <Checkbox class="reverse" ref="reverse" v-bind:name="'Reverse'" v-on:toggle="updateReverse($event)"/>
    </div>
  </div>
</template>

<script>
import Checkbox from './Checkbox.vue'
import { mapGetters, mapMutations } from 'vuex'

import '../utils.js'

let interval;

export default {
  data () {
    return {
      dampen: 0,
      reverse: false
    }
  },

  methods: {
    updateReverse: function(checked) {
      const reverseMsg = {
        'type': 'ReverseDrive',
        'reverse': checked
      }
      this.$parent.publish('/teleop_reverse_drive', reverseMsg);
    }
  },

  methods: {
    updateReverse: function(checked) {
      const reverseMsg = {
        'type': 'ReverseDrive',
        'reverse': checked
      }
      this.$parent.publish('/teleop_reverse_drive', reverseMsg);
    }
  },

  components: {
    Checkbox
  },

  computed: {

    dampenDisplay: function() {
      return (this.dampen * 100).toFixed(2);
    },

    ...mapGetters('autonomy', {
      autonEnabled: 'autonEnabled'
    }),
  },

  beforeDestroy: function () {
    window.clearInterval(interval);
  },

  created: function () {

    const JOYSTICK_CONFIG = {
      'left_right': 0,
      'forward_back': 1,
      'twist': 2,
      'dampen': 3,
      'pan': 4,
      'tilt': 5
    }

    const updateRate = 0.05;
    interval = window.setInterval(() => {
      if (!this.autonEnabled) {
        const gamepads = navigator.getGamepads()
        for (let i = 0; i < 4; i++) {
          const gamepad = gamepads[i]
          if (gamepad) {
            if (gamepad.id.includes('Logitech')) {
              // Make dampen 1 when slider is pushed forward, 0 when pulled back
              // Raw value is -1 to 1
              this.dampen = gamepad.axes[JOYSTICK_CONFIG['dampen']] * -0.5 + 0.5
              // -1 multiplier to make turning left a positive value
              let rotation = -1 * (gamepad.axes[JOYSTICK_CONFIG['left_right']] + gamepad.axes[JOYSTICK_CONFIG['twist']])
              if (rotation > 1) {
                rotation = 1
              }
              else if (rotation < -1) {
                rotation = -1
              }
              const joystickData = {
                'type': 'Joystick',
                // forward on joystick is -1, so invert
                'forward_back': -1 * gamepad.axes[JOYSTICK_CONFIG['forward_back']],
                'left_right': rotation,
                'dampen': this.dampen
              }
              this.$parent.publish('/drive_control', joystickData)
            }
          }
        }
      }
    }, updateRate*1000)
  },

  components: {
    Checkbox
  }
}
</script>

<style scoped>

.wrap {
  display: inline-block;
  align-items: center;
}

.controls {
  display: flex;
  align-items: center;
}

.reverse {
  margin-left: 20px;
}

</style>
