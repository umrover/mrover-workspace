<template>
  <div class="wrap">
    <span>Speed Limiter: {{ dampenDisplay }}%</span>
    <Checkbox ref="reverse" v-bind:name="'Reverse'" v-on:toggle="updateReverse($event)"/>
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
      dampen: 1.0,
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
      'forward_back': 1,
      'left_right': 2,
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
              this.dampen = gamepad.axes[JOYSTICK_CONFIG['dampen']] * 0.5 + 0.5

              const joystickData = {
                'type': 'Joystick',
                'forward_back': gamepad.axes[JOYSTICK_CONFIG['forward_back']],
                'left_right': gamepad.axes[JOYSTICK_CONFIG['left_right']],
                'dampen': this.dampen
              }

              this.$parent.publish('/drive_control', joystickData)
            }
          }
        }
      }
    }, updateRate*1000)
  }
}
</script>

<style scoped>

.wrap {
  display: flex;
  align-items: center;
}

</style>
