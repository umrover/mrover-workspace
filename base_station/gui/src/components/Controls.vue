<template>
  <div class="wrap">
    <div class="control_buttons">
        <Checkbox ref="arm" v-bind:name="'Arm Controls'" v-on:toggle="updateControlMode('arm', $event)"/>
        <Checkbox ref="arm_ik" v-bind:name="'Inverse Kinematics'" v-on:toggle="updateControlMode('arm_ik', $event)"/>
    </div>
    <div class="speed_limiter">
      <p>
        Speed Limiter:
        <span>{{ dampenDisplay }}%</span>
      </p>
    </div>
    <div class="mode_info">
      <div v-if="controlMode === 'arm'">
        <p>arm</p>
      </div>
    </div>
  </div>
</template>

<script>
import Checkbox from './Checkbox.vue'
import { mapGetters, mapMutations } from 'vuex'

export default {
  data () {
    return {
      dampen: 0
    }
  },

  computed: {

    dampenDisplay: function () {
      return (this.dampen * -50 + 50).toFixed(2)
    },

    ...mapGetters('controls', {
      controlMode: 'controlMode'
    }),

    ...mapGetters('autonomy', {
      autonEnabled: 'autonEnabled'
    }),
  },

  created: function () {

    const JOYSTICK_CONFIG = {
      'forward_back': 1,
      'left_right': 2,
      'dampen': 3,
      'kill': 4,
      'restart': 5,
      'pan': 4,
      'tilt': 5
    }

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

    const updateRate = 0.05;
    window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      for (let i = 0; i < 4; i++) {
        const gamepad = gamepads[i]
        if (gamepad) {
          if (gamepad.id.includes('Logitech')) {
            const joystickData = {
              'type': 'Joystick',
              'forward_back': gamepad.axes[JOYSTICK_CONFIG['forward_back']],
              'left_right': gamepad.axes[JOYSTICK_CONFIG['left_right']],
              'dampen': gamepad.axes[JOYSTICK_CONFIG['dampen']],
              'kill': gamepad.buttons[JOYSTICK_CONFIG['kill']]['pressed'],
              'restart': gamepad.buttons[JOYSTICK_CONFIG['restart']]['pressed']
            }
            this.dampen = gamepad.axes[JOYSTICK_CONFIG['dampen']]

            if (!this.autonEnabled) {
              this.$parent.publish('/drive_control', joystickData)
            }
          } else if (gamepad.id.includes('Microsoft') || gamepad.id.includes('Xbox')) {
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

            if (this.controlMode === 'arm') {
              this.$parent.publish('/arm_control', xboxData)
              this.$parent.publish('/arm_toggles_button_data', arm_toggles)
            } else if(this.controlMode === 'arm_ik') {
              this.$parent.publish('/arm_toggles_button_data', arm_toggles)
              let speed = 0.25;
              const deltaPos = {
                'type': 'IkArmControl',
                'deltaX': (xboxData['left_js_x']**2)*speed*updateRate*(xboxData['left_js_x']<0 ? -1 : 1),
                'deltaZ': (xboxData['left_js_y']**2)*speed*updateRate*(xboxData['left_js_y']>0 ? 1 : -1),
                'deltaJointE': (xboxData['right_js_x']**2)*0.4*updateRate*(xboxData['right_js_x']>0 ? -1 : 1),
                'deltaTilt': (xboxData['right_js_y']**2)*0.4*updateRate*(xboxData['right_js_y']<0 ? -1 : 1)
              }

              deltaPos.deltaY = (xboxData['d_pad_up'] ? 1 : (xboxData['d_pad_down'] ? -1 : 0)) * speed * updateRate

              if(Math.abs(deltaPos.deltaX) < 0.001){
                deltaPos.deltaX=0;
              }
              if(Math.abs(deltaPos.deltaZ) < 0.001){
                deltaPos.deltaZ=0;
              }
              if(Math.abs(deltaPos.deltaJointE) < 0.001){
                deltaPos.deltaJointE=0;
              }
              if(Math.abs(deltaPos.deltaTilt) < 0.001){
                deltaPos.deltaTilt=0;
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
  grid-template-areas: "control_buttons speed_limiter mode_info";
  grid-template-columns: 1fr 1fr 1fr;
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

.speed_limiter {
  display: flex;
  grid-area: speed_limiter;
  align-items: center;
  justify-content: center;
}

.mode_info {
  grid-area: mode_info;
  display: flex;
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
