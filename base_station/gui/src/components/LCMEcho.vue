<template>
  <div class="wrapper">
    <div class="box header">
      <img src="/static/mrover.png" alt="MRover" title="MRover" width="48" height="48" />
      <h1>LCM Echo</h1>
      <div class="spacer"></div>
      <div class="comms">
        <ul id="vitals">
          <li><CommIndicator v-bind:connected="connections.websocket" name="Web Socket" /></li>
          <li><CommIndicator v-bind:connected="connections.lcm" name="Rover Connection Status" /></li>
        </ul>
      </div>
      <div class="spacer"></div>
    </div>
    <div class="box checklist">
      <ul id="channels">
        <li v-for="(checked, channel) in viewing" :key="channel">
          <input type="checkbox" :id="channel" v-model="viewing[channel]">
          <label :for="channel">{{ channel }}</label>
        </li>
      </ul>
    </div>
    <div class="box messagefeed" ref="messagefeed">
      <ul id="feed">
        <li v-for="msg, i in messages" :key="i">
          {{ msg }}
        </li>
      </ul>
    </div>



  </div>
</template>

<script>
  import LCMBridge from 'lcm_bridge_client/dist/bridge.js';
  import CommIndicator from './CommIndicator.vue'

  export default {
    name: 'LCMEcho',
    data () {
      return {
        lcm_: null,
        connections: {
          websocket: false,
          lcm: false
        },
        messages: [],
        viewing: {
          '/ik_ra_control': false,
          '/auton': false,
          '/bearing': false,
          '/camera_servos': false,
          '/course': false,
          '/debugMessage': false,
          '/motor': false,
          '/encoder': false,
          '/ik_arm_control': false,
          '/drive_control': false,
          '/autonomous': false,
          '/kill_switch': false,
          '/nav_status': false,
          '/obstacle': false,
          '/odometry': false,
          '/arm_motors': false,
          '/pi_camera': false,
          '/pi_settings': false,
          '/sa_motors': false,
          '/sensors': false,
          '/sensor_switch': false,
          '/set_demand': false,
          '/set_param': false,
          '/temperature': false,
          '/tennis_ball': false,
          '/sa_control': false,
          '/arm_control': false
        },
        subscriptions: [
          {'topic': '/ik_ra_control', 'type': 'ArmPosition', 'checked': false},
          {'topic': '/auton', 'type': 'AutonState', 'checked': false},
          {'topic': '/bearing', 'type': 'Bearing', 'checked': false},
          {'topic': '/camera_servos', 'type': 'CameraServos', 'checked': false},
          {'topic': '/course', 'type': 'Course', 'checked': false},
        //   {'topic': '', 'type': 'CurrentDraw', 'checked': false},
          {'topic': '/debugMessage', 'type': 'DebugMessage', 'checked': false},
          {'topic': '/motor', 'type': 'DriveMotors', 'checked': false},
          {'topic': '/encoder', 'type': 'Encoder', 'checked': false},
        //   {'topic': '', 'type': 'Heartbeat', 'checked': false},
          {'topic': '/ik_arm_control', 'type': 'IkArmControl', 'checked': false},
          {'topic': '/drive_control', 'type': 'Joystick', 'checked': false},
          {'topic': '/autonomous', 'type': 'Joystick', 'checked': false},
          {'topic': '/kill_switch', 'type': 'KillSwitch', 'checked': false},
          {'topic': '/nav_status', 'type': 'NavStatus', 'checked': false},
          {'topic': '/obstacle', 'type': 'Obstacle', 'checked': false},
          {'topic': '/odometry', 'type': 'Odometry', 'checked': false},
          {'topic': '/arm_motors', 'type': 'OpenLoopRAMotors', 'checked': false},
          {'topic': '/pi_camera', 'type': 'PiCamera', 'checked': false},
          {'topic': '/pi_settings', 'type': 'PiSettings', 'checked': false},
          {'topic': '/sa_motors', 'type': 'SAMotors', 'checked': false},
          {'topic': '/sensors', 'type': 'Sensors', 'checked': false},
          {'topic': '/sensor_switch', 'type': 'SensorSwitch', 'checked': false},
          {'topic': '/set_demand', 'type': 'SetDemand', 'checked': false},
          {'topic': '/set_param', 'type': 'SetParam', 'checked': false},
          {'topic': '/temperature', 'type': 'Temperature', 'checked': false},
          {'topic': '/tennis_ball', 'type': 'TennisBall', 'checked': false},
          {'topic': '/sa_control', 'type': 'Xbox', 'checked': false},
          {'topic': '/arm_control', 'type': 'Xbox', 'checked': false}
        ]
      }
    },

    mounted () {;
    },

    methods: {
      updateScroll: function () {
        this.$refs.messagefeed.scrollTop = this.$refs.messagefeed.scrollHeight
      }
    },

    created: function () {
      this.lcm_ = new LCMBridge(
        'ws://localhost:8001',
        // Update WebSocket connection state
        (online) => {
          this.lcm_.setHomePage()
          this.connections.websocket = online
        },
        // Update connection states
        (online) => {
          this.connections.lcm = online[0]
        },
        // Subscribed LCM message received
        (msg) => {
          if (this.viewing[msg.topic]){
            this.updateScroll()
            this.messages.push(msg.message)
            this.messages = this.messages.slice(-100)
          }
        },
        // Subscriptions
        this.subscriptions
      )
    },
    components: {
      CommIndicator
    }
  }
</script>

<style scoped>
  .wrapper {
    display: grid;
    grid-gap: 10px;
    grid-template-columns: 1fr 3fr;
    grid-template-rows: 60px 1fr;
    grid-template-areas: "header header" "checklist feed";

    font-family: sans-serif;
    height: 98vh;
  }

  .header {
    grid-area: header;
    display: flex;
    align-items: center;
  }

  .header h1 {
    margin-left: 5px;
  }

  .box {
    border-radius: 5px;
    padding: 10px;
    border: 1px solid black;
  }

  .spacer {
    flex-grow: 0.8;
  }

  .comms {
    display: flex;
    flex-direction: column;
    align-items: flex-start;
  }

  .comms * {
    margin-top: 2px;
    margin-bottom: 2px;
  }

  .messagefeed {
    overflow: auto;
  }

  p {
    margin-bottom: 1em;
  }

  ul#channels li {
    display: block;
    padding: 0px 10px 0px 0px;
  }

  ul#feed li {
    display: block;
    padding: 0px 10px 0px 0px;
  }

  ul#vitals li {
    display: inline;
    float: left;
    padding: 0px 10px 0px 0px;
  }
</style>
