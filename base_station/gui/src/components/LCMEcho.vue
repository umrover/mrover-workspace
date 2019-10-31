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
          '/arm_control': false,
          '/arm_motors': false,
          '/arm_toggles_button_data': false,
          '/arm_toggles_toggle_data': false,
          '/auton': false,
          '/autonomous': false,
          '/camera_servos': false,
          '/config_pid': false,
          '/course': false,
          '/debugMessage': false,
          '/drive_control': false,
          '/encoder': false,
          '/gps': false,
          '/ik_arm_control': false,
          '/ik_ra_control': false,
          '/imu': false,
          '/kill_switch': false,
          '/microscope': false,
          '/motor': false,
          '/nav_status': false,
          '/obstacle': false,
          '/odometry': false,
          '/pi_camera': false,
          '/pi_settings': false,
          '/sa_controls': false,
          '/sa_motors': false,
          '/sensor_switch': false,
          '/sensors': false,
          '/set_demand': false,
          '/temperature': false,
          '/tennis_ball': false

        },
        subscriptions: [
          {'topic': '/arm_toggles_button_data', 'type': 'ArmToggles'},
          {'topic': '/arm_toggles_toggle_data', 'type': 'ArmToggles'},
          {'topic': '/ik_ra_control', 'type': 'ArmPosition'},
          {'topic': '/auton', 'type': 'AutonState'},
          {'topic': '/camera_servos', 'type': 'CameraServos'},
          {'topic': '/course', 'type': 'Course'},
          // {'topic': '', 'type': 'CurrentDraw'},
          {'topic': '/debugMessage', 'type': 'DebugMessage'},
          {'topic': '/motor', 'type': 'DriveMotors'},
          {'topic': '/encoder', 'type': 'Encoder'},
          // {'topic': '', 'type': 'Heartbeat'},
          {'topic': '/ik_arm_control', 'type': 'IkArmControl'},
          {'topic': '/drive_control', 'type': 'Joystick'},
          {'topic': '/autonomous', 'type': 'Joystick'},
          {'topic': '/kill_switch', 'type': 'KillSwitch'},
          {'topic': '/nav_status', 'type': 'NavStatus'},
          {'topic': '/obstacle', 'type': 'Obstacle'},
          {'topic': '/odometry', 'type': 'Odometry'},
          {'topic': '/arm_motors', 'type': 'OpenLoopRAMotor'},
          {'topic': '/pi_camera', 'type': 'PiCamera'},
          {'topic': '/pi_settings', 'type': 'PiSettings'},
          {'topic': '/sa_motors', 'type': 'SAMotors'},
          {'topic': '/sensors', 'type': 'Sensors'},
          {'topic': '/sensor_switch', 'type': 'SensorSwitch'},
          {'topic': '/set_demand', 'type': 'SetDemand'},
          {'topic': '/config_pid', 'type': 'PIDConstants'},
          {'topic': '/temperature', 'type': 'Temperature'},
          {'topic': '/tennis_ball', 'type': 'TennisBall'},
          {'topic': '/sa_controls', 'type': 'Xbox'},
          {'topic': '/arm_control', 'type': 'Xbox'},
          {'topic': '/imu', 'type': 'IMU'},
          {'topic': '/gps', 'type': 'GPS'},
          {'topic': '/microscope', 'type': 'Microscope'}
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
    white-space: nowrap;
  }

  ul#vitals li {
    display: inline;
    float: left;
    padding: 0px 10px 0px 0px;
  }
</style>
