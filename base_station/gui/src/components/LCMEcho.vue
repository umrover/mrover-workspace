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
      <button id="clear-button" @click="clearFeed()">Clear Feed</button>
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
          '/arm_control_state': false,
          '/arm_control_state_to_gui': false,
          '/arm_adjustments': false,
          '/arm_motors': false,
          '/ra_ik_cmd': false,
          '/arm_preset': false,
          '/arm_toggles_button_data': false,
          '/arm_toggles_toggle_data': false,
          '/auton': false,
          '/autonomous': false,
          '/cameras_cmd': false,
          '/camera_servos': false,
          '/carousel_closedloop_cmd': false,
          '/carousel_data': false,
          '/carousel_openloop_cmd': false,
          '/config_pid': false,
          '/course': false,
          '/debugMessage': false,
          '/drive_control': false,
          '/encoder': false,
          '/fk_transform': false,
          '/gimbal_control': false,
          '/mast_gimbal_cmd': false,
          '/gps': false,
          '/hand_openloop_cmd': false,
          '/heater_cmd': false,
          '/heater_state_data': false,
          '/heater_auto_shutdown_cmd': false,
          '/heater_auto_shutdown_data': false,
          '/ik_arm_control': false,
          '/ra_ik_cmd': false,
          '/imu': false,
          '/kill_switch': false,
          '/locked_joints': false,
          '/microscope': false,
          '/mosfet_cmd': false,
          '/drive_vel_cmd': false,
          '/nav_status': false,
          '/obstacle': false,
          '/odometry': false,
          '/pdb_data': false,
          '/radio_update': false,
          '/radio_setup': false,
          '/ra_control': false,
          '/ra_openloop_cmd': false,
          '/ra_pidconfig_cmd': false,
          '/rr_drop_complete': false,
          '/rr_drop_init': false,
          '/sa_control': false,
          '/sa_endeffector_cmd': false,
          '/sa_openloop_cmd': false,
          '/sa_pidconfig_cmd': false,
          '/sa_position': false,
          '/sa_motors': false,
          '/sa_zero_trigger': false,
          '/sensor_switch': false,
          '/sensors': false,
          '/servo_cmd': false,
          '/set_demand': false,
          '/spectral_data': false,
          '/spectral_triad_data': false,
          '/target_list': false,
          '/target_orientation': false,
          '/temperature': false,
          '/tennis_ball': false,
          '/thermistor_data': false,
          '/zed_gimbal_cmd': false,
          '/zed_gimbal_data': false,
          '/zero_position': false
        },
        subscriptions: [
          {'topic': '/arm_adjusments', 'type': 'ArmAdjustments'},
          {'topic': '/arm_control_state', 'type': 'ArmControlState'},
          {'topic': '/arm_control_state_to_gui', 'type': 'ArmControlState'},
          {'topic': '/arm_motors', 'type': 'OpenLoopRAMotor'},
          {'topic': '/arm_toggles_button_data', 'type': 'ArmToggles'},
          {'topic': '/arm_toggles_toggle_data', 'type': 'ArmToggles'},
          {'topic': '/arm_preset', 'type': 'ArmPreset'},
          {'topic': '/auton', 'type': 'AutonState'},
          {'topic': '/autonomous', 'type': 'Joystick'},
          {'topic': '/cameras_cmd', 'type': 'Cameras'},
          {'topic': '/camera_servos', 'type': 'CameraServos'},
          {'topic': '/carousel_closedloop_cmd', 'type': 'CarouselClosedLoopCmd'},
          {'topic': '/carousel_data', 'type': 'CarouselData'},
          {'topic': '/carousel_openloop_cmd', 'type': 'CarouselOpenLoopCmd'},
          {'topic': '/course', 'type': 'Course'},
          {'topic': '/debugMessage', 'type': 'DebugMessage'},
          {'topic': '/drive_control', 'type': 'Joystick'},
          {'topic': '/drive_vel_cmd', 'type': 'DriveVelCmd'},
          {'topic': '/encoder', 'type': 'Encoder'},
          {'topic': '/fk_transform', 'type': 'FKTransform'},
          {'topic': '/gimbal_control', 'type': 'Keyboard'},
          {'topic': '/gps', 'type': 'GPS'},
          {'topic': '/hand_openloop_cmd', 'type': 'HandCmd'},
          {'topic': '/heater_cmd', 'type': 'Heater'},
          {'topic': '/heater_state_data', 'type': 'Heater'},
          {'topic': '/heater_auto_shutdown_cmd', 'type': 'HeaterAutoShutdown'},
          {'topic': '/heater_auto_shutdown_data', 'type': 'HeaterAutoShutdown'},
          {'topic': '/ik_arm_control', 'type': 'IkArmControl'},
          {'topic': '/imu', 'type': 'IMU'},
          {'topic': '/kill_switch', 'type': 'KillSwitch'},
          {'topic': '/locked_joints', 'type': 'LockJoints'},
          {'topic': '/mast_gimbal_cmd', 'type': 'MastGimbalCmd'},
          {'topic': '/microscope', 'type': 'Microscope'},
          {'topic': '/mosfet_cmd', 'type': 'MosfetCmd'},
          {'topic': '/motion_execute', 'type': 'MotionExecute'},
          {'topic': '/nav_status', 'type': 'NavStatus'},
          {'topic': '/obstacle', 'type': 'Obstacle'},
          {'topic': '/odometry', 'type': 'Odometry'},
          {'topic': '/pdb_data', 'type': 'PDBData'},
          {'topic': '/radio_setup', 'type': 'Signal'},
          {'topic': '/ra_position', 'type': 'RAPosition'},
          {'topic': '/radio_update', 'type': 'RadioSignalStrength'},
          {'topic': '/ra_control', 'type': 'Xbox'},
          {'topic': '/ra_ik_cmd', 'type': 'RAPosition'},
          {'topic': '/ra_openloop_cmd', 'type': 'RAOpenLoopCmd'},
          {'topic': '/ra_pidconfig_cmd', 'type': 'PIDConstants'},
          {'topic': '/rr_drop_complete', 'type': 'RepeaterDrop'},
          {'topic': '/rr_drop_init', 'type': 'RepeaterDrop'},
          {'topic': '/sensors', 'type': 'Sensors'},
          {'topic': '/sensor_switch', 'type': 'SensorSwitch'},
          {'topic': '/servo_cmd', 'type': 'ServoCmd'},
          {'topic': '/set_demand', 'type': 'SetDemand'},
          {'topic': '/spectral_data', 'type': 'SpectralData'},
          {'topic': '/spectral_triad_data', 'type': 'SpectralData'},
          {'topic': '/sa_control', 'type': 'Xbox'},
          {'topic': '/sa_endeffector_cmd', 'type': 'SAEndEffectorCmd'},
          {'topic': '/sa_motors', 'type': 'SAMotors'},
          {'topic': '/sa_position', 'type': 'SAPosition'},
          {'topic': '/sa_openloop_cmd', 'type': 'SAOpenLoopCmd'},
          {'topic': '/sa_pidconfig_cmd', 'type': 'PIDConstants'},
          {'topic': '/sa_zero_trigger', 'type': 'Signal'},
          {'topic': '/target_list', 'type': 'TargetList'},
          {'topic': '/target_orientation', 'type': 'TargetOrientation'},
          {'topic': '/temperature', 'type': 'Temperature'},
          {'topic': '/tennis_ball', 'type': 'TennisBall'},
          {'topic': '/thermistor_data', 'type': 'ThermistorData'},
          {'topic': '/zed_gimbal_cmd', 'type': 'ZedGimbalPosition'},
          {'topic': '/zed_gimbal_data', 'type': 'ZedGimbalPosition'},
          {'topic': '/zero_position', 'type': 'ZeroPosition'}
        ]
      }
    },

    mounted () {;
    },

    methods: {
      updateScroll: function () {
        this.$refs.messagefeed.scrollTop = this.$refs.messagefeed.scrollHeight
      },

      clearFeed: function() {
        this.messages = []
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

  #clear-button {
    font-size: 16px;
    text-decoration: none;
    padding: 10px;
    background: #ffcb05;
    color: #00274c;
    border-radius: 8px;
    border: none;
  }

  #clear-button:active {
    background: #f0c000;
  }
</style>
