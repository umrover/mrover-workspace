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
          '/arm_adjustments': false,
          '/arm_control_state': false,
          '/arm_preset': false,
          '/arm_preset_path': false,
          '/auton': false,
          '/autonomous': false,
          '/auton_drive_control': false,
          '/auton_reverse': false,
          '/cameras_cmd': false,
          '/carousel_calib_data': false,
          '/carousel_closed_loop_cmd': false,
          '/carousel_open_loop_cmd': false,
          '/carousel_pos_data': false,
          '/carousel_zero_cmd': false,
          '/course': false,
          '/custom_preset': false,
          '/drive_control': false,
          '/drive_state_data': false,
          '/drive_vel_cmd': false,
          '/encoder': false,
          '/estimated_gate_location': false,
          '/fk_transform': false,
          '/foot_open_loop_cmd': false,
          '/gimbal_control': false,
          '/gps': false,
          '/hand_openloop_cmd': false,
          '/imu_data': false,
          '/heater_auto_shutdown_cmd': false,
          '/heater_auto_shutdown_data': false,
          '/heater_cmd': false,
          '/heater_state_data': false,
          '/locked_joints': false,
          '/mast_gimbal_cmd': false,
          '/microscope': false,
          '/mosfet_cmd': false,
          '/motion_execute': false,
          '/nav_status': false,
          '/obstacle': false,
          '/odometry': false,
          '/pdb_data': false,
          '/projected_points': false,
          '/radio_setup': false,
          '/radio_update': false,
          '/ra_b_calib_data': false,
          '/ra_control': false,
          '/ra_ik_cmd': false,
          '/ra_open_loop_cmd': false,
          '/ra_pidconfig_cmd': false,
          '/sa_b_calib_data': false,
          '/sa_control': false,
          '/sa_ik_cmd': false,
          '/sa_open_loop_cmd': false,
          '/sa_pidconfig_cmd': false,
          '/sa_position': false,
          '/sa_motors': false,
          '/sa_zero_trigger': false,
          '/target_bearing': false,
          '/scoop_limit_switch_enable_cmd': false,
          '/set_demand': false,
          '/simulation_mode': false,
          '/spectral_data': false,
          '/target_list': false,
          '/target_orientation': false,
          '/teleop_reverse_drive': false,
          '/thermistor_data': false,
          '/use_orientation': false,
          '/zero_position': false
        },
        subscriptions: [
          {'topic': '/arm_adjusments', 'type': 'ArmAdjustments'},
          {'topic': '/arm_control_state', 'type': 'ArmControlState'},
          {'topic': '/arm_preset', 'type': 'ArmPreset'},
          {'topic': '/auton_reverse', 'type': 'ReverseDrive'},
          {'topic': '/auton_drive_control', 'type': 'AutonDriveControl'},
          {'topic': '/arm_preset_path', 'type': 'ArmPresetPath'},
          {'topic': '/auton', 'type': 'AutonState'},
          {'topic': '/autonomous', 'type': 'Joystick'},
          {'topic': '/cameras_cmd', 'type': 'Cameras'},
          {'topic': '/carousel_calib_data', 'type': 'Calibrate'},
          {'topic': '/carousel_closed_loop_cmd', 'type': 'CarouselPosition'},
          {'topic': '/carousel_open_loop_cmd', 'type': 'CarouselOpenLoopCmd'},
          {'topic': '/carousel_pos_data', 'type': 'CarouselPosition'},
          {'topic': '/carousel_zero_cmd', 'type': 'Signal'},
          {'topic': '/course', 'type': 'Course'},
          {'topic': '/custom_preset', 'type': 'CustomPreset'},
          {'topic': '/drive_control', 'type': 'Joystick'},
          {'topic': '/drive_state_data', 'type': 'DriveStateData'},
          {'topic': '/drive_vel_cmd', 'type': 'DriveVelCmd'},
          {'topic': '/encoder', 'type': 'Encoder'},
          {'topic': '/estimated_gate_location', 'type': 'EstimatedGateLocation'},
          {'topic': '/fk_transform', 'type': 'FKTransform'},
          {'topic': '/foot_open_loop_cmd', 'type': 'FootCmd'},
          {'topic': '/gimbal_control', 'type': 'Keyboard'},
          {'topic': '/gps', 'type': 'GPS'},
          {'topic': '/hand_open_loop_cmd', 'type': 'HandCmd'},
          {'topic': '/heater_auto_shutdown_cmd', 'type': 'HeaterAutoShutdown'},
          {'topic': '/heater_auto_shutdown_data', 'type': 'HeaterAutoShutdown'},
          {'topic': '/heater_cmd', 'type': 'Heater'},
          {'topic': '/heater_state_data', 'type': 'Heater'},
          {'topic': '/imu_data', 'type': 'IMUData'},
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
          {'topic': '/ra_b_calib_data', 'type': 'Calibrate'},
          {'topic': '/ra_position', 'type': 'RAPosition'},
          {'topic': '/ra_control', 'type': 'Xbox'},
          {'topic': '/ra_ik_cmd', 'type': 'RAPosition'},
          {'topic': '/ra_open_loop_cmd', 'type': 'RAOpenLoopCmd'},
          {'topic': '/ra_pidconfig_cmd', 'type': 'PIDConstants'},
          {'topic': '/sa_b_calib_data', 'type': 'Calibrate'},
          {'topic': '/sa_control', 'type': 'Xbox'},
          {'topic': '/sa_ik_cmd', 'type': 'SAPosition'},
          {'topic': '/sa_open_loop_cmd', 'type': 'SAOpenLoopCmd'},
          {'topic': '/sa_position', 'type': 'SAPosition'},
          {'topic': '/sa_zero_trigger', 'type': 'Signal'},
          {'topic': '/scoop_limit_switch_enable_cmd', 'type': 'ScoopLimitSwitchEnable'},
          {'topic': '/set_demand', 'type': 'SetDemand'},
          {'topic': '/simulation_mode', 'type': 'SimulationMode'},
          {'topic': '/spectral_data', 'type': 'SpectralData'},
          {'topic': '/target_bearing', 'type': 'TargetBearing'},
          {'topic': '/target_list', 'type': 'TargetList'},
          {'topic': '/target_orientation', 'type': 'TargetOrientation'},
          {'topic': '/teleop_reverse_drive', 'type': 'ReverseDrive'},
          {'topic': '/thermistor_data', 'type': 'ThermistorData'},
          {'topic': '/use_orientation', 'type': 'UseOrientation'},
          {'topic': '/zero_position', 'type': 'Signal'}
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
