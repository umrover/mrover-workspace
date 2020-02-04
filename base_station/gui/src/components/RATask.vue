<template>
  <div class="wrapper">
    <div class="box header">
      <img src="/static/mrover.png" alt="MRover" title="MRover" width="48" height="48" />
      <h1>Dashboard</h1>
      <div class="spacer"></div>
      <div class="comms">
        <ul id="vitals">
          <li><CommIndicator v-bind:connected="connections.websocket" name="Web Socket" /></li>
          <li><CommIndicator v-bind:connected="connections.lcm" name="Rover Connection Status" /></li>
          <li><CommIndicator v-bind:connected="connections.motors && connections.lcm" name="Driving" /></li>
        </ul>
      </div>
      <div class="spacer"></div>
      <div class="help">
        <img src="/static/help.png" alt="Help" title="Help" width="48" height="48" />
      </div>
      <div class="helpscreen"></div>
      <div class="helpimages" style="display: flex; align-items: center; justify-content: space-evenly">
        <img v-if="controlMode === 'arm'" src="/static/arm.png" alt="Robot Arm" title="Robot Arm Controls" style="width: auto; height: 70%; display: inline-block" />
        <img v-else-if="controlMode === 'soil_ac'" src="/static/soil_ac.png" alt="Soil Acquisition" title="Soil Acquisition Controls" style="width: auto; height: 70%; display: inline-block" />
        <img src="/static/joystick.png" alt="Joystick" title="Joystick Controls" style="width: auto; height: 70%; display: inline-block" />
      </div>
    </div>

    <div class="box odom light-bg">
      <OdometryReading v-bind:odom="odom"/>
    </div>
    <div class="box cameras light-bg">
      <Cameras v-bind:servosData="lastServosMessage" v-bind:connections="connections.cameras"/>
    </div>
    <div class="box map light-bg">
      <RoverMap v-bind:odom="odom"/>
    </div>
    <div class="box waypoints light-bg">
      <WaypointEditor v-bind:odom="odom" />
    </div>
    <div class="box controls light-bg">
      <ArmControls/>
      <EncoderCounts/>
      <DriveControls/>
    </div>
  </div>
</template>

<script>
import { mapGetters, mapMutations } from 'vuex'
import Cameras from './Cameras.vue'
import RoverMap from './RoverMap.vue'
import CommIndicator from './CommIndicator.vue'
import OdometryReading from './OdometryReading.vue'
import ArmControls from './ArmControls.vue'
import DriveControls from './DriveControls.vue'
import EncoderCounts from './EncoderCounts.vue'
import WaypointEditor from './WaypointEditor.vue'
import LCMBridge from 'lcm_bridge_client/dist/bridge.js'

let interval;

export default {
  name: 'RATask',
  data () {
    return {
      lcm_: null,

      lastServosMessage: {
        pan: 0,
        tilt: 0
      },

      odom: {
        latitude_deg: 38,
        latitude_min: 24.38226,
        longitude_deg: -110,
        longitude_min: -47.51724,
        bearing_deg: 0,
        speed: 0
      },

      connections: {
        websocket: false,
        lcm: false,
        motors: false,
        cameras: [false, false, false, false, false, false, false, false]
      },

      nav_status: {
        completed_wps: 0,
        total_wps: 0
      }
    }
  },

  methods: {
    publish: function (channel, payload) {
      this.lcm_.publish(channel, payload)
    },

    subscribe: function (channel, callbackFn) {
      if( (typeof callbackFn !== "function") || (callbackFn.length !== 1)) {
        console.error("Callback Function is invalid (should take 1 parameter)")
      }
      this.lcm_.subscribe(channel, callbackFn)
    }
  },

  computed: {
    ...mapGetters('autonomy', {
      autonEnabled: 'autonEnabled'
    }),

    ...mapGetters('controls', {
      controlMode: 'controlMode'
    }),
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
        this.connections.lcm = online[0],
        this.connections.cameras = online.slice(1)
      },
      // Subscribed LCM message received
      (msg) => {
        if (msg.topic === '/odometry') {
          this.odom = msg.message
        } else if (msg.topic === '/kill_switch') {
          this.connections.motors = !msg.message.killed
        } else if (msg.topic === '/debugMessage') {
          if (msg['message']['isError']) {
            console.error(msg['message']['message'])
          } else {
            console.log(msg['message']['message'])
          }
        }
      },
      // Subscriptions
      [
        {'topic': '/odometry', 'type': 'Odometry'},
        {'topic': '/sensors', 'type': 'Sensors'},
        {'topic': '/temperature', 'type': 'Temperature'},
        {'topic': '/kill_switch', 'type': 'KillSwitch'},
        {'topic': '/camera_servos', 'type': 'CameraServos'},
        {'topic': '/encoder', 'type': 'Encoder'},
        {'topic': '/nav_status', 'type': 'NavStatus'},
        {'topic': '/debugMessage', 'type': 'DebugMessage'}
      ]
    )

    const servosMessage = {
      'type': 'CameraServos',
      'pan': 0,
      'tilt': 0
    }

    const JOYSTICK_CONFIG = {
      'forward_back': 1,
      'left_right': 2,
      'dampen': 3,
      'kill': 4,
      'restart': 5,
      'pan': 4,
      'tilt': 5
    }

    interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      for (let i = 0; i < 2; i++) {
        const gamepad = gamepads[i]
        if (gamepad) {
          if (gamepad.id.includes('Logitech')) {
            const servosSpeed = 0.8
            servosMessage['pan'] += gamepad.axes[JOYSTICK_CONFIG['pan']] * servosSpeed / 10
            servosMessage['tilt'] += -gamepad.axes[JOYSTICK_CONFIG['tilt']] * servosSpeed / 10
          }
        }
      }

      const clamp = function (num, min, max) {
        return num <= min ? min : num >= max ? max : num
      }

      servosMessage['pan'] = clamp(servosMessage['pan'], -1, 1)
      servosMessage['tilt'] = clamp(servosMessage['tilt'], -1, 1)
      this.lastServosMessage = servosMessage

      this.lcm_.publish('/camera_servos', servosMessage)
    }, 100)
  },

  components: {
    RoverMap,
    Cameras,
    CommIndicator,
    ArmControls,
    DriveControls,
    EncoderCounts,
    OdometryReading,
    WaypointEditor
  }
}
</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>
  .wrapper {
    display: grid;
    grid-gap: 10px;
    grid-template-columns: 1fr 1fr;
    grid-template-rows: 60px 3fr 1fr 2fr 70px 60px;
    grid-template-areas: "header header" "map cameras" "map waypoints" "map waypoints" "controls waypoints" "odom waypoints";
    font-family: sans-serif;
    height: 98vh;
  }

  .box {
    border-radius: 5px;
    padding: 10px;
    border: 1px solid black;
  }

  .light-bg {
    background-color: LightGrey;
  }

  img {
    border: none;
    border-radius: 0px;
  }

  .header {
    grid-area: header;
    display: flex;
    align-items: center;
  }

  .header h1 {
    margin-left: 5px;
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

  .helpscreen {
    z-index: 1000000000;
    display: block;
    visibility: hidden;
    background-color: black;
    opacity: 0.8;
    position: absolute;
    left: 0px;
    top: 0px;
    width: 100%;
    height: 100%;
  }

  .helpimages {
    z-index: 1000000001;
    visibility: hidden;
    position: absolute;
    left: 5%;
    top: 5%;
    width: 90%;
    height: 90%;
  }

  .help {
    z-index: 1000000002;
    display: flex;
    float: right;
    opacity: 0.8;
    cursor: auto;
  }

  .help:hover {
    opacity: 1.0;
    cursor: pointer;
  }

  .help:hover ~ .helpscreen, .help:hover ~ .helpimages {
    visibility: visible;
  }

  .odom {
    grid-area: odom;
    font-size: 1em;
  }

  .diags {
    grid-area: diags;
  }

  .map {
    grid-area: map;
  }

  .waypoints {
    grid-area: waypoints;
  }

  .controls {
    grid-area: controls;
    font-size: 1em;
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
  }

  ul#vitals li {
    display: inline;
    float: left;
    padding: 0px 10px 0px 0px;
  }
</style>
