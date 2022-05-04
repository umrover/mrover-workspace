<template>
  <div class="wrapper">
    <div class="box header">
      <img src="/static/mrover.png" alt="MRover" title="MRover" width="48" height="48" />
      <h1>Sample Acquisition Dashboard</h1>
      <div class="spacer"></div>
      <div class="comms">
        <ul id="vitals">
          <li><CommIndicator v-bind:connected="connections.websocket" name="Web Socket" /></li>
          <li><CommIndicator v-bind:connected="connections.lcm" name="Rover Connection Status" /></li>
        </ul>
      </div>
    </div>

    <div class="box cameras light-bg">
      <Cameras v-bind:numCams="2" v-bind:mission="'Science'" v-bind:channel="'/cameras_control'"/>
    </div>
    <div class="box drivecontrols light-bg">
      <DriveControls/>
    </div>
    <div class="box drivedata light-bg">
      <DriveVelDataH/>
    </div>
    <div class="box scoopUV light-bg">
      <ScoopUV v-bind:mosfetIDs="mosfetIDs"/>
    </div>
    <div class="box SAArm light-bg">
      <SAArm/>
    </div>
    <div class="box PDBFuse light-bg">
      <PDBFuse/>
    </div>
  </div>
</template>

<script>
import Cameras from './Cameras.vue'
import CommIndicator from './CommIndicator.vue'
import DriveControls from './DriveControls.vue'
import Raman from './Raman.vue'
import LCMBridge from 'lcm_bridge_client/dist/bridge.js'
import DriveVelDataH from './DriveVelDataH.vue'
import ScoopUV from './ScoopUV.vue'
import SAArm from './SAArm.vue'
import PDBFuse from './PDBFuse.vue'

let interval;

export default {
  name: 'Dashboard',
  data () {
    return {
      lcm_: null,

      connections: {
        websocket: false,
        lcm: false
      },

      mosfetIDs: {
        red_led: 0,
        green_led: 1,
        blue_led: 2,
        ra_laser: 3,
        uv_led: 4,
        white_led: 5,
        uv_bulb: 6,
        raman_laser: 7
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

  beforeDestroy: function () {
    window.clearInterval(interval);
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
      (msg) => { },
      // Subscriptions
      [
        {'topic': '/test_enable', 'type': 'TestEnable'},
        {'topic': '/drive_vel_data', 'type': 'DriveVelData'},
        {'topic': '/drive_state_data', 'type': 'DriveStateData'},
        {'topic': '/sa_position', 'type': 'SAPosition'},
        {'topic': '/sa_offset_pos', 'type': 'SAPosition'},
        {'topic': '/arm_control_state_to_gui', 'type': 'ArmControlState'},
        {'topic': '/pdb_data', 'type': 'PDBData'},
        {'topic': '/fuse_data', 'type': 'FuseData'},
        {'topic': '/scoop_limit_switch_enable_cmd', 'type': 'ScoopLimitSwitchEnable'},
        {'topic': '/ra_b_calib_data', 'type': 'Calibrate'},
        {'topic': '/sa_b_calib_data', 'type': 'Calibrate'}
      ]
    )
  },

  components: {
    Cameras,
    CommIndicator,
    DriveControls,
    Raman,
    SpectralData,
    Chlorophyll,
    StripTest,
    DriveVelDataV,
    Amino,
    GenerateReport,
    Carousel,
    ScoopUV,
    SAArm,
    PDBFuse
  }
}
</script>

<style scoped>

    .wrapper {
        display: grid;
        grid-gap: 10px;
        grid-template-columns: auto auto;
        grid-template-rows: 60px auto auto auto auto;
        grid-template-areas: "header header" 
                             "cameras drivecontrols" 
                             "cameras scoopUV" 
                             "SAArm PDBFuse"
                             "drivedata PDBFuse";
        font-family: sans-serif;
        height: auto;
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

    .raman {
        grid-area: raman;
        font-size: 1em;
    }

    .amino{
      grid-area: amino;
    } 

    .striptest{
      grid-area: striptest;
    }
    
    .chlorophyll{
      grid-area: chlorophyll;
    }

    .diags {
        grid-area: diags;
    }

    .drives {
      grid-area: drives;
    }

    .cameras {
      grid-area: cameras;
    }

    .drivecontrols {
      grid-area: drivecontrols;
    }

    .scoopUV {
      grid-area: scoopUV;
    }

    .SAArm {
      grid-area: SAArm;
    }

    .PDBFuse {
      grid-area: PDBFuse;
    }

    ul#vitals li {
        display: inline;
        float: left;
        padding: 0px 10px 0px 0px;
    }
</style>
