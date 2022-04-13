<template>
  <div class="wrapper">
    <div class="box header">
      <img src="/static/mrover.png" alt="MRover" title="MRover" width="48" height="48" />
      <h1>Science Dashboard</h1>
      <div class="spacer"></div>
      <div class="comms">
        <ul id="vitals">
          <li><CommIndicator v-bind:connected="connections.websocket" name="Web Socket" /></li>
          <li><CommIndicator v-bind:connected="connections.lcm" name="Rover Connection Status" /></li>
        </ul>
      </div>
      <div class="spacer"></div>
      <div class="help">
        <img src="/static/help.png" alt="Help" title="Help" width="48" height="48" />
      </div>
      <div class="helpscreen"></div>
      <div class="helpimages" style="display: flex; align-items: center; justify-content: space-evenly">
        <img src="/static/joystick.png" alt="Joystick" title="Joystick Controls" style="width: auto; height: 70%; display: inline-block" />
      </div>
    </div>

    <div class="box raman light-bg">
      <Raman v-bind:mosfetIDs="mosfetIDs"/>
    </div>
    <div class="box cameras light-bg">
      <Cameras/>
    </div>
    <div class="box spectral light-bg">
      <SpectralData v-bind:spectral_triad_data="spectral_triad_data"/>
    </div>
    <div class = "box light-bg chlorophyll">
      <Chlorophyll v-bind:mosfetIDs="mosfetIDs" v-bind:spectral_data="spectral_data"/> 
      <GenerateReport v-bind:spectral_data="spectral_data"/>
    </div>
    <div class="box striptest light-bg">
      <StripTest/>
    </div>
    <div class="box amino light-bg">
      <Amino v-bind:mosfetIDs="mosfetIDs"/>
    </div>
    <div class="box drives light-bg">
      <DriveVelDataV/>
    </div>
    <div class="box carousel light-bg">
      <Carousel/>
    </div>
    <div class="box scoopUV light-bg">
      <ScoopUV v-bind:mosfetIDs="mosfetIDs"/>
    </div>
    <div class="box SAArm light-bg">
      <SAArm/>
    </div>
    <div class="box PDB light-bg">
      <PDBFuse/>
    </div>
  </div>
</template>

<script>
import Cameras from './Cameras.vue'
import CommIndicator from './CommIndicator.vue'
import Raman from './Raman.vue'
import WaypointEditor from './WaypointEditor.vue'
import LCMBridge from 'lcm_bridge_client/dist/bridge.js'
import SpectralData from './SpectralData.vue'
import Chlorophyll from './Chlorophyll.vue'
import StripTest from './StripTest.vue'
import DriveVelDataV from './DriveVelDataV.vue'
import Amino from './Amino.vue'
import GenerateReport from './GenerateReport.vue'
import Carousel from './Carousel.vue'
import ScoopUV from './ScoopUV.vue'
import SAArm from './SAArm.vue'
import PDBFuse from './PDBFuse.vue'

let interval;

export default {
  name: 'Dashboard',
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
        longitude_deg: 110,
        longitude_min: 47.51724,
        bearing_deg: 0
      },

      connections: {
        websocket: false,
        lcm: false
      },

      nav_status: {
        completed_wps: 0,
        total_wps: 0
      },
      spectral_data: {
          d0_1:0,
          d0_2:0,
          d0_3:0,
          d0_4:0,
          d0_5:0,
          d0_6:0,
          d1_1:0,
          d1_2:0,
          d1_3:0,
          d1_4:0,
          d1_5:0,
          d1_6:0,
          d2_1:0,
          d2_2:0,
          d2_3:0,
          d2_4:0,
          d2_5:0,
          d2_6:0
      },
      spectral_triad_data: {
          d0_1:0,
          d0_2:0,
          d0_3:0,
          d0_4:0,
          d0_5:0,
          d0_6:0,
          d1_1:0,
          d1_2:0,
          d1_3:0,
          d1_4:0,
          d1_5:0,
          d1_6:0,
          d2_1:0,
          d2_2:0,
          d2_3:0,
          d2_4:0,
          d2_5:0,
          d2_6:0
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
      (msg) => {
        if (msg.topic === '/odometry') {
          this.odom = msg.message
        } else if (msg.topic ==='/spectral_data'){
          this.spectral_data = msg.message
        } else if (msg.topic ==='/spectral_triad_data'){
          this.spectral_triad_data = msg.message
        } else if (msg.topic ==='/thermistor_data'){
          this.thermistor_data = msg.message
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
        {'topic': '/nav_status', 'type': 'NavStatus'},
        {'topic': '/test_enable', 'type': 'TestEnable'},
        {'topic': '/debugMessage', 'type': 'DebugMessage'},
        {'topic': '/spectral_data', 'type': 'SpectralData'},
        {'topic': '/spectral_triad_data', 'type': 'SpectralData'},
        {'topic': '/thermistor_data', 'type': 'ThermistorData'},
        {'topic': '/mosfet_cmd', 'type': 'MosfetCmd'},
        {'topic': '/drive_vel_data', 'type': 'DriveVelData'},
        {'topic': '/drive_state_data', 'type': 'DriveStateData'},
        {'topic': '/carousel_data', 'type': 'CarouselPosition'},
        {'topic': '/sa_position', 'type': 'SAPosition'},
        {'topic': '/sa_offset_pos', 'type': 'SAPosition'},
        {'topic': '/arm_control_state_to_gui', 'type': 'ArmControlState'},
        {'topic': '/heater_state_data', 'type': 'Heater'},
        {'topic': '/heater_auto_shutdown_data', 'type': 'HeaterAutoShutdown'},
        {'topic': '/pdb_data', 'type': 'PDBData'},
        {'topic': '/fuse_data', 'type': 'FuseData'},
        {'topic': '/scoop_limit_switch_enable_cmd', 'type': 'ScoopLimitSwitchEnable'}
      ]
    )
  },

  components: {
    Cameras,
    CommIndicator,
    Raman,
    WaypointEditor,
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
        grid-template-columns: auto auto auto;
        grid-template-rows: 60px auto auto auto auto auto;
        grid-template-areas: "header header header" 
                             "cameras cameras cameras" 
                             "carousel chlorophyll raman" 
                             "spectral chlorophyll scoopUV" 
                             "spectral chlorophyll drives"
                             "SAArm striptest drives"
                             "SAArm amino drives"
                             "PDBFuse amino drives";
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

    .waypoints {
        grid-area: waypoints;
    }

    .drives {
      grid-area: drives;
    }

    .cameras {
      grid-area: cameras;
    }

    .spectral {
      grid-area: spectral;
    }

    .controls {
        grid-area: controls;
        font-size: 1em;
    }

    .carousel {
      grid-area: carousel;
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
