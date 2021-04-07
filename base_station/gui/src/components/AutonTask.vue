<template>
  <div class="wrapper">
    <div class="box header">
      <img src="/static/mrover.png" alt="MRover" title="MRover" width="48" height="48" />
      <h1>Auton Dashboard</h1>
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

    <div class="box1 data" v-bind:style="{backgroundColor: nav_state_color}">
     <h1>Nav State: {{this.nav_status.nav_state_name}}</h1>
     <div class="raw-data raw-sensors">
        <RawSensorData v-bind:GPS="GPS" v-bind:IMU="IMU"/>
        <br>
        <br>
        <Obstacle v-bind:Obstacle="Obstacle"/>
        <br>
        <TargetList v-bind:TargetList="TargetList"/>
        <DriveControls/>
        <DriveVelDataH/>
        <SaveAutonData v-bind:odom="odom" v-bind:IMU="IMU" v-bind:GPS="GPS" v-bind:TargetBearing="TargetBearing" v-bind:nav_status="nav_status" v-bind:AutonDriveControl="AutonDriveControl" v-bind:TargetList="TargetList"/>
        <PlaybackAutonData/>
     </div>
    </div>
    <div class="box odom light-bg">
      <OdometryReading v-bind:odom="odom"/>
    </div>
    <div class="box map light-bg">
      <RoverMap v-bind:odom="odom" v-bind:GPS="GPS" v-bind:TargetBearing="TargetBearing"/>
    </div>
    <div class="box waypoints light-bg">
      <AutonWaypointEditor v-bind:odom="odom" v-bind:AutonDriveControl="AutonDriveControl"/>
    </div>
    <div class="box cameras light-bg">
      <Cameras v-if="!this.autonEnabled" v-bind:numCams="4" v-bind:mission="'Auton'" v-bind:channel="'/cameras_control'"/>
    </div>
  </div>

</template>

<script>
import { mapGetters } from 'vuex'
import RoverMap from './AutonRoverMap.vue'
import Cameras from './Cameras.vue'
import CommIndicator from './CommIndicator.vue'
import OdometryReading from './OdometryReading.vue'
import ArmControls from './ArmControls.vue'
import DriveControls from './DriveControls.vue'
import AutonWaypointEditor from './AutonWaypointEditor.vue'
import RawSensorData from './RawSensorData.vue'
import LCMBridge from 'lcm_bridge_client/dist/bridge.js'
import Obstacle from './Obstacle.vue'
import TargetList from './TargetList.vue'
import DriveVelDataH from './DriveVelDataH.vue'
import SaveAutonData from './SaveAutonData.vue'
import PlaybackAutonData from './PlaybackAutonData.vue'

const navBlue = "#4695FF"
const navGreen = "yellowgreen"
const navRed = "lightcoral"
const navGrey = "lightgrey"

export default {
  name: 'AutonTask',
  data () {
    return {
      lcm_: null,

      odom: {
        latitude_deg: 0,
        latitude_min: 0,
        longitude_deg: 0,
        longitude_min: 0,
        bearing_deg: 0,
        speed: 0
      },

      connections: {
        websocket: false,
        lcm: false
      },

      nav_status: {
        nav_state_name: "Off",
        completed_wps: 0,
        total_wps: 0
      },
      nav_state_color: "red",
      nav_counter: 0,

      GPS: {
        latitude_deg: 38,
        latitude_min: 24.38226,
        longitude_deg: -110,
        longitude_min: -47.51724,
        bearing_deg: 0,
        speed: 0
      },
     
      Obstacle: {
	      detected: false,
	      bearing: 0,
        distance: 0
      },

      TargetBearing: {
        target_bearing: 0
      },

      TargetList: [
        {bearing: 0, distance: -1, id: 0},
        {bearing: 0, distance: -1, id: 0}
      ],

      AutonDriveControl: {
        left_percent_velocity: 0,
        right_percent_velocity: 0
      },

      IMU: {
        accel_x_g: 0,
        accel_y_g: 0,
        accel_z_g: 0,
        gyro_x_dps: 0,
        gyro_y_dps: 0,
        gyro_z_dps: 0,
        mag_x_uT: 0,
        mag_y_uT: 0,
        mag_z_uT: 0,
        roll_rad: 0,
        pitch_rad: 0,
        yaw_rad: 0,
        calibration_sys: 0,
        calibration_gyro: 0,
        calibration_accel: 0,
        calibration_mag: 0,
        bearing_deg: 0
      },
      
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
  },

  created: function () {

    setInterval(() => {
      if(this.nav_status.nav_state_name == "Off"){
        this.nav_state_color = navBlue
      }
      else if(this.nav_status.nav_state_name == "Done"){
        if(this.nav_state_color == navBlue || this.nav_state_color == navRed){
          this.nav_state_color = navGreen
        }
        else if(this.nav_counter >= 5 && this.nav_state_color == navGreen){
          this.nav_state_color = navGrey
          this.nav_counter = 0
        }
        else if(this.nav_counter >= 5 && this.nav_state_color == navGrey){
          this.nav_state_color = navGreen
          this.nav_counter = 0
        }
      }
      else{
        this.nav_state_color = navRed
      }
      this.nav_counter = this.nav_counter + 1
      if(this.nav_counter >= 5){
        this.nav_count = 0
      }
    }, 100);

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
        } else if (msg.topic === '/gps') {
          this.GPS = msg.message
        } else if (msg.topic === '/target_bearing') {
          this.TargetBearing = msg.message
        } else if (msg.topic === '/imu_data') {
          this.IMU = msg.message
         }else if (msg.topic === '/auton_drive_control') {
          this.AutonDriveControl = msg.message
        } else if (msg.topic === '/kill_switch') {
          this.connections.motors = !msg.message.killed
        } else if (msg.topic === '/obstacle') {
          this.Obstacle = msg.message
        } else if (msg.topic === '/target_list') {
          this.TargetList = msg.message.targetList
        } else if (msg.topic === '/nav_status') {
          this.nav_status = msg.message
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
        {'topic': '/auton_drive_control', 'type': 'AutonDriveControl'},
        {'topic': '/temperature', 'type': 'Temperature'},
        {'topic': '/kill_switch', 'type': 'KillSwitch'},
        {'topic': '/estimated_gate_location', 'type': 'EstimatedGateLocation'},
        {'topic': '/nav_status', 'type': 'NavStatus'},
        {'topic': '/gps', 'type': 'GPS'},
        {'topic': '/imu_data', 'type': 'IMUData'},
        {'topic': '/debugMessage', 'type': 'DebugMessage'},
        {'topic': '/obstacle', 'type': 'Obstacle'},
        {'topic': '/target_list', 'type': 'TargetList'},
        {'topic': '/drive_vel_data', 'type': 'DriveVelData'},
        {'topic': '/drive_state_data', 'type': 'DriveStateData'},
        {'topic': '/projected_points', 'type': 'ProjectedPoints'},
        {'topic': '/target_bearing', 'type': 'TargetBearing'}
      ]
    )
  },

  components: {
    RoverMap,
    Cameras,
    CommIndicator,
    ArmControls,
    DriveControls,
    OdometryReading,
    RawSensorData,
    AutonWaypointEditor,
    Obstacle,
    TargetList,
    DriveVelDataH,
    SaveAutonData,
    PlaybackAutonData
  }
}
</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>
  .wrapper {
    display: grid;
    overflow:hidden;
    min-height: 98vh;
    grid-gap: 10px;
    grid-template-columns: 2fr 1.25fr 0.75fr;
    grid-template-rows: 50px 2fr 1fr 6vh 24vh;
    grid-template-areas: "header header header" 
                         "map waypoints waypoints"
                         "map waypoints waypoints" 
                         "data waypoints waypoints" 
                         "data angles odom";
    font-family: sans-serif;
    height: auto;
    width: auto;
  }

  .box {
    border-radius: 5px;
    padding: 10px;
    border: 1px solid black;
  }

  .box1 {
    border-radius: 5px;
    background-color: LightGrey;
    padding: 10px;
    border: 1px solid black;
    overflow-y: scroll;
  }

  .box2 {
    display: block;
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
    /* font-size: 1em;
    height: 41%;
    display: inline-block; */
    grid-area: odom;
  }

  .cameras {
    overflow: auto;
  }

  .diags {
    grid-area: diags;
  }

  .map {
    grid-area: map;
  }

  .angles{
    grid-area: angles;
  }
  .waypoints {
    grid-area: waypoints;
  }

  .Joystick {
    font-size: 1em;
    height: 41%;
    width: 93%;
    display: inline-block;
  }

  .raw-sensors{
    font-size: 1em;
  }

  .GPS{
    grid-area: gps;
  }

  .IMU{
    grid-area: imu;
  }

  .data{
    grid-area: data;
  }

  .controls {
    font-size: 1em;
    height: 40.5%;
    display: block;
  }

  ul#vitals li {
    display: inline;
    float: left;
    padding: 0px 10px 0px 0px;
  }
</style>
