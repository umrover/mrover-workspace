<template>
  <div class="wrapper">
    <div class="box header">
      <img src="/static/mrover.png" alt="MRover" title="MRover" width="48" height="48" />
      <h1>ISH Dashboard</h1>
      <div class="spacer"></div>
      <div class="comms">
        <ul id="vitals">
          <li><CommIndicator v-bind:connected="connections.websocket" name="Web Socket" /></li>
          <li><CommIndicator v-bind:connected="connections.lcm" name="Rover Connection Status" /></li>
        </ul>
      </div>
    </div>

    <div class="box raman light-bg">
      <Raman v-bind:mosfetIDs="mosfetIDs"/>
    </div>
    <div class="box cameras light-bg">
      <Cameras v-bind:numCams="2" v-bind:mission="Science" v-bind:channel="'/cameras_control_ish'"/>
    </div>
    <div class = "box light-bg chlorophyll">
      <Chlorophyll v-bind:mosfetIDs="mosfetIDs" v-bind:spectral_data="spectral_data"/>
    </div>
    <div class="box striptest light-bg">
      <StripTest/>
    </div>
    <div class="box amino light-bg">
      <Amino v-bind:mosfetIDs="mosfetIDs"/>
    </div>
    <div class="box carousel light-bg">
      <Carousel/>
    </div>
  </div>
</template>

<script>
import Cameras from './Cameras.vue'
import CommIndicator from './CommIndicator.vue'
import Raman from './Raman.vue'
import LCMBridge from 'lcm_bridge_client/dist/bridge.js'
import Chlorophyll from './Chlorophyll.vue'
import StripTest from './StripTest.vue'
import Amino from './Amino.vue'
import Carousel from './Carousel.vue'

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
        if (msg.topic ==='/spectral_data'){
          this.spectral_data = msg.message
        } else if (msg.topic ==='/thermistor_data'){
          this.thermistor_data = msg.message
        }
      },
      // Subscriptions
      [
        {'topic': '/test_enable', 'type': 'TestEnable'},
        {'topic': '/spectral_data', 'type': 'SpectralData'},
        {'topic': '/thermistor_data', 'type': 'ThermistorData'},
        {'topic': '/mosfet_cmd', 'type': 'MosfetCmd'},
        {'topic': '/carousel_data', 'type': 'CarouselPosition'},
        {'topic': '/heater_state_data', 'type': 'Heater'},
        {'topic': '/heater_auto_shutdown_data', 'type': 'HeaterAutoShutdown'},
      ]
    )
  },

  components: {
    Cameras,
    CommIndicator,
    Raman,
    Chlorophyll,
    StripTest,
    Amino,
    Carousel
  }
}
</script>

<style scoped>

    .wrapper {
        display: grid;
        grid-gap: 10px;
        grid-template-columns: auto auto auto auto;
        grid-template-rows: 60px auto auto auto;
        grid-template-areas: "header header header header" 
                             "cameras cameras cameras raman"
                             "chlorophyll chlorophyll carousel carousel"
                             "amino amino striptest striptest";
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

    .cameras {
      grid-area: cameras;
    }

    .carousel {
      grid-area: carousel;
    }

    ul#vitals li {
        display: inline;
        float: left;
        padding: 0px 10px 0px 0px;
    }
</style>
