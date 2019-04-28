<template>
  <div class="wrapper">
    <div class="box header">
      <img src="/static/mrover.png" alt="MRover" title="MRover" width="48" height="48" />
      <h1>SA Dashboard</h1>
      <div class="spacer"></div>
      <div class="comms">
      </div>
    </div>

    <div class="box">
      <div class="horiz" v-for="esc, i in throttles">
        <input v-model="throttles[i]"></p>
        <button v-on:click='set(i)'>Set Thottle {{i}}</button>
      </div>
    </div>
  </div>
</template>

<script>
import LCMBridge from 'lcm_bridge_client/dist/bridge.js'
import Checkbox from './Checkbox.vue'

let interval;

export default {
  name: 'ESCTest',

  data() {
    return {
      throttles: [
        0, 0
      ]
    }
  },

  methods: {

    set: function(i) {
      this.publish("/esc_throttle", {
        'type': 'ESCThrottle',
        'esc_id': (i==0 ? "vacuum_1" : "vacuum_2"),
        'percent': parseFloat(this.throttles[i])
      })
    },

    publish: function (channel, payload) {
      this.lcm_.publish(channel, payload)
    }
  },

  created: function () {
    this.lcm_ = new LCMBridge(
      'ws://localhost:8001',
      // Update WebSocket connection state
      (online) => {
      },
      // Update connection states
      (online) => {
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
      []
    )
  },

  components: {
    Checkbox
  }

}</script>
<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>

    .wrapper {
        display: grid;
        grid-gap: 10px;
        grid-template-columns: 1fr;
        grid-template-rows: 60px 1fr;
        grid-template-areas: "header";
        font-family: sans-serif;
        height: 98vh;
    }

    .box {
        border-radius: 5px;
        padding: 10px;
        border: 1px solid black;
    }

    .horiz {
      display: flex;
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
    }

    ul#vitals li {
        display: inline;
        float: left;
        padding: 0px 10px 0px 0px;
    }
</style>
