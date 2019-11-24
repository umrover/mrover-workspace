<template>
  <div class="wrapper">
    <div class="box header">
      <img src="/static/mrover.png" alt="MRover" title="MRover" width="48" height="48" />
      <h1>LCM Send</h1>
      <div class="spacer"></div>
      <div class="comms">
        <ul id="vitals">
          <li><CommIndicator v-bind:connected="connections.websocket" name="Web Socket" /></li>
          <li><CommIndicator v-bind:connected="connections.lcm" name="Rover Connection Status" /></li>
        </ul>
      </div>
      <div class="spacer"></div>
    </div>
    <div class="box">

      <label style="float: left" :for="'channel'">Channel:  </label>
      <input id='channel' type="text" v-model='channel_name' />

      <br/>
      <br/>

      <label style="float: left" :for="'message_type'">Message Type</label>
      <!-- <input type="text" :id="'Channel'" class ="lcm_send_input" v-model="channel_name">-->
      <select class="lcm_send_input" id="message_type" v-model="selectedOption" @change="switch_message_type()" required> 
        <option value="" selected>select a message type</option>
        <option v-for="option in options" v-bind:value="option.text">
          {{ option.text }}
        </option>
      </select>



      <br/>
      <br/>
      
      <div class="lcm_send_input"> 
        <div class="param" v-for="datum in commandParams"> 
          <label class="param_type" > {{ datum.type }} </label>
          <label class="param_label"> {{ datum.label }} </label>
          <input type="text" v-model="datum.value"/>
        </div>
      </div>     

      <!--<textarea class="lcm_send_input message_textarea" :id="'Message'" v-model="message_text"></textarea> -->

      <br/>

      <button type="button" v-on:click="send_message()">Send</button>
    </div>



  </div>
</template>

<script>
  import LCMBridge from 'lcm_bridge_client/dist/bridge.js';
  import CommIndicator from './CommIndicator.vue'
  import msgs from '../static/rover_msgs.json'

  export default {
    name: 'LCMSend',
    mounted() {
      this.populate();
    },
    data () {
      return {
        lcm_: null,
        connections: {
          websocket: false,
          lcm: false
        },
        channel_name: "",
        message_text: "",
        options: [],
        selectedOption: '',
        commandParams: []
      }
    },

    methods: {
      send_message: function () {

        var msg = {type: this.selectedOption};

        for(var i = 0; i < this.commandParams.length; ++i) {
          var param = this.commandParams[i];

          //Convert to boolean, 
          if(param.type == "boolean") {
            if(param.value.toLowerCase() == "true") {
              msg[param.label] = true;
            }

            else if(param.value.toLowerCase() == "false") {
              msg[param.label] = false;
            }

            else {
              console.error("Input for boolean parameter '" + param.label + "' is not 'true' or 'false'");
              return;
            }
          }

          //convert to number
          else if(param.type == "double" || param.type == "float" || param.type.startsWith("int")) {
            msg[param.label] = Number(param.value);
          }

          //leave as string
          else if(param.type == "string") {
            msg[param.label] = param.value;
          }
          
          //otherwise, convert to JS object
          else {
            msg[param.label] = JSON.parse(param.value);
          }
        }

        
        console.log("Sending message:", msg);

        this.lcm_.publish(this.channel_name, msg);

        //Example:
        //this.lcm_.publish('/auton', {type: 'AutonState', is_auton: this.auton_enabled})
      },

      switch_message_type: function() {

        //Clear commandParams array
        this.commandParams.length = 0;

        //Populate commandParams arary
        for(var i = 0; i < msgs[this.selectedOption].length; ++i)
        {
          this.commandParams.push({ type: msgs[this.selectedOption][i][0], label: msgs[this.selectedOption][i][1] });
        }
      },

      populate() {
        
        for(var m in msgs)
         {
           this.options.push({text: m, value: m });
         }
        
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
    grid-template-columns: 1fr 1fr;
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

  .lcm_send_input {
    width: 100%;
  }

  .message_textarea {
    height: 500px;
  }

  .param {
    margin-bottom: 5px
  }

  .param_type {
    color: grey;
    margin-right: 2px;
  }

  .param_label {
    margin-right: 2px;
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

  textarea {
    resize: none;
  }
</style>
