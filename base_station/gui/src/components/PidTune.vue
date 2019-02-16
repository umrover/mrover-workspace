<template>
  <div class="wrapper">
    <div class="box header">
      <img src="/static/mrover.png" alt="MRover" title="MRover" width="48" height="48" />
      <h1>PID Tuning</h1>
    </div>

    <div class="box pid">
      <h2>Set PID constant</h2><br>
      <p>Device ID: <input v-model="deviceID"></p>
      <p>K<sub>P</sub>: <input v-model="Kp"></p>
      <p>K<sub>I</sub>: <input v-model="Ki"></p>
      <p>K<sub>D</sub>: <input v-model="Kd"></p>
      <p>K<sub>F</sub>: <input v-model="Kf"></p>

      <button v-on:click='configPID()'>Set Parameters</button>
      <br>
      <button v-on:click='setDefaultPIDs("arm")'>Set Arm PID Values</button>
      <button v-on:click='setDefaultPIDs("science")'>Set Science PID Values</button>
    </div>

    <div class="box demand">
      <p>Device ID: <input v-model="deviceID"></p>
      <p>Control Mode:
        <select v-model="controlMode">
          <option value='0'>Throttle</option>
          <option value='5'>Follower Mode</option>
          <option value='4'>Voltage Mode</option>
          <option value='1'>Position Mode</option>
          <option value='2'>Speed Mode</option>
          <option value='3'>Current Mode</option>
          <option value='6'>Motion Profile Mode</option>
          <option value='7'>Motion Magic</option>
          <option value='15'>Disabled</option>
        </select>
      </p>
      <p>Demand: <input v-model="demand"></p>
      <button v-on:click='setDemand()'>Set Demand</button>
    </div>
    
    <div class="box graph">
        <PidChart v-bind:chartData="chartData" :width="400" :height="310" ref="pidChart"/>
    </div>
  </div>
</template>

<script>
  import LCMBridge from 'lcm_bridge_client/dist/bridge.js';
  import PidChart from './PidChart.vue'

  const time=10;
  const refresh=0.1;
  export default {
    name: 'PidTune',
    data () {
      return {
        lcm_: null,
        connections: {
          websocket: false,
          lcm: false
        },
        deviceID: 0,
        Kp: 0.0,
        Ki: 0.0,
        Kd: 0.0,
        Kf: 0.0,
        controlMode: 0,
        demand: 0,
        chartData: [
          { data: [],
            fill: false,
            borderColor: 'rgba(0,0,255,0.8)',
            xAxisID: 'x-axis',
            yAxisID: 'y-axis',
            label: "Encoder Data"
          },
          { data: [{x:0, y:0}, {x:10, y:0}],
            fill: false,
            borderColor: 'rgba(0,255,0,0.8)',
            xAxisID: 'x-axis',
            yAxisID: 'y-axis',
            label: "Target Demand"
          }
        ],
        defaultPIDS:{
          "arm":{
            "4":{
              "kp": 0.0,
              "ki": 0.0,
              "kd": 0.0,
              "kf": 0.0
            },
            "5":{
              "kp": 0.0,
              "ki": 0.0,
              "kd": 0.0,
              "kf": 0.0
            },
            "6":{
              "kp": 0.0,
              "ki": 0.0,
              "kd": 0.0,
              "kf": 0.0
            },
            "7":{
              "kp": 0.0,
              "ki": 0.0,
              "kd": 0.0,
              "kf": 0.0
            },
            "8":{
              "kp": 0.0,
              "ki": 0.0,
              "kd": 0.0,
              "kf": 0.0
            },
            "9":{
              "kp": 0.0,
              "ki": 0.0,
              "kd": 0.0,
              "kf": 0.0
            }
          },

          "science":{
            "4":{
              "kp": 0.0,
              "ki": 0.0,
              "kd": 0.0,
              "kf": 0.0
            },
            "5":{
              "kp": 0.0,
              "ki": 0.0,
              "kd": 0.0,
              "kf": 0.0
            },
            "6":{
              "kp": 0.0,
              "ki": 0.0,
              "kd": 0.0,
              "kf": 0.0
            },
            "7":{
              "kp": 0.0,
              "ki": 0.0,
              "kd": 0.0,
              "kf": 0.0
            },
            "8":{
              "kp": 0.0,
              "ki": 0.0,
              "kd": 0.0,
              "kf": 0.0
            },
            "9":{
              "kp": 0.0,
              "ki": 0.0,
              "kd": 0.0,
              "kf": 0.0
            }
          }
        }
      }
    },

    mounted () {
      let initData=[];

      for (let i = 0; i < Math.round(time/refresh); i++) {
        initData.push({x:i*refresh, y:0});
      };
      this.chartData[0].data = initData;
      this.$refs.pidChart.renderPIDChart();
    },

    methods: {
      configPID(){
        const msg = {
          'type':'PIDConstants',
          'deviceID': parseInt(this.deviceID),
          'kP': parseFloat(this.Kp),
          'kI': parseFloat(this.Ki),
          'kD': parseFloat(this.Kd),
          'kF': parseFloat(this.Kf)
        }
        this.lcm_.publish('/config_pid', msg);
      },

      setDemand(){
        this.chartData[1].data = [{x:0, y:this.demand}, {x:10, y:this.demand}];
        this.$refs.pidChart.renderPIDChart();
        this.lcm_.publish("/set_demand", {
          'type': 'SetDemand',
          'deviceID': parseInt(this.deviceID),
          'control_mode': parseInt(this.controlMode),
          'value': parseInt(this.demand)
        });
      },

      setDefaultPIDs(mode){
        const pids=this.defaultPIDS[mode];
        Object.keys(pids).forEach( (id) => {
          const constants = {
            'kp': pids[id]["kp"],
            'ki': pids[id]["ki"],
            'kd': pids[id]["kd"],
            'kf': pids[id]["kf"]
          }
          this.sendPIDs(id, constants)
        });
      },
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
          if (msg.topic === '/encoder') {
            let val=0;
            switch(parseInt(this.deviceID)){
              case 4:
                val=msg.message.joint_a;
                break;
              case 5:
                val=msg.message.joint_b;
                break;
              case 6:
                val=msg.message.joint_c;
                break;
              case 7:
                val=msg.message.joint_d;
                break;
              case 8:
                val=msg.message.joint_e;
                break;
            }

            const data=this.chartData[0].data;
            for (let i = 1; i < data.length; i++) {
              data[i-1].y=data[i].y;
            }
            data[data.length-1].y=val;
            this.$refs.pidChart.renderPIDChart();
            // chart.update(); and update demand

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
          {'topic': '/encoder', 'type': 'Encoder'},
          {'topic': '/debugMessage', 'type': 'DebugMessage'}
        ]
      )
    },

    components: {
      PidChart
    }
  }
</script>

<style scoped>
  .wrapper {
    display: grid;
    grid-gap: 10px;
    grid-template-columns: 2fr 3fr;
    grid-template-rows: 60px 3fr 2fr;
    grid-template-areas: "header header" "pid graph" "demand graph";

    font-family: sans-serif;
    height: 100%;
  }

  .header {
    grid-area: header;
    display: flex;
    align-items: center;
  }

  .graph{
    grid-area: graph;
    display: grid;
  }

  .pid {
    grid-area: pid;
    display: grid;
  }

  .demand {
    grid-area: demand;
    display: grid;
  }

  .header h1 {
    margin-left: 5px;
  }

  .box {
    border-radius: 5px;
    padding: 10px;
    border: 1px solid black;
  }

  p {
    margin-bottom: 1em;
  }
</style>
