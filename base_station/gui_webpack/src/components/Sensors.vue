<template>
  <div class="wrap">
    <div class="box">
      <Checkbox v-bind:name="'Recording'" v-on:toggle="toggleRecording($event)"/>
    </div>
    <div class='box'></div>

    <SensorItem v-bind='sensor' v-for='(sensor, index) in sensorData' :key='index'/>
    <div v-if='sensorData.length%2==1' class='box' ></div>
  </div>
</template>

<script>
import SensorItem from './SensorItem.vue'
import Checkbox from './Checkbox.vue'
import {mapMutations} from 'vuex'

export default {
  name: 'Sensors',

  data() {
    return {
      rawData: {
        temperature: 0,
        moisture: 0,
        conductivity: 0,
        pH: 0,
        O2: 0,
        CO2: 0,
        bcpu_temp: 0,
        gpu_temp: 0,
        tboard_temp: 0
      }
    }
  },

  computed: {
    color: function () {
      return this.recording ? 'green' : 'red'
    },
    sensorData: function () {
      return [
        {
          name: 'Temperature',
          value: this.rawData.temperature,
          unit: 'ºC'
        },
        {
          name: 'Moisture',
          value: this.rawData.moisture
        },
        {
          name: 'Soil Conductivity',
          value: this.rawData.conductivity,
          unit: 'µS/cm'
        },
        {
          name: 'pH',
          value: this.rawData.pH
        },
        {
          name: 'Oxygen',
          value: this.rawData.O2,
          unit: 'ppm'
        },
        {
          name: 'Carbon Dioxide',
          value: this.rawData.CO2,
          unit: 'ppm'
        },
        {
          name: 'CPU Temperature',
          value: this.rawData.bcpu_temp / 1000,
          unit: 'ºC'
        },
        {
          name: 'GPU Temperature',
          value: this.rawData.gpu_temp / 1000,
          unit: 'ºC'
        },
        {
          name: 'Overall Board Temperature',
          value: this.rawData.tboard_temp / 1000,
          unit: 'ºC'
        }
      ]
    }
  },

  methods: {
    toggleRecording: function (record) {
      const msg = {
        'type': 'SensorSwitch',
        'should_record': record
      }
      this.$parent.publish('/sensor_switch', msg)
    },
  },

  created: function() {
    this.$parent.subscribe('/sensors', (msg) => {
      this.rawData = Object.assign(this.rawData, msg)
    })

    this.$parent.subscribe('/temperature', (msg) => {
      this.rawData = Object.assign(this.rawData, msg)
    })
  },

  components: {
    Checkbox,
    SensorItem
  }
}
</script>

<!-- Add 'scoped' attribute to limit CSS to this component only -->
<style scoped>
.wrap {
  display: grid;
  grid-template-columns: 1fr 1fr;
  grid-template-rows: repeat(auto-fill, 35px);
  height: 100%;
}

.white-text{
  color: white;
}

.box {
  padding: 0px;
  padding-left: 5px;
  padding-right: 5px;
  border: none;
}

.green {
  background-color: green;
}

.red {
  background-color: red;
}
</style>
