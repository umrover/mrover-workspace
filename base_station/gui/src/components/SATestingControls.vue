<template>
  <div class="wrap">
    <div class="flex">
      <SASiteControls v-bind:site="1"/>
    </div>

    <div class="temperatureBox">
      <span>
        Temperature: {{thermistor_data.temperature}}<br>
      </span>
    </div>
    <div class="flex">
      <SASiteControls v-bind:site="2"/>
    </div>
    <div class="spectrometerBox">
      <div class="spectrometerInput">
        <input type="number" v-model="id">
        <button v-on:click="sendSpectralCmd(id)">Get Spectral Data</button>
      </div>
      <div class="spectrometerOutput">
        <span>
          r: {{SpectralData.r}}<br>
          g: {{SpectralData.g}}<br>
          a: {{SpectralData.a}}<br>
          s: {{SpectralData.s}}<br>
          h: {{SpectralData.h}}<br>
          b: {{SpectralData.b}}<br>
          t: {{SpectralData.t}}<br>
          i: {{SpectralData.i}}<br>
          c: {{SpectralData.c}}<br>
          u: {{SpectralData.u}}<br>
          j: {{SpectralData.j}}<br>
          d: {{SpectralData.d}}<br>
          v: {{SpectralData.v}}<br>
          k: {{SpectralData.k}}<br>
          e: {{SpectralData.e}}<br>
          w: {{SpectralData.w}}<br>
          l: {{SpectralData.l}}<br>
          f: {{SpectralData.f}}
        </span>
      </div>
    </div>
    <div class="box">
      <h4>GPS Data</h4>
      Latitude: {{gps_data.latitude_deg}}<br>
      Min Latitude: {{gps_data.latitude_min}}<br>
      Longitude: {{gps_data.longitude_deg}}<br>
      Min Longitude: {{gps_data.longitude_min}}<br>
      Bearing: {{gps_data.bearing_deg}}<br>
      Speed: {{gps_data.speed}}
    </div>
    <div class="box">
      <div class="center">
        <h4>Mosfet Devices</h4>
      </div>
      <div class="deviceToggles">
      <template v-for="id in 10">
          <Checkbox v-bind:name="id - 1" v-on:toggle="setDevice(id - 1, $event)"/>
      </template>
      </div>
    </div>
    <div class="flex">
      <button ref="raman" class="button" v-on:click="sendCollect($event)"> <span>Raman Test</span> </button>
    </div>
  </div>
</template>

<style scoped>
  .wrap {
    display: grid;
    grid-gap: 10px;
    grid-template-columns: 1fr 1fr;
    grid-template-rows: 1fr 1fr 1fr;
  }

  .box {
    height: 200px;
    border-radius: 5px;
    padding: 10px;
    border: 1px solid black;
  }

  .flex {
    display: flex;
    align-items: center;
    justify-content: center;
  }

  .deviceToggles {
    display: flex;
  }

  .spectrometerBox {
    display: flex;
    align-items: center;
    justify-content: center;
    height: 200px;
    border-radius: 5px;
    padding: 10px;
    border: 1px solid black;
  }

  .spectrometerInput {
    display: grid;
    grid-template-rows: 1fr 1fr 1fr 1fr;
    justify-items: center;
    padding-right: 10px;
  }

  .spectrometerOutput {
    height: 200px;
    width: 80px;
    overflow: auto;
    grid-template-rows: 1fr;
    justify-items: center;
    padding: 10px;
  }

  .temperatureBox {
    height: 200px;
    border-radius: 5px;
    padding: 10px;
    border: 1px solid black;    
    display: flex;
    align-items: center;
    justify-content: center;
  }

  .center {
    text-align: center;
  }
</style>

<script>
  import SASiteControls from './SASiteControls.vue'
  import Checkbox from './Checkbox.vue'
  
  export default {
    data() {
      return {
        id: 0,

        SpectralData: {
          r: 0,
          g: 0,
          a: 0,
          s: 0,
          h: 0,
          b: 0,
          t: 0,
          i: 0,
          c: 0,
          u: 0,
          j: 0,
          d: 0,
          v: 0,
          k: 0,
          e: 0,
          w: 0,
          l: 0,
          f: 0
        },
        
        thermistor_data: {
          temperature: 0
        },

        gps_data: {
          latitude_deg: 0,
          latitude_min: 0,
          longitude_deg: 0,
          longitude_min: 0,
          bearing_deg: 0,
          speed: 0
        }
      }
    },

    beforeDestroy: function () {

    },

    mounted: function () {
      this.$refs["rgb"].active = true
    },

    props: {

    },

    created: function () {
      this.$parent.subscribe('/thermistor_data', (msg) => {
        this.thermistor_data = msg
      })

      this.$parent.subscribe('/gps_data', (msg) => {
        this.gps_data = msg
      })

      this.$parent.subscribe('/spectral_data', (msg) => { 
        this.SpectralData = msg
      })
    },

    methods: {

      sendCollect: function (button) {
        this.$parent.publish("/raman_collect", {"type": "Signal"})
        let obj = this.$refs["raman"]
        obj.disabled = true
        setTimeout(function() {
          obj.disabled = false;
        }, 2000);
      },

      setDevice: function(id, enabled) {
        this.$parent.publish("/mosfet_cmd", {
          'type': 'MosfetCmd',
          'device': Number(id),
          'enable' : enabled
        })
      },
      
      sendSpectralCmd: function(id) {
        this.$parent.publish("/spectral_cmd",
         {'type' : "SpectralCmd",
           'id' : Number(id)})
      },
    },

    components: {
      SASiteControls,
      Checkbox
    }
  }
</script>
