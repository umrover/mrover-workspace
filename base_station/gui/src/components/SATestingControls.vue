<template>
  <div class="wrap">
    <div class="flex">
      <SASiteControls v-bind:site="'White'"/>
    </div>
    <div class="flex">
      <SASiteControls v-bind:site="'Yellow'"/>
    </div>
    <div class="flex">
      <SASiteControls v-bind:site="'Blue'"/>
    </div>
    <div class="flex">
      <button ref="raman" class="button" v-on:click="sendCollect($event)"> <span>Raman Test</span> </button>
    </div>
      <div>
        <p>Thermistor Data</p>
          Thermistor: {{thermistor_data.temperature_C}}&deg;C<br>
    </div>
    </div>
</template>
<style scoped>
  .wrap {
    display: grid;
    grid-gap: 10px;
    grid-template-columns: 1fr 1fr;
    grid-template-rows: 1fr 1fr;
    /* height: 300px; */
  }
  .box {
    border-radius: 5px;
    padding: 10px;
    border: 1px solid black;
  }
  .flex {
    display: flex;
    align-items: center;
    justify-content: center;
    height: 200px;
  }
  .led-toggles {
    display: grid;
    grid-template-rows: 1fr 1fr 1fr;
    justify-items: center;
  }
</style>
<script>
  import SASiteControls from './SASiteControls.vue'
  import Checkbox from './Checkbox.vue'
  export default {
    data() {
      return {
        spectral_data: {
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
          temperature_C: 0,
          thermistor: ""
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
      this.$parent.subscribe('/spectral_data', (msg) => {
        this.spectral_data = msg
      }),
      this.$parent.subscribe('/thermistor_data', (msg) => {
        this.thermistor_data = msg
      })
    },
    methods: {
      setPart: function(id, enabled) {
        this.$parent.publish("/mosfet", {
          'type': 'Mosfet',
          'id': id,
          'enable': enabled
        })
      },
      setRGBLeds: function(enabled) {
        this.$parent.publish("/rgb_leds", {
          'type': 'RGBLED',
          'on': enabled
        })
      },

      sendCollect: function (button) {
        this.$parent.publish("/raman_collect", {"type": "Signal"})
        let obj = this.$refs["raman"]
        obj.disabled = true
        setTimeout(function() {
          obj.disabled = false;
        }, 2000);
      }
    },
    components: {
      SASiteControls,
      Checkbox
    }
  }
</script>