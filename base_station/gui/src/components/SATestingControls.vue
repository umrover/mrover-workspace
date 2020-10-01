<template>
  <div class=“wrap”>
    <div class=“flex”>
      <SASiteControls v-bind:site="'White'"/>
    </div>
    <div class=“flex”>
      <SASiteControls v-bind:site="'Yellow'"/>
    </div>
    <div class=“flex”>
      <SASiteControls v-bind:site="'Blue'"/>
    </div>
    <div class=“flex”>
      <button ref="raman" class="button" v-on:click="sendCollect($event)"> <span>Raman Test</span> </button>
      <p>Spectral Data<p>
      r: {{SpectralData.r}}, g: {{SpectralData.g}}, a: {{SpectralData.a}}<br>
      s: {{SpectralData.s}}, h: {{SpectralData.h}}, b: {{SpectralData.b}}<br>
      t: {{SpectralData.t}}, i: {{SpectralData.i}}, c: {{SpectralData.c}}<br>
      u: {{SpectralData.u}}, j: {{SpectralData.j}}. d: {{SpectralData.d}}<br>
      v: {{SpectralData.v}}, k: {{SpectralData.k}}, e: {{SpectralData.e}}<br>
      w: {{SpectralData.w}}, l: {{SpectralData.l}}, f: {{SpectralData.f}}<br>
    </div>
  </div>
</template>
<style scoped>
  .wrap {
    display: grid;
    grid-gap: 10px;
    grid-template-columns: 1fr 1fr;
    grid-template-rows: 1fr 1fr;
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
      this.$parent.$parent.subscribe('/spectral_data', (msg) => {
        if(msg.site == this.site) {
          this.SpectralData = msg
        }
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