<template>
  <div class="wrap">
    <div class="flex">
      <SASiteControls v-bind:site="1"/>
    </div>
    <div class="led-toggles">
      <Checkbox v-bind:name="'Toggle Backlights'" v-on:toggle="setPart('backlights', $event)"/>
      <Checkbox v-bind:name="'Toggle UV Lights'" v-on:toggle="setPart('uv_leds', $event)"/>
      <Checkbox ref="rgb" v-bind:name="'Toggle RGB Sensor Lights'" v-on:toggle="setRGBLeds($event)"/>
    </div>
    <div class="flex">
      <SASiteControls v-bind:site="2"/>
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
