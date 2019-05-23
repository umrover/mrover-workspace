<template>
  <div class="wrap">
    <div class="led-toggles">
      <Checkbox  v-bind:name="'Toggle Backlights'" v-on:toggle="setPart('backlights', $event)"/>
      <Checkbox v-bind:name="'Toggle UV Lights'" v-on:toggle="setPart('uv_leds', $event)"/>
    </div>
    <div class="site-0">
      <SASiteControls v-bind:site="1"/>
    </div>
    <div class="site-1">
      <SASiteControls v-bind:site="2"/>
    </div>
    <div class="raman_button">
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
    grid-template-areas: "site_0 led_toggles" "site_1 raman_button";

    font-family: sans-serif;
    height: 100%;
  }

  .box {
    border-radius: 5px;
    padding: 10px;
    border: 1px solid black;
  }

  .site-0 {
    display: flex;
    grid-area: site_0;
    align-items: center;
    justify-content: center;
  }

  .site-1 {
    display: flex;
    grid-area: site_1;
    align-items: center;
    justify-content: center;
  }

  .led-toggles {
    display: flex;
    grid-area: led_toggles;
    align-items: center;
    justify-content: center;
  }

  .raman_button {
    display: flex;
    grid-area: raman_button;
    align-items: center;
    justify-content: center;
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

    created: function () {

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
