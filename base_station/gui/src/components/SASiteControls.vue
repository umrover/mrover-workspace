<template>
  <div class="wrap2 box">
    <div>
      <h4>Site {{site}}</h4>
    </div>
    <div class="horizontal-buttons">
      <button v-on:click="startTest('Flouresence')" :disabled="!enable_tests">Flouresence Test</button>
      <button v-on:click="startTest('Biuret')" :disabled="!enable_tests">Biuret Test</button>
      <button v-on:click="startTest('Ammonia')" :disabled="!enable_tests">Ammonia Test</button>
      <button v-on:click="sendMicroCam()">MicroCam</button>
    </div>
    <div class="servo">
      <input type="number" v-model="degrees">
      <button v-on:click='sendServo()'>Update Servo Degrees</button>
    </div>
  </div>
</template>

<style scoped>
  .wrap2 {
    display: grid;
    grid-gap: 10px;
    grid-template-rows: 60px 1fr 1fr;

    font-family: sans-serif;
    height: 100%;
  }

  .box {
    border-radius: 5px;
    padding: 10px;
    border: 1px solid black;
  }

  .horizontal-buttons {
    grid-template-areas: none;
    display: flex;
    padding: 0px 0px 0px 0px;
    height: 100%;
  }

</style>

<script>
  import Checkbox from './Checkbox.vue'

  export default {
    data() {
      return {
        enable_tests: true,
        degrees: 0.0
      }
    },

    props : {
      site: {
        type: Number,
        required: true
      }
    },

    created: function () {
      this.$parent.$parent.subscribe('/test_enable', (msg) => {
        if(msg.site == this.site) {
          this.enable_tests = msg.enabled
        }
      })
    },

    methods: {
      startTest: function(test_name) {
        this.$parent.$parent.publish("/start_test", {
          'type': 'StartTest',
          'test': test_name,
          'site': this.site
        })
      },

      sendServo: function() {
        this.$parent.$parent.publish("/servo", {
          'type': 'Servo',
          'id': 'servo_' + this.site,
          'degrees': parseFloat(this.degrees)
        })
      },

      sendMicroCam: function() {
        this.$parent.$parent.publish("/microcam", {
          'type': 'MicroCam',
          'id': 'camera_' + this.site
        })
      }
    },

    components: {
      Checkbox
    }
  }
</script>
