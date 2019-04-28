<template>
  <div class="wrap2 box">
    <div>
      <h4>Site {{site}}</h4>
    </div>
    <div class="horizontal-buttons">
      <Checkbox v-if="site==0" v-bind:name="'Toggle Main LED'" v-on:toggle="setPart('backlights', $event)"/>
      <Checkbox v-bind:name="'Toggle Solenoid'" v-on:toggle="setPart(site==0?'solenoid_1':'solenoid_2', $event)"></Checkbox>
    </div>
    <div class="horizontal-buttons">
      <button v-on:click="startTest('Flouresence')" :disabled="!enable_tests">Flouresence Test</button>
      <button v-on:click="startTest('Biuret')" :disabled="!enable_tests">Biuret Test</button>
      <button v-on:click="startTest('Ammonia')" :disabled="!enable_tests">Ammonia Test</button>
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
        enable_tests: true
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

      setPart: function(id, enabled) {
        this.$parent.$parent.publish("/mosfet", {
          'type': 'Mosfet',
          'id': id,
          'enable': enabled
        })
      }
    },

    components: {
      Checkbox
    }
  }
</script>
