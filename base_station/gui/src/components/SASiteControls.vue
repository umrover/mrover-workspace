<template>
  <div class="wrap2 box">
    <div>
      <h4>{{ site }}</h4>
    </div>
    <div class="horizontal-buttons">
      <div id="toReplace">
        <div :is="currentComponent"></div>
        <div v-show="!currentComponent" v-for="component in componentsArray">
          <button @click="swapComponent(component)">{{component}} Test</button>
        </div>
      </div>
      <button @click="swapComponent(null)">Close</button>
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
        degrees: 0.0,
        currentComponent: null,
        componentsArray: ['Chlorophyll', 'Ammonia', 'Amino']
      }
    },

    props : {
      site: {
        type: String,
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
      },
      swapComponent: function(component)
      {
        this.currentComponent = component;
      }
    },

    components: {
      Checkbox,
      'Chlorophyll': {
        template: '<div><p>Turn off White LEDs</p><button @click="swapComponent(\'WhiteLEDsOff\')">Execute</button></div>',
        methods: {
          swapComponent: function(component)
          {
            this.$parent.$parent.$parent.publish("/mosfet_cmd", {
              'type': 'MosfetCmd',
              'device': 2,
              'enable': false
            })
            this.$emit("click", this.$parent.swapComponent(component))
          }
        }
      },
      'WhiteLEDsOff': {
        template: '<div><p>Turn on UV LED</p><button @click="swapComponent(\'UVLEDOn\')">Execute</button></div>',
        methods: {
          swapComponent: function(component)
          {
            this.$parent.$parent.$parent.publish("/mosfet_cmd", {
              'type': 'MosfetCmd',
              'device': 1,
              'enable': true
            })
            this.$emit("click", this.$parent.swapComponent(component))
          }
        }
      },
      'UVLEDOn': {
        template: '<div><p>Turn off UV LED</p><button @click="swapComponent(\'UVLEDOff\')">Execute</button></div>',
        methods: {
          swapComponent: function(component)
          {
            this.$parent.$parent.$parent.publish("/mosfet_cmd", {
              'type': 'MosfetCmd',
              'device': 1,
              'enable': true
            })
            this.$emit("click", this.$parent.swapComponent(component))
          }
        }
      },
      'UVLEDOff': {
        template: '<div><p>Turn on White LEDs</p><button @click="swapComponent(\'WhiteLEDsOn\')">Execute</button></div>',
        methods: {
          swapComponent: function(component)
          {
            this.$parent.$parent.$parent.publish("/mosfet_cmd", {
              'type': 'MosfetCmd',
              'device': 2,
              'enable': false
            })
            this.$emit("click", this.$parent.swapComponent(component))
          }
        }
      },
      'WhiteLEDsOn': {
        template: '<div><p>Finish</p><button @click="swapComponent(null)">Execute</button></div>',
        methods: {
          swapComponent: function(component)
          {
            this.$emit("click", this.$parent.swapComponent(component))
          }
        }
      },
      'Ammonia': {
        template: '<div><p>Run Ammonia Servo Script</p><button @click="swapComponent(null)">Execute</button></div>',
        methods: {
           swapComponent: function(component) {
             this.$parent.$parent.$parent.publish('/servo_cmd', {
                'type': 'ServoCmd',
                'id': 'ammonia_' + this.$parent.site,
                'degree': 0
             }),
             setTimeout(() => {
               this.$parent.$parent.$parent.publish('/servo_cmd', {
                 'type': 'ServoCmd',
                 'id': 'ammonia_' + this.$parent.site,
                 'position': 90
               })
               }, 5000);
            }
        }
      },
      'Amino': {
        template: '<div><p>Start Amino Test</p><button @click="swapComponent(\'StartAmino\')">Execute</button></div>',
        methods: {
          swapComponent: function(component) {
            this.$parent.$parent.$parent.publish("/servo_cmd", {
              'type': 'ServoCmd',
              'id': 'amino_' + this.$parent.site,
              'degree': 90
            })
            var device = 0;
            if (this.$parent.site == 'White') {
              device = 0
            } else if (this.$parent.site == 'Blue') {
              device = 3
            } else if (this.$parent.site == 'Yellow') {
              device = 4
            }
            this.$parent.$parent.$parent.publish("/mosfet_cmd", {
              'type': 'MosfetCmd',
              'device': device,
              'enable': true
            })
            this.$parent.$parent.$parent.publish("/mosfet_cmd", {
              'type': 'MosfetCmd',
              'device': device,
              'enable': true
            })
            this.$emit("click", this.$parent.swapComponent(component))
          }
        }
      },
      'StartAmino': {

      }
    }
  }
</script>
