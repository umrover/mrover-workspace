<template>
  <div class="wrap">
    <h3>Cameras</h3>
    <div class="input">
      Camera name: <input type='message' v-model ='cameraName'>
      Camera number: <input type='Number' min="0" max="9" v-model ='cameraIdx'>
      <button v-on:click="addCameraName()">Change name</button>
    </div>
    <div class="cameraselection">
      <CameraSelection class="cameraspace1" v-bind:camsEnabled="camsEnabled" v-bind:names="names" v-bind:numCams="numCams" v-on:cam_index="setCamIndex($event)"/>
    </div>
  </div>
</template>

<script>
  import CameraSelection from './CameraSelection.vue'
  import CommIndicator from './CommIndicator.vue'
  import Checkbox from "./Checkbox.vue"

  let interval;

  export default {
    data() {
      return {
        camsEnabled: [
          false,
          false,
          false,
          false,
          false,
          false,
          false,
          false,
          false,
        ],
        names: [
          "Camera 0",
          "Camera 1",
          "Camera 2",
          "Camera 3",
          "Camera 4",
          "Camera 5",
          "Camera 6",
          "Camera 7",
          "Camera 8"
        ],
        cameraIdx: 1,
        cameraName: ""
      }
    },

    beforeDestroy: function () {
      window.clearInterval(interval);
    },

    props: {
      numCams: {
        type: Number,
        required: true
      },
      channel: {
        type: String,
        required: true
      }
    },

    created: function () {
      // const JOYSTICK_CONFIG = {
      //   'down_left_button': 6,
      //   'up_left_button': 7,
      //   'down_middle_button': 8,
      //   'up_middle_button': 9,
      //   'down_right_button': 10,
      //   'up_right_button': 11
      // }

      // const CAMERA_NUM = {
      //   'down_left_button': 1,
      //   'up_left_button': 2,
      //   'down_middle_button': 3,
      //   'up_middle_button': 4,
      //   'down_right_button': 5,
      //   'up_right_button': 6
      // }

      // window.addEventListener('keydown', (e) => {
      //   const activeElement = document.activeElement;
      //   const inputs = ['input', 'select', 'textarea'];

      //   //Prevent camera change if inside text area
      //   if (activeElement && inputs.indexOf(activeElement.tagName.toLowerCase()) !== -1) {
      //       return;
      //   }

      //   if(e.keyCode>=49 && e.keyCode<=54)  //keys 1 to 6
      //     this.cam_index_1 = e.keyCode-48
      // })

      // // Change cam index based on joystick button
      // interval = window.setInterval(() => {
      //   const gamepads = navigator.getGamepads()
      //   for (let i = 0; i < 2; i++) {
      //     const gamepad = gamepads[i]
      //     if (gamepad) {
      //       if (gamepad.id.includes('Logitech')) {
      //         if (gamepad.buttons[JOYSTICK_CONFIG['down_left_button']]['pressed']) {
      //           this.cam_index_1 = CAMERA_NUM['down_left_button']
      //         } else if (gamepad.buttons[JOYSTICK_CONFIG['up_left_button']]['pressed']) {
      //           this.cam_index_1 = CAMERA_NUM['up_left_button']
      //         } else if (gamepad.buttons[JOYSTICK_CONFIG['down_middle_button']]['pressed']) {
      //           this.cam_index_1 = CAMERA_NUM['down_middle_button']
      //         } else if (gamepad.buttons[JOYSTICK_CONFIG['up_middle_button']]['pressed']) {
      //           this.cam_index_1 = CAMERA_NUM['up_middle_button']
      //         } else if (gamepad.buttons[JOYSTICK_CONFIG['down_right_button']]['pressed']) {
      //           this.cam_index_1 = CAMERA_NUM['down_right_button']
      //         } else if (gamepad.buttons[JOYSTICK_CONFIG['up_right_button']]['pressed']) {
      //           this.cam_index_1 = CAMERA_NUM['up_right_button']
      //         }
      //       }
      //     }
      //   }
      // }, 250)
    },

    methods: {
      setCamIndex: function (index) {
        this.camsEnabled.splice(index, 1, !this.camsEnabled[index])
        this.sendCameras();
      },

      sendCameras: function() {
        let ports = []

        for (let i = 0; i < this.camsEnabled.length; i++) {
          if (this.camsEnabled[i]) {
            ports.push(i)
          }
        }

        for (let i = ports.length; i < this.numCams; i++) {
          ports.push(-1)
        }

        this.$parent.publish(this.channel, {
          'type': 'GUICameras',
          'port_0': ports[0],
          'port_1': ports[1],
        })
      },

      addCameraName: function() {
        this.names.splice(this.cameraIdx, 1, this.cameraName)
      }
    },

    components: {
      CameraSelection,
      CommIndicator,
      Checkbox
    }
  }
</script>

<style>
  .wrap {
    display: grid;
    grid-gap: 10px;
    grid-template-columns: 1fr;
    grid-template-rows: 60px;
    grid-template-areas: "header";
    font-family: sans-serif;
    height: 100%;
  }

  .cameraselection {
    display: grid;
    grid-template-columns: 1fr 1fr;
    grid-template-areas: "cameraspace1"
  }

  .cam_buttons {
    height:20px;
    width:100px;
  }

  .box {
    border-radius: 5px;
    padding: 10px;
    border: 1px solid black;
  }

  img {
    border: none;
    border-radius: 0px;
  }

  .header {
    grid-area: header;
    display: flex;
    align-items: center;
  }

  .header h1 {
    margin-left: 5px;
  }

  .spacer {
    flex-grow: 0.8;
  }

  .comms {
    display: flex;
    flex-direction: column;
    align-items: flex-start;
  }

  .comms * {
    margin-top: 2px;
    margin-bottom: 2px;
    display: flex;
  }

  ul#vitals li {
    display: inline;
    padding: 0px 10px 0px 0px;
  }
</style>
