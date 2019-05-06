<template>
  <div class="wrap2">
    <span v-if="pi_index >= 0" class="title">Current Camera: {{pi_index}}</span>
    <div class="buttons">
      <template v-for="i in 3">
        <button class="pi_buttons" ref="cams" v-on:click="$emit('pi_index', i-1)"> <span>{{cameras[i-1]}}</span> </button>
        <div class="fixed-spacer"></div>
      </template>
    </div>
    <div class="buttons">
      <template v-for="i in 3">
        <button class="pi_buttons" ref="cams" v-on:click="$emit('pi_index', i+2)"> <span>{{cameras[i+2]}}</span> </button>
        <div class="fixed-spacer"></div>
      </template>
    </div>
    <div class="buttons">
      <template v-for="i in 3">
        <button class="pi_buttons" ref="cams" v-on:click="$emit('pi_index', i+5)"> <span>{{cameras[i+5]}}</span> </button>
        <div class="fixed-spacer"></div>
      </template>
    </div>


    <div class="options">
      <button v-on:click="takePicture()"><span>Take Pic</span></button>
      <Checkbox v-bind:name="'Vertical Flip'" v-on:toggle="toggleVFlip()"/>
      <div class="fixed-spacer"></div>
      <input type="range" name="shutterslider" v-model="sspeed" min="8" max="16" step="1">
      <label for="shutterslider">Shutter Speed</label>
    </div>
  </div>
</template>

<script>
  import Checkbox from "./Checkbox.vue"
  import * as jsonconfig from "./../conf.json"

  export default {
    data() {
      return {
        vflip: false,
        sspeed: "12",
        cameras: jsonconfig["default"]["cameras"]
      }
    },

    props: {
      pi_index: {
        type: Number,
        required: true
        },
      dual_stream: {
        type: Boolean,
        required: true
      }
    },

    components: {
      Checkbox
    },

    watch: {
      pi_index: function (newIdx) {
        for (let i = 0; i <= 8; i++) {
          this.$refs["cams"][i].disabled = (i == newIdx)
        }
      },
      vflip: function(newFlip) {
        this.sendSettings()
      },
      sspeed: function(newSpeed) {
        this.sendSettings()
      }
    },

    methods: {
      sendSettings: function() {
        this.$parent.$parent.publish("/pi_settings", {
          type: 'PiSettings',
          pi_index: this.pi_index,
          shutter_speed: Math.round(Math.pow(10, parseInt(this.sspeed)/4)),
          vflip: this.vflip,
          height: 480,
          width: 854
        });
      },

      toggleVFlip: function () {
        this.vflip = !this.vflip
      },

      takePicture: function () {
        this.$parent.$parent.publish("/pi_picture", {
          type: 'PiPicture',
          index: this.pi_index
        })
      }
    }
  }
</script>

<style>
  /* .wrap2 {
    grid-gap: 10px;
    grid-template-columns: 1fr;
    grid-template-rows: 1fr 1fr 1fr 1fr 1fr;
    grid-template-areas: "title" "microscope" "cameras1" "cameras2" "options";

    font-family: sans-serif;
    height: 100%;
  } */

  /* .wrap2 {
    display: grid;
    grid-gap: 10px;
    grid-template-columns: 1fr;
    grid-template-rows: 20px 20px 20px;
    grid-template-areas: "title" "buttons" "options";

    font-family: sans-serif;
    height: 100%;
    padding: 10px 0px 10px 0px;
  } */

  .title {
    grid-area: title;
    margin: auto;
    width: auto;
  }

  .buttons {
    display: flex;
    align-items: center;
    justify-content: center;
    grid-area: buttons;
  }

  .pi_buttons {
    height:20px;
    width:100px;
  }

  .options {
    display: flex;
    align-items: center;
    justify-content: center;
    grid-area: options;
  }

  .fixed-spacer {
    width:10px;
    height:auto;
  }
</style>
