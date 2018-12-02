<template>
  <div class="wrap2">
    <span v-if="pi_index >= 0" class="title">Current Camera: {{pi_index}}</span>
    <div class="buttons">
      <template v-for="i in 6">
        <button ref="cams" v-on:click="$emit('pi_index', i)"> <span>Camera {{i}}</span> </button>
        <div class="fixed-spacer"></div>
      </template>
    </div>

    <div class="options">
      <Checkbox v-bind:name="'Vertical Flip'" v-on:toggle="toggleVFlip()"/>
      <div class="fixed-spacer"></div>
      <Checkbox v-bind:name="'High Quality'" v-on:toggle="toggleHQ()"/>
      <div class="fixed-spacer"></div>
      <span>Shutter Speed:</span>
      <input type="number" v-model="sspeed">
      <div class="fixed-spacer"></div>
      <button v-on:click='sendSettings()'>Update</button>
    </div>
  </div>
</template>

<script>
  import Checkbox from "./Checkbox.vue"

  export default {
    data() {
      return {
        vflip: false,
        high_quality: false,
        sspeed: "10000"
      }
    },

    props: {
      pi_index: {
        type: Number,
        required: true
      }
    },

    components: {
      Checkbox
    },

    watch: {
      pi_index: function (newIdx) {
        for (let i = 1; i <= 6; i++) {
          this.$refs["cams"][i-1].disabled = (i == newIdx)
        }
      }
    },

    methods: {
      sendSettings: function() {
        this.$parent.$parent.publish("/pi_settings", {
          type: 'PiSettings',
          shutter_speed: parseInt(this.sspeed),
          vflip: this.vflip,
          height: this.high_quality ? 720 : 480,
          width: this.high_quality ? 1280 : 854
        });
      },

      toggleHQ: function () {
        this.high_quality = !this.high_quality
      },

      toggleVFlip: function () {
        this.vflip = !this.vflip
      }
    }
  }
</script>

<style>
  .wrap2 {
    display: grid;
    grid-gap: 10px;
    grid-template-columns: 1fr;
    grid-template-rows: 20px 20px 20px;
    grid-template-areas: "title" "buttons" "options";

    font-family: sans-serif;
    height: 100%;
  }

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
