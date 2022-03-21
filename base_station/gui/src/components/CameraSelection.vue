<template>
  <div class="wrap2">
    <span v-if="cam_index > -1" class="title">Current Camera: {{cam_index}}</span>
    <div class="buttons">
      <template v-for="i in 3">
        <button class="cam_buttons" ref="cams" v-on:click="$emit('cam_index', i-2)"> <span>{{cameras[i-2]}}</span> </button>
        <div class="fixed-spacer"></div>
      </template>
    </div>
    <div class="buttons">
      <template v-for="i in 3">
        <button class="cam_buttons" ref="cams" v-on:click="$emit('cam_index', i+1)"> <span>{{cameras[i+1]}}</span> </button>
        <div class="fixed-spacer"></div>
      </template>
    </div>
    <div class="buttons">
      <template v-for="i in 3">
        <button class="cam_buttons" ref="cams" v-on:click="$emit('cam_index', i+4)"> <span>{{cameras[i+4]}}</span> </button>
        <div class="fixed-spacer"></div>
      </template>
    </div>
  </div>
</template>

<script>
  import * as jsonconfig from "./../conf.json"

  export default {
    data() {
      return {
        cameras: jsonconfig["default"]["cameras"]
      }
    },

    props: {
      cam_index: {
        type: Number,
        required: true
      }
    },

    watch: {
      cam_index: function (newIdx) {
        for (let i = 0; i <= 8; i++) {
          this.$refs["cams"][i].disabled = (i - 1 == newIdx)
        }
      }
    },
  }
</script>

<style>

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

  .cam_buttons {
    height:20px;
    width:100px;
  }

  .fixed-spacer {
    width:10px;
    height:auto;
  }
</style>
