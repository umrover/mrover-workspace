<template>
  <div class="wrap">
    <div>
      <SASiteControls v-bind:site="0"/>
    </div>
    <div>
      <SASiteControls v-bind:site="1"/>
    </div>
  </div>
</template>

<style scoped>
  .wrap {
    display: grid;
    grid-gap: 10px;
    grid-template-columns: 1fr 1fr;
    grid-template-rows: 1fr;
    grid-template-areas: "header" "servos";

    font-family: sans-serif;
    height: 100%;
  }

  .box {
    border-radius: 5px;
    padding: 10px;
    border: 1px solid black;
  }
</style>

<script>
  import SASiteControls from './SASiteControls.vue'
  
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

    },

    components: {
      SASiteControls
    }
  }
</script>
