<template>
  <div class="wrap">
    <div>
    <h5>Obstacle Data:</h5>
    <p>Detected: {{this.Obstacle.detected}}<br/>Bearing: {{this.Obstacle.bearing}}<br/>Distance: {{this.Obstacle.distance}}</p>
    </div>
  </div>
</template>

<script>
export default {
  props: {
    Obstacle: {
      type: Object,
      required: true
    },

  },

  computed: {
    color: function () {
      return this.Obstacle.detected ? 'green' : 'red'
    }
  }
}
</script>

<style scoped>
   
  .wrap {
      padding: 0px;
      padding-left: 5px;
      padding-right: 5px;
      border: none;
      margin: -45px 0px 0px 0px;
      grid-template-rows: auto;
  }

  .wrap p {
    display: inline;
  }

</style>