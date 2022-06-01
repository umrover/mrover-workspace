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
    padding: 5px;
    border: none;
    grid-template-rows: auto;
  }

  .wrap p {
    display: inline;
  }

  h5 {
    padding: 0;
    padding-top: 5px;
    margin: 0;
  }

  p {
    margin: 0;
    padding: 0;
  }

</style>