<template>
  <div class="wrap">
    <div>
    <p><b>Obstacle Data: </b>&nbsp;
      Detected: {{this.Obstacle.detected}}&nbsp;
      Bearing: {{this.Obstacle.bearing}}&nbsp;
      Distance: {{this.Obstacle.distance}}</p>
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
    padding: 2px;
    border: none;
    grid-template-rows: auto;
  }

  .wrap p {
    display: inline;
  }

  h5 {
    padding: 0;
    padding-top: 2px;
    margin: 0;
  }

  p {
    margin: 0;
    padding: 0;
  }

</style>
