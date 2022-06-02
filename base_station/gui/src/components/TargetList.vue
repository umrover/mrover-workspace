<template>
  <div class="wrap">
    <div>
      <p><b>Target 1: </b>&nbsp;
        Bearing: {{ this.TargetList[0].bearing.toFixed(2) }}&nbsp;
        Distance: {{ this.TargetList[0].distance.toFixed(2) }}&nbsp;
        id: {{ this.TargetList[0].id.toFixed(0) }}&nbsp;
        <b>Target 2: </b>&nbsp;
        Bearing: {{ this.TargetList[1].bearing.toFixed(2) }}&nbsp;
        Distance: {{ this.TargetList[1].distance.toFixed(2) }}&nbsp;
        id: {{ this.TargetList[1].id.toFixed(0) }}</p>
    </div>
  </div>
</template>

<script>
export default {
  props: {
    TargetList: {
      type: Array,
      required: true
    },

  },
}
</script>

<style scoped>
   
  .wrap {
    padding: 2px;
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