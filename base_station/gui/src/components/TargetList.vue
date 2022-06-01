<template>
  <div class="wrap">
    <div>
      <h5>Target 1</h5>
      <p>Bearing: {{this.TargetList[0].bearing}}<br/>Distance: {{this.TargetList[0].distance}}<br/>id: {{this.TargetList[0].id}}</p>
      <h5>Target 2</h5>
      <p>Bearing: {{this.TargetList[1].bearing}}<br/>Distance: {{this.TargetList[1].distance}}<br/>id: {{this.TargetList[1].id}}</p>
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
    padding: 5px;
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