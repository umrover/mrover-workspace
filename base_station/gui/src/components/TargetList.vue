<template>
  <div class="wrap">
    <div>
      <h5>Target 1</h5>
      <p>Bearing: {{this.TargetList[0].bearing}}<br/>;Distance: {{this.TargetList[0].distance}}<br/>;id: {{this.TargetList[0].id}}</p>
      <h5>Target 2</h5>
      <p>Bearing: {{this.TargetList[1].bearing}}<br/>;Distance: {{this.TargetList[1].distance}}<br/>;id: {{this.TargetList[1].id}}</p>
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
      padding: 0px;
      padding-left: 5px;
      padding-right: 5px;
      border: none;
      margin: 0px 0px 0px 0px;
      grid-template-rows: auto;
  }

  .wrap p {
    display: inline;
  }

</style>