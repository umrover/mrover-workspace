<template>
  <div class="wrap">
    <div>
    <p>Target 1:
        Bearing: {{this.TargetList[0].bearing}} &nbsp;Distance: {{this.TargetList[0].distance}}&nbsp; id: {{this.TargetList[0].id}}<br/>
        Target 2:
        Bearing: {{this.TargetList[1].bearing}}&nbsp; Distance: {{this.TargetList[1].distance}}&nbsp; id: {{this.TargetList[1].id}}</p>
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