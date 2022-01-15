<template>
    <div class = "wrap">
        <p>Zed Gimbal Angle: {{angle}}°</p>
    </div>
</template>

<script>
export default {
  data () {
    return {
      angle: 0
    }
  },
  created: function () {
    this.$parent.subscribe('/zed_gimbal_data', (msg) => {
      this.angle = msg.angle;
    })
  }
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
