<template>
<div class = "main-content">
 <h4>Auton Commands</h4>
  <div class="joystick-wrap">
    <span>Left: {{AutonDriveControl.left_percent_velocity.toFixed(3)}}<br>Right: {{AutonDriveControl.right_percent_velocity.toFixed(3)}}</span>
  </div>
</div>
</template>

<script>
export default {
  props: {
    AutonDriveControl: {
      type: Object,
      required: true
    },
  }
}
</script>

<style scoped>

  .main-content{
    margin-top: 0.5rem;
    border-radius: 5px;
    padding: 10px;
    border: 1px solid black;
    align-content: center;
  }

  .main-content h4{
    margin: 0px 0px 5px 0px; 
    padding: 0px 0px 0px 0px;
    padding-left: 5px;
  }

  .joystick-wrap {
      padding: 0px;
      padding-left: 5px;
      padding-right: 5px;
      border: none;
  }

  .p{
    padding: 0px;
    margin: 0px;
  }

  .h4{
    padding: 0px;
    margin: 0px;
  }
</style>
