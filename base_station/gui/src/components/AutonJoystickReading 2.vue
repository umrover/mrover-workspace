<template>
<div class = "main-content">
 <h3>Joystick Reading</h3>
  <div class="joystick-wrap">
    <span>forward_back: {{Joystick.forward_back}}<br>left_right: {{Joystick.left_right}}<br>dampen: {{Joystick.dampen}}</span>
  </div>
</div>
</template>

<script>
export default {
  props: {
    Joystick: {
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
  }

  .main-content h3{
    margin: 0px 0px 5px 0px; 
    padding: 0px 0px 0px 0px;
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

  .h3{
    padding: 0px;
    margin: 0px;
  }
</style>
