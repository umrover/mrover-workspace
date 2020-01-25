<template>
  <div class="wrap">
    
    <h3 class="centered" >Signal Strength</h3>
    <h3 class="centered"> {{signal_strength}} </h3>
    
  </div>
</template>

<script>
import LCMBridge from 'lcm_bridge_client/dist/bridge.js';

export default {
  data () {
    return {
      signal_strength: " - "
    }
  },

  created: function () {
    this.$parent.subscribe('/radio', (msg) => {

      this.signal_strength = msg.signal_strength
    })
  }
}
</script>

<style scoped>

.wrap {
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  text-align: center;
}

</style>
