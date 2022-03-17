<template>
  <div class="wrap">
    
    <h3 class="centered" >Signal Strength</h3>
    <h3 class="centered"> {{signal_strength}} </h3>

    <br/>

    <button v-on:click="send_setup_message" :disabled="setup_button_clicked"> radio setup </button>
    
  </div>
</template>

<script>
import LCMBridge from 'lcm_bridge_client/dist/bridge.js';

export default {
  data () {
    return {
      signal_strength: " - ",
      setup_button_clicked: false
    }
  },

  methods: {
    send_setup_message: function(event) {
      this.setup_button_clicked = true

      // Diable and reenable 40s later. Prevents double-click
      // Note that radio setup causes the radio to reboot
      // This also prevents a radio setup call while radio is rebooting
      this.timeout = setTimeout(() => {
        this.setup_button_clicked = false
      }, 40000)
      
      var msg = {type: "Signal"};

      this.$parent.lcm_.publish('/radio_setup', msg)
    }
  },

  created: function () {
    this.$parent.subscribe('/radio_update', (msg) => {
      console.log("Radio update receieved")

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
