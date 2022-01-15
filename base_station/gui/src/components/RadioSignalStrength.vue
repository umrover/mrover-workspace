<template>
  <div class="wrap">
    <p>Radio Signal Strength: {{this.RadioSignalStrength.signal_strength}}%</p>
   <!--button v-on:click="send_setup_message" :disabled="setup_button_clicked"> radio setup </button-->
  </div>
</template>

<script>
import LCMBridge from 'lcm_bridge_client/dist/bridge.js';

export default {
  props: {
      RadioSignalStrength: {
      type: Object,
      required: true
    }
  },

  /*methods: {
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
  }*/
  }
</script>

<style scoped>

  .wrap {
      padding: 0px;
      padding-left: 5px;
      padding-right: 5px;
      border: none;
      margin: -5px 0px 0px 0px;
  }

  .wrap p {
    display: inline;
  }

  .p {
    margin: 0;
    padding: 0;
  }

</style>
