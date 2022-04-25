<template>
<div class="wrap">
  <div class="title">
    <h3>Amino Test Controls</h3>
  </div>
  <div class="box1 select">
    <label for="site">Choose a site:</label>
    <select v-model="site" name="site" id="site">
      <option value=0>Site 0</option>
      <option value=1>Site 1</option>
      <option value=2>Site 2</option>
    </select>
  </div>
  <div class="box1 heaters">
    <ToggleButton id="heater" v-bind:currentState="heaters[site].intended" :labelEnableText="'Heater '+(site)+' Intended'" :labelDisableText="'Heater '+(site)+' Intended'" v-on:change="toggleHeater(site)"/>
    <p v-bind:style="{color: heaters[site].color}">Thermistor {{site}}: {{heaters[site].temp.toFixed(2)}} C°</p>
  </div>
  <div class="comms heaterStatus">
    <CommIndicator v-bind:connected="heaters[site].enabled" v-bind:name="'Heater '+(site)+' Status'"/>
  </div>
  <div class="box1 shutdown">
    <ToggleButton id="autoshutdown" v-bind:currentState="autoShutdownIntended" :labelEnableText="'Auto Shutdown Intended'" :labelDisableText="'Auto Shutdown Intended'" v-on:change="sendAutoShutdownCmd(!autoShutdownIntended)"/>
  </div>
  <div class="comms shutdownStatus">
    <CommIndicator v-bind:connected="autoShutdownEnabled" v-bind:name="'Auto Shutdown Status'"/>
  </div>
</div>  
</template>

<script>
import ToggleButton from './ToggleButton.vue'
import CommIndicator from './CommIndicator.vue'

export default {
  data () {
    return {
      site: 0,

      heaters: [
        {
          enabled: false,
          intended: false,
          temp: 0,
          color: 'grey'
        },
        {
          enabled: false,
          intended: false,
          temp: 0,
          color: 'grey'
        },
        {
          enabled: false,
          intended: false,
          temp: 0,
          color: 'grey'
        }
      ],

      autoShutdownEnabled: true,
      autoShutdownIntended: true
    }
  },

  created: function () {
    this.$parent.subscribe('/thermistor_data', (msg) => {
      
      // For some reason, splice has to be used to reactively update array
      this.heaters[0].temp = msg.temp0
      this.heaters[1].temp = msg.temp1
      this.heaters[2].temp = msg.temp2

      for (let i = 0; i < this.heaters.length; i++) {
        if (this.heaters[i].temp > 100) {
          this.heaters[i].color = 'red'
        }
        else {
          this.heaters[i].color = 'black'
        }
      }
    })

    this.$parent.subscribe('/heater_state_data', (msg) => {
      this.heaters[msg.device].enabled = msg.enabled
    })

    this.$parent.subscribe('/heater_auto_shutdown_data', (msg) => {
      this.autoShutdownEnabled = msg.auto_shutdown_enabled
    })
  },

  components: {
    ToggleButton,
    CommIndicator
  },

  methods: {
    toggleHeater: function(id) {
      this.sendHeaterCmd(parseInt(id), !this.heaters[id].intended);
    },

    sendHeaterCmd: function(id, enabled) {
      this.heaters[id].intended = enabled

      this.$parent.publish('/heater_cmd', {
        'type': 'Heater',
        'device': id,
        'enabled': enabled
      })
    },

    sendAutoShutdownCmd: function(enabled) {
      this.autoShutdownIntended = enabled

      if (enabled == this.autoShutdownEnabled) {
        if (enabled) {
          this.autoShutdownMessage = 'On'
        }
        else {
          this.autoShutdownMessage = 'Off'
        }
      }
      else {
        this.autoShutdownMessage = '...'
      }

      this.$parent.publish('/heater_auto_shutdown_cmd', {
        'type': 'HeaterAutoShutdown',
        'auto_shutdown_enabled': enabled
      })
    }
  }
}
</script>

<style scoped>
  .wrap {
    display: grid;
    grid-gap: 5px;
    grid-template-columns: auto auto;
    grid-template-rows: auto auto auto auto;
    grid-template-areas:  "title ."
                          "select select"
                          "heaters heaterStatus"
                          "shutdown shutdownStatus";
    height: auto;
  }
  .title {
    grid-area: title;
    text-align: left;
  }
  .select {
    grid-area: select;
  }
  .heaters {
    grid-area: heaters;
  }
  .heaterStatus {
    grid-area: heaterStatus;
  }
  .shutdown {
    grid-area: shutdown;
  }
  .shutdownStatus {
    grid-area: shutdownStatus;
  }

  .box1 {
    text-align: left;
    vertical-align: top;
    display: inline-block;
  }
  .comms {
        display: flex;
        flex-direction: column;
        align-items: flex-start;
    }

        .comms * {
            margin-top: 2px;
            margin-bottom: 2px;
        }
</style>
