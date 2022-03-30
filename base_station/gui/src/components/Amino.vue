<template>
<div class="wrap">
  <div>
    <h3>Amino Test Controls</h3>
  </div>
  <div class="box1">
    <label for="site">Choose a site:</label>
    <select v-model = "site" name="site" id="site">
      <option value="0">Site 0</option>
      <option value="1">Site 1</option>
      <option value="2">Site 2</option>
    </select>
  </div>
  <div class="box1" :class="{'active': heaters[site].enabled}">
    <ToggleButton id="heater_toggle" :labelEnableText="'Heater '+(site)+' '+(heaters[site].message)" :labelDisableText="'Heater '+(site)+' '+(heaters[site].message)" v-on:change="toggleHeater(site)"/>
    <p v-bind:style="{color: heaters[site].color}">Thermistor {{site}}: {{heaters[site].temp.toFixed(2)}} C°</p>
  </div>
  <!-- <div class="box1" v-if="site == 0">
    <label for="toggle_button" :class="{'active': heaters[0].enabled}" class="toggle__button">
      <span class="toggle__label" >Heater 0 {{heaters[0].message}}</span>
      <input type="checkbox" id="toggle_button">
      <span class="toggle__switch" v-on:click="sendHeaterCmd(0, !heaters[0].intended)"></span>
    </label>
    <p v-bind:style="{color: heaters[0].color}">Thermistor 0: {{heaters[0].temp.toFixed(2)}} C°</p>
  </div>
  <div class="box1" v-if="site == 1">
    <label for="toggle_button" :class="{'active': heaters[1].enabled}" class="toggle__button">
      <span class="toggle__label" >Heater 1 {{heaters[1].message}}</span>
      <input type="checkbox" id="toggle_button">
      <span class="toggle__switch" v-on:click="sendHeaterCmd(1, !heaters[1].intended)"></span>
    </label>
    <p v-bind:style="{color: heaters[1].color}">Thermistor 1: {{heaters[1].temp.toFixed(2)}} C°</p>
  </div>
  <div class="box1" v-if="site == 2">
    <label for="toggle_button" :class="{'active': heaters[2].enabled}" class="toggle__button">
      <span class="toggle__label" >Heater 2 {{heaters[2].message}}</span>
      <input type="checkbox" id="toggle_button">
      <span class="toggle__switch" v-on:click="sendHeaterCmd(2, !heaters[2].intended)"></span>
    </label>
    <p v-bind:style="{color: heaters[2].color}">Thermistor 2: {{heaters[2].temp.toFixed(2)}} C°</p>
  </div> -->
  <div>
    <label for="toggle_button" :class="{'active': autoShutdownEnabled}" class="toggle__button">
      <span class="toggle__label" >Auto Shutdown {{autoShutdownMessage}}</span>
      <input type="checkbox" id="toggle_button">
      <span class="toggle__switch" v-on:click="sendAutoShutdownCmd(!autoShutdownIntended)"></span>
    </label>
  </div>
</div>  
</template>

<script>
import ToggleButton from './ToggleButton.vue'

export default {
  data () {
    return {
      site: 0,

      heaters: [
        {
          enabled: false,
          intended: false,
          message: 'Off',
          temp: 0,
          color: 'grey'
        },
        {
          enabled: false,
          intended: false,
          message: 'Off',
          temp: 0,
          color: 'grey'
        },
        {
          enabled: false,
          intended: false,
          message: 'Off',
          temp: 0,
          color: 'grey'
        }
      ],

      autoShutdownEnabled: true,
      autoShutdownIntended: true,
      autoShutdownMessage: 'On'
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
      this.heaters[msg.device].intended = msg.enabled

      let enabled = 'Off'
      if (msg.enabled == 1) {
        enabled = 'On'
      }
      this.heaters[msg.device].message = enabled
    })

    this.$parent.subscribe('/heater_auto_shutdown_data', (msg) => {
      this.autoShutdownEnabled = msg.auto_shutdown_enabled
      this.autoShutdownIntended = msg.auto_shutdown_enabled

      let enabled = 'Off'
      if (msg.auto_shutdown_enabled == 1) {
        enabled = 'On'
      }
      this.autoShutdownMessage = enabled
    })
  },

  components: {
    ToggleButton
  },

  methods: {
    toggleHeater: function(id) {
      if (this.heaters[id].enabled) {
        this.sendHeaterCmd(id, false)
      } else {
        this.sendHeaterCmd(id, true)
      }
    },
    
    sendHeaterCmd: function(id, enabled) {
      this.heaters[id].intended = enabled

      if (enabled == this.heaters[id].enabled) {
        if (enabled) {
          this.heaters[id].message = 'On'
        }
        else {
          this.heaters[id].message = 'Off'
        }
      }
      else {
        this.heaters[id].message = '...'
      }

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
    display: inline-block;
    vertical-align: top;
    align-content: left;
  }
  .box1 {
    text-align: left;
    vertical-align: top;
    display: inline-block;
  }

  .toggle__button {
      vertical-align: middle;
      user-select: none;
      cursor: pointer;
  }
  .toggle__button input[type="checkbox"] {
      opacity: 0;
      position: absolute;
      width: 1px;
      height: 1px;
  }
  .toggle__button .toggle__switch {
      display:inline-block;
      height:12px;
      border-radius:6px;
      width:40px;
      background: #BFCBD9;
      box-shadow: inset 0 0 1px #BFCBD9;
      position:relative;
      margin-left: 10px;
      transition: all .25s;
  }

  .toggle__button .toggle__switch::after, 
  .toggle__button .toggle__switch::before {
      content: "";
      position: absolute;
      display: block;
      height: 18px;
      width: 18px;
      border-radius: 50%;
      left: 0;
      top: -3px;
      transform: translateX(0);
      transition: all .25s cubic-bezier(.5, -.6, .5, 1.6);
  }

  .toggle__button .toggle__switch::after {
      background: #4D4D4D;
      box-shadow: 0 0 1px #666;
  }
  .toggle__button .toggle__switch::before {
      background: #4D4D4D;
      box-shadow: 0 0 0 3px rgba(0,0,0,0.1);
      opacity:0;
  }
  .active .toggle__switch {
    background: #FFEA9B;
    box-shadow: inset 0 0 1px #FFEA9B;
  }
  .active .toggle__switch::after,
  .active .toggle__switch::before{
      transform:translateX(40px - 18px);
  }
  .active .toggle__switch::after {
      left: 23px;
      background: #FFCB05;
      box-shadow: 0 0 1px #FFCB05;
  }
</style>
