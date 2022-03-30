<template>
<div class="wrap">
  <div>
    <h3> Scoop UV Bulb </h3>
  </div>
  <label for="toggle_button" :class="{'active': uv_bulb == 1}" class="toggle__button">
      <span v-if="uv_bulb == 1" class="toggle__label" >Scoop UV On</span>
      <span v-if="uv_bulb == 0" class="toggle__label" >Scoop UV Off</span>

      <input type="checkbox" id="toggle_button">
      <span class="toggle__switch" v-on:click="toggleUVBulb()"></span>
  </label>
  <div class="shutdown">
    <label for="toggle_button" :class="{'active': shutdown == 1}" class="toggle__button">
      <span v-if="shutdown == 1" class="toggle__label" >UV Auto shutoff On</span>
      <span v-if="shutdown == 0" class="toggle__label" >UV Auto shutoff Off</span>
      
      <input type="checkbox" id="toggle_button">
      <span class="toggle__switch" v-on:click="toggleShutdown()"></span>
    </label>

    <label for="toggle_button" :class="{'active': scoopLimit == 1}" class="toggle__button">
        <span v-if="scoopLimit == 1" class="toggle__label" >Limit Switch On</span>
        <span v-if="scoopLimit == 0" class="toggle__label" >Limit Switch Off</span>

        <input type="checkbox" id="toggle_button">
        <span class="toggle__switch" v-on:click="toggleLimit()"></span>
    </label>

    <div :class="{'active': scoopActive}">
      <ToggleButton id="scoop_button" labelEnableText="Scoop UV On" labelDisableText="Scoop UV Off" v-on:change="toggleScoop()"/>
    </div>
</div>  
</template>

<script>
import ToggleButton from './ToggleButton.vue'

export default {
  data () {
    return {
      scoopUV: 0,
      scoopLimit: 1,
      scoopActive: false
    }
  },

  components: {
    ToggleButton
  },

  props: {
    mosfetIDs: {
      type: Object,
      required: true
    }
  },  
  methods: {
    toggleScoop: function () {
      this.scoopActive = !this.scoopActive
      this.setPart(this.mosfetIDs.scoopUV, this.scoopActive)
    },
    
    setPart: function(id, enabled) {
      this.$parent.publish("/mosfet_cmd", {
        'type': 'MosfetCmd',
        'device': this.mosfetIDs.uv_bulb,
        'enable': val
      })
    },

    scoop_limit(val) {
      this.$parent.publish("/scoop_limit_switch_enable_cmd", {
        'type': 'ScoopLimitSwitchEnable',
        'enable': val
      })
    }
  },

  methods: {
    toggleUVBulb: function() {
      this.uv_bulb = !this.uv_bulb

      if (this.uv_bulb) {
        this.timeoutID = setTimeout(() => {
          if (this.uv_bulb && this.shutdown) {
            this.uv_bulb = false
          }
        }, 2 * 60000) // 2 minutes
      }
      else {
        clearTimeout(this.timeoutID)
      }
    },
    
    toggleShutdown: function() {
      this.shutdown = !this.shutdown
    },
      
    toggleLimit: function() {
      this.scoop_limit = !this.scoop_limit
    }
  }
}
</script>


<style scoped>
  .wrap {
    display: inline-block;
    align-content: center;
  }
</style>