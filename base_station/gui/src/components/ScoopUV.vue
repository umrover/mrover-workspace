<template>
<div class="wrap">
  <div>
    <h3> Scoop UV Bulb </h3>
  </div>

  <div :class="{'active': scoopActive}">
    <ToggleButton id="scoop_button" labelEnableText="Scoop UV On" labelDisableText="Scoop UV Off" v-on:change="UVShutdown()"/>
  </div>
    
  <div :class="{'active': shutdownActive}">
    <ToggleButton id="shutdown" labelEnableText="UV Auto shutoff On" labelDisableText="UV Auto shutoff Off" v-on:change="switchShutdown()"/>
  </div>

  <div :class="{'active': scoopLimitActive}">
    <ToggleButton id="scoop_limit_switch" :defaultState="true" labelEnableText="Limit Switch On" labelDisableText="Limit Switch Off" v-on:change="toggleLimit()"/>
  </div>
</div>  
</template>

<script>
import ToggleButton from './ToggleButton.vue'

export default {
  data () {
    return {
      scoopActive: false,
      scoopLimitActive: true,
      shutdown: true,
      uvBulb: false,
      timeoutID: true
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
    switchShutdown: function() {
      this.shutdown = !this.shutdown
    },
    
    toggleLimit: function () {
      this.scoopLimitActive = !this.scoopLimitActive
      this.setLimit(this.scoopLimitActive)
    },
    
    setPart: function(id, enabled) {
      this.$parent.publish("/mosfet_cmd", {
        'type': 'MosfetCmd',
        'device': id,
        'enable': enabled
      })
    },
    
    UVshutdown: function() {
      this.scoopActive = !this.scoopActive
      this.setPart(this.mosfetIDs.uv_bulb, this.scoopActive);
      if (this.shutdown === 1) {
        clearTimeout(this.timeoutID);
        this.timeoutID = setTimeout(() => {
          this.uv_bulb = 0;
          this.setPart(this.mosfetIDs.uv_bulb, false);
        }, 120000) //2 minutes
      }
      else {
        clearTimeout(this.timeoutID);
      }
    },
      
    setLimit: function(enabled) {
      this.$parent.publish("/scoop_limit_switch_enable_cmd", {
        'type': 'ScoopLimitSwitchEnable',
        'enable': enabled
      })
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
