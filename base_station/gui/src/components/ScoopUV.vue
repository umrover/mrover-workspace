<template>
<div class="wrap">
    <div>
      <h3> Scoop UV Bulb </h3>
    </div>

  <div :class="{'active': scoopActive}">
    <ToggleButton id="scoop_button" labelEnableText="Scoop UV On" labelDisableText="Scoop UV Off" v-on:change="UVshutdown()"/>
  </div>
    
  <div :class="{'active': shutdownActive}">
    <ToggleButton id="shutdown" :defaultState="true" labelEnableText="UV Auto shutoff On" labelDisableText="UV Auto shutoff Off" v-on:change="switchShutdown()"/>
  </div>

  <div :class="{'active': scoopLimitActive}">
    <ToggleButton id="scoop_limit_switch" :defaultState="true" labelEnableText="Scoop Limit Switch On" labelDisableText="Scoop Limit Switch Off" v-on:change="toggleLimit()"/>
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
      shutdownActive: true,
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
    toggleScoop: function () {
      this.scoopActive = !this.scoopActive
      this.setPart(this.mosfetIDs.uvBulb, this.scoopActive)
    },

    toggleLimit: function () {
      this.scoopLimitActive = !this.scoopLimitActive
      this.setLimit(this.scoopLimitActive)
    },
    
    setPart: function(id, enabled) {
      this.$parent.publish("/mosfet_cmd", {
        'type': 'MosfetCmd',
        'device': this.mosfetIDs.uv_bulb,
        'enable': val
      })
    },
    
    UVshutdown: function() {
      this.scoopActive = !this.scoopActive
      this.setPart(this.mosfetIDs.uv_bulb, this.scoopActive);
      if (this.shutdownActive === true) {
        clearTimeout(this.timeoutID);
        this.timeoutID = setTimeout(() => {
          this.setPart(this.mosfetIDs.uv_bulb, false);
        }, 120000) //2 minutes
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
