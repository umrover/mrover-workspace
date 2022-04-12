<template>
<div class="wrap">
    <div>
      <h3> Scoop UV Bulb </h3>
    </div>

  <div :class="{'active': scoopUVActive}">
    <ToggleButton id="scoop_button" :defaultState=false labelEnableText="Scoop UV On" labelDisableText="Scoop UV Off" v-on:change="toggleUVBulb()"/>
  </div>
    
  <div :class="{'active': shutdownActive}">
    <ToggleButton id="shutdown" :defaultState=true labelEnableText="UV Auto shutoff On" labelDisableText="UV Auto shutoff Off" v-on:change="switchShutdown()"/>
  </div>

  <div :class="{'active': scoopLimitActive}">
    <ToggleButton id="scoop_limit_switch" :defaultState=true labelEnableText="Limit Switch On" labelDisableText="Limit Switch Off" v-on:change="toggleLimit()"/>
  </div>
</div>
</template>

<script>
import ToggleButton from './ToggleButton.vue'

export default {
  data () {
    return {
      scoopUVActive: false,
      shutdownActive: true,
      timeoutID: 0,

      scoopLimitActive: true
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

  watch: {
    scoopUVActive() {
      this.$parent.publish("/mosfet_cmd", {
        'type': 'MosfetCmd',
        'device': this.mosfetIDs.uv_bulb,
        'enable': this.scoopUVActive
      })
    }
  },

  methods: {
    switchShutdown: function() {
      this.shutdownActive = !this.shutdownActive
    },

    toggleUVBulb: function() {
      this.scoopUVActive = !this.scoopUVActive

      if (this.scoopUVActive) {
        this.timeoutID = setTimeout(() => {
          if (this.scoopUVActive && this.shutdownActive) {
            this.scoopUVActive = false
          }
        }, 2 * 6000) // 2 minutes
      }
      else {
        clearTimeout(this.timeoutID)
      }
    },

    toggleLimit: function () {
      this.scoopLimitActive = !this.scoopLimitActive
      this.setLimit(this.scoopLimitActive)
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
