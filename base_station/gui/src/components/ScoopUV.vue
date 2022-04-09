<template>
<div class="wrap">
    <div>
      <h3> Scoop UV Bulb </h3>
    </div>

    <div :class="{'active': scoopActive}">
      <ToggleButton id="scoop_button" labelEnableText="Scoop UV On" labelDisableText="Scoop UV Off" v-on:change="toggleScoop()"/>
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
        'device': id,
        'enable': enabled
      })
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
