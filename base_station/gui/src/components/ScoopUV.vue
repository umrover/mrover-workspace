<template>
<div class="wrap">
    <div>
      <h3> Scoop UV Bulb </h3>
    </div>
    <label for="toggle_button" :class="{'active': scoopUV == 1}" class="toggle__button">
        <span v-if="scoopUV == 1" class="toggle__label" >Scoop UV On</span>
        <span v-if="scoopUV == 0" class="toggle__label" >Scoop UV Off</span>

        <input type="checkbox" id="toggle_button">
        <span class="toggle__switch" v-if="scoopUV == 0" v-on:click="scoopUV = 1, setPart(mosfetIDs.uvBulb, true)"></span>
        <span class="toggle__switch" v-if="scoopUV == 1" v-on:click="scoopUV = 0, setPart(mosfetIDs.uvBulb, false)"></span>
    </label>

    <label for="toggle_button" :class="{'active': scoopLimit == 1}" class="toggle__button">
        <span v-if="scoopLimit == 1" class="toggle__label" >Limit Switch On</span>
        <span v-if="scoopLimit == 0" class="toggle__label" >Limit Switch Off</span>

        <input type="checkbox" id="toggle_button">
        <span class="toggle__switch" v-if="scoopLimit == 0" v-on:click="scoopLimit = 1, setLimit(true)"></span>
        <span class="toggle__switch" v-if="scoopLimit == 1" v-on:click="scoopLimit = 0, setLimit(false)"></span>
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