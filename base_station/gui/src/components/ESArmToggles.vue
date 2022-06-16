<template>
  <div class="wrap">
    <div>
      <ToggleButton id="laser" v-bind:currentState="laserActive" labelEnableText="Laser On" labelDisableText="Laser Off" v-on:change="toggleLaser()"/>
    </div>
    <div>
      <ToggleButton id="arm-slow" v-bind:currentState="armSlow" labelEnableText="Slowing On" labelDisableText="Slowing Off" v-on:change="toggleSlow()"/>
    </div>
  </div>
</template>

<script>
import ToggleButton from './ToggleButton.vue'

export default {
  data() {
    return {
      laserActive: false,
      armSlow: false
    }
  },

  methods: {
    toggleLaser: function() {
      this.laserActive = !this.laserActive

      const msg = {
        'type': 'MosfetCmd',
        'device': 3,
        'enable': this.laserActive
      }

      this.$parent.publish('/mosfet_cmd', msg)
    },

    toggleSlow: function() {
      this.armSlow = !this.armSlow

      const msg = {
        'type': 'Enable',
        'enabled': this.armSlow
      }

      this.$parent.publish('/arm_slow_mode', msg)
    }
  },

  components: {
    ToggleButton
  }
}
</script>

<style scoped>

.wrap {
  display: flex;
  gap: 10px;
}

</style>