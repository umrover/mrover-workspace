<template>
  <div class="odom-wrap">
    <h3> Raman laser </h3>
    <div class="sacontrols buttons">
      <div :class="{'active': ramanActive}">
        <ToggleButton id="raman_button" labelEnableText="Raman Laser On" labelDisableText="Raman Laser Off" v-on:change="toggleRaman()"/>
      </div>
    </div>
  </div>
</template>

<script>
import ToggleButton from './ToggleButton.vue'

export default {
  data () {
    return {
      ramanActive: false
    }
  },

  components: {
    ToggleButton
  },

  props: {
    mosfetIDs: {
      type: Object,
      required: true
    },
  },

    methods: {
      toggleRaman: function () {
        this.ramanActive = !this.ramanActive
        this.setPart(this.mosfetIDs.raman_laser, this.ramanActive)
      },
      
      setPart: function(id, enabled) {
        this.$parent.publish("/mosfet_cmd", {
          'type': 'MosfetCmd',
          'device': id,
          'enable': enabled
        })
      }
    }
}
</script>

<style scoped>
  .odom-wrap {
    display: inline-block;
    align-items: center;
    justify-items: center;
  }

  .odom {
    grid-area: odom;
  }

  .buttons {
    grid-area: buttons;
  }

  .odom-wrap p {
    display: inline;
  }
</style>
