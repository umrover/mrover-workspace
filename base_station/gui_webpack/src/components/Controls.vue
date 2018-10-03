<template>
  <div class="wrap">
    <div class="control_buttons">
        <Checkbox ref="arm" v-bind:name="'Arm Controls'" v-on:toggle="updateControlMode('arm', $event)"/>
        <Checkbox ref="soil_ac" v-bind:name="'Soil Acquisition'" v-on:toggle="updateControlMode('soil_ac', $event)"/>
    </div>
    <div class="speed_limiter">
      <p>
        Speed Limiter:
        <span>{{ dampenDisplay }}%</span>
      </p>
    </div>
    <div class="mode_info">
      <div v-if="controlMode === 'arm'">
        <p>arm</p>
      </div>
      <template v-else-if="controlMode === 'soil_ac'">
          <span v-bind:class="['led', saDrillColor]"/>
          <span style="padding-right: 10px;" class="name">Drill</span>
          <span v-bind:class="['led', saDoorColor]"/>
          <span class="name">Door</span>
      </template>
    </div>
  </div>
</template>

<script>
import Checkbox from './Checkbox.vue'
import { mapGetters, mapMutations } from 'vuex'

export default {
  props: {
    dampen: {
      type: Number,
      required: true
    },

    saMotor: {
      type: Object,
      required: true
    }
  },

  computed: {
    saDrillColor: function () {
      return this.saMotor.drill !== 0 ? 'lightgreen' : 'red'
    },

    saDoorColor: function () {
      if (this.saMotor.door_actuator > 0) {
        return 'lightgreen'
      } else if (this.saMotor.door_actuator < 0) {
        return 'red'
      } else {
        return 'orange'
      }
    },

    dampenDisplay: function () {
      return (this.dampen * -50 + 50).toFixed(2)
    },

    ...mapGetters('controls', {
      controlMode: 'controlMode'
    }),
  },

  methods: {
    updateControlMode: function (mode, checked) {
      if (checked) {
        if (this.controlMode !== ''){
          this.$refs[this.controlMode].toggle()
        }

        this.setControlMode(mode)
      } else {
        this.setControlMode('')
      }
    },

    ...mapMutations('controls', {
      setControlMode: 'setControlMode'
    })
  },

  components: {
    Checkbox
  }
}
</script>

<style scoped>
.wrap {
  display: grid;
  grid-template-areas: "control_buttons speed_limiter mode_info";
  grid-template-columns: 1fr 1fr 1fr;
  align-items: center;
  height: 100%;
  padding: 0px 0px 0px 20px;
}

.control_buttons {
  grid-area: control_buttons;
  grid-template-areas: none;
  display: flex;
  padding: 0px 0px 0px 0px;
  height: 100%;
}

.speed_limiter {
  display: flex;
  grid-area: speed_limiter;
  align-items: center;
  justify-content: center;
}

.mode_info {
  grid-area: mode_info;
  display: flex;
}

.led {
  width: 16px;
  height: 16px;
  border-radius: 8px;
  border: 1px solid;
}

.lightgreen {
  background-color: lightgreen;
}

.red {
  background-color: red;
}

.orange {
  background-color: orange;
}

.name {
  margin-left: 5px;
}
</style>
