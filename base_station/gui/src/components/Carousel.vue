<template>
  <div class="carousel-wrap">
    <div>
        <h3>Carousel Controls</h3>
    </div>
    <div class="box1">
      <div v-if="calibrated">
        <label for="position">Rotate carousel to position:</label>
        <button v-on:click="targetSite = 0"> Position 0 </button>
        <button v-on:click="targetSite = 1"> Position 1 </button>
        <button v-on:click="targetSite = 2"> Position 2 </button>
      </div>
      <div v-else>
        <p style="color: 'red'"><b>Not Calibrated</b></p>
        <div v-if="calibrating">
          <button v-on:click="cancelCalibration()"> Cancel </button>
        </div>
        <div v-else>
          <button v-on:click="calibrate()"> Calibrate </button>
        </div>
      </div>
      <br>
      <div class="commands">
        <button v-on:click="zero()"> Zero encoder </button>
        <p>Current mode: {{this.closedLoop ? 'Closed Loop' : 'Open Loop'}}</p>
        <p>Motor position: {{this.positionDegrees}}</p>
        <div v-if="atSite">(At site {{site}})</div>
      </div>
      <br>
      <label for="openloop">Openloop command:</label>
      <input type="text" name="openloop" @keydown="keydown" @keyup="keyup"/>
    </div>
  </div>
</template>

<script>

const sitePositions = [
  -Math.PI,
  Math.PI / 3,
  -Math.PI / 3
]

export default {
  data () {
    return {
      calibrated: false,
      calibrating: false,

      targetSite: -1,
      position: 0,

      atSite: false,
      site: -1,

      closedLoop: true,
      openLoopSpeed: 0.0,
    }
  },

  created: function () {
    this.$parent.subscribe('/carousel_calib_data', (msg) => {
      // If we are now calibrated after not being calibrated, stop moving
      if (!this.calibrated && msg.calibrated) {
        this.openLoopSpeed = 0.0
        this.calibrating = false
      }

      this.calibrated = msg.calibrated
    })

    this.$parent.subscribe('/carousel_pos_data', (msg) => {
      this.position = msg.position

      let new_site = false
      for (let i = 0; i < 3; i++) {
        if (Math.abs(this.position - sitePositions[i]) < 0.1) {
          new_site = true
          this.site = i
        }
      }
      this.atSite = new_site
    })

    window.setInterval(() => {
      if (!this.closedLoop) {
        this.$parent.publish('/carousel_open_loop_cmd', {
          'type': 'CarouselOpenLoopCmd',
          'throttle': this.openLoopSpeed
        })
      }
    }, 100)
  },

  computed: {
    positionDegrees() {
      return this.position * 180 / Math.PI
    }
  },

  methods: {
    publishClosedLoop: function() {
      this.$parent.publish('/carousel_closed_loop_cmd', {
        'type': 'CarouselPosition',
        'position': sitePositions[this.targetSite]
      })
    },

    keydown: function(event) {
      if (event.key === 'ArrowLeft') {
        this.closedLoop = false
        this.targetSite = -1

        this.openLoopSpeed = -1.0
      }
      else if (event.key === 'ArrowRight') {
        this.closedLoop = false
        this.targetSite = -1

        this.openLoopSpeed = 1.0
      }
    },

    keyup: function(event) {
      if (event.key === 'ArrowLeft' || event.key === 'ArrowRight') {
        this.openLoopSpeed = 0.0
      }
    },

    calibrate: function() {
      this.closedLoop = false
      this.targetSite = -1

      this.calibrating = true
      this.openLoopSpeed = 1
    },

    cancelCalibration: function() {
      this.calibrating = false
      this.openLoopSpeed = 0.0
    },

    zero: function() {
      this.$parent.publish('/carousel_zero_cmd', {
        'type': 'Signal'
      })
    }
  },

  watch: {
    targetSite() {
      this.closedLoop = true
      this.openLoopSpeed = 0.0

      this.publishClosedLoop()
    }
  }
}
</script>

<style scoped>
  .carousel-wrap {
    display: inline-block;
    align-content: center;
  }
  .box1 {
    text-align: left;
    vertical-align: top;
    display: inline-block;
  }
  .commands{
    text-align: left;
    display: inline-block;
  }
</style>