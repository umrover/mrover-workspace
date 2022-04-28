<template>
  <div class="carousel-wrap">
    <div>
        <h3>Carousel Controls</h3>
    </div>
    <div class="box1">
        <label for="position">Rotate carousel to position:</label>
        <button v-on:click="target = 0"> Position 0 </button>
        <button v-on:click="target = 1"> Position 1 </button>
        <button v-on:click="target = 2"> Position 2 </button>
        <br>
        <div class="commands">
          <div v-if="closedLoop">
            <div v-if="reachedPos">
              <p>At position {{this.position}}</p>
            </div>
            <div v-else>
              <p>Turning to {{this.target}}</p>
            </div>
          </div>
          <div v-else>
            <p>Last at position {{this.position}}</p>
          </div>
        </div>
        <br>
        <label for="openloop">Openloop command:</label>
        <input type="text" name="openloop" @keydown="keydown" @keyup="keyup"/>
    </div>
  </div>
</template>

<script>

export default {
  data () {
    return {
      target: 0,
      position: 0,
      reachedPos: true,

      closedLoop: true,
      openLoopSpeed: 0.0,

      
    }
  },

  created: function () {
    this.$parent.subscribe('/carousel_pos_data', (msg) => {
      this.position = msg.position

      if (this.closedLoop) {
        if (msg.position == this.target) {
          this.reachedPos = true
        }
      }
    })

    window.setInterval(() => {
      if (!this.closedLoop) {
        this.$parent.publish('/carousel_openloop_cmd', {
          'type': 'CarouselOpenLoopCmd',
          'throttle': this.openLoopSpeed
        })
      }
    }, 100)
  },

  methods: {
    publishClosedLoop: function() {
      this.$parent.publish('/carousel_closedloop_cmd', {
        'type': 'CarouselPosition',
        'position': Number(this.target)
      })

      this.reachedPos = false
      this.closedLoop = true
      this.openLoopSpeed = 0.0
    },

    keydown: function(event) {
      if (event.key === 'ArrowLeft') {
        this.closedLoop = false
        this.reachedPos = false

        this.openLoopSpeed = -1.0
      }
      else if (event.key === 'ArrowRight') {
        this.closedLoop = false
        this.reachedPos = false

        this.openLoopSpeed = 1.0
      }
    },

    keyup: function(event) {
      if (event.key === 'ArrowLeft' || event.key === 'ArrowRight') {
        this.openLoopSpeed = 0.0
      }
    },
  },

  watch: {
    target() {
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