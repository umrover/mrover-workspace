<template>
  <div class="carousel-wrap">
    <div>
        <h3>Carousel Controls</h3>
    </div>
    <div class="box1">
        <label for="position">Rotate carousel to position:</label>
        <select v-model="target" name="target" id="target">
            <option value="2">A</option>
            <option value="1">B</option>
            <option value="0">C</option>
        </select>
        <br>
        <div class="commands">
          <div v-if="closedLoop">
            <div v-if="reachedPos">
              <p>At position {{this.posMap.get(this.position)}}</p>
            </div>
            <div v-else>
              <p>Turning to {{this.posMap.get(this.target)}}</p>
            </div>
          </div>
          <div v-else>
            <p>Last at position {{this.posMap.get(this.position)}}</p>
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
      target: '2',
      position: '2',
      reachedPos: true,

      closedLoop: true,
      openLoopSpeed: 0.0,

      posMap:new Map([
        ['2', 'A'],
        ['1', 'B'],
        ['0', 'C'],
      ]),
    }
  },

  created: function () {
    this.$parent.subscribe('/carousel_data', (msg) => {
      this.position = msg.position.toString()

      if (this.closedLoop) {
        if (msg.position == Number(this.target)) {
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
        'type': 'CarouselClosedLoopCmd',
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