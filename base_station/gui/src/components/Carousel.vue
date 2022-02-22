<template>
  <div class="carousel-wrap">
    <div>
        <h3>Carousel Controls</h3>
    </div>
    <div class="box1">
        <label for="position">Rotate carousel to position:</label>
        <select v-model="position" name="position" id="position">
            <option value="2">A</option>
            <option value="1">B</option>
            <option value="0">C</option>
        </select>
        <br>
        <div class="commands">
          <p>{{reachedPos ? "At" : "Turning to"}} position {{this.posMap.get(this.position)}}</p>
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
      if (this.closedLoop) {
        if (msg.position == Number(this.position)) {
          this.reachedPos = true
        }
      }

      else {
        this.position = msg.position
        this.reachedPos = true;
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
      this.$parent.publish("/carousel_closedloop_cmd", {
        'type': 'CarouselClosedLoopCmd',
        'position': Number(this.position)
      })

      this.reachedPos = false
      this.closedLoop = true
      this.openLoopSpeed = 0.0
    },

    keydown: function(event) {
      this.closedLoop = false

      if (event.key === 'ArrowLeft') {
        this.openLoopSpeed = -1.0
      }
      else if (event.key === 'ArrowRight') {
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
    position() {
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