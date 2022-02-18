<template>
  <div class="carousel-wrap">
    <div>
        <h3> Carousel Data </h3>
    </div>
    <div class="box1">
        <label for="position">Rotate carousel to position:</label>
        <select v-model = "position" name="position" id="position">
            <option value="2">A</option>
            <option value="1">B</option>
            <option value="0">C</option>
        </select>
        <div class="commands">
          <p>{{reachedPos?  "Reached": "Turning to"}} position {{this.posMap.get(this.position)}}</p>
        </div>
    </div>
  </div>
</template>

<script>

export default {
  data () {
    return {
        position: '2',
        reachedPos: true,
        posMap:new Map([
        ['2', 'A'],
        ['1', 'B'],
        ['0', 'C'],
        ])
    }
  },
  created: 
    function () {
        this.$parent.subscribe('/carousel_data', (msg) => {
          if (msg.position == Number(this.position)) {
            this.reachedPos = true
          }
        })
      },
  methods: {
      setPart: function() {
        this.$parent.publish("/carousel_closedloop_cmd", {
        'type': 'CarouselClosedLoopCmd',
        'position': Number(this.position)
      })
      this.reachedPos = false
    }
  },
  watch: {
    position() {
      this.setPart()
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