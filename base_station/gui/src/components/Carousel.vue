<template>
  <div class="carousel-wrap">
    <div>
        <h3> Carousel Data </h3>
    </div>
    <div class="box1">
        <label for="position">Rotate carousel to position:</label>
        <select v-model = "position" name="position" id="position">
            <option value=1>A</option>
            <option value=0>B</option>
            <option value=2>C</option>
        </select>
        <div class="commands" v-if="strip == 1">
            <p> Turning to position A </p>
        </div>
        <div class="commands" v-if="strip == 1">
            <p> Turning to position A </p>
        </div>
        <div class="commands" v-if="strip == 1">
            <p> Turning to position A </p>
        </div>
          <!--<p>Turning to position {{this.position}}</p>-->
        </div>
    </div>
  </div>
</template>

<script>
export default {
  data () {
    return {
        position: 1
    }
  },
  created: {
    function () {
      this.$parent.subscribe('/carousel_cmd', (msg) => {
        this.position = msg.position;
      })
    }
  },
  methods: {
      setPart: function() {
        this.$parent.publish("/carousel_data", {
        'type': 'CarouselData',
        'position': Number(this.position)
      })
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