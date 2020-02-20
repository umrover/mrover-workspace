<template>
    <div class="wrap">
        <div v-show='false' id="key">
          <input v-on:keydown="keymonitor">
          <input v-on:keyup="keyzero">
      </div>
    </div>
</template>

<script>
export default {
  data() {
    return {
      keymonitor: (event) => {
        const keyboardData = {
          "type": "Keyboard",
          "w": event.keyCode === 87,
          "a": event.keyCode === 65,
          "s": event.keyCode === 83,
          "d": event.keyCode === 68,
          "i": event.keyCode === 73,
          "j": event.keyCode === 74,
          "k": event.keyCode === 75,
          "l": event.keyCode === 76
        }

        this.$parent.$parent.publish('/gimbal_control', keyboardData)
      },

      keyzero: (event) => {
        const keyboardData = {
          "type": "Keyboard",
          "w": 0,
          "a": 0,
          "s": 0,
          "d": 0,
          "i": 0,
          "j": 0,
          "k": 0,
          "l": 0
        }

        this.$parent.$parent.publish('/gimbal_control', keyboardData)
      }
    }
  },

  beforeDestroy: function () {
    document.removeEventListener('keydown', this.keymonitor);
    document.removeEventListener('keyup', this.keyzero);
  },

  created: function () {
    document.addEventListener('keydown', this.keymonitor);
    document.addEventListener('keyup', this.keyzero);
  }
}
</script>