<template>
    <div class="wrap">
        <div v-show='false' id="key">
          <input v-on:keyup="keymonitor">
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
          "l": event.keyCode === 76,
        }

        this.$parent.$parent.publish('/gimbal_control', keyboardData)
      }
    }
  },

  beforeDestroy: function () {
    document.removeEventListener('keyup', this.keymonitor);
  },

  created: function () {
    document.addEventListener('keyup', this.keymonitor);
  }
}
</script>