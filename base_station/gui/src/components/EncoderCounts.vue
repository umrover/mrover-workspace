 <template>
  <div class="encoders">
    <div>joint a: {{encoderCounts.joint_a}}</div>
    <div>joint b: {{encoderCounts.joint_b}}</div>
    <div>joint c: {{encoderCounts.joint_c}}</div>
    <div>joint d: {{encoderCounts.joint_d}}</div>
    <div>joint e: {{encoderCounts.joint_e}}</div>
    <div>joint f: {{encoderCounts.joint_f}}</div>
  </div>
</template>

<script>
export default {
  data () {
    return {
      encoderCounts: {
        joint_a: 0,
        joint_b: 0,
        joint_c: 0,
        joint_d: 0,
        joint_e: 0,
        joint_f: 0
      }
    }
  },

  created: function () {
    this.$parent.subscribe('/encoder', (msg) => {
      this.encoderCounts = msg
    })
  }
}
</script>

<style scoped>

.encoders {
  display: grid;
  grid-template-rows: 1fr 1fr 1fr;
  grid-auto-flow: column;
}

</style>
