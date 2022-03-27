 <template>
  <div class="wrap">
    <h3>Encoder Counts</h3>
    <div class="encoders">
      <div>joint a: {{encoderCounts.joint_a.toFixed(5)}}</div>
      <div>joint b: {{encoderCounts.joint_b.toFixed(5)}}</div>
      <div>joint c: {{encoderCounts.joint_c.toFixed(5)}}</div>
      <div>joint d: {{encoderCounts.joint_d.toFixed(5)}}</div>
      <div>joint e: {{encoderCounts.joint_e.toFixed(5)}}</div>
      <div>joint f: {{encoderCounts.joint_f.toFixed(5)}}</div>
    </div>
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
    this.$parent.subscribe('/arm_position', (msg) => {
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
    column-gap: 20px;
}

.wrap {
  display: inline-block;
}

</style>
