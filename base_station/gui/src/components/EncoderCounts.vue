 <template>
  <div class="encoders">
    <div>joint a: {{Math.round((encoderCounts.joint_a + Number.EPSILON) * 10000) / 10000}}</div>
    <div>joint b: {{Math.round((encoderCounts.joint_b + Number.EPSILON) * 10000) / 10000}}</div>
    <div>joint c: {{Math.round((encoderCounts.joint_c + Number.EPSILON) * 10000) / 10000}}</div>
    <div>joint d: {{Math.round((encoderCounts.joint_d + Number.EPSILON) * 10000) / 10000}}</div>
    <div>joint e: {{Math.round((encoderCounts.joint_e + Number.EPSILON) * 10000) / 10000}}</div>
    <div>joint f: {{Math.round((encoderCounts.joint_f + Number.EPSILON) * 10000) / 10000}}</div>
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

</style>
