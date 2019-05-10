 <template>
    <div class="wrap">
        <div>
            <span>joint a: {{encoderCounts.joint_a}}</span>
            <span>joint b: {{encoderCounts.joint_b}}</span>
            <span>joint c: {{encoderCounts.joint_c}}</span>
        </div>
        <div>
            <span>joint d: {{encoderCounts.joint_d}}</span>
            <span>joint e: {{encoderCounts.joint_e}}</span>
            <span>joint f: {{encoderCounts.joint_f}}</span>
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
    this.$parent.$parent.subscribe('/encoder', (msg) => {
      this.encoderCounts = msg
    })
  }
}
</script>

<style>
    .wrap {
        display: grid;
        align-items: center;
        grid-template-rows: 1fr 1fr;
        height: 100%;
    }
</style>
