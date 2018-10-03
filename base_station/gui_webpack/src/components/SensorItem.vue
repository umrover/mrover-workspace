<template>
  <div class="box">
      <p>
          {{name}}
          <span>
            {{value.toFixed(2)}}{{unit}}
          </span>
      </p>
  </div>
</template>

<script>
export default {
  props: {
    name: {
      type: String,
      required: true
    },
    value: {
      type: Number,
      required: true
    },
    unit: {
      type: String,
      default: ''
    }
  }
}
</script>

<style scoped>
  .box {
      padding: 0px;
      padding-left: 5px;
      padding-right: 5px;
      border: none;
  }

  p {
      margin: 0px;
      font-size: 1rem;
      margin-top: 5px;
      text-align: left;
  }

  span{
      float: right;
  }
</style>
