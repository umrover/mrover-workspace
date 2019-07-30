<template>
  <div class="wrap">
    <p>
      Current odometry reading: {{ odom.latitude_deg }}º {{ Math.floor(odom.latitude_min) }}' {{ ((odom.latitude_min % 1)*60).toFixed(4) }}" N by {{ -odom.longitude_deg }}º {{ -Math.ceil(odom.longitude_min) }}' {{ -((odom.longitude_min % 1)*60).toFixed(4) }}" W bearing {{ odom.bearing_deg.toFixed(2) }}
    </p>
  </div>
</template>

<script>
export default {
  props: {
    odom: {
      type: Object,
      required: true
    },
  }
}
</script>

<style scoped>
  .wrap {
      padding: 0px;
      padding-left: 5px;
      padding-right: 5px;
      border: none;
  }
</style>