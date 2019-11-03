<template>
<div class = "main-content">
 <h3>Raw Data Reading</h3>
  <div class="wrap">
    <span> GPS: {{ GPS.latitude_deg }}º {{ Math.floor(GPS.latitude_min) }}' {{ ((GPS.latitude_min % 1)*60).toFixed(4) }}" N by {{ -GPS.longitude_deg }}º {{ -Math.ceil(GPS.longitude_min) }}' {{ -((GPS.longitude_min % 1)*60).toFixed(4) }}" W bearing {{ GPS.bearing_deg.toFixed(2) }}
     <br><br>IMU: accel_x: {{ IMU.accel_x }} accel_y: {{ IMU.accel_y  }} accel_z: {{ IMU.accel_z }} gyro_x: {{ IMU.gyro_x }} gyro_y: {{ IMU.gyro_y }} gyro_z: {{ IMU.gyro_z }}
      mag_x: {{ IMU.mag_x }} mag_y: {{ IMU.mag_y  }} mag_z: {{ IMU.mag_z }} bearing: {{ IMU.bearing.toFixed(2) }}
    </span>
  </div>
</div>
</template>

<script>
export default {
  props: {
    GPS: {
      type: Object,
      required: true
    },

    IMU: {
      type: Object,
      required: true
    },
  }
}
</script>

<style scoped>

  .main-content h3{
    margin: 0px 0px 5px 0px; 
    padding: 0px 0px 0px 0px;
  }

  .wrap {
      padding: 0px;
      padding-left: 5px;
      padding-right: 5px;
      border: none;
  }

  .p {
    margin: 0;
    padding: 0;
  }

  .h3{
    padding: 0px;
    margin: 0px;
  }

</style>