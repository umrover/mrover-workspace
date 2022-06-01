<template>
<div class = "main-content">
 <h3>Raw Data Reading</h3>
  <div class="wrap">
    <div>
      <h5>IMU</h5>
      <p>accel_x_g: {{ IMU.accel_x_g }}</p>
      <p>accel_y_g: {{ IMU.accel_y_g }}</p>
      <p>accel_z_g: {{ IMU.accel_z_g }}</p>
      <p>gyro_x_dps: {{ IMU.gyro_x_dps }}</p>
      <p>gyro_y_dps: {{ IMU.gyro_y_dps }}</p>
      <p>gyro_z_dps: {{ IMU.gyro_z_dps }}</p>
      <p>mag_x_uT: {{ IMU.mag_x_uT }}</p>
      <p>mag_y_uT: {{ IMU.mag_y_uT  }}</p>
      <p>mag_z_uT: {{ IMU.mag_z_uT }}</p>
      <p>roll_rad: {{ IMU.roll_rad }}</p>
      <p>pitch_rad: {{ IMU.pitch_rad}}</p>
      <p>yaw_rad: {{ IMU.yaw_rad }}</p>
      <p>bearing_deg:{{ IMU.bearing_deg.toFixed(2) }}</p>
    </div>
    <div>
      <h5>Calibration</h5>
      <p>sys: {{ IMU.calibration_sys}}</p>
      <p>gyro: {{ IMU.calibration_gyro }}</p>
      <p>accel: {{ IMU.calibration_accel}}</p>
      <p>mag: {{ IMU.calibration_mag }}</p>
    </div>
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
  },
}
</script>

<style scoped>

  .main-content h3{
    margin: 0px 0px 0px 5px;
    padding: 0px 0px 0px 0px;
  }

  .wrap {
      padding: 0px;
      padding-left: 5px;
      padding-right: 5px;
      border: none;
      grid-template-rows: auto;
  }

  .wrap p {
    display: inline;
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
