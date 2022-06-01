<template>
<div class = "main-content">
 <h3>Raw Data Reading</h3>
  <div class="wrap">
    <div>
      <h5>IMU</h5>
      <p>accel_x_g: {{ IMU.accel_x_g }}<br/>
        accel_y_g: {{ IMU.accel_y_g }}<br/>
        accel_z_g: {{ IMU.accel_z_g }}<br/>
        gyro_x_dps: {{ IMU.gyro_x_dps }}<br/>
        gyro_z_dps: {{ IMU.gyro_z_dps }}<br/>
        gyro_y_dps: {{ IMU.gyro_y_dps }}<br/>
        mag_x_uT: {{ IMU.mag_x_uT }}<br/>
        mag_y_uT: {{ IMU.mag_y_uT  }}<br/>
        mag_z_uT: {{ IMU.mag_z_uT }}<br/>
        roll_rad: {{ IMU.roll_rad }}<br/>
        pitch_rad: {{ IMU.pitch_rad}}<br/>
        yaw_rad: {{ IMU.yaw_rad }}<br/>
        bearing_deg:{{ IMU.bearing_deg.toFixed(2) }}</p>
    </div>
    <div>
      <h5>Calibration</h5>
      <p>sys: {{ IMU.calibration_sys}}<br/>
        gyro: {{ IMU.calibration_gyro }}<br/>
        accel: {{ IMU.calibration_accel}}<br/>
        mag: {{ IMU.calibration_mag }}</p>
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
      padding: 5px;
      border: none;
      grid-template-rows: auto;
  }

  .wrap p {
    display: inline;
  }

  h3{
    padding: 0px;
    margin: 0px;
  }

  h5 {
    padding: 0;
    padding-top: 5px;
    margin: 0;
  }

  p {
    margin: 0;
    padding: 0;
  }

</style>
