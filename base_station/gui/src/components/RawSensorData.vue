<template>
<div class = "main-content">
  <div class="wrap">
    <div>
      <p><b>IMU: </b> accel_x_g: {{ IMU.accel_x_g.toFixed(2) }}&nbsp;
        accel_y_g: {{ IMU.accel_y_g.toFixed(2) }}&nbsp;
        accel_z_g: {{ IMU.accel_z_g.toFixed(2) }}&nbsp;
        gyro_x_dps: {{ IMU.gyro_x_dps.toFixed(2) }}&nbsp;
        gyro_y_dps: {{ IMU.gyro_y_dps.toFixed(2) }}&nbsp;
        gyro_z_dps: {{ IMU.gyro_z_dps.toFixed(2) }}&nbsp;
        mag_x_uT: {{ IMU.mag_x_uT.toFixed(2) }}&nbsp;
        mag_y_uT: {{ IMU.mag_y_uT.toFixed(2)  }}&nbsp;
        mag_z_uT: {{ IMU.mag_z_uT.toFixed(2) }}&nbsp;
        roll_rad: {{ IMU.roll_rad.toFixed(2) }}&nbsp;
        pitch_rad: {{ IMU.pitch_rad.toFixed(2) }}&nbsp;
        yaw_rad: {{ IMU.yaw_rad.toFixed(2) }}&nbsp;
        bearing_deg:{{ IMU.bearing_deg.toFixed(2) }}</p>
    </div>
    <div>
      <p><b>Calibration: </b> sys: {{ IMU.calibration_sys.toFixed(2) }}&nbsp;
        gyro: {{ IMU.calibration_gyro.toFixed(2) }}&nbsp;
        accel: {{ IMU.calibration_accel.toFixed(2) }}&nbsp;
        mag: {{ IMU.calibration_mag.toFixed(2) }}</p>
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
      padding: 2px;
      border: none;
      grid-template-rows: auto;
  }

  .wrap p {
    display: inline;
  }

  p {
    margin: 0;
    padding: 0;
  }

</style>
