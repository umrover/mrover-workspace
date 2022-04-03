<template>
<div class = "main-content">
 <h3>Raw Data Reading</h3>
  <div class="wrap">
    <!-- <div>
      <p>GPS:</p>
      <p>Lat:</p>
      <p>{{formatted_gps.lat.d}}º</p>
      <p v-if="this.min_enabled">{{formatted_gps.lat.m}}'</p>
      <p v-if="this.sec_enabled">{{formatted_gps.lat.s}}"</p>
      N,
      <p>&nbsp;Lon:</p>
      <p>{{formatted_gps.lon.d}}º</p>
      <p v-if="this.min_enabled">{{formatted_gps.lon.m}}'</p>
      <p v-if="this.sec_enabled">{{formatted_gps.lon.s}}"</p>
      W
      <p>&nbsp;Bearing: {{GPS.bearing_deg.toFixed(2)}}º</p>
    </div> -->
    <div>
      <p>IMU: accel_x_g: {{ IMU.accel_x_g }} accel_y_g: {{ IMU.accel_y_g  }} accel_z_g: {{ IMU.accel_z_g }} &nbsp;gyro_x_dps: {{ IMU.gyro_x_dps }} gyro_y_dps: {{ IMU.gyro_y_dps }} gyro_z_dps: {{ IMU.gyro_z_dps }}
      &nbsp;mag_x_uT: {{ IMU.mag_x_uT }} mag_y_uT: {{ IMU.mag_y_uT  }} mag_z_uT: {{ IMU.mag_z_uT }} roll_rad: {{ IMU.roll_rad }} pitch_rad: {{ IMU.pitch_rad}} yaw_rad: {{ IMU.yaw_rad }} bearing_deg: {{ IMU.bearing_deg.toFixed(2) }}</p>
    </div>
    <div>
      <p>Calibration: sys: {{ IMU.calibration_sys}} gyro: {{ IMU.calibration_gyro }} accel: {{ IMU.calibration_accel}} mag: {{ IMU.calibration_mag }}</p>
    </div>
  </div>
</div>
</template>

<script>
import {convertDMS} from '../utils.js';
import {mapGetters} from 'vuex';

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

  // computed: {
  //   ...mapGetters('autonomy', {
  //     odom_format: 'odomFormat'
  //   }),

  //   formatted_gps: function() {
  //     return {
  //       lat: convertDMS({d: this.GPS.latitude_deg, m: this.GPS.latitude_min, s: 0}, this.odom_format),
  //       lon: convertDMS({d: -this.GPS.longitude_deg, m: -this.GPS.longitude_min, s: 0}, this.odom_format)
  //     };
  //   },

  //   min_enabled: function() {
  //     return this.odom_format != 'D';
  //   },

  //   sec_enabled: function() {
  //     return this.odom_format == 'DMS';
  //   }
  // }
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
