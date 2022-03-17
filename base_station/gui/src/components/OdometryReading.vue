<template>
  <div class="wrap">
    <p>Current odometry reading:</p>
    <div>
      <p>{{formatted_odom.lat.d}}º</p>
      <p v-if="this.min_enabled">{{formatted_odom.lat.m}}'</p>
      <p  v-if="this.sec_enabled">{{formatted_odom.lat.s}}"</p>
      N
    </div>
    <div>
      <p>{{formatted_odom.lon.d}}º</p>
      <p v-if="this.min_enabled">{{formatted_odom.lon.m}}'</p>
      <p  v-if="this.sec_enabled">{{formatted_odom.lon.s}}"</p>
      W
    </div>
  </div>
</template>

<script>
import {convertDMS} from '../utils.js';
import {mapGetters} from 'vuex';

export default {
  props: {
    odom: {
      type: Object,
      required: true
    },
  },

  computed: {
    ...mapGetters('autonomy', {
      odom_format: 'odomFormat'
    }),

    formatted_odom: function() {
      return {
        lat: convertDMS({d: this.odom.latitude_deg, m: this.odom.latitude_min, s: 0}, this.odom_format),
        lon: convertDMS({d: -this.odom.longitude_deg, m: -this.odom.longitude_min, s: 0}, this.odom_format)
      };
    },

    min_enabled: function() {
      return this.odom_format != 'D';
    },

    sec_enabled: function() {
      return this.odom_format == 'DMS';
    }
  },
}
</script>

<style scoped>
  .wrap {
      padding: 0px;
      padding-left: 5px;
      padding-right: 5px;
      border: none;
  }

  .wrap p {
    display: inline;
  }
</style>
