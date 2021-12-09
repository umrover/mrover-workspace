<template>
  <div class="odom-wrap">
    <div class="sacontrols buttons">
    <button ref="raman" class=raman v-on:click="sendCollect(mosfetIDs.ramanLaser)"> <span>Raman Test</span> </button>
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
    mosfetIDs: {
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

    methods: {
      sendCollect: function (ramanLaser) {
        this.$parent.publish("/mosfet_cmd", {
        'type': 'MosfetCmd',
        'device': ramanLaser,
        'enable': true
      })
        let obj = this.$refs["raman"]
        obj.disabled = true
        setTimeout(this.pubHelper, 2000, ramanLaser);
      },
      pubHelper: function (ramanLaser){
        this.$refs["raman"].disabled = false;
        this.$parent.publish("/mosfet_cmd", {
            'type': 'MosfetCmd',
            'device': ramanLaser,
            'enable': false
          })
      }
    },
}
</script>

<style scoped>
  .odom-wrap {
      display: flex;
      align-items: center;
      justify-items: center;
      font-family: sans-serif;
      height: 10vh;
  }

  .odom {
    grid-area: odom;
  }

  .buttons {
    grid-area: buttons;
  }

  .odom-wrap p {
    display: inline;
  }
</style>
