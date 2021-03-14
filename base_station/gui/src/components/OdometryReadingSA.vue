<template>
  <div class="odom-wrap">
    <div class="sacontrols odom">
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
        <br/>
        <p>Bearing: {{odom.bearing_deg.toFixed(2)}}º</p>
      </div>
    </div>
    <!--div class = "pdb">
    <PDBData/>
    </div-->
    <div class="sacontrols buttons">
    <button ref="raman" class=raman v-on:click="sendCollect($event)"> <span>Raman Test</span> </button>
    <SAControls/>
    </div>
  </div>
</template>

<script>
import {convertDMS} from '../utils.js';
import {mapGetters} from 'vuex';
import SAControls from './SAControls.vue';
import PDBData from './PDBData.vue';

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

    methods: {
      sendCollect: function (button) {
        this.$parent.publish("/mosfet_cmd", {
        'type': 'MosfetCmd',
        'device': 5,
        'enable': true
      })
        let obj = this.$refs["raman"]
        obj.disabled = true
        setTimeout(this.pubHelper, 2000);
      },
      pubHelper: function (){
        this.$refs["raman"].disabled = false;
        this.$parent.publish("/mosfet_cmd", {
            'type': 'MosfetCmd',
            'device': 5,
            'enable': false
          })
      }
    },
  components: {
    SAControls
    //PDBData
  }
}
</script>

<style scoped>
  .odom-wrap {
      padding: 0px;
      padding-left: 10px;
      padding-right: 0px;
      border: none;
      margin-top: 0.5rem;
      display: grid;
      grid-gap: 3px;
      grid-template-columns: 1fr 1fr;
      grid-template-rows: 1fr 1fr;
      grid-template-areas: "odom buttons" "pdb buttons";
      font-family: sans-serif;
      height: 24vh;
  }
  .odom{
    grid-area: odom;
  }
  .buttons{
    grid-area: buttons;
  }
  .raman{
    margin-left: 20px;
  }
  .pdb{
    grid-area: pdb;
  }
  .odom-wrap p {
    display: inline;
  }

  .sacontrols{
    display:inline-block;
  }
</style>
