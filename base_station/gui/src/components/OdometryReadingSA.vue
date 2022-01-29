<template>
  <div class="odom-wrap">
    <h3> Raman laser </h3>
    <div class="sacontrols buttons">
      <label for="toggle_button" :class="{'active': ramanLaser == true}" class="toggle__button">
        <span v-if="ramanLaser == false" class="toggle__label" >Raman Laser Off</span>
        <span v-if="ramanLaser == true" class="toggle__label" >Raman Laser On</span>

        <input type="checkbox" id="toggle_button" v-model="checkedValue">
          <span class="toggle__switch" v-if="ramanLaser== true" v-on:click="ramanLaser=false,pubHelper(mosfetIDs.ramanLaser)"></span>
          <span class="toggle__switch" v-if="ramanLaser == false" v-on:click="ramanLaser=true,sendCollect(mosfetIDs.ramanLaser)"></span>
      </label>
    </div>
  </div>
</template>

<script>
import {convertDMS} from '../utils.js';
import {mapGetters} from 'vuex';

export default {
  data () {
    return {
      ramanLaser: false
    }
  },
  
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
      display: inline-block;
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

  .toggle__button {
    vertical-align: middle;
    user-select: none;
    cursor: pointer;
  }

  .toggle__button input[type="checkbox"] {
      opacity: 0;
      position: absolute;
      width: 1px;
      height: 1px;
  }

  .toggle__button .toggle__switch {
      display:inline-block;
      height:12px;
      border-radius:6px;
      width:40px;
      background: #BFCBD9;
      box-shadow: inset 0 0 1px #BFCBD9;
      position:relative;
      margin-left: 10px;
      transition: all .25s;
  }

  .toggle__button .toggle__switch::after, 
  .toggle__button .toggle__switch::before {
      content: "";
      position: absolute;
      display: block;
      height: 18px;
      width: 18px;
      border-radius: 50%;
      left: 0;
      top: -3px;
      transform: translateX(0);
      transition: all .25s cubic-bezier(.5, -.6, .5, 1.6);
  }

  .toggle__button .toggle__switch::after {
      background: #4D4D4D;
      box-shadow: 0 0 1px #666;
  }

  .toggle__button .toggle__switch::before {
      background: #4D4D4D;
      box-shadow: 0 0 0 3px rgba(0,0,0,0.1);
      opacity:0;
  }

  .active .toggle__switch {
    background: #FFEA9B;
    box-shadow: inset 0 0 1px #FFEA9B;
  }

  .active .toggle__switch::after,
  .active .toggle__switch::before{
      transform:translateX(40px - 18px);
  }

  .active .toggle__switch::after {
      left: 23px;
      background: #FFCB05;
      box-shadow: 0 0 1px #FFCB05;
  }
</style>
