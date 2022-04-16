<template>
<div class="wrap">
  <div>
    <h3> Scoop UV Bulb </h3>
  </div>
  <label for="toggle_button" :class="{'active': uv_bulb == 1}" class="toggle__button">
      <span v-if="uv_bulb == 1" class="toggle__label" >Scoop UV On</span>
      <span v-if="uv_bulb == 0" class="toggle__label" >Scoop UV Off</span>

      <input type="checkbox" id="toggle_button">
      <span class="toggle__switch" v-on:click="toggleUVBulb()"></span>
  </label>
  <div class="shutdown">
    <label for="toggle_button" :class="{'active': shutdown == 1}" class="toggle__button">
      <span v-if="shutdown == 1" class="toggle__label" >UV Auto shutoff On</span>
      <span v-if="shutdown == 0" class="toggle__label" >UV Auto shutoff Off</span>
      
      <input type="checkbox" id="toggle_button">
      <span class="toggle__switch" v-on:click="toggleShutdown()"></span>
    </label>
  </div>
    <label for="toggle_button" :class="{'active': scoop_limit == 1}" class="toggle__button">
        <span v-if="scoop_limit == 1" class="toggle__label" >Limit On</span>
        <span v-if="scoop_limit == 0" class="toggle__label" >Limit Off</span>

        <input type="checkbox" id="toggle_button">
        <span class="toggle__switch" v-on:click="toggleLimit()"></span>
    </label>
</div>  
</template>

<style scoped>
  .wrap {
    display: inline-block;
    align-content: center;
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

  .shutdown {
    padding-top: 5%;
    padding-bottom: 5%;
  }
</style>

<script>
export default {
  data () {
    return {
      uv_bulb: 0,
      shutdown: 1,
      timeoutID: 1,
      scoop_limit: 1
    }
  },
  props: {
    mosfetIDs: {
      type: Object,
      required: true
    }
  },

  watch: {
    uv_bulb(val) {
      this.$parent.publish("/mosfet_cmd", {
        'type': 'MosfetCmd',
        'device': this.mosfetIDs.uv_bulb,
        'enable': val
      })
    },

    scoop_limit(val) {
      this.$parent.publish("/scoop_limit_switch_enable_cmd", {
        'type': 'ScoopLimitSwitchEnable',
        'enable': val
      })
    }
  },

  methods: {
    toggleUVBulb: function() {
      this.uv_bulb = !this.uv_bulb

      if (this.uv_bulb) {
        this.timeoutID = setTimeout(() => {
          if (this.uv_bulb && this.shutdown) {
            this.uv_bulb = false
          }
        }, 2 * 60000) // 2 minutes
      }
      else {
        clearTimeout(this.timeoutID)
      }
    },
    
    toggleShutdown: function() {
      this.shutdown = !this.shutdown
    },
      
    toggleLimit: function() {
      this.scoop_limit = !this.scoop_limit
    }
  }
}
</script>