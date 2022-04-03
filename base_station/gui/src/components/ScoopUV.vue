<template>
<div class="wrap">
  <div>
    <h3> Scoop UV Bulb </h3>
  </div>
  <label for="toggle_button" :class="{'active': uvBulb == 1}" class="toggle__button">
      <span v-if="uvBulb == 1" class="toggle__label" >Scoop UV On</span>
      <span v-if="uvBulb == 0" class="toggle__label" >Scoop UV Off</span>

      <input type="checkbox" id="toggle_button">
      <span class="toggle__switch" v-if="uvBulb == 0" v-on:click="uvBulb = 1, UVshutdown(true)"></span>
      <span class="toggle__switch" v-if="uvBulb == 1" v-on:click="uvBulb = 0, UVshutdown(false)"></span>
  </label>
  <div class="shutdown">
    <label for="toggle_button" :class="{'active': shutdown == 1}" class="toggle__button">
      <span v-if="shutdown == 1" class="toggle__label" >UV Auto shutoff On</span>
      <span v-if="shutdown == 0" class="toggle__label" >UV Auto shutoff Off</span>

      <input type="checkbox" id="toggle_button">
      <span class="toggle__switch" v-if="shutdown == 0" v-on:click="shutdown=1"></span>
      <span class="toggle__switch" v-if="shutdown == 1" v-on:click="shutdown=0"></span>
    </label>
  </div>
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
      uvBulb: 0,
      shutdown: 1
    }
  },
  props: {
    mosfetIDs: {
      type: Object,
      required: true
    },
  },  
  methods: {
    setPart: function(id, enabled) {
      this.$parent.publish("/mosfet_cmd", {
        'type': 'MosfetCmd',
        'device': id,
        'enable': enabled
      })
    },
    UVshutdown: function(status) {
      this.setPart(this.mosfetIDs.uvBulb, status);
      if (this.shutdown === 1) {
        setTimeout(() => {
          this.uvBulb = 0;
          this.setPart(this.mosfetIDs.uvBulb, false);
        }, 120000) //2 minutes
      }
    }
  }
}
</script>