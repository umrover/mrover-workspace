<template>
<div class="wrap">
    <div>
        <h3> Reverse</h3>
    </div>

    <label for="toggle_button" :class="{'active': reverse == 1}" class="toggle__button">
        <span v-if="reverse == 1" class="toggle__label" >Reverse</span>
        <span v-if="reverse == 0" class="toggle__label" >Forward</span>

        <input type="checkbox" id="toggle_button" v-model="checkedValue">
        <span class="toggle__switch" v-if="reverse == 1" v-on:click="reverse=0, setPart(false)"></span>
        <span class="toggle__switch" v-if="reverse == 0" v-on:click="reverse=1, setPart(true)"></span>
    </label>
</div>
</template>

<style scoped>
  .wrap {
    display: inline-block;
    align-content: center;
    /* height: 300px; */
  }
  .box1 {
    /*border-radius: 5px;
    border: 1px solid black;*/
    text-align: right;
    vertical-align: top;
    display: inline-block;
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

<script>
export default {
  data () {
    return {
      reverse: 0
    }
  },
  methods: {
    setPart: function(enabled) {
      this.$parent.publish("/reverse_drive", {
        'type': 'ReverseDrive',
        'reverse': enabled
      })
    }
  }
}
</script>
