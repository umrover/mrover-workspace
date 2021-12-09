<template>
<div class="wrap">
    <div>
      <h3>Amino Test Controls</h3>
    </div>
    <div class="box1">
        <label for="site">Choose a site:</label>
        <select v-model = "site" name="site" id="site">
            <option value="0">Site A</option>
            <option value="1">Site B</option>
            <option value="2">Site C</option>
        </select>
    </div>
    <div class="box1" v-if="site == 0">
        <label for="toggle_button" :class="{'active': heater0 == 0}" class="toggle__button">
            <span v-if="heater0 == 0" class="toggle__label" >Heater A On</span>
            <span v-if="heater0 == 1" class="toggle__label" >Heater A Off</span>

            <input type="checkbox" id="toggle_button" v-model="checkedValue">
            <span class="toggle__switch" v-if="heater0 == 1" v-on:click="heater0=0,setPart(mosfetIDs.heater0,false)"></span>
            <span class="toggle__switch" v-if="heater0 == 0" v-on:click="heater0=1,setPart(mosfetIDs.heater0,true)"></span>
        </label>

        <p>Thermistor A: {{temp0}} C°</p>
    </div>
    <div class="box1" v-if="site == 1">
        <label for="toggle_button" :class="{'active': heater1 == 0}" class="toggle__button">
            <span v-if="heater1 == 0" class="toggle__label" >Heater B On</span>
            <span v-if="heater1 == 1" class="toggle__label" >Heater B Off</span>

            <input type="checkbox" id="toggle_button" v-model="checkedValue">
            <span class="toggle__switch" v-if="heater1 == 1" v-on:click="heater1=0,setPart(mosfetIDs.heater1,false)"></span>
            <span class="toggle__switch" v-if="heater1 == 0" v-on:click="heater1=1,setPart(mosfetIDs.heater1,true)"></span>
        </label>

        <p>Thermistor B: {{temp1}} C°</p>
    </div>
    <div class="box1" v-if="site == 2">
        <label for="toggle_button" :class="{'active': heater2 == 0}" class="toggle__button">
            <span v-if="heater2 == 0" class="toggle__label" >Heater C On</span>
            <span v-if="heater2 == 1" class="toggle__label" >Heater C Off</span>

            <input type="checkbox" id="toggle_button" v-model="checkedValue">
            <span class="toggle__switch" v-if="heater2 == 1" v-on:click="heater2=0,setPart(mosfetIDs.heater2,false)"></span>
            <span class="toggle__switch" v-if="heater2 == 0" v-on:click="heater2=1,setPart(mosfetIDs.heater2,true)"></span>
        </label>

        <p>Thermistor C: {{temp2}} C°</p>
    </div>
    <div>
        <input type="checkbox" id="autoShutdown" name="autoShutdown" v-model="autoShutdown">
        <label for="autoShutdown"> Nichrome Auto-shutdown</label><br>
    </div>
</div>  
</template>

<style scoped>
  .wrap {
    display: inline-block;
    vertical-align: top;
    align-content: left;
  }
  .box1 {
    text-align: left;
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
        site: 0,
        pump0: 0,
        pump1: 0,
        pump2: 0,
        heater0: 0,
        heater1: 0,
        heater2: 0,
        temp0: 0,
        temp1: 0,
        temp2: 0,
        temp0hook: 0,
        temp1hook: 0,
        temp2hook: 0,
        color0: "black",
        color1: "black",
        color2: "black",
        autoShutdown: true
    }
  },
  created:
    function () {
      this.$parent.subscribe('/thermistor_data', (msg) => {
        this.temp0 = msg.temp0
        this.temp1 = msg.temp1
        this.temp2 = msg.temp2
        this.tempLimitHelper();
      })
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
    tempLimit: function() {
            console.log("tempLimit");
            if(this.temp0>100 && this.temp0hook == 0){
                this.temp0hook = 1;
                this.heater0 = 0;
                this.setPart(9,false);
                this.color0 = "red";
                alert("Thermistor 0 over temp limit, shuttting down till 90°")
            }
            else if (this.temp0hook == 1 && this.heater0 == 1){
                this.temp0hook = 0;
                this.setPart(9,true);
                this.color0 = "black";
            }
            else if (this.temp0hook == 1 && this.temp0 <= 90){
                this.temp0hook = 0;
                this.heater0 = 1;
                this.setPart(9,true);
                this.color0 = "black";
                alert("Thermistor 0 turned back on")
            }
            if(this.temp1>100 && this.temp1hook == 0){
                this.temp1hook = 1;
                this.heater1 = 0;
                this.setPart(10,false);
                this.color1 = "red";
                alert("Thermistor 1 over temp limit, shuttting down till 90°")
            }
            else if (this.temp1hook == 1 && this.heater1 == 1){
                this.temp1hook = 0;
                this.setPart(10,true);
                this.color1 = "black";
            }
            else if (this.temp1hook == 1 && this.temp1 <= 90){
                this.temp1hook = 0;
                this.heater1 = 1;
                this.color1 = "black";
                this.setPart(10,true);
                alert("Thermistor 1 turned back on")
                
            }
            if(this.temp2>100 && this.temp2hook == 0){
                this.temp2hook = 1;
                this.heater2 = 0;
                this.setPart(11,false);
                this.color2 = "red";
                alert("Thermistor 2 over temp limit, shuttting down till 90°")
            }
            else if (this.temp2hook == 1 && this.heater2 == 1){
                this.temp2hook = 0;
                this.setPart(11,true);
                this.color2 = "black";
            }
            else if (this.temp2hook == 1 && this.temp2 <= 90){
                this.temp2hook = 0;
                this.heater2 = 1;
                this.color2 = "black";
                this.setPart(11,true);
                alert("Thermistor 2 turned back on")
            }
    },
    tempLimitHelper: function(){
        if(this.autoShutdown){
            this.tempLimit();
        }
        else{
            this.color0 = "black"
            this.color1 = "black"
            this.color2 = "black"
            this.temp0hook = 0
            this.temp1hook = 0
            this.temp2hook = 0
        }
    }
  }
}
</script>