<template>
<div class="wrap">
    <div>
      <h3>Amino Testing Controls</h3>
    </div>
    <div class="box1">
        <label for="site">Choose a site:</label>
        <select v-model = "site" name="site" id="site">
            <option value="0">Site 0</option>
            <option value="1">Site 1</option>
            <option value="2">Site 2</option>
        </select>
    </div>
    <div class="box1" v-if="site == 0">
        <button v-if="pump0==0" v-on:click="pump0 = 1, setPart(6,true)">
            Pump 0 on
        </button>
        <button v-if="pump0==1" v-on:click="pump0 = 0, setPart(6,false)">
            Pump 0 off
        </button>
        <button v-if="heater0==0" v-on:click="heater0 = 1, setPart(9,true)" v-bind:style="{color: color0}">
            Heater 0 on
        </button>
        <button v-if="heater0==1" v-on:click="heater0 = 0, setPart(9,false)" v-bind:style="{color: color0}">
            Heater 0 off
        </button>
        <p>Thermistor 0: {{temp0}} C°</p>
    </div>
    <div class="box1" v-if="site == 1">
        <button v-if="pump1==0" v-on:click="pump1 = 1, setPart(7,true)">
            Pump 1 on
        </button>
        <button v-if="pump1==1" v-on:click="pump1 = 0, setPart(7,false)">
            Pump 1 off
        </button>
        <button v-if="heater1==0" v-on:click="heater1 = 1, setPart(10,true)" v-bind:style="{color: color1}">
            Heater 1 on
        </button>
        <button v-if="heater1==1" v-on:click="heater1 = 0, setPart(10,false)" v-bind:style="{color: color1}">
            Heater 1 off
        </button>
        <p>Thermistor 1: {{temp1}} C°</p>
    </div>
    <div class="box1" v-if="site == 2">
        <button v-if="pump2==0" v-on:click="pump2 = 1, setPart(8,true)">
            Pump 2 on
        </button>
        <button v-if="pump2==1" v-on:click="pump2 = 0, setPart(8,false)">
            Pump 2 off
        </button>
        <button v-if="heater2==0" v-on:click="heater2 = 1, setPart(11,true)" v-bind:style="{color: color2}">
            Heater 2 on
        </button>
        <button v-if="heater2==1" v-on:click="heater2 = 0, setPart(11,false)" v-bind:style="{color: color2}">
            Heater 2 off
        </button>
        <p>Thermistor 2: {{temp2}} C°</p>
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
    /* height: 300px; */
  }
  .box1 {
    /*border-radius: 5px;
    border: 1px solid black;*/
    text-align: left;
    vertical-align: top;
    display: inline-block;
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