<template>
<div class="wrap">
    <div>
      <h3> Chlorophyll Info</h3>
    </div>
    <!--div class="box1">
      <label for="sensors">Choose a site:</label>
      <select name="sensors" id="sensors" v-model="chloroSite">
      <option >0</option>
      <option >1</option>
      <option >2</option>
      </select>
      UV Status: {{siteUVs.UV0}}
    </div-->
    <div class="box1" v-if="whiteLEDS == 0">
        <button  v-on:click="whiteLEDS=1,setPart(mosfetIDs.sciWLed,true)">
        White LEDs On
        </button>
    </div>
    <div class="box1" v-if="whiteLEDS == 1">
        <button  v-on:click="whiteLEDS=0,setPart(mosfetIDs.sciWLed,false)">
        White LEDs Off
        </button>
    </div>
    <div class="box1">
        <button v-if="siteUVs.UV0 == 0" v-on:click="siteUVs.UV0 = 1,setPart(mosfetIDs.sciUV,true)">
        UV On
        </button>
        <button v-if="siteUVs.UV0 == 1" v-on:click="siteUVs.UV0 = 0,setPart(mosfetIDs.sciUV,false)">
        UV Off
        </button>
    </div>
    <!--div class="box1" v-if="chloroSite == 1">
        <button v-if="siteUVs.UV1 == 0" v-on:click="siteUVs.UV1 = 1">
        UV On
        </button>
        <button v-if="siteUVs.UV1 == 1" v-on:click="siteUVs.UV1 = 0">
        UV Off
        </button>
    </div>
    <div class="box1" v-if="chloroSite == 2">
        <button v-if="siteUVs.UV2 == 0" v-on:click="siteUVs.UV2 = 1">
        UV On
        </button>
        <button v-if="siteUVs.UV2 == 1" v-on:click="siteUVs.UV2 = 0">
        UV Off
        </button>
    </div-->
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
</style>

<script>
export default {
  data () {
    return {
      whiteLEDS: 0,
      chloroSite: 0,
      siteUVs:{
        UV0: 0,
        UV1: 0,
        UV2: 0
      },
    }
  },
  created:{ 
    function () {
      this.$parent.subscribe('/mosfet_cmd', (msg) => {
        this.siteUVs = msg
      })
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
    }
  }
}
</script>