<template>
<div>
  <div class="buttons">
    <div>
      <h3> Chlorophyll Data</h3>
    </div>

    <label for="toggle_button" :class="{'active': whiteLEDS == 1}" class="toggle__button">
      <span v-if="whiteLEDS == 1" class="toggle__label" >White LEDs On</span>
      <span v-if="whiteLEDS == 0" class="toggle__label" >White LEDs Off</span>

      <input type="checkbox" id="toggle_button">
        <span class="toggle__switch" v-if="whiteLEDS == 0" v-on:click="whiteLEDS=1, setPart(mosfetIDs.whiteLED, true)"></span>
        <span class="toggle__switch" v-if="whiteLEDS == 1" v-on:click="whiteLEDS=0, setPart(mosfetIDs.whiteLED, false)"></span>
    </label>

    <label for="toggle_button" :class="{'active': UVLED == 1}" class="toggle__button">
      <span v-if="UVLED == 1" class="toggle__label" >UV LEDs On</span>
      <span v-if="UVLED == 0" class="toggle__label" >UV LEDs Off</span>

      <input type="checkbox" id="toggle_button">
      <span class="toggle__switch" v-if="UVLED == 0" v-on:click="UVLED=1, setPart(mosfetIDs.UVLED, true)"></span>
      <span class="toggle__switch" v-if="UVLED == 1" v-on:click="UVLED=0, setPart(mosfetIDs.UVLED, false)"></span>
    </label>
  </div>
  <br>
  <div class="wrap-table">
    <div>
        <h3> Chlorophyll Spectral data </h3>
    </div>
    <table class="tableFormat" style="undefined;table-layout: fixed; width: 434px">
    <colgroup>
      <col style="width: 63px">
      <col style="width: 53px">
      <col style="width: 53px">
      <col style="width: 53px">
      <col style="width: 53px">
      <col style="width: 53px">
      <col style="width: 53px">
      <col style="width: 53px">
    </colgroup>
    <thead>
      <tr>
        <th class = "tableElement"></th>
        <th class = "tableElement">1</th>
        <th class = "tableElement">2</th>
        <th class = "tableElement">3</th>
        <th class = "tableElement">4</th>
        <th class = "tableElement">5</th>
        <th class = "tableElement">6</th>
      </tr>
    </thead>
    <tbody>
      <tr>
        <td class = "tableElement">Spec 0</td>
        <td class = "tableElement">{{spectral_data.d0_1}}</td>
        <td class = "tableElement">{{spectral_data.d0_2}}</td>
        <td class = "tableElement">{{spectral_data.d0_3}}</td>
        <td class = "tableElement">{{spectral_data.d0_4}}</td>
        <td class = "tableElement">{{spectral_data.d0_5}}</td>
        <td class = "tableElement">{{spectral_data.d0_6}}</td>
      </tr>
      <tr>
        <td class = "tableElement">Spec 1</td>
        <td class = "tableElement">{{spectral_data.d1_1}}</td>
        <td class = "tableElement">{{spectral_data.d1_2}}</td>
        <td class = "tableElement">{{spectral_data.d1_3}}</td>
        <td class = "tableElement">{{spectral_data.d1_4}}</td>
        <td class = "tableElement">{{spectral_data.d1_5}}</td>
        <td class = "tableElement">{{spectral_data.d1_6}}</td>
      </tr>
      <tr>
        <td class = "tableElement">Spec 2</td>
        <td class = "tableElement">{{spectral_data.d2_1}}</td>
        <td class = "tableElement">{{spectral_data.d2_2}}</td>
        <td class = "tableElement">{{spectral_data.d2_3}}</td>
        <td class = "tableElement">{{spectral_data.d2_4}}</td>
        <td class = "tableElement">{{spectral_data.d2_5}}</td>
        <td class = "tableElement">{{spectral_data.d2_6}}</td>
      </tr>
    </tbody>
    </table>
  </div>  
</div>
</template>

<style scoped>
  .buttons {
    display: inline-block;
    align-content: center;
    height: 10vh;
  }

  .wrap-table {
    display: inline-block;
    align-content: center;
    height: 40vh;
    padding-top: 5vh;
  }

  .report {
    height: 5vh;
    align-items: center;
    padding-top: 3vh;
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

  .tableFormat{
    border-collapse:collapse;
    border-spacing:0;
  }

  .tableFormat td{
    border-color:black;
    border-style:solid;
    border-width:1px;
    font-size:14px;
    overflow:hidden;
    padding:5px 5px;
    word-break:normal
  }

  .tableFormat th{
    border-color:black;
    border-style:solid;
    border-width:1px;
    font-size:14px;
    font-weight:normal;
    overflow:hidden;
    padding:10px 5px;
    word-break:normal;
  }

  .tableFormat .tableElement{
    border-color:inherit;
    text-align:center;
    vertical-align:top
  }

</style>

<script>
import GenerateReport from './GenerateReport.vue';

export default {
  data () {
    return {
      whiteLEDS: 0,
      UVLED: 0
    }
  },
  props: {
    mosfetIDs: {
      type: Object,
      required: true
    },
    spectral_data: {
      type: Object,
      required: true
    }
  },  
  methods: {
    setPart: function(id, enabled) {
      this.$parent.publish("/mosfet_cmd", {
        'type': 'MosfetCmd',
        'device': id,
        'enable': enabled
      })
    }
  },
  components: {
    GenerateReport
  }
}
</script>