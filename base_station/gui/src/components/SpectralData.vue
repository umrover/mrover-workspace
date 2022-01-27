<template>
<div>
    <div>
        <h3> End Effector Spectral Triad Data </h3>
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
  <br>
  <div class="report">
    <GenerateReport v-bind:spectral_data="spectral_data"/>
  </div>
</div>
</template>

<style scoped>
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

  .report {
    height: 7vh;
    align-items: center;
  }
</style>

<script>
import GenerateReport from './GenerateReport.vue';

export default {
  props: {
    spectral_data: {
      type: Object,
      required: true
    }
  },

  components: {
    GenerateReport
  }

}
</script>