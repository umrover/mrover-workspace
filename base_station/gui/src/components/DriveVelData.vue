<template>
<div class="wrap">
    <div>
        <h3> Motor Data </h3>
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
      <td class = "tableElement">Drive State</td>
      <td class = "tableElement">{{}}</td>
      <td class = "tableElement">{{}}</td>
    </tr>
  </thead>
  <tbody>
    <tr>
      
    </tr>
    <tr>
      
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
</template>

<style scoped>
    .wrap {
        display: inline-block;
        align-content: center;
        /* height: 300px; */
    }
    .box {
        border-radius: 5px;
        padding: 10px;
        border: 1px solid black;
        text-align: right;
        vertical-align: top;
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
    padding:10px 5px;
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
export default {
    methods: {
        
    }
}
</script>