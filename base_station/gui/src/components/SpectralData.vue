<template>
<div class = "wrap">
  <table class="tableFormat" style="undefined;table-layout: fixed; width: 434px">
  <colgroup>
    <col style="width: 53px">
    <col style="width: 25px">
    <col style="width: 25px">
    <col style="width: 25px">
    <col style="width: 25px">
    <col style="width: 25px">
    <col style="width: 25px">
  </colgroup>
  <thead>
    <tr>
      <th> </th>
      <th>1</th>
      <th>2</th>
      <th >3</th>
      <th >4</th>
      <th >5</th>
      <th >6</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td >Spec 0</td>
      <td >{{spectral_data.d0_1}}</td>
      <td >{{spectral_data.d0_2}}</td>
      <td >{{spectral_data.d0_3}}</td>
      <td >{{spectral_data.d0_4}}</td>
      <td >{{spectral_data.d0_5}}</td>
      <td >{{spectral_data.d0_6}}</td>
    </tr>
    <tr>
      <td >Spec 1</td>
      <td >{{spectral_data.d1_1}}</td>
      <td >{{spectral_data.d1_2}}</td>
      <td >{{spectral_data.d1_3}}</td>
      <td >{{spectral_data.d1_4}}</td>
      <td >{{spectral_data.d1_5}}</td>
      <td >{{spectral_data.d1_6}}</td>
    </tr>
    <tr>
      <td >Spec 2</td>
      <td >{{spectral_data.d2_1}}</td>
      <td >{{spectral_data.d2_2}}</td>
      <td >{{spectral_data.d2_3}}</td>
      <td >{{spectral_data.d2_4}}</td>
      <td >{{spectral_data.d2_5}}</td>
      <td >{{spectral_data.d2_6}}</td>
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
    /*border-radius: 5px;
    padding: 10px;*/
    border: 1px solid black;
    text-align: right;
    vertical-align: top;
  }
  .tableFormat{
    border-collapse:collapse;
    border-spacing:0;
  }
  .tableFormat thead{
    display: table-header-group;
    text-align: center;
    
  }
  .tableFormat tbody{
    display: table-row;
    text-align: center;
  }
  .tableFormat th td{
    border: 1px solid black;
    border-collapse: collapse;
    overflow:hidden;
    padding:10px 5px;
    word-break:normal;
    text-align: center;
  }
</style>

<script>
export default {
  props: {
    spectral_data: {
      type: Object,
      required: true
    },
  }
}
</script>