<template>
<div class="wrap">
    <div>
        <h3> Motor Data </h3>
    </div>
    <table class="tableFormat" style="undefined;table-layout: fixed; width: 250px">
  <colgroup>
    <col style="width: 80px">
    <col style="width: 85px">
    <col style="width: 85px">
  </colgroup>
  <thead>
    <tr class="bold">
      <td class = "tableElement">Side</td>
      <td class = "tableElement">Left</td>
      <td class = "tableElement">Right</td>
    </tr>
  </thead>
  <tbody>
    <tr class="bold">
      <td class = "tableElement">Front</td>
      <td class = "tableElement">0</td>
      <td class = "tableElement">1</td>
    </tr>
    <tr>
      <td class = "tableElement">Current</td>
      <td class = "tableElement">{{drive_vel_data.frontLeft.current}} amps</td>
      <td class = "tableElement">{{drive_vel_data.frontRight.current}} amps</td>
    </tr>
    <tr>
      <td class = "tableElement">Velocity</td>
      <td class = "tableElement">{{drive_vel_data.frontLeft.velocity}} m/sec</td>
      <td class = "tableElement">{{drive_vel_data.frontRight.velocity}} m/sec</td>
    </tr>
    <tr class="bold">
      <td class = "tableElement">Middle</td>
      <td class = "tableElement">2</td>
      <td class = "tableElement">3</td>
    </tr>
    <tr>
      <td class = "tableElement">Current</td>
      <td class = "tableElement">{{drive_vel_data.middleLeft.current}} amps</td>
      <td class = "tableElement">{{drive_vel_data.middleRight.current}} amps</td>
    </tr>
    <tr>
      <td class = "tableElement">Velocity</td>
      <td class = "tableElement">{{drive_vel_data.middleLeft.velocity}} m/sec</td>
      <td class = "tableElement">{{drive_vel_data.middleRight.velocity}} m/sec</td>
    </tr>
    <tr class="bold">
      <td class = "tableElement">Back</td>
      <td class = "tableElement">4</td>
      <td class = "tableElement">5</td>
    </tr>
    <tr>
      <td class = "tableElement">Current</td>
      <td class = "tableElement">{{drive_vel_data.backLeft.current}} amps</td>
      <td class = "tableElement">{{drive_vel_data.backRight.current}} amps</td>
    </tr>
    <tr>
      <td class = "tableElement">Velocity</td>
      <td class = "tableElement">{{drive_vel_data.backLeft.velocity}} m/sec</td>
      <td class = "tableElement">{{drive_vel_data.backRight.velocity}} m/sec</td>
    </tr>
  </tbody>
  </table>
  <div>
        <h3> Drive State </h3>
  </div>
  <table class="tableFormat" style="undefined;table-layout: fixed; width: 255px">
      <colgroup>
        <col style="width: 85px">
        <col style="width: 85px">
        <col style="width: 85px">
      </colgroup>
      <thead>
          <tr class="bold">
              <td class = "tableElement">Front</td>
              <td class = "tableElement">Middle</td>
              <td class = "tableElement">Back</td>
          </tr>
      </thead>
      <tbody>
          <tr>
              <td class = "tableElement">{{drive_state_data.frontState}}</td>
              <td class = "tableElement">{{drive_state_data.middleState}}</td>
              <td class = "tableElement">{{drive_state_data.backState}}</td>
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
    font-size:13px;
    overflow:hidden;
    padding:10px 5px;
    word-break:normal
    }
    .tableFormat th{
    border-color:black;
    border-style:solid;
    border-width:1px;
    font-size:13px;
    font-weight:normal;
    overflow:hidden;
    padding:10px 5px;
    word-break:normal;
    }
    .bold{
      font-weight: bold;
      border: 2px solid black;
    }
    .tableFormat .tableElement{
    border-color:inherit;
    text-align:center;
    vertical-align:top
    }
</style>

<script>
export default {
    data() {
      return {
        drive_vel_data: {
          frontLeft:{current: 0, velocity: 0},
          frontRight:{current: 0, velocity: 0},
          middleLeft:{current: 0, velocity: 0},
          middleRight:{current: 0, velocity: 0},
          backLeft:{current: 0, velocity: 0},
          backRight:{current: 0, velocity: 0}
        },
        drive_state_data: {
          frontState: "Disconnected",
          middleState: "Disconnected",
          backState: "Disconnected"
        }
      }
    },
    created: function(){
      this.$parent.subscribe('/drive_vel_data', (msg) => {
        if(msg.axis == 0){
          this.drive_vel_data.frontLeft.current = msg.current_amps;
          this.drive_vel_data.frontLeft.velocity = msg.vel_m_s;
        }
        if(msg.axis == 1){
          this.drive_vel_data.frontRight.current = msg.current_amps;
          this.drive_vel_data.frontRight.velocity = msg.vel_m_s;
        }
        if(msg.axis == 2){
          this.drive_vel_data.middleLeft.current = msg.current_amps;
          this.drive_vel_data.middleLeft.velocity = msg.vel_m_s;
        }
        if(msg.axis == 3){
          this.drive_vel_data.middleRight.current = msg.current_amps;
          this.drive_vel_data.middleRight.velocity = msg.vel_m_s;
        }
        if(msg.axis == 4){
          this.drive_vel_data.backLeft.current = msg.current_amps;
          this.drive_vel_data.backLeft.velocity = msg.vel_m_s;
        }
        if(msg.axis == 5){
          this.drive_vel_data.backRight.current = msg.current_amps;
          this.drive_vel_data.backRight.velocity = msg.vel_m_s;
        }
      })
      this.$parent.subscribe('/drive_state_data', (msg) => {
        if(msg.odrive_index == 0){
          this.drive_state_data.frontState = msg.state;
        }
        if(msg.odrive_index == 1){
          this.drive_state_data.middleState = msg.state;
        }
        if(msg.odrive_index == 2){
          this.drive_state_data.backState = msg.state;
        }
      })
    }
}
</script>