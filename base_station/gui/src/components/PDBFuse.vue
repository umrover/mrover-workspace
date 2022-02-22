<template>
<div class="wrap">
    <div>
        <h3> PDB and Fuse Data </h3>
    </div>
    <div class="tables">
      <div class="table1">
      <table class="tableFormat" style="undefined;table-layout: fixed; width: 250px">
        <colgroup>
          <col style="width: 80px">
          <col style="width: 85px">
        </colgroup>
        <thead>
          <tr>
            <td class = "tableElement"></td>
            <td class = "tableElement">Temperature</td>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td class = "tableElement">3V3</td>
            <td class = "tableElement">{{pdb_data['3V3'].temp}}</td>
          </tr>
          <tr>
            <td class = "tableElement">5V</td>
            <td class = "tableElement">{{pdb_data['5V'].temp}}</td>
          </tr>
          <tr>
            <td class = "tableElement">12V</td>
            <td class = "tableElement">{{pdb_data['12V'].temp}}</td>
          </tr>
        </tbody>
      </table>
      </div>
      <div class="table2">
      <table class="tableFormat" style="undefined;table-layout: fixed; width: 250px">
        <colgroup>
          <col style="width: 80px">
          <col style="width: 85px">
          <col style="width: 85px">
        </colgroup>
        <thead>
          <tr>
            <td class = "tableElement"></td>
            <td class = "tableElement">Current</td>
            <td class = "tableElement">Voltage</td>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td class = "tableElement">3V3</td>
            <td class = "tableElement">{{pdb_data['3V3'].current}}</td>
            <td class = "tableElement">{{pdb_data['3V3'].voltage}}</td>
          </tr>
          <tr>
            <td class = "tableElement">5V</td>
            <td class = "tableElement">{{pdb_data['5V'].current}}</td>
            <td class = "tableElement">{{pdb_data['5V'].voltage}}</td>
          </tr>
          <tr>
            <td class = "tableElement">12V</td>
            <td class = "tableElement">{{pdb_data['12V'].current}}</td>
            <td class = "tableElement">{{pdb_data['12V'].voltage}}</td>
          </tr>
          <tr>
            <td class = "tableElement">Cytron 1</td>
            <td class = "tableElement">{{fuse_data.cytron1.current}}</td>
            <td class = "tableElement">{{fuse_data.cytron1.voltage}}</td>
          </tr>
          <tr>
            <td class = "tableElement">Cytron 2</td>
            <td class = "tableElement">{{fuse_data.cytron2.current}}</td>
            <td class = "tableElement">{{fuse_data.cytron2.voltage}}</td>
          </tr>
          <tr>
            <td class = "tableElement">Cytron 3</td>
            <td class = "tableElement">{{fuse_data.cytron3.current}}</td>
            <td class = "tableElement">{{fuse_data.cytron3.voltage}}</td>
          </tr>
          <tr>
            <td class = "tableElement">Cytron 4</td>
            <td class = "tableElement">{{fuse_data.cytron4.current}}</td>
            <td class = "tableElement">{{fuse_data.cytron4.voltage}}</td>
          </tr>
          <tr>
            <td class = "tableElement">Cytron 5</td>
            <td class = "tableElement">{{fuse_data.cytron5.current}}</td>
            <td class = "tableElement">{{fuse_data.cytron5.voltage}}</td>
          </tr>
          <tr>
            <td class = "tableElement">Cytron 6</td>
            <td class = "tableElement">{{fuse_data.cytron6.current}}</td>
            <td class = "tableElement">{{fuse_data.cytron6.voltage}}</td>
          </tr>
          <tr>
            <td class = "tableElement">Nichrome 1</td>
            <td class = "tableElement">{{fuse_data.nichrome1.current}}</td>
            <td class = "tableElement">{{fuse_data.nichrome1.voltage}}</td>
          </tr>
          <tr>
            <td class = "tableElement">Nichrome 2</td>
            <td class = "tableElement">{{fuse_data.nichrome2.current}}</td>
            <td class = "tableElement">{{fuse_data.nichrome2.voltage}}</td>
          </tr>
          <tr>
            <td class = "tableElement">Nichrome 3</td>
            <td class = "tableElement">{{fuse_data.nichrome3.current}}</td>
            <td class = "tableElement">{{fuse_data.nichrome3.voltage}}</td>
          </tr>
          <tr>
            <td class = "tableElement">HBridge 1</td>
            <td class = "tableElement">{{fuse_data.hbridge1.current}}</td>
            <td class = "tableElement">{{fuse_data.hbridge1.voltage}}</td>
          </tr>
          <tr>
            <td class = "tableElement">HBridge 2</td>
            <td class = "tableElement">{{fuse_data.hbridge2.current}}</td>
            <td class = "tableElement">{{fuse_data.hbridge2.voltage}}</td>
          </tr>
        </tbody>
    </table>
    </div>
  </div>
</div>
</template>

<style scoped>
    .tables {
      display: flex;
    }

    .wrap {
        display: inline-block;
        align-content: center;
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
        // drive_vel_data: {
        //   frontLeft:{current: 0, velocity: 0},
        //   frontRight:{current: 0, velocity: 0},
        //   middleLeft:{current: 0, velocity: 0},
        //   middleRight:{current: 0, velocity: 0},
        //   backLeft:{current: 0, velocity: 0},
        //   backRight:{current: 0, velocity: 0}
        // },
        // drive_state_data: {
        //   leftState: "Boot",
        //   rightState: "Boot"
        // }
        pdb_data: {
          '3V3':{temp: 0, current: 0, voltage: 0},
          '5V':{temp: 0, current: 0, voltage: 0},
          '12V':{temp: 0, current: 0, voltage: 0}
        },
        fuse_data: {
          cytron1:{current: 0, voltage: 0},
          cytron2:{current: 0, voltage: 0},
          cytron3:{current: 0, voltage: 0},
          cytron4:{current: 0, voltage: 0},
          cytron5:{current: 0, voltage: 0},
          cytron6:{current: 0, voltage: 0},
          nichrome1:{current: 0, voltage: 0},
          nichrome2:{current: 0, voltage: 0},
          nichrome3:{current: 0, voltage: 0},
          hbridge1:{current: 0, voltage: 0},
          hbridge2:{current: 0, voltage: 0}
        }
      }
    },
    created: function(){
      this.$parent.subscribe('/pdb_data', (msg) => {
        this.pdb_data['3V3'].temp = msg.temp[0];
        this.pdb_data['3V3'].current = msg.current[0];
        this.pdb_data['3V3'].voltage = msg.voltage[0];
        this.pdb_data['5V'].temp = msg.temp[1];
        this.pdb_data['5V'].current = msg.current[1];
        this.pdb_data['5V'].voltage = msg.voltage[1];
        this.pdb_data['12V'].temp = msg.temp[2];
        this.pdb_data['12V'].current = msg.current[2];
        this.pdb_data['12V'].voltage = msg.voltage[2];
      })
      this.$parent.subscribe('/fuse_data', (msg) => {
        this.fuse_data.cytron1.current = msg.current[0];
        this.fuse_data.cytron2.current = msg.current[1];
        this.fuse_data.cytron3.current = msg.current[2];
        this.fuse_data.cytron4.current = msg.current[3];
        this.fuse_data.cytron5.current = msg.current[4];
        this.fuse_data.cytron6.current = msg.current[5];
        this.fuse_data.nichrome1.current = msg.current[6];
        this.fuse_data.nichrome2.current = msg.current[7];
        this.fuse_data.nichrome3.current = msg.current[8];
        this.fuse_data.hbridge1.current = msg.current[9];
        this.fuse_data.hbridge2.current = msg.current[10];

        this.fuse_data.cytron1.voltage = msg.voltage[0];
        this.fuse_data.cytron2.voltage = msg.voltage[1];
        this.fuse_data.cytron3.voltage = msg.voltage[2];
        this.fuse_data.cytron4.voltage = msg.voltage[3];
        this.fuse_data.cytron5.voltage = msg.voltage[4];
        this.fuse_data.cytron6.voltage = msg.voltage[5];
        this.fuse_data.nichrome1.voltage = msg.voltage[6];
        this.fuse_data.nichrome2.voltage = msg.voltage[7];
        this.fuse_data.nichrome3.voltage = msg.voltage[8];
        this.fuse_data.hbridge1.voltage = msg.voltage[9];
        this.fuse_data.hbridge2.voltage = msg.voltage[10];
      })
    }
    // created: function(){
    //   this.$parent.subscribe('/drive_vel_data', (msg) => {
    //     if(msg.axis == 0){
    //       this.drive_vel_data.frontLeft.current = msg.measuredCurrent;
    //       this.drive_vel_data.frontLeft.velocity = msg.estimatedVel;
    //     }
    //     if(msg.axis == 1){
    //       this.drive_vel_data.frontRight.current = msg.measuredCurrent;
    //       this.drive_vel_data.frontRight.velocity = msg.estimatedVel;
    //     }
    //     if(msg.axis == 2){
    //       this.drive_vel_data.middleLeft.current = msg.measuredCurrent;
    //       this.drive_vel_data.middleLeft.velocity = msg.estimatedVel;
    //     }
    //     if(msg.axis == 3){
    //       this.drive_vel_data.middleRight.current = msg.measuredCurrent;
    //       this.drive_vel_data.middleRight.velocity = msg.estimatedVel;
    //     }
    //     if(msg.axis == 4){
    //       this.drive_vel_data.backLeft.current = msg.measuredCurrent;
    //       this.drive_vel_data.backLeft.velocity = msg.estimatedVel;
    //     }
    //     if(msg.axis == 5){
    //       this.drive_vel_data.backRight.current = msg.measuredCurrent;
    //       this.drive_vel_data.backRight.velocity = msg.estimatedVel;
    //     }
    //   })
    //   this.$parent.subscribe('/drive_state_data', (msg) => {
    //     if(msg.controller == 0){
    //       if(msg.state == 0){
    //         this.drive_state_data.leftState = "Boot";
    //       }
    //       if(msg.state == 1){
    //         this.drive_state_data.leftState = "Disarmed";
    //       }
    //       if(msg.state == 2){
    //         this.drive_state_data.leftState = "Armed";
    //       }
    //       if(msg.state == 3){
    //         this.drive_state_data.leftState = "Calibrating";
    //       }
    //       if(msg.state == 4){
    //         this.drive_state_data.leftState = "Error";
    //       }
    //       if(msg.state == 5){
    //         this.drive_state_data.leftState = "Exit";
    //       }
    //     }
    //     if(msg.controller == 1){
    //       if(msg.state == 0){
    //         this.drive_state_data.rightState = "Boot";
    //       }
    //       if(msg.state == 1){
    //         this.drive_state_data.rightState = "Disarmed";
    //       }
    //       if(msg.state == 2){
    //         this.drive_state_data.rightState = "Armed";
    //       }
    //       if(msg.state == 3){
    //         this.drive_state_data.rightState = "Calibrating";
    //       }
    //       if(msg.state == 4){
    //         this.drive_state_data.rightState = "Error";
    //       }
    //       if(msg.state == 5){
    //         this.drive_state_data.rightState = "Exit";
    //       }
    //     }
    //   })
    // }
}
</script>