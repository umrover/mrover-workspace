<template>
<div class="wrap">
  <div>
    <h3>Power Distribution Board</h3>
  </div>
  <div class="table">
    <table class="tableFormat" style="undefined;table-layout: fixed">
      <colgroup>
        <col style="width: 80px">
        <col style="width: 100px">
        <col style="width: 100px">
        <col style="width: 100px">
      </colgroup>
      <thead>
        <tr>
          <td class = "tableElement"></td>
          <td class = "tableElement">Temperature</td>
          <td class = "tableElement">Current</td>
          <td class = "tableElement">Voltage</td>
        </tr>
      </thead>
      <tbody>
        <tr>
          <td class = "tableElement">3.3V</td>
          <td class = "tableElement" v-bind:style="{color: pdb_data.three.temp.color}">{{pdb_data.three.temp.val.toFixed(5)}}°C</td>
          <td class = "tableElement" v-bind:style="{color: pdb_data.three.current.color}">{{pdb_data.three.current.val.toFixed(5)}}A</td>
          <td class = "tableElement" v-bind:style="{color: pdb_data.three.voltage.color}">{{pdb_data.three.voltage.val.toFixed(5)}}V</td>
        </tr>
        <tr>
          <td class = "tableElement">5V</td>
          <td class = "tableElement" v-bind:style="{color: pdb_data.five.temp.color}">{{pdb_data.five.temp.val.toFixed(5)}}°C</td>
          <td class = "tableElement" v-bind:style="{color: pdb_data.five.current.color}">{{pdb_data.five.current.val.toFixed(5)}}A</td>
          <td class = "tableElement" v-bind:style="{color: pdb_data.five.voltage.color}">{{pdb_data.five.voltage.val.toFixed(5)}}V</td>
        </tr>
        <tr>
          <td class = "tableElement">12V</td>
          <td class = "tableElement" v-bind:style="{color: pdb_data.twelve.temp.color}">{{pdb_data.twelve.temp.val.toFixed(5)}}°C</td>
          <td class = "tableElement" v-bind:style="{color: pdb_data.twelve.current.color}">{{pdb_data.twelve.current.val.toFixed(5)}}A</td>
          <td class = "tableElement" v-bind:style="{color: pdb_data.twelve.voltage.color}">{{pdb_data.twelve.voltage.val.toFixed(5)}}V</td>
        </tr>
      </tbody>
    </table>
  </div>
  <div>
    <h3>Fuse Board</h3>
  </div>
  <div class="table">
    <table class="tableFormat" style="undefined;table-layout: fixed">
      <colgroup>
        <col style="width: 80px">
        <col style="width: 100px">
        <col style="width: 100px">
      </colgroup>
      <thead>
        <tr>
          <td class="tableElement"></td>
          <td class="tableElement">Current</td>
          <td class="tableElement">Voltage</td>
        </tr>
      </thead>
      <tbody>
        <tr>
          <td class="tableElement">Cytron 1</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.cytron1.current.color}">{{fuse_data.cytron1.current.val.toFixed(5)}}A</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.cytron1.voltage.color}">{{fuse_data.cytron1.voltage.val.toFixed(5)}}V</td>
        </tr>
        <tr>
          <td class="tableElement">Cytron 2</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.cytron2.current.color}">{{fuse_data.cytron2.current.val.toFixed(5)}}A</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.cytron2.voltage.color}">{{fuse_data.cytron2.voltage.val.toFixed(5)}}V</td>
        </tr>
        <tr>
          <td class="tableElement">Cytron 3</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.cytron3.current.color}">{{fuse_data.cytron3.current.val.toFixed(5)}}A</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.cytron3.voltage.color}">{{fuse_data.cytron3.voltage.val.toFixed(5)}}V</td>
        </tr>
        <tr>
          <td class="tableElement">Cytron 4</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.cytron4.current.color}">{{fuse_data.cytron4.current.val.toFixed(5)}}A</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.cytron4.voltage.color}">{{fuse_data.cytron4.voltage.val.toFixed(5)}}V</td>
        </tr>
        <tr>
          <td class="tableElement">Cytron 5</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.cytron5.current.color}">{{fuse_data.cytron5.current.val.toFixed(5)}}A</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.cytron5.voltage.color}">{{fuse_data.cytron5.voltage.val.toFixed(5)}}V</td>
        </tr>
        <tr>
          <td class="tableElement">Cytron 6</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.cytron6.current.color}">{{fuse_data.cytron6.current.val.toFixed(5)}}A</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.cytron6.voltage.color}">{{fuse_data.cytron6.voltage.val.toFixed(5)}}V</td>
        </tr>
        <tr>
          <td class="tableElement">Nichrome 1</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.nichrome1.current.color}">{{fuse_data.nichrome1.current.val.toFixed(5)}}A</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.nichrome1.voltage.color}">{{fuse_data.nichrome1.voltage.val.toFixed(5)}}V</td>
        </tr>
        <tr>
          <td class="tableElement">Nichrome 2</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.nichrome2.current.color}">{{fuse_data.nichrome2.current.val.toFixed(5)}}A</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.nichrome2.voltage.color}">{{fuse_data.nichrome2.voltage.val.toFixed(5)}}V</td>
        </tr>
        <tr>
          <td class="tableElement">Nichrome 3</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.nichrome3.current.color}">{{fuse_data.nichrome3.current.val.toFixed(5)}}A</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.nichrome3.voltage.color}">{{fuse_data.nichrome3.voltage.val.toFixed(5)}}V</td>
        </tr>
        <tr>
          <td class="tableElement">HBridge 1</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.hbridge1.current.color}">{{fuse_data.hbridge1.current.val.toFixed(5)}}A</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.hbridge1.voltage.color}">{{fuse_data.hbridge1.voltage.val.toFixed(5)}}V</td>
        </tr>
        <tr>
          <td class="tableElement">HBridge 2</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.hbridge2.current.color}">{{fuse_data.hbridge2.current.val.toFixed(5)}}A</td>
          <td class = "tableElement" v-bind:style="{color: fuse_data.hbridge2.voltage.color}">{{fuse_data.hbridge2.voltage.val.toFixed(5)}}V</td>
        </tr>
      </tbody>
    </table>
  </div>
</div>
</template>

<script>
export default {
  data() {
    return {
      pdb_data: {
        three: {
          temp: {val: 0.0, color: "grey"},
          current: {val: 0.0, color: "grey"},
          voltage: {val: 0.0, color: "grey"}
        },
        five: {
          temp: {val: 0.0, color: "grey"},
          current: {val: 0.0, color: "grey"},
          voltage: {val: 0.0, color: "grey"}
        },
        twelve: {
          temp: {val: 0.0, color: "grey"},
          current: {val: 0.0, color: "grey"},
          voltage: {val: 0.0, color: "grey"}
        }
      },

      fuse_data: {
        cytron1: {
          current: {val: 0.0, color: "grey"},
          voltage: {val: 0.0, color: "grey"}
        },
        cytron2: {
          current: {val: 0.0, color: "grey"},
          voltage: {val: 0.0, color: "grey"}
        },
        cytron3: {
          current: {val: 0.0, color: "grey"},
          voltage: {val: 0.0, color: "grey"}
        },
        cytron4: {
          current: {val: 0.0, color: "grey"},
          voltage: {val: 0.0, color: "grey"}
        },
        cytron5: {
          current: {val: 0.0, color: "grey"},
          voltage: {val: 0.0, color: "grey"}
        },
        cytron6: {
          current: {val: 0.0, color: "grey"},
          voltage: {val: 0.0, color: "grey"}
        },
        nichrome1: {
          current: {val: 0.0, color: "grey"},
          voltage: {val: 0.0, color: "grey"}
        },
        nichrome2: {
          current: {val: 0.0, color: "grey"},
          voltage: {val: 0.0, color: "grey"}
        },
        nichrome3: {
          current: {val: 0.0, color: "grey"},
          voltage: {val: 0.0, color: "grey"}
        },
        hbridge1: {
          current: {val: 0.0, color: "grey"},
          voltage: {val: 0.0, color: "grey"}
        },
        hbridge2: {
          current: {val: 0.0, color: "grey"},
          voltage: {val: 0.0, color: "grey"}
        },
      }
    }
  },

  created: function() {

    this.$parent.subscribe('/pdb_data', (msg) => {
      this.updateVal(this.pdb_data.three.temp, msg.temp[0], 100)
      this.updateVal(this.pdb_data.three.current, msg.current[0], 2)
      this.updateVal(this.pdb_data.three.voltage, msg.voltage[0], 3.6)
      
      this.updateVal(this.pdb_data.five.temp, msg.temp[1], 100)
      this.updateVal(this.pdb_data.five.current, msg.current[1], 8)
      this.updateVal(this.pdb_data.five.voltage, msg.voltage[1], 5.3)
      
      this.updateVal(this.pdb_data.twelve.temp, msg.temp[2], 100)
      this.updateVal(this.pdb_data.twelve.current, msg.current[2], 8)
      this.updateVal(this.pdb_data.twelve.voltage, msg.voltage[2], 14)
    })

    this.$parent.subscribe('/fuse_data', (msg) => {
      this.updateVal(this.fuse_data.cytron1.current, msg.current[0], 2.5)
      this.updateVal(this.fuse_data.cytron2.current, msg.current[1], 2.5)
      this.updateVal(this.fuse_data.cytron3.current, msg.current[2], 2.5)
      this.updateVal(this.fuse_data.cytron4.current, msg.current[3], 2.5)
      this.updateVal(this.fuse_data.cytron5.current, msg.current[4], 2.5)
      this.updateVal(this.fuse_data.cytron6.current, msg.current[5], 2.5)
      this.updateVal(this.fuse_data.nichrome1.current, msg.current[6], 2.5)
      this.updateVal(this.fuse_data.nichrome2.current, msg.current[7], 2.5)
      this.updateVal(this.fuse_data.nichrome3.current, msg.current[8], 2.5)
      this.updateVal(this.fuse_data.hbridge1.current, msg.current[9], 2.5)
      this.updateVal(this.fuse_data.hbridge2.current, msg.current[10], 2.5)

      this.updateVal(this.fuse_data.cytron1.voltage, msg.voltage[0], 45)
      this.updateVal(this.fuse_data.cytron2.voltage, msg.voltage[1], 45)
      this.updateVal(this.fuse_data.cytron3.voltage, msg.voltage[2], 45)
      this.updateVal(this.fuse_data.cytron4.voltage, msg.voltage[3], 45)
      this.updateVal(this.fuse_data.cytron5.voltage, msg.voltage[4], 45)
      this.updateVal(this.fuse_data.cytron6.voltage, msg.voltage[5], 45)
      this.updateVal(this.fuse_data.nichrome1.voltage, msg.voltage[6], 45)
      this.updateVal(this.fuse_data.nichrome2.voltage, msg.voltage[7], 45)
      this.updateVal(this.fuse_data.nichrome3.voltage, msg.voltage[8], 45)
      this.updateVal(this.fuse_data.hbridge1.voltage, msg.voltage[9], 45)
      this.updateVal(this.fuse_data.hbridge2.voltage, msg.voltage[10], 45)
    })
  },

  methods: {
    updateVal: function(struct, val, threshold) {
      struct.val = val

      if (val > threshold || val < 0.0) {
        struct.color = "red"
      }
      else {
        struct.color = "black"
      }
    }
  }
}
</script>

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

  .tableFormat {
    border-collapse: collapse;
    border-spacing: 0;
    padding: 10px;
  }

  .tableFormat td {
    border-color: black;
    border-style: solid;
    border-width: 1px;
    font-size: 13px;
    overflow: hidden;
    padding: 10px 5px;
    word-break: normal;
  }

  .tableFormat th {
    border-color: black;
    border-style: solid;
    border-width: 1px;
    font-size: 13px;
    font-weight: normal;
    overflow: hidden;
    padding: 10px 5px;
    word-break: normal;
  }
  .bold{
    font-weight: bold;
    border: 2px solid black;
  }
  .tableFormat .tableElement {
    border-color: inherit;
    text-align: center;
    vertical-align: top
  }
</style>
