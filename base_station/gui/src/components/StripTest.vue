<template>
<div class="status wrap" v-bind:style="{backgroundColor: background_color}">
    <div class="inner_pad">
        <div>
            <h3> Strip Test Controls </h3>
        </div>
        <div class="controls">
            <div>
                <label for="strip">Perform strip test on strip:</label>
                    <select v-model = "strip" name="strip" id="strip">
                        <option value=0>0</option>
                        <option value=1>1</option>
                        <option value=2>2</option>
                    </select>
                <div class="commands" v-if="strip == 0">
                    <button v-on:click="stripTest0()"> Start Strip 0 Test </button>
                </div>
                <div class="commands" v-if="strip == 1">
                    <button v-on:click="stripTest1()"> Start Strip 1 Test </button>
                </div>
                <div class="commands" v-if="strip == 2">
                    <button v-on:click="stripTest2()"> Start Strip 2 Test </button>
                </div>
            </div>
        </div>
    <button v-on:click="back_to_start()">Back to start</button>
    <div class="input">
        Angle: <input type='number' v-model.number='angle'>
        <button v-on:click="userInput()"> Move to angle</button>
    </div>
    </div>
</div>
</template>

<script>
const serv2_start = 100;
const serv2_dip = 30;
const serv1_start = 100;
const serv1_dip = 30;
const serv0_start = 80;
const serv0_dip = 10;


const undipped = '#A54657';
const dipping = '#FABC2A';
const dipped = '#AEF78E';




export default {
    data () {
        return {
            strip: 0,
            angle0: serv0_start,
            angle1: serv1_start,
            angle2: serv2_start,
            serv2_start: serv2_start,
            serv2_dip: serv2_dip,
            serv2_has_dip: false,
            serv2_color: undipped,
            serv1_start: serv1_start,
            serv1_dip: serv1_dip,
            serv1_has_dip: false,
            serv1_color: undipped,
            serv0_start: serv0_start,
            serv0_dip: serv0_dip,
            serv0_has_dip: false,
            serv0_color: undipped,
            background_color: undipped,
            undipped: undipped,
            dipping: dipping,
            dipped: dipped
        }
    },
    methods: {
        setPart: function() {
            this.$parent.publish("/servo_cmd", {
            'type': 'ServoCmd',
            'angle0': this.angle0,
            'angle1': this.angle1,
            'angle2': this.angle2
            })
        },

        stripTest2: function() {
            this.angle2 = this.serv2_dip;
            this.setPart();
            setTimeout(() => {
                this.angle2 = this.serv2_start;
                this.serv2_has_dip = true;
                this.setPart();
                }, 10000);
        },

        stripTest1: function() {
            this.angle1 = this.serv1_dip;
            this.setPart();
            setTimeout(() => {
                this.angle1 = this.serv1_start;
                this.serv1_has_dip = true;
                this.setPart();
                }, 10000);
        },
        
        stripTest0: function() {
            this.angle0 = this.serv0_dip;
            this.setPart();
            setTimeout(() => {
                this.angle0 = this.serv0_start;
                this.serv0_has_dip = true;
                this.setPart();
                }, 10000);
        },
    },
    watch: {
        strip: function (val) {
            if (val == 2) {
                this.background_color = this.serv2_color;
            } else if (val == 1) {
                this.background_color = this.serv1_color;
            } else {
                this.background_color = this.serv0_color;
            }
        },
        angle2: function (val) {
            if (val == 100) {
                if (this.serv2_has_dip) {
                    this.serv2_color = this.dipped;
                } else {
                    this.serv2_color = this.undipped;
                }
            } else if (val == 30) {
                this.serv2_color = this.dipping;
            }
            if (this.strip == 2) {
                this.background_color = this.serv2_color;
            }
        },
        angle1: function (val) {
            if (val == 100) {
                if (this.serv1_has_dip) {
                    this.serv1_color = this.dipped;
                } else {
                    this.serv1_color = this.undipped;
                }
            } else if (val == 30) {
                this.serv1_color = this.dipping;
            }
            if (this.strip == 1) {
                this.background_color = this.serv1_color;
            }
        },
        angle0: function (val) {
            if (val == 80) {
                if (this.serv0_has_dip) {
                    this.serv0_color = this.dipped;
                } else {
                    this.serv0_color = this.undipped;
                }
            } else if (val == 10) {
                this.serv0_color = this.dipping;
            }
            if (this.strip == 0) {
                this.background_color = this.serv0_color;
            }
        }
    }
}
</script>

<style scoped>
    .wrap {
        display: inline-block;
        align-content: center;
    }
    .status {
        border-radius: 5px;
        border: 1px solid black;
        display: flex;
        justify-content: space-around;
        flex-direction: column;
    }

    .inner_pad {
        padding: 10px;
    }

    .controls {
        display: inline-block;
        vertical-align: top;
        align-content: left;
    }

    .commands{
        text-align: left;
        display: inline-block;
    }
</style>