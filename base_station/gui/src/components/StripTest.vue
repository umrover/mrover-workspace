<template>
<div class="wrap">
    <div>
        <h3> Strip Test Controls </h3>
    </div>
    <div class="controls">
        <div class="strip_test">
            <label for="strip">Perform strip test on strip:</label>
                <select v-model = "strip" name="strip" id="strip">
                    <option value=2>A</option>
                    <option value=1>B</option>
                    <option value=0>C</option>
                </select>
            <div class="commands" v-if="strip == 2">
                <button v-on:click="stripTestA()"> Start Strip A Test </button>
            </div>
            <div class="commands" v-if="strip == 1">
                <button v-on:click="stripTestB()"> Start Strip B Test </button>
            </div>
            <div class="commands" v-if="strip == 0">
                <button v-on:click="stripTestC()"> Start Strip C Test </button>
            </div>
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

// const undipped = '#A54657';
// const dipping = '#FABC2A';
// const dipped = '#AEF78E';


export default {
    data () {
        return {
            strip: 2,
            angle0: serv0_start,
            angle1: serv1_start,
            angle2: serv2_start,
            serv2_start: serv2_start,
            serv2_dip: serv2_dip,
            serv1_start: serv1_start,
            serv1_dip: serv1_dip,
            serv0_start: serv0_start,
            serv0_dip: serv0_dip,
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

        stripTestA: function() {
            this.angle2 = this.serv2_dip;
            this.setPart();
            setTimeout(() => {
                this.angle2 = this.serv2_start;
                // this.serv2_has_dip = true;
                this.setPart();
                }, 10000);
        },

        stripTestB: function() {
            this.angle1 = this.serv1_dip;
            this.setPart();
            setTimeout(() => {
                this.angle1 = this.serv1_start;
                // this.serv1_has_dip = true;
                this.setPart();
                }, 10000);
        },
        
        stripTestC: function() {
            this.angle0 = this.serv0_dip;
            this.setPart();
            setTimeout(() => {
                this.angle0 = this.serv0_start;
                // this.serv0_has_dip = true;
                this.setPart();
                }, 10000);
        },

        // changeStripColor: function(event) {
        //     if (event.target.value == 2) {
        //         if (angle2 == 100) {
        //             if (serv2_has_dip) {
        //                 this.background_color = this.dipped;
        //             } else {
        //                 this.background_color = this.undipped;
        //             }
        //         } else if (angle2 == 30) {
        //             this.background_color = this.dipping;
        //         }
        //     } else if (event.target.value == 1) {
        //         if (angle1 == 100) {
        //             if (serv1_has_dip) {
        //                 this.background_color = this.dipped;
        //             } else {
        //                 this.background_color = this.undipped;
        //             }
        //         } else if (angle1 == 30) {
        //             this.background_color = this.dipping;
        //         }
        //     } else {
        //         if (angle0 == 80) {
        //             if (serv0_has_dip) {
        //                 this.background_color = this.dipped;
        //             } else {
        //                 this.background_color = this.undipped;
        //             }
        //         } else if (angle0 == 10) {
        //             this.background_color = this.dipping;
        //         }
        //     }
        // }
    },
    // watch: {
    //     strip: function (val) {
            
    //     }
    // }
}
</script>

<style scoped>
    .wrap {
        display: inline-block;
        align-content: center;
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