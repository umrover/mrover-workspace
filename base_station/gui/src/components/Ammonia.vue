<template>
<div class="wrap">
    <div>
        <h3> Strip Test Controls </h3>
    </div>
    <div class="controls">
        <div class="strip_test">
            <label for="strip">Perform strip test on strip:</label>
                <select v-model = "strip" name="strip" id="strip">
                    <option value=1>A</option>
                    <option value=0>B</option>
                    <option value=2>C</option>
                </select>
            <div class="commands" v-if="strip == 1">
                <button v-on:click="stripTestA()"> Start Strip A Test </button>
            </div>
            <div class="commands" v-if="strip == 0">
                <button v-on:click="stripTestB()"> Start Strip B Test </button>
            </div>
            <div class="commands" v-if="strip == 2">
                <button v-on:click="stripTestC()"> Start Strip C Test </button>
            </div>
        </div>
    </div>
</div>
</template>


<script>
const start = 0;
const dip = 10;
const picture = 20;

export default {
    data () {
        return {
            strip: 1,
            angle0: start,
            angle1: start,
            angle2: start,
            start: start,
            dip: dip,
            picture: picture
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
            alert("Strip A test started");
            this.angle1 = this.dip;
            this.setPart();
            setTimeout(() => {
                alert("Taking strips out");
                this.angle1 = this.picture;
                this.setPart();
                }, 10000);
            setTimeout(() => {
                this.angle1 = this.start;
                this.setPart()
                alert("Strip test completed");
                }, 15000);
        },

        stripTestB: function() {
            alert("Strip B Test Started");
            this.angle0 = this.dip;
            this.setPart();
            setTimeout(() => {
                alert("Taking strips out");
                this.angle0 = this.picture;
                this.setPart();
                }, 10000);
            setTimeout(() => {
                this.angle0 = this.start;
                this.setPart();
                alert("Strip test completed");
                }, 15000);
        },
        
        stripTestC: function() {
            alert("Strip C Test Started");
            this.angle2 = this.dip;
            this.setPart();
            setTimeout(() => {
                alert("Taking strips out");
                this.angle2 = this.picture;
                this.setPart();
                }, 10000);
            setTimeout(() => {
                this.angle2 = this.start;
                this.setPart();
                alert("Strip test completed");
                }, 15000);
        }
    }
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