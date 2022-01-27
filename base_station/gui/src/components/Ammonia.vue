<template>
<div class="wrap">
    <div>
        <h3> Strip Test Controls </h3>
    </div>
    <div class="controls">
        <div class="strip_test">
            <label for="strip">Perform strip test on strip:</label>
                <select v-model = "strip" name="strip" id="strip">
                    <option value=0>A</option>
                    <option value=1>B</option>
                    <option value=2>C</option>
                </select>
            <div class="commands" v-if="strip == 0">
                <button v-on:click="stripTestA()"> Start Strip A Test </button>
            </div>
            <div class="commands" v-if="strip == 1">
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
export default {
    data () {
        return {
            strip: 0,
            servo_0: false,
            servo_1: false,
            servo_2: false
        }
    },
    
    created:
        function () {
            this.$parent.subscribe('/servo_cmd', (msg) => {
                this.servo_0 = msg.servo_0
                this.servo_1 = msg.servo_1
                this.servo_2 = msg.servo_2
            })
        },

    methods: {

        stripTestA: function() {
            alert("Strip A test started");
            this.servo_0 = true;
            setTimeout(() => {
                alert("Taking strips out");
                this.servo_0 = false;
                }, 10000);
            setTimeout(() => {
                alert("Strip test completed");
                }, 15000);
        },

        stripTestB: function() {
            alert("Strip B Test Started");
            this.servo_1 = true;
            setTimeout(() => {
                alert("Taking strips out");
                this.servo_1 = false;
                }, 10000);
            setTimeout(() => {
                alert("Strip test completed")
                }, 15000);
        },
        
        stripTestC: function() {
            alert("Strip C Test Started");
            this.servo_2 = true;
            setTimeout(() => {
                alert("Taking strips out");
                this.servo_2 = false;
                }, 10000);
            setTimeout(() => {
                alert("Strip test completed")
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