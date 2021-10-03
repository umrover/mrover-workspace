<template>
<div class="wrap">
    <div>
        <h3> Ammonia Controls </h3>
    </div>
    <div class="box">
        <button v-on:click="setPart(1)">
            Up
        </button>
        <button v-on:click="setPart(-1)">
            Down
        </button>
        <button v-on:click="setPart(0)">
            Stop
        </button>
    </div>
</div>
</template>
<!-- Reminder to attach github page for AmmoniaCmd to Teleop/ESW Interface google doc -->
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
</style>

<script>
export default {
    methods: {
        setPart: function(speed) {
        this.$parent.publish("/ammonia_cmd", {
            'type': 'AmmoniaCmd',
            'speed': speed
        })
        }
    }
}
</script>