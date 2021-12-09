<template>
<div class="wrap">
    <div>
        <h3> Strip Test Controls </h3>
    </div>
    <div class="box">
        <button v-on:click="setPart(1)">
            Up
        </button>
        <button v-on:click="setPart(-1)">
            Down
        </button>
    </div>
</div>
</template>
<!-- Reminder to attach github page for AmmoniaCmd to Teleop/ESW Interface google doc -->

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

<style scoped>
    .wrap {
        display: inline-block;
        align-content: center;
        /* height: 300px; */
    }
    .box {
        display: flex;
        border-radius: 5px;
        padding: 10px;
        border: 1px solid black;
        text-align: right;
        vertical-align: top;
        align-items: center;
        justify-content: center;
    }
</style>