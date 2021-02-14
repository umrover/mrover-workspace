<template>
<div class="wrap">
    <div>
        <h3> Ammonia Controls </h3>
    </div>
    <div class="box">
        <button>
            Up
        </button>
        <button>
            Down
        </button>
        <button>
            Stop
        </button>
    </div>
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
</style>

<script>
export default {
    data () {
        return {

        }
    }
}
</script>