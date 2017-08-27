<template>
    <div class="wrapper">
        <span class="led" :class="{ green: state, red: !state }">
        </span>
        <span class="name">
            {{ name }}
        </span>
    </div>
</template>

<script>
export default {
    props: ['state', 'name']
}
</script>

<style scoped>
.wrapper {
    display: flex;
    align-items: center;
    height: 100%;
}

.led {
    width: 16px;
    height: 16px;
    border-radius: 8px;
    border: 1px solid;
}

.green {
    background-color: lightgreen;
}

.red {
    background-color: red;
}

.name {
    margin-left: 5px;
}
</style>
