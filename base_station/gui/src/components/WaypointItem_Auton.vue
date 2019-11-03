<template>
  <div class="waypoint-item">
    <div class="buttons">
      <button class="red" v-on:click="$emit('add', {'list': list, 'index': index})">Add</button>
      <button v-bind:class="[waypoint.search ? 'green' : 'red']" v-on:click="$emit('toggleSearch', {'list': list, 'index': index})">Search</button>
      <button v-bind:class="[waypoint.gate ? 'green' : 'red']" v-on:click="$emit('toggleGate', {'list': list, 'index': index})">Gate</button>
      <button class="red" v-on:click="$emit('delete', {'list': list, 'index': index})">Delete</button>
    </div>
    <div class="name">
      <p>{{waypoint.name}}</p>
    </div>
    <div class="location">
      <p>{{waypoint.latLng.lat}}ºN {{waypoint.lat_min}}' {{waypoint.lat_sec}}", {{-waypoint.latLng.lng}}ºW {{waypoint.lon_min}}' {{waypoint.lon_sec}}"</p>
    </div>
  </div>
</template>

<script>
export default {

  props: {
    waypoint: {
      type: Object,
      required: true
    },

    list: {
      type: Number,
      required: true
    },

    index: {
      type: Number,
      required: true
    }
  },
}
</script>

<style scoped>
  .waypoint-item {
    background-color: rgb(180, 180, 180);
    border-radius: 5px;
    padding: 10px;
    border: 1px solid black;

    margin: 5px;
  }
  .location {
    grid-area: location;
  }

  .buttons {
    grid-area: buttons;
    align-self: center;
    justify-self: center;
    display: block;
  }

  .red {
    background-color: red;
  }

  .green {
    background-color: green;
  }

  button {
    width: auto;
    height: auto;
    padding: 7px;
    font-weight: bold;
  }

  p {
    margin: 5px
  }
</style>