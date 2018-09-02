<template>
  <div class="wrap">
    <div class="box">
      Name: <input v-model="name"><br>
      <input v-model="lat">ºN
      <input v-model="lon">ºW
      <button v-on:click="addWaypoint(lat, lon)">Add Waypoint</button>
    </div>
    <div class="box">
      <Checkbox name="Autonomy Mode" bind:toggled="auton_enabled"/><br>
      <span>
        Navigation State: {{nav_state}}<br>
        Waypoints Traveled: {{nav_status.completed_wps}}/{{nav_status.total_wps}}<br>
        Missed Waypoints: {{nav_status.missed_wps}}/{{nav_status.total_wps}}
      </span>
    </div>
    <div class="box">
      <draggable v-model="storedWaypoints" class="dragArea" :options="{scroll: true, group: 'waypoints'}">
        <WaypointItem v-for="waypoint, i in storedWaypoints" :key="i" v-bind:waypoint="waypoint" v-bind:list="0" v-bind:index="i" v-on:delete="deleteItem($event)"/>
      </draggable>
    </div>
    <div class="box">
      <draggable v-model="route" class="dragArea" :options="{scroll: true, group: 'waypoints'}">
        <WaypointItem v-for="waypoint, i in route" :key="i" v-bind:waypoint="waypoint" v-bind:list="1" v-bind:index="i" v-on:delete="deleteItem($event)"/>
      </draggable>
    </div>
  </div>
</template>

<script>
import draggable from 'vuedraggable'
import WaypointItem from './WaypointItem.vue'
import {mapMutations} from 'vuex'

export default {

  data () {
    return {
      lon: "",
      lat: "",

      storedWaypoints: [
        {
          name: 'Waypoint 1',
          latLng: L.latLng(38.416371, -110.791954)
        },
        {
          name: 'Waypoint 2',
          latLng: L.latLng(38.406371, -110.781954)
        },
        {
          name: 'Waypoint 3',
          latLng: L.latLng(38.416371, -110.781954)
        },
        {
          name: 'Waypoint 4',
          latLng: L.latLng(38.416371, -110.791954)
        },
        {
          name: 'Waypoint 5',
          latLng: L.latLng(38.406371, -110.781954)
        },
        {
          name: 'Waypoint 6',
          latLng: L.latLng(38.416371, -110.781954)
        },
        {
          name: 'Waypoint 7',
          latLng: L.latLng(38.416371, -110.791954)
        },
        {
          name: 'Waypoint 8',
          latLng: L.latLng(38.406371, -110.781954)
        },
        {
          name: 'Waypoint 9',
          latLng: L.latLng(38.416371, -110.781954)
        },
        {
          name: 'Waypoint 10',
          latLng: L.latLng(38.416371, -110.791954)
        },
        {
          name: 'Waypoint 11',
          latLng: L.latLng(38.406371, -110.781954)
        },
        {
          name: 'Waypoint 12',
          latLng: L.latLng(38.416371, -110.781954)
        }
      ],
      route: []
    }
  },

  methods: {
    ...mapMutations('autonomy',{
      setRoute: 'setRoute'
    }),

    deleteItem: function (payload) {
      if(payload.list === 0) {
        this.storedWaypoints.splice(payload.index, 1)
      } else if(payload.list === 1) {
        this.route.splice(payload.index, 1)
      }
    },

    addWaypoint: function (lat, lon) {
      const parseCoordinate = function (input) {
        const nums = input.split(" ")
        switch (nums.length) {
          case 1:
            return parseFloat(nums[0])
          case 2:
            return parseFloat(nums[0]) + parseFloat(nums[1])/60
          case 3:
            return parseFloat(nums[0]) + parseFloat(nums[1])/60 + parseFloat(nums[2])/360
          default:
            return 0
        }
      }

      this.storedWaypoints.push({
        name: name,
        latLng: L.latLng(parseCoordinate(lat), -parseCoordinate(lon))
      })
    }
  },

  watch: {
    route: function (newRoute) {
      this.setRoute(newRoute)
    }
  },

  components: {
    draggable,
    WaypointItem
  }

}
</script>

<style scoped>
  
  .wrap {
    display: grid;
    grid-template-columns: 1fr 1fr;
    grid-template-rows: 5rem 1fr;
    grid-gap: 6px;
    height: 100%;
  }

  .dragArea {
    height: 100%;
  }

  .box {
    border-radius: 5px;
    padding: 10px;
    border: 1px solid black;

    overflow: scroll;
  }
</style>