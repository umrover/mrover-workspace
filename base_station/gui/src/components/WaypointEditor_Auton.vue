<template>
  <div class="wrap">
    <div class="box">
      <div class="identification">
        Name: <input v-model="name" v-on:click="select($event)">
        ID: <input v-model="id" type="number" max="249" min="-1" step="1">
      </div>
      <br>
      <input type="radio" v-model="odom_format_in" value="D" class="checkbox"><font size="2">D</font>
      <input type="radio" v-model="odom_format_in" value="DM" class="checkbox"><font size="2">DM</font>
      <input type="radio" v-model="odom_format_in" value="DMS" class="checkbox"><font size="2">DMS</font><br>
      <div class="wp-input">
        <p><input v-model.number="input.lat.d" size="13" v-on:click="select($event)">º</p>
        <p v-if="this.min_enabled"><input v-model.number="input.lat.m" size="13" v-on:click="select($event)">'</p>
        <p  v-if="this.sec_enabled"><input v-model.number="input.lat.s" size="13" v-on:click="select($event)">"</p>
        N
      </div>
      <div class="wp-input">
        <p><input v-model.number="input.lon.d" size="13" v-on:click="select($event)">º</p>
        <p v-if="this.min_enabled"><input v-model.number="input.lon.m" size="13" v-on:click="select($event)">'</p>
        <p  v-if="this.sec_enabled"><input v-model.number="input.lon.s" size="13" v-on:click="select($event)">"</p>
        W
      </div>
      <br>
      <button v-on:click="addWaypoint(input)">Add Waypoint</button>
      <button v-on:click="addWaypoint(formatted_odom)">Drop Waypoint</button>
    </div>
    <div class="box">
      <Checkbox ref="checkbox" v-bind:name="'Autonomy Mode'" v-on:toggle="toggleAutonMode($event) "/><br>
      <span>
        Navigation State: {{nav_status.nav_state_name}}<br>
        Waypoints Traveled: {{nav_status.completed_wps}}/{{nav_status.total_wps}}<br>
      </span>
    </div>
    <div class="box1">
      <h3>All Waypoints</h3>
      <draggable v-model="storedWaypoints" class="dragArea" draggable=".item'">
        <WaypointItem v-for="waypoint, i in storedWaypoints" :key="i" v-bind:waypoint="waypoint" v-bind:list="0" v-bind:index="i" v-on:delete="deleteItem($event)" v-on:toggleSearch="toggleSearch($event)" v-on:toggleGate="toggleGate($event)" v-on:add="addItem($event)"/>
      </draggable>
    </div>
    <div class="box1">
      <h3>Current Course</h3>
      <draggable v-model="route" class="dragArea" draggable=".item'">
        <WaypointItem v-for="waypoint, i in route" :key="i" v-bind:waypoint="waypoint" v-bind:list="1" v-bind:index="i" v-bind:name="name" v-bind:id="id" v-on:delete="deleteItem($event)" v-on:toggleSearch="toggleSearch($event)" v-on:toggleGate="toggleGate($event)" v-on:add="addItem($event)"/>
      </draggable>
    </div>
  </div>
</template>

<script>
import Checkbox from './Checkbox.vue'
import draggable from 'vuedraggable'
import {convertDMS} from '../utils.js';
import WaypointItem from './WaypointItem_Auton.vue'
import {mapMutations, mapGetters} from 'vuex'
import _ from 'lodash';
import fnvPlus from 'fnv-plus';
import L from 'leaflet'

let interval;

export default {

  props: {
    odom: {
      type: Object,
      required: true
    },
  },

  data () {
    return {
      name: "Waypoint",
      id: -1,
      odom_format_in: 'DM',
      input: {
        lat: {
          d: 0,
          m: 0,
          s: 0
        },
        lon: {
          d: 0,
          m: 0,
          s: 0
        }
      },

      nav_status: {
        nav_state_name: "Off",
        completed_wps: 0,
        total_wps: 0
      },

      storedWaypoints: [],
      route: []
    }
  },

  beforeDestroy: function () {
    window.clearInterval(interval);
  },

  created: function () {

    this.$parent.subscribe('/nav_status', (msg) => {
      this.nav_status = msg
    })

    interval = window.setInterval(() => {
        if(this.auton_enabled && this.nav_status.nav_state_name === 'Done'){
          this.$refs.checkbox.toggleAndEmit()
        }

        this.$parent.publish('/auton', {type: 'AutonState', is_auton: this.auton_enabled})

        let course = {
            num_waypoints: this.route.length,
            waypoints: _.map(this.route, (waypoint) => {
              const lat = waypoint.lat.d + waypoint.lat.m/60 + waypoint.lat.s/3600;
              const lon = waypoint.lon.d + waypoint.lon.m/60 + waypoint.lon.s/3600;
              const latitude_deg = Math.trunc(lat)
              const longitude_deg = Math.trunc(lon)
              return {
                  type: "Waypoint",
                  search: waypoint.search,
                  gate: waypoint.gate,
                  id: Math.trunc(waypoint.id),
                  odom: {
                      latitude_deg: latitude_deg,
                      latitude_min: (lat - latitude_deg) * 60,
                      longitude_deg: -longitude_deg,
                      longitude_min: (lon - longitude_deg) * 60,
                      bearing_deg: 0,
                      speed: -1,
                      type: "Odometry"
                  },
              }
            })
        };
        course.hash = fnvPlus.fast1a52(JSON.stringify(course));
        course.type = 'Course'
        this.$parent.publish('/course', course)

    }, 100);
  },

  methods: {
    ...mapMutations('autonomy',{
      setRoute: 'setRoute',
      setWaypointList: 'setWaypointList',
      setAutonMode: 'setAutonMode',
      setOdomFormat: 'setOdomFormat'
    }),

    deleteItem: function (payload) {
      if(payload.list === 0) {
        this.storedWaypoints.splice(payload.index, 1)
      } else if(payload.list === 1) {
        this.route.splice(payload.index, 1)
      }
    },

    // Add item from all waypoints div to current waypoints div
    addItem: function (payload) {
       if(payload.list === 0) {
        this.route.push(this.storedWaypoints[payload.index])
      } else if(payload.list === 1) {
        this.storedWaypoints.push(this.route[payload.index])
      }
    },

    toggleSearch: function (payload) {
      if(payload.list === 0) {
        this.storedWaypoints[payload.index].search = !this.storedWaypoints[payload.index].search
      } else if(payload.list === 1) {
        this.route[payload.index].search = !this.route[payload.index].search
      }
    },

    toggleGate: function (payload) {
      if(payload.list === 0) {
        this.storedWaypoints[payload.index].gate = !this.storedWaypoints[payload.index].gate
      } else if(payload.list === 1) {
        this.route[payload.index].gate = !this.route[payload.index].gate
      }
    },

    addWaypoint: function (coord) {
      this.storedWaypoints.push({
        name: this.name,
        id: this.id,
        lat: Object.assign({}, coord.lat),
        lon: Object.assign({}, coord.lon),
        search: false,
        gate: false
      });
    },

    select: function(payload) {
      console.log(payload.toElement);
      payload.toElement.select();
    },

    toggleAutonMode: function (val) {
      this.setAutonMode(val)
    }
  },

  watch: {
    route: function (newRoute) {
      const waypoints = newRoute.map((waypoint) => {
        const lat = waypoint.lat.d + waypoint.lat.m/60 + waypoint.lat.s/3600;
        const lon = waypoint.lon.d + waypoint.lon.m/60 + waypoint.lon.s/3600;
        return { latLng: L.latLng(lat, lon) };
      });
      this.setRoute(waypoints);
    },

    storedWaypoints: function (newList) {
      const waypoints = newList.map((waypoint) => {
        const lat = waypoint.lat.d + waypoint.lat.m/60 + waypoint.lat.s/3600;
        const lon = waypoint.lon.d + waypoint.lon.m/60 + waypoint.lon.s/3600;
        return { latLng: L.latLng(lat, lon) };
      });
      this.setWaypointList(waypoints);
    },

    odom_format_in: function (newOdomFormat) {
      this.setOdomFormat(newOdomFormat);
      this.input.lat = convertDMS(this.input.lat, newOdomFormat);
      this.input.lon = convertDMS(this.input.lon, newOdomFormat);
      this.storedWaypoints.map((waypoint) => {
        waypoint.lat = convertDMS(waypoint.lat, newOdomFormat);
        waypoint.lon = convertDMS(waypoint.lon, newOdomFormat);
        return waypoint;
      });
      this.route.map((waypoint) => {
        waypoint.lat = convertDMS(waypoint.lat, newOdomFormat);
        waypoint.lon = convertDMS(waypoint.lon, newOdomFormat);
        return waypoint;
      });
    }
  },

  computed: {
    ...mapGetters('autonomy', {
      auton_enabled: 'autonEnabled',
      odom_format: 'odomFormat'
    }),

    formatted_odom: function() {
      return {
        lat: convertDMS({d: this.odom.latitude_deg, m: this.odom.latitude_min, s: 0}, this.odom_format),
        lon: convertDMS({d: -this.odom.longitude_deg, m: -this.odom.longitude_min, s: 0}, this.odom_format)
      }
    },

    min_enabled: function() {
      return this.odom_format != 'D';
    },

    sec_enabled: function() {
      return this.odom_format == 'DMS';
    }
  },

  components: {
    draggable,
    WaypointItem,
    Checkbox
  }

}
</script>

<style scoped>

  .wrap {
    display: grid;
    grid-template-columns: 1fr 1fr;
    grid-template-rows: 10.5rem 1fr;
    grid-gap: 6px;
    height: 100%;
  }

  .dragArea {
    height: 100%;
  }

  .identification{
    display: inline-block;
  }

  .box {
    border-radius: 5px;
    padding: 10px;
    border: 1px solid black;
    height: 150px;
    overflow: auto;
  }
    .box1 {
    border-radius: 5px;
    padding: 10px;
    border: 1px solid black;
    overflow: auto;
  }

  .wp-input p {
    display: inline;
  }

</style>
