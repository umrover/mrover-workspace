<template>
  <div class="wrap">
    <div class="box">
      <div class="top">
        Name: <input v-model="name">
        <input type="checkbox" v-model="convert" value="yes" class="checkbox"><font size="2">Convert</font><br>
      </div>
      <br>
      <input type="radio" v-model="input_fields" value="D" class="checkbox"><font size="2">In Degrees</font><br>
      <input type="radio" v-model="input_fields" value="DM" class="checkbox"><font size="2">In Degree Minutes</font><br>
        <input type="radio" v-model="input_fields" value="DMS" class="checkbox"><font size="2">In Degree Minutes Seconds</font><br>
        <input type="radio" v-model="convert_fields" value="convertD" class="checkbox"><font size="2">Convert to Degrees</font><br>
        <input type="radio" v-model="convert_fields" value="convertDM" class="checkbox"><font size="2">Convert to Degrees Minutes</font><br>
        <input type="radio" v-model="convert_fields" value="convertDMS" class="checkbox"><font size="2">Convert to Degrees Minutes Seconds</font><br>
        <input v-model="lat" size="15">ºN 
        <input v-model="lat_mins" size="15">' 
        <input v-model="lat_sec" size="15">"<br>
        <input v-model="lon" size="15">ºW 
        <input v-model="lon_mins" size="15">' 
        <input v-model="lon_sec" size="15">"<br>
      <button v-on:click="parseWaypoint()">Add Waypoint</button>
      <button v-on:click="dropWaypoint()">Drop Waypoint</button>
    </div>
    <div class="box">
      <Checkbox ref="checkbox" v-bind:name="'Autonomy Mode'" v-on:toggle="toggleAutonMode($event) "/><br>
      <span>
        Navigation State: {{nav_status.nav_state_name}}<br>
        Waypoints Traveled: {{nav_status.completed_wps}}/{{nav_status.total_wps}}<br>
        Missed Waypoints: {{nav_status.missed_wps}}/{{nav_status.total_wps}}<br>
        Tennis Balls: {{nav_status.found_tbs}}/{{nav_status.total_tbs}}
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
        <WaypointItem v-for="waypoint, i in route" :key="i" v-bind:waypoint="waypoint" v-bind:list="1" v-bind:index="i" v-on:delete="deleteItem($event)" v-on:toggleSearch="toggleSearch($event)" v-on:toggleGate="toggleGate($event)" v-on:add="addItem($event)"/>
      </draggable>
    </div>
  </div>
</template>

<script>
import Checkbox from './Checkbox.vue'
import draggable from 'vuedraggable'
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
      name: "",
      lon: "",
      lon_mins: "",
      lon_sec: "",
      lat: "",
      lat_mins: "",
      lat_sec: "",
      convert_fields: "convertD",
      input_fields: "D",

      nav_status: {
        nav_state_name: "Off",
        completed_wps: 0,
        missed_wps: 0,
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
              let lat = waypoint.latLng.lat
              let lng = waypoint.latLng.lng
              let latitude_deg = Math.trunc(lat)
              let latitude_min = (lat - latitude_deg) * 60
              let longitude_deg = Math.trunc(lng)
              let longitude_min = (lng - longitude_deg) * 60
              return {
                  type: "Waypoint",
                  search: waypoint.search,
                  gate: waypoint.gate,
                  odom: {
                      latitude_deg: latitude_deg,
                      latitude_min: latitude_min,
                      longitude_deg: longitude_deg,
                      longitude_min: longitude_min,
                      bearing_deg: 0,
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
      setAutonMode: 'setAutonMode'
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

    addWaypoint: function (lat, lat_mins, lat_sec, lon, lon_mins, lon_sec) {
        this.storedWaypoints.push({
          name: this.name,
          latLng: L.latLng(lat, lon),
          lat_min: lat_mins,
          lat_sec: lat_sec,
          lon_min: lon_mins,
          lon_sec: lon_sec,
          search: false,
          gate: false,
        })

    },

    dropWaypoint: function () {
      if(this.input_fields == "D"){ // given and want in degrees
        this.addWaypoint(this.odom.latitude_deg+(this.odom.latitude_min/60), 0, 0, this.odom.longitude_deg+(this.odom.longitude_min/60),0,0)
      }
      else if(this.input_fields == "DM"){ // given and want in degrees minutes
        this.addWaypoint(this.odom.latitude_deg, this.odom.latitude_min, 0, this.odom.longitude_deg, -this.odom.longitude_min, 0)
      }
      else if(this.input_fields == "DMS"){ // given and want in degrees minutes seconds
        this.addWaypoint(this.odom.latitude_deg, Math.floor(this.odom.latitude_min), ((this.odom.latitude_min%1)*60).toFixed(4), this.odom.longitude_deg, Math.floor(-this.odom.longitude_min), ((-this.odom.longitude_min%1)*60).toFixed(4))
      }

    },

    parseWaypoint: function () {

      const parseCoordinate = function (input) {
        const nums = input.split(" ")
        switch (nums.length) {
          case 1:
             return parseFloat(nums[0])
          case 2:
            return parseFloat(nums[0]) + parseFloat(nums[1])/60
          case 3:
            return parseFloat(nums[0]) + parseFloat(nums[1])/60 + parseFloat(nums[2])/3600
          default:
            return 0
        }
      }

      if(this.input_fields == "D"){ // in degrees
         this.addWaypoint(parseCoordinate(this.lat), 0, 0, -parseCoordinate(this.lon), 0, 0)
      }
      else if(this.input_fields == "DM"){ // in degrees
         this.addWaypoint(parseCoordinate(this.lat), parseCoordinate(this.lat_mins), 0, -parseCoordinate(this.lon), parseCoordinate(this.lon_mins), 0)
      }
      else if(this.input_fields == "DMS"){ // in degrees
         this.addWaypoint(parseCoordinate(this.lat), parseCoordinate(this.lat_mins), parseCoordinate(this.lat_sec), -parseCoordinate(this.lon), parseCoordinate(this.lon_mins), parseCoordinate(this.lon_sec))
      }

      if(this.convert.checked == true){
        if(this.convert_fields == "convertD"){ // convert to degrees
         this.addWaypoint(parseCoordinate(this.lat) + parseCoordinate(this.lat_mins)/60 + parseCoordinate(this.lat_sec)/3600, 0, 0, -parseCoordinate(this.lon) + -parseCoordinate(this.lon_mins)/60 + -parseCoordinate(this.lon_sec)/3600, 0, 0)
        }
        else if(this.convert_fields == "convertDM"){ // convert to degrees
          this.addWaypoint(parseCoordinate(this.lat) + parseCoordinate(this.lat_mins)/60 + parseCoordinate(this.lat_sec)/3600, 0, 0, -parseCoordinate(this.lon) + -parseCoordinate(this.lon_mins)/60 + -parseCoordinate(this.lon_sec)/3600, 0, 0)
        }
        else if(this.convert_fields == "convertDMS"){ // convert to degrees
          this.addWaypoint(parseCoordinate(this.lat) + parseCoordinate(this.lat_mins)/60 + parseCoordinate(this.lat_sec)/3600, 0, 0, -parseCoordinate(this.lon) + -parseCoordinate(this.lon_mins)/60 + -parseCoordinate(this.lon_sec)/3600, 0, 0)
        }
      }
    },

   toggleAutonMode: function (val) {
      this.setAutonMode(val)
    },
   },

  watch: {
    route: function (newRoute) {
      this.setRoute(newRoute)
    },

    storedWaypoints: function (newList) {
      this.setWaypointList(newList)
    }
  },

  computed: {
    ...mapGetters('autonomy', {
      auton_enabled: 'autonEnabled',
    })
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

  .top{
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

</style>
