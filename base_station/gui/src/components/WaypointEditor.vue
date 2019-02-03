<template>
  <div class="wrap">
    <div class="box">
      Name: <input v-model="name"><br>
      <input v-model="lat">ºN
      <input v-model="lon">ºW
      <button v-on:click="parseWaypoint()">Add Waypoint</button>
      <button v-on:click="dropWaypoint()">Drop Waypoint</button>
    </div>
    <div class="box">
      <Checkbox ref="checkbox" v-bind:name="'Autonomy Mode'" v-on:toggle="toggleAutonMode($event) "/><br>
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
import Checkbox from './Checkbox.vue'
import draggable from 'vuedraggable'
import WaypointItem from './WaypointItem.vue'
import {mapMutations, mapGetters} from 'vuex'
import _ from 'lodash';
import fnvPlus from 'fnv-plus';

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
      lat: "",

      nav_state: "None",

      nav_status: {
        completed_wps: 0,
        total_wps: 0,
        missed_wps: 0
      },

      storedWaypoints: [],
      route: []
    }
  },

  created: function () {

    this.$parent.subscribe('/nav_status', (msg) => {
      this.nav_status = msg.message
      
      nav_state_textual: (nav_state) => {
          switch(nav_state) {
              case 0: return 'Off';
              case 1: return 'Done';
              case 10: return 'Turn';
              case 11: return 'Drive';
              case 20: return 'Search Face North';
              case 21: return 'Search Face 120';
              case 22: return 'Search Face 240';
              case 23: return 'Search Face 360';
              case 24: return 'Search Turn';
              case 25: return 'Search Drive';
              case 28: return 'Turn To Ball';
              case 29: return 'Drive To Ball';
              case 30: return 'Turn Around Obstacle';
              case 31: return 'Drive Around Obstacle';
              case 32: return 'Search Turn Around Obstacle';
              case 33: return 'Search Drive Around Obstacle';
              default: return 'Unknown';
          }
      },
      this.nav_state = nav_state_textual(msg.nav_state);
    })

    window.setInterval(() => {
        if(this.auton_enabled && this.nav_state === 'Done'){
          this.$refs.checkbox.toggleAndEmit()
        }

        this.$parent.publish('/auton', {type: 'AutonState', is_auton: this.auton_enabled})

        let course = {
            num_waypoints: this.route.length,
            waypoints: _.map(this.route, (waypoint) => {
                return {
                    type: "Waypoint",
                    search: waypoint.search,
                    odom: {
                        latitude_deg: waypoint.latitude_deg|0,
                        latitude_min: waypoint.latitude_min,
                        longitude_deg: waypoint.longitude_deg|0,
                        longitude_min: waypoint.longitude_min,
                        bearing_deg: 0,
                        type: "Odometry"
                    }
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

    addWaypoint: function (lat, lon) {
      this.storedWaypoints.push({
        name: this.name,
        latLng: L.latLng(lat, -lon)
      })
    },

    dropWaypoint: function () {
      let lat = this.odom.latitude_deg + this.odom.latitude_min/60
      let lon = this.odom.longitude_deg + this.odom.longitude_min/60
      this.addWaypoint(lat, lon)
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
            return parseFloat(nums[0]) + parseFloat(nums[1])/60 + parseFloat(nums[2])/360
          default:
            return 0
        }
      }

      this.addWaypoint(parseCoordinate(this.lat), parseCoordinate(this.lon))
    },

    toggleAutonMode(val){
      this.setAutonMode(val)
    }
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
    grid-template-rows: 7rem 1fr;
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