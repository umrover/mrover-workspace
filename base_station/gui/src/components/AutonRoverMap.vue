<template>
  <div class="wrap">
    <!-- Map goes here       -->
    <l-map ref="map" class="map" :zoom="15" :center="center" v-on:click="getClickedLatLon($event)">
      <l-control-scale :imperial="false"/>
      <l-tile-layer :url="this.online ? this.onlineUrl : this.offlineUrl" :attribution="attribution" :options="tileLayerOptions"/>
      <l-marker ref="tangent" :lat-lng="this.playbackEnabled ? this.playbackPath[this.playbackPath.length-1] : odomLatLng" :icon="tangentIcon"/>
      <l-marker ref="target_bearing" :lat-lng="this.playbackEnabled ? this.playbackPath[this.playbackPath.length-1] : odomLatLng" :icon="targetBearingIcon"/>
      <l-marker ref="rover" :lat-lng="this.playbackEnabled ? this.playbackPath[this.playbackPath.length-1] : odomLatLng" :icon="locationIcon"/>

      <l-marker :lat-lng="waypoint.latLng" :icon="waypointIcon" v-for="(waypoint, index) in waypointList" :key="index">
         <l-tooltip :options="{permanent: 'true', direction: 'top'}"> {{ waypoint.name }}, {{ index }} </l-tooltip>
      </l-marker>

      <l-marker :lat-lng="projected_point.latLng" :icon="projectedPointIcon" v-for="(projected_point, index) in projectedPoints" :key="index">
         <l-tooltip :options="{permanent: 'true', direction: 'top'}">{{ projectedPointsType }} {{ index }}</l-tooltip>
      </l-marker>

      <l-marker :lat-lng="post1" :icon="postIcon" v-if="post1">
         <l-tooltip :options="{permanent: 'true', direction: 'top'}">Post 1</l-tooltip>
      </l-marker>
      <l-marker :lat-lng="post2" :icon="postIcon" v-if="post2">
         <l-tooltip :options="{permanent: 'true', direction: 'top'}">Post 2</l-tooltip>
      </l-marker>

      <l-polyline :lat-lngs="this.playbackEnabled ? polylinePlaybackPath : polylinePath" :color="'red'" :dash-array="'5, 5'"/>
      <l-polyline :lat-lngs="projectedPath" :color="'black'" :dash-array="'5, 5'" :fill="false"/>
      <l-polyline :lat-lngs="odomPath" :color="'blue'"/>
      <l-polyline :lat-lngs="playbackPath" :color="'green'"/>
    </l-map>
    <div class="controls">
      <div v-if="this.playbackEnabled">
        <input type="range" min="0" :max='this.playbackLength-1' value ='0' v-model='playbackSlider'>
      </div>
      <div class="online">
        <label><input type="checkbox" v-model="online" />Online</label>
      </div> 
    </div>
  </div>
</template>

<script>
import { LMap, LTileLayer, LMarker, LPolyline, LPopup, LTooltip, LControlScale } from 'vue2-leaflet'
import { mapGetters, mapMutations } from 'vuex'
import L from '../leaflet-rotatedmarker.js'

const MAX_ODOM_COUNT = 1000
const DRAW_FREQUENCY = 10
const onlineUrl = 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png'
const offlineUrl = '/static/map/{z}/{x}/{y}.png'

export default {
  name: 'RoverMap',

  components: {
    LMap,
    LTileLayer,
    LMarker,
    LPolyline,
    LPopup,
    LTooltip,
    LControlScale
  },

  created: function () {
    this.locationIcon = L.icon({
      iconUrl: '/static/location_marker_icon.png',
      iconSize: [40, 40],
      iconAnchor: [20, 20]
    })
    this.tangentIcon = L.icon({
      iconUrl: '/static/gps_tangent_icon.png',
      iconSize: [44, 80],
      iconAnchor: [22, 60]
    })
    this.targetBearingIcon = L.icon({
      iconUrl: '/static/gps_tangent_icon.png',
      iconSize: [30, 56],
      iconAnchor: [15, 42]
    })
    this.waypointIcon = L.icon({
      iconUrl: '/static/map_marker.png',
      iconSize: [64, 64],
      iconAnchor: [32, 64],
      popupAnchor: [0, -32]
    })
    this.projectedPointIcon = L.icon({
      iconUrl: '/static/map_marker_projected.png',
      iconSize: [64, 64],
      iconAnchor: [32, 64],
      popupAnchor: [0, -32]
    })
    this.postIcon = L.icon({
      iconUrl: '/static/gate_location.png',
      iconSize: [64, 64],
      iconAnchor: [32, 64],
      popupAnchor: [0, -32]
    })

    this.$parent.subscribe('/projected_points', (msg) => {
      let newProjectedList = msg.points
      this.projectedPoints = newProjectedList.map((projected_point) => {
        return {
          latLng: L.latLng(
            projected_point.latitude_deg + projected_point.latitude_min/60,
            projected_point.longitude_deg + projected_point.longitude_min/60
          )
        }
      })

      this.projectedPointsType = msg.path_type
    })

    this.$parent.subscribe('/estimated_gate_location', (msg) => {
      this.post1 =  L.latLng(
        msg.post1.latitude_deg + msg.post1.latitude_min/60,
        msg.post1.longitude_deg + msg.post1.longitude_min/60
      )
      this.post2 = L.latLng(
        msg.post2.latitude_deg + msg.post2.latitude_min/60,
        msg.post2.longitude_deg + msg.post2.longitude_min/60
      )
    })

  },

  computed: {
    ...mapGetters('autonomy', {
      route: 'route',
      waypointList: 'waypointList',
      playbackEnabled: 'playbackEnabled',
      playbackLength: 'playbackLength',
      playback: 'playback',
      playbackOdomLat: 'playbackOdomLat',
      playbackOdomLon: 'playbackOdomLon',
      playbackOdomBearing: 'playbackOdomBearing',
      playbackGpsBearing: 'playbackGpsBearing',
      playbackTargetBearing: 'playbackTargetBearing'
    }),

    odomLatLng: function () {
      return L.latLng(this.odom.latitude_deg + this.odom.latitude_min/60, this.odom.longitude_deg + this.odom.longitude_min/60)
    },

    polylinePath: function () {
      return [this.odomLatLng].concat(this.route.map(waypoint => waypoint.latLng))
    },

    projectedPath: function () {
      return [this.odomLatLng].concat(this.projectedPoints.map(projected_point => projected_point.latLng))
    },

    polylinePlaybackPath: function () {
      return this.route.map(waypoint => waypoint.latLng)
    }
  },

  data () {
    return {
      center: L.latLng(38.406371, -110.791954),
      attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors',
      online: true,
      onlineUrl: onlineUrl,
      offlineUrl: offlineUrl,
      roverMarker: null,
      waypointIcon: null,
      map: null,
      odomCount: 0,
      locationIcon: null,
      odomPath: [],

      tangentMarker: null,
      tangentIcon: null,

      targetBearingMarker: null,
      targetBearingIcon: null,

      projectedPoints: [],
      projectedPointsType: '',

      post1: null,
      post2: null,

      findRover: false,

      playbackPath: [],
      playbackSlider: 0,

      options: {
        type: Object,
        default: () => ({})
      },
      tileLayerOptions: {
        maxNativeZoom: 18,
        maxZoom: 100
      }
    }
  },

  props: {
    odom: {
      type: Object,
      required: true
    },
    GPS: {
      type: Object,
      required: true
    },
    TargetBearing: {
      type: Object,
      required: true
    }
  },

  methods: {
    getClickedLatLon: function (e) {
      this.setClickPoint(
          { 
            lat: e.latlng.lat,
            lon: e.latlng.lng
          }
        )
    },

    ...mapMutations('autonomy',{
      setClickPoint: 'setClickPoint',
      setWaypointList: 'setWaypointList',
      setAutonMode: 'setAutonMode',
      setOdomFormat: 'setOdomFormat'
    }),
  },


  watch: {
    odom: function (val) {
      // Trigger every time rover odom is changed

      const lat = val.latitude_deg + val.latitude_min / 60
      const lng = val.longitude_deg + val.longitude_min / 60
      const angle = val.bearing_deg

      const latLng = L.latLng(lat, lng)

      // Move to rover on first odom message
      if (!this.findRover) {
        this.findRover = true
        this.center = latLng
      }
      
      // Update the rover marker
      this.roverMarker.setRotationAngle(angle)

      this.roverMarker.setLatLng(latLng)
      this.tangentMarker.setLatLng(latLng)
      this.targetBearingMarker.setLatLng(latLng)

      // Update the rover path
      this.odomCount++
      if (this.odomCount % DRAW_FREQUENCY === 0) {
        if (this.odomCount > MAX_ODOM_COUNT * DRAW_FREQUENCY) {
          this.odomPath.splice(0, 1)
        }
        this.odomPath.push(latLng)
      }

      this.odomPath[this.odomPath.length - 1] = latLng
    },

    GPS: function (val) {
      this.tangentMarker.setRotationAngle(val.bearing_deg)
    },

    TargetBearing: function (val) {
      this.targetBearingMarker.setRotationAngle(val.target_bearing)
    },
    
    playbackEnabled: function(val) {
      this.odomPath = []
      this.playbackPath = []
      this.playbackSlider = 0

      if (val) {
        const latLng = L.latLng(this.playbackOdomLat[0], this.playbackOdomLon[0])
        this.center = latLng

        this.roverMarker.setRotationAngle(this.playbackOdomBearing[0])
        this.roverMarker.setLatLng(latLng)

        this.tangentMarker.setRotationAngle(this.playbackGpsBearing[0])
        this.tangentMarker.setLatLng(latLng)

        this.targetBearingMarker.setRotationAngle(this.playbackTargetBearing[0])
        this.targetBearingMarker.setLatLng(latLng)
      }
    },

    playbackSlider: function (val) {
      const latLng = L.latLng(this.playbackOdomLat[val], this.playbackOdomLon[val])

      this.roverMarker.setRotationAngle(this.playbackOdomBearing[val])
      this.roverMarker.setLatLng(latLng)

      this.tangentMarker.setRotationAngle(this.playbackGpsBearing[val])
      this.tangentMarker.setLatLng(latLng)

      this.targetBearingMarker.setRotationAngle(this.playbackTargetBearing[val])
      this.targetBearingMarker.setLatLng(latLng)

      let length_diff = val - this.playbackPath.length

      if (length_diff > 0) {
        for (let i = this.playbackPath.length; i < val; i++) {
          this.playbackPath.push(L.latLng(
            this.playbackOdomLat[i],
            this.playbackOdomLon[i]
          ))
        }
      }
      else if (length_diff < 0) {
        this.playbackPath.splice(val, -1*length_diff)
      }
    }
  },

  mounted: function () {
    this.$nextTick(() => {
      this.map = this.$refs.map.mapObject
      this.roverMarker = this.$refs.rover.mapObject
      this.tangentMarker = this.$refs.tangent.mapObject
      this.targetBearingMarker = this.$refs.target_bearing.mapObject
    })
  }
}
</script>

<style scoped>
.map {
  height: 100%;
  width: 100%;
  grid-area: "map";
}

.controls {
  grid-area: "controls";
  display: inline;
}

.controls label{
  font-size: 12px;
}

.controls div{
  display: inline-block;
}

.online{
  float:right;
}

.wrap {
  align-items: center;
  height: 100%;
  display: grid;
  overflow:hidden;
  min-height: 100%;
  grid-gap: 3px;
  grid-template-columns: 1fr;
  grid-template-rows: 94% 6%;
  grid-template-areas:"map" 
                      "controls";
}
.custom-tooltip {
    display: inline-block;
    margin: 10px 20px;
    opacity: 1;
    position: relative;
}

.custom-tooltip .tooltip-inner {
	background: #0088cc;
}

.custom-tooltip.top .tooltip-arrow {
	border-top-color: #0088cc;
}


</style>
