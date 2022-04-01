<template>
  <div class="wrap">
    <!-- Map goes here       -->
    <l-map ref="map" class="map" :zoom="15" :center="center">
      <l-control-scale :imperial="false"/>
      <l-tile-layer :url="url" :attribution="attribution" :options="tileLayerOptions"/>
      <l-marker ref="tangent" :lat-lng="this.playbackEnabled ? this.playbackPath[this.playbackPath.length-1] : odomLatLng" :icon="tangentIcon"/>
      <l-marker ref="rover" :lat-lng="this.playbackEnabled ? this.playbackPath[this.playbackPath.length-1] : odomLatLng" :icon="locationIcon"/>
      <l-marker :lat-lng="waypoint.latLng" :icon="waypointIcon" v-for="(waypoint,index) in route" :key="waypoint.id" >
        <l-tooltip :options="{ permanent: 'true', direction: 'top'}"> {{ waypoint.name }}, {{ index }} </l-tooltip>
      </l-marker>

      <l-marker :lat-lng="waypoint.latLng" :icon="waypointIcon" v-for="(waypoint,index) in list" :key="waypoint.id">
         <l-tooltip :options="{ permanent: 'true', direction: 'top'}"> {{ waypoint.name }}, {{ index }} </l-tooltip>
      </l-marker>

      <l-polyline :lat-lngs="this.playbackEnabled ? polylinePlaybackPath : polylinePath" :color="'red'" :dash-array="'5, 5'"/>
      <l-polyline :lat-lngs="odomPath" :color="'blue'"/>
      <l-polyline :lat-lngs="playbackPath" :color="'green'"/>
    </l-map>
    <div class="slider" v-if="this.playbackEnabled">
      <input type="range" min="0" :max='this.playbackLength-1' value ='0' v-model='playbackSlider'>
    </div>
  </div>
</template>

<script>
import { LMap, LTileLayer, LMarker, LPolyline, LPopup, LTooltip, LControlScale } from 'vue2-leaflet'
import { mapGetters } from 'vuex'
import L from '../leaflet-rotatedmarker.js'

const MAX_ODOM_COUNT = 1000
const DRAW_FREQUENCY = 10

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
      iconSize: [64, 64],
      iconAnchor: [32, 32]
    })
    this.tangentIcon = L.icon({
      iconUrl: '/static/gps_tangent_icon.png',
      iconSize: [44, 80],
      iconAnchor: [22, 60]
    })
    this.waypointIcon = L.icon({
      iconUrl: '/static/map_marker.png',
      iconSize: [64, 64],
      iconAnchor: [32, 64],
      popupAnchor: [0, -32]
    })
  },

  computed: {
    ...mapGetters('autonomy', {
      route: 'route',
      list: 'waypointList',
      playbackEnabled: 'playbackEnabled',
      playbackLength: 'playbackLength',
      playback: 'playback',
      playbackOdomLat: 'playbackOdomLat',
      playbackOdomLon: 'playbackOdomLon',
      playbackOdomBearing: 'playbackOdomBearing',
      playbackGpsBearing: 'playbackGpsBearing',
    }),

    odomLatLng: function () {
      return L.latLng(this.odom.latitude_deg + this.odom.latitude_min/60, this.odom.longitude_deg + this.odom.longitude_min/60)
    },

    polylinePath: function () {
      return [this.odomLatLng].concat(this.route.map(waypoint => waypoint.latLng))
    },

    polylinePlaybackPath: function () {
      return this.route.map(waypoint => waypoint.latLng)
    }
  },

  data () {
    return {
      center: L.latLng(38.406371, -110.791954),
      url: 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
      attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors',
      roverMarker: null,
      tangentMarker: null,
      waypointIcon: null,
      map: null,
      odomCount: 0,
      locationIcon: null,
      tangentIcon: null,
      odomPath: [],
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
    }
  },

  watch: {
    odom: function (val) {
      // Trigger every time rover odom is changed

      const lat = val.latitude_deg + val.latitude_min / 60
      const lng = val.longitude_deg + val.longitude_min / 60
      const angle = val.bearing_deg

      // Move to rover on first odom message
      if (!this.findRover) {
        this.findRover = true
        this.center = L.latLng(lat, lng)
      }
      
      // Update the rover marker
      this.roverMarker.setRotationAngle(angle)

      this.roverMarker.setLatLng(L.latLng(lat, lng))
      this.tangentMarker.setLatLng(L.latLng(lat, lng))

      // Update the rover path
      this.odomCount++
      if (this.odomCount % DRAW_FREQUENCY === 0) {
        if (this.odomCount > MAX_ODOM_COUNT * DRAW_FREQUENCY) {
          this.odomPath.splice(0, 1)
        }
        this.odomPath.push(L.latLng(lat, lng))
      }

      this.odomPath[this.odomPath.length - 1] = L.latLng(lat, lng)
    },

    GPS: function (val) {
      const angle = val.bearing_deg
      this.tangentMarker.setRotationAngle(angle)
    },
    
    playbackEnabled: function(val) {
      this.odomPath = []
      this.playbackPath = []
      this.playbackSlider = 0

      if (val) {
        this.center = L.latLng(this.playbackOdomLat[0], this.playbackOdomLon[0])

        this.roverMarker.setRotationAngle(this.playbackOdomBearing[0])
        this.roverMarker.setLatLng(L.latLng(this.playbackOdomLat[0], this.playbackOdomLon[0]))

        this.tangentMarker.setRotationAngle(this.playbackGpsBearing[0])
        this.tangentMarker.setLatLng(L.latLng(this.playbackOdomLat[0], this.playbackOdomLon[0]))
      }
    },

    playbackSlider: function (val) {
      this.roverMarker.setRotationAngle(this.playbackOdomBearing[val])
      this.roverMarker.setLatLng(L.latLng(this.playbackOdomLat[val], this.playbackOdomLon[val]))

      this.tangentMarker.setRotationAngle(this.playbackGpsBearing[val])
      this.tangentMarker.setLatLng(L.latLng(this.playbackOdomLat[val], this.playbackOdomLon[val]))

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

.slider {
  grid-area: "slider";
}

.wrap {
  align-items: center;
  height: 100%;
  display: grid;
  overflow:hidden;
  min-height: 100%;
  grid-gap: 5px;
  grid-template-columns: 1fr;
  grid-template-rows: 96% 4%;
  grid-template-areas:"map" 
                      "slider";
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
