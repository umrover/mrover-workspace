<template>
  <div class="wrap">
    <!-- Map goes here       -->

    <l-map ref="map" class="map" :zoom="15" :center="center">
      <l-control-scale :imperial="false"/>
      <l-tile-layer :url="url" :attribution="attribution"/>
      <l-marker ref="rover" :lat-lng="odomLatLng" :icon="locationIcon"/>
      <l-marker :lat-lng="waypoint.latLng" :icon="waypointIcon" v-for="(waypoint,index) in route" :key="waypoint.id" >
        <l-tooltip :options="{ permanent: 'true', direction: 'top'}"> {{ index }} </l-tooltip>
      </l-marker>

      <l-marker :lat-lng="waypoint.latLng" :icon="waypointIcon" v-for="(waypoint,index) in list" :key="waypoint.id">
         <l-tooltip :options="{ permanent: 'true', direction: 'top'}"> {{ index }} </l-tooltip>
      </l-marker>

      <l-polyline :lat-lngs="polylinePath" :color="'red'" :dash-array="'5, 5'"/>
      <l-polyline :lat-lngs="odomPath" :color="'blue'"/>
    </l-map>

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
      list: 'waypointList'
    }),

    odomLatLng: function () {
      return L.latLng(this.odom.latitude_deg + this.odom.latitude_min/60, this.odom.longitude_deg + this.odom.longitude_min/60)
    },

    polylinePath: function () {
      return [this.odomLatLng].concat(this.route.map(waypoint => waypoint.latLng))
    }
  },

  data () {
    return {
      center: L.latLng(38.406371, -110.791954),
      url: '/static/map/{z}/{x}/{-y}.png',
      attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors',
      roverMarker: null,
      waypointIcon: null,
      map: null,
      odomCount: 0,
      locationIcon: null,
      odomPath: [],
      options: {
        type: Object,
        default: () => ({})
      }
    }
  },

  props: {
    odom: {
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

      // Update the rover marker
      this.roverMarker.setRotationAngle(angle)

      this.roverMarker.setLatLng(L.latLng(lat, lng))

      // Update the rover path
      this.odomCount++
      if (this.odomCount % DRAW_FREQUENCY === 0) {
        if (this.odomCount > MAX_ODOM_COUNT * DRAW_FREQUENCY) {
          this.odomPath.splice(0, 1)
        }
        this.odomPath.push(L.latLng(lat, lng))
      }

      this.odomPath[this.odomPath.length - 1] = L.latLng(lat, lng)
    }
  },

  mounted: function () {
    this.$nextTick(() => {
      this.map = this.$refs.map.mapObject
      this.roverMarker = this.$refs.rover.mapObject
    })
  }
}
</script>

<style scoped>
.map {
  height: 100%;
  width: 100%;
}

.wrap {
  display: flex;
  align-items: center;
  height: 100%;
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
