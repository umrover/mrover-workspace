<template>
  <div class="wrap">
    <!-- Map goes here       -->
    <l-map ref="map" class="map" :zoom="15" :center="center">
      <l-control-scale :imperial="false"/>
      <l-tile-layer :url="url" :attribution="attribution" :options="tileLayerOptions"/>
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
      playbackLat: 'playbackLat',
      playbackLon: 'playbackLon',
      playbackBearing: 'playbackBearing',
      playbackWaypointLat: 'playbackWaypointLat',
      playbackWaypointLon: 'playbackWaypointLon'
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
      waypointIcon: null,
      map: null,
      odomCount: 0,
      locationIcon: null,
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
    
    playbackEnabled: function (val) {
      console.log('val:', val)
      if (val) {
        console.log(this.playbackLon[0])
        this.center = L.latLng(this.playbackLat[0], this.playbackLon[0])
        this.roverMarker.setRotationAngle(this.playbackBearing[0])
        this.roverMarker.setLatLng(L.latLng(this.playbackLat[0], this.playbackLon[0]))

        for (let i = 0; i < this.playbackWaypointLat.length; i++) {
          this.route.push( {latLng: L.latLng(this.playbackWaypointLat[i], this.playbackWaypointLon[i])} )
        }
      }
    },

    playbackSlider: function (val) {
      this.roverMarker.setRotationAngle(this.playbackBearing[val])
      this.roverMarker.setLatLng(L.latLng(this.playbackLat[val], this.playbackLon[val]))

      let length_diff = val - this.playbackPath.length

      if (length_diff > 0) {
        for (let i = this.playbackPath.length; i < val; i++) {
          this.playbackPath.push(L.latLng(
            this.playbackLat[i],
            this.playbackLon[i]
          ))
        }
      }
      else if (length_diff < 0) {
        this.playbackPath.splice(val, -1*length_diff)
      }
    }
  },

  methods: {
    upload_log: function () {
      console.log('Uploading file...')
      console.log(this.playback)

      let files = document.getElementById('read_log').files
      if (files && files[0]) {
        let file = files[0]

        let reader = new FileReader()

        reader.addEventListener('load', (e) => {

          let raw_data = e.target.result
          let parsed_data = []

          let lines = raw_data.split('\n')
          for (let i = 0; i < lines.length; i++) {
            parsed_data.push(lines[i].split(','))
          }

          let rover_lat_deg_idx = 0
          let rover_lat_min_idx = 0
          let rover_lon_deg_idx = 0
          let rover_lon_min_idx = 0

          let waypoint_lat_idx = 0
          let waypoint_lon_idx = 0

          for (let i = 0; i < parsed_data[0].length; i++) {
            switch (parsed_data[0][i]) {
              case 'Odom Degrees Lat':
                  rover_lat_deg_idx = i
                  break
              case 'Odom Minutes Lat':
                  rover_lat_min_idx = i
                  break
              case 'Odom Degrees Lon':
                  rover_lon_deg_idx = i
                  break
              case 'Odom Minutes Lon':
                  rover_lon_min_idx = i
                  break
              case 'First Waypoint Lat':
                  waypoint_lat_idx = i
                  break
              case 'First Waypoint Lon':
                  waypoint_lon_idx = i
                  break
            }
          }

          for (let i = 1; i < parsed_data.length - 1; i++) {
            this.playback.lat.push(
              parseFloat(parsed_data[i][rover_lat_deg_idx]) + parseFloat(parsed_data[i][rover_lat_min_idx])/60.0
            )
            this.playback.lon.push(
              parseFloat(parsed_data[i][rover_lon_deg_idx]) + parseFloat(parsed_data[i][rover_lon_min_idx])/60.0
            )

            let new_waypoint = true
            let waypoint_lat = parseFloat(parsed_data[i][waypoint_lat_idx])
            let waypoint_lon = parseFloat(parsed_data[i][waypoint_lon_idx])

            if (isNaN(waypoint_lat) || isNaN(waypoint_lon)) {
              new_waypoint = false
            }
            else {
              for (let j = 0; j < this.playback.waypoint_lat.length; j++) {
                if (waypoint_lat == this.playback.waypoint_lat[j] && waypoint_lon == this.playback.waypoint_lon[j]) {
                  new_waypoint = false
                }
              }
            }

            if (new_waypoint) {
              this.playback.waypoint_lat.push(waypoint_lat)
              this.playback.waypoint_lon.push(waypoint_lon)
            }
          }

          console.log("SUCCESSFULLY READ DATA!")
          this.playback.enabled = true
        }) // end callback

        reader.readAsBinaryString(file)
      }
      else {
          console.error("AUTON LOG NOT FOUND!")
      }
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
