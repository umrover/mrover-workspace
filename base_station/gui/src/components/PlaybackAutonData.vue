<template>
    <div class="log">
        <input type="file" id="read_log" accept=".csv"/>
        <button v-on:click="upload_log()">Upload log</button>
    </div>
</template>

<script>
import { mapMutations } from 'vuex'
import L from '../leaflet-rotatedmarker.js'

export default {

  data () {
    return {
      route: [],
      odom_lat: [],
      odom_lon: [],
      odom_bearing: [],
      gps_bearing: [],
      target_bearing: [],
      waypoint_lat: [],
      waypoint_lon: []
    }
  },

  methods: {
    ...mapMutations('autonomy', {
      setPlaybackEnabled: 'setPlaybackEnabled',
      setRoute: 'setRoute',
      setPlaybackOdomLat: 'setPlaybackOdomLat',
      setPlaybackOdomLon: 'setPlaybackOdomLon',
      setPlaybackOdomBearing: 'setPlaybackOdomBearing',
      setPlaybackGpsBearing: 'setPlaybackGpsBearing',
      setPlaybackTargetBearing: 'setPlaybackTargetBearing'
    }),

    upload_log: function() {
      console.log('Uploading file...')

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
          let rover_bearing_idx = 0

          let gps_bearing_idx = 0

          let target_bearing_idx = 0

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
              case 'Odom bearing':
                rover_bearing_idx = i
                break
              case 'GPS Bearing':
                gps_bearing_idx = i
                break
              case 'Target Bearing':
                target_bearing_idx = i
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
            this.odom_lat.push(
              parseFloat(parsed_data[i][rover_lat_deg_idx]) + parseFloat(parsed_data[i][rover_lat_min_idx])/60.0
            )
            this.odom_lon.push(
              parseFloat(parsed_data[i][rover_lon_deg_idx]) + parseFloat(parsed_data[i][rover_lon_min_idx])/60.0
            )
            this.odom_bearing.push(
              parseFloat(parsed_data[i][rover_bearing_idx])
            )
            this.gps_bearing.push(
              parseFloat(parsed_data[i][gps_bearing_idx])
            )
            this.target_bearing.push(
              parseFloat(parsed_data[i][target_bearing_idx])
            )

            let new_waypoint = true
            let waypoint_lat = parseFloat(parsed_data[i][waypoint_lat_idx])
            let waypoint_lon = parseFloat(parsed_data[i][waypoint_lon_idx])

            if (isNaN(waypoint_lat) || isNaN(waypoint_lon)) {
              new_waypoint = false
            }
            else {
              for (let j = 0; j < this.waypoint_lat.length; j++) {
                if (waypoint_lat == this.waypoint_lat[j] && waypoint_lon == this.waypoint_lon[j]) {
                  new_waypoint = false
                }
              }
            }

            if (new_waypoint) {
              this.waypoint_lat.push(waypoint_lat)
              this.waypoint_lon.push(waypoint_lon)
            }
          }

          for (let i = 0; i < this.waypoint_lat.length; i++) {
            this.route.push( {latLng: L.latLng(this.waypoint_lat[i], this.waypoint_lon[i])} )
          }

          this.setRoute(this.route)
          this.setPlaybackOdomLat(this.odom_lat)
          this.setPlaybackOdomLon(this.odom_lon)
          this.setPlaybackOdomBearing(this.odom_bearing)
          this.setPlaybackGpsBearing(this.gps_bearing)
          this.setPlaybackTargetBearing(this.target_bearing)

          console.log("SUCCESSFULLY READ DATA!")
          this.setPlaybackEnabled(true)

          this.route = []
          this.odom_lat = []
          this.odom_lon = []
          this.odom_bearing = []
          this.gps_bearing = []
          this.target_bearing = []
          this.waypoint_lat = []
          this.waypoint_lon = []
        }) // end callback

        reader.readAsBinaryString(file)
      }
      else {
          console.error("AUTON LOG NOT FOUND!")
      }
    }
  }
}
</script>

<style scoped>
  .playback {
      padding: 10px
  }
</style>
