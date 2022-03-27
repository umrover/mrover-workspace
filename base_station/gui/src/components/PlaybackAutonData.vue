<template>
    <div class="log">
        <input type="file" id="read_log" accept=".csv"/>
        <button v-on:click="upload_log()">Upload log</button>
    </div>
</template>

<script>
import { mapGetters, mapMutations } from 'vuex'

export default {

  data () {
    return {
      playback: {
        lat: [],
        lon: [],
        bearing: [],
        waypoint_lat: [],
        waypoint_lon: []
      }
    }
  },

  methods: {
    ...mapMutations('autonomy', {
      setPlaybackEnabled: 'setPlaybackEnabled',
      setPlaybackLat: 'setPlaybackLat',
      setPlaybackLon: 'setPlaybackLon',
      setPlaybackBearing: 'setPlaybackBearing',
      setPlaybackWaypointLat: 'setPlaybackWaypointLat',
      setPlaybackWaypointLon: 'setPlaybackWaypointLon',
    }),

    upload_log: function() {
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
          let rover_bearing_idx = 0

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
            this.playback.bearing.push(
              parseFloat(parsed_data[i][rover_bearing_idx])
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

          this.setPlaybackLat(this.playback.lat)
          this.setPlaybackLon(this.playback.lon)
          this.setPlaybackBearing(this.playback.bearing)
          this.setPlaybackWaypointLat(this.playback.waypoint_lat)
          this.setPlaybackWaypointLon(this.playback.waypoint_lon)

          console.log("SUCCESSFULLY READ DATA!")
          this.setPlaybackEnabled(true)

          this.playback.lat = []
          this.playback.lon = []
          this.playback.bearing = []
          this.playback.waypoint_lat = []
          this.playback.waypoint_lon = []
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
