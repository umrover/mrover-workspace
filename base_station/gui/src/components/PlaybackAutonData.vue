<template>
    <div class="log">
        <input type="file" id="read_log" accept=".csv"/>
        <button v-on:click="upload_log()">Upload log</button>
    </div>
</template>

<script>
export default {
    props: {
        playback: {
            type: Boolean,
            required: true
        }
    },

    methods: {

        upload_log() {
            console.log('Uploading file...')

            let files = document.getElementById('read_log').files
            if (files && files[0]) {
                let file = files[0]
                console.log('file:', file)

                let reader = new FileReader()

                reader.addEventListener('load', function(e) {
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
                                break;
                            case 'Odom Minutes Lat':
                                rover_lat_min_idx = i
                                break;
                            case 'Odom Degrees Lon':
                                rover_lon_deg_idx = i
                                break;
                            case 'Odom Minutes Lon':
                                rover_lon_min_idx = i
                                break;
                            case 'First Waypoint Lat':
                                waypoint_lat_idx = i
                                break;
                            case 'First Waypoint Lon':
                                waypoint_lon_idx = i
                                break;
                        }
                    }

                    console.log(parsed_data[0][0])
                    console.log(parsed_data[0])
                    console.table(parsed_data)

                    console.log(rover_lat_deg_idx)
                    console.log(rover_lat_min_idx)
                    console.log(rover_lon_deg_idx)
                    console.log(rover_lon_min_idx)
                    console.log(waypoint_lat_idx)
                    console.log(waypoint_lon_idx)
                })

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
