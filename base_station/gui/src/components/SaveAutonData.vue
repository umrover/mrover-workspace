<template>
    <div class="log">
        <button v-on:click="download_log()">Save log</button>
    </div>
</template>

<script>

import {convertDMS} from '../utils.js'
import {mapGetters} from 'vuex'

export default {
    data() {
        return {
            num_logs: 0,
            timestamp: [],

            degrees_north: [],
            minutes_north: [],
            degrees_west: [],
            minutes_west: []
        }
    },

    props: {
        odom: {
            type: Object,
            required: true
        }
    },

    computed: {
        ...mapGetters('autonomy', {
            odom_format: 'odomFormat'
        }),

        formatted_odom: function() {
            return {
                lat: convertDMS({d: this.odom.latitude_deg, m: this.odom.latitude_min, s: 0}, this.odom_format),
                lon: convertDMS({d: -this.odom.longitude_deg, m: -this.odom.longitude_min, s: 0}, this.odom_format)
            }
        }
    },

    created: function() {
        // time between interations in seconds
        const update_rate = 1

        const seconds_to_save = 10
        const overflow_amt = 5

        window.setInterval(() => {
            this.num_logs += 1

            const time = new Date(Date.now())
            const time_string = time.toTimeString().substring(0,17) + ' ' + time.toDateString()
            this.timestamp.push(time_string)

            this.degrees_north.push(this.formatted_odom.lat.d)
            this.minutes_north.push(this.formatted_odom.lat.m)
            this.degrees_west.push(this.formatted_odom.lon.d)
            this.minutes_west.push(this.formatted_odom.lon.m)

            if (this.num_logs > (seconds_to_save / update_rate) + overflow_amt) {
                this.num_logs -= overflow_amt

                this.degrees_north.splice(0, overflow_amt)
                this.minutes_north.splice(0, overflow_amt)
                this.degrees_west.splice(0, overflow_amt)
                this.minutes_west.splice(0, overflow_amt)
            }

        }, update_rate * 1000)
    },

    methods: {
        download_log() {
            var csv = 'Timestamp,Degrees North,Minutes North,Degrees East,Minutes East\n'

            for (let i = 0; i < this.num_logs; i++) {
                csv += this.timestamp[i] + ','
                
                csv += this.degrees_north[i] + ','
                csv += this.minutes_north[i] + ','
                csv += this.degrees_west[i] + ','
                csv += this.minutes_west[i]

                csv += '\n'
            }

            var hiddenElement = document.createElement('a')
            hiddenElement.href = 'data:text/csv;charset=utf-8,' + encodeURI(csv)
            hiddenElement.target = '_blank'

            const time = new Date(Date.now())
            const time_string = time.toTimeString().substring(0,17) + ' ' + time.toDateString()

            hiddenElement.download = 'AutonLog-' + time_string + '.csv'
            hiddenElement.click()
        }
    }
}
</script>

<style scoped>
    .log {
        padding: auto;
    }
</style>
