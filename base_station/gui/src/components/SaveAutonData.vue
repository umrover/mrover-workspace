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
            minutes_west: [],

            accel_x: [],
            accel_y: [],
            accel_z: [],
            gyro_x: [],
            gyro_y: [],
            gyro_z: [],
            mag_x: [],
            mag_y: [],
            mag_z: [],
            bearing: [],

            nav_state: [],
            completed_wps: [],
            total_wps: [],

            forward_back: [],
            left_right: [],
            dampen: [],
            kill: [],
            restart: []
        }
    },

    props: {
        odom: {
            type: Object,
            required: true
        },

        IMU: {
            type: Object,
            required: true
        },

        nav_status: {
            type: Object,
            required: true
        },

        Joystick: {
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

        const seconds_to_save = 1800
        const overflow_amt = 60

        window.setInterval(() => {
            this.num_logs += 1

            const time = new Date(Date.now())
            const time_string = time.toTimeString().substring(0,17) + ' ' + time.toDateString()
            this.timestamp.push(time_string)

            this.degrees_north.push(this.formatted_odom.lat.d)
            this.minutes_north.push(this.formatted_odom.lat.m)
            this.degrees_west.push(this.formatted_odom.lon.d)
            this.minutes_west.push(this.formatted_odom.lon.m)

            this.accel_x.push(this.IMU.accel_x)
            this.accel_y.push(this.IMU.accel_y)
            this.accel_z.push(this.IMU.accel_z)
            this.gyro_x.push(this.IMU.gyro_x)
            this.gyro_y.push(this.IMU.gyro_y)
            this.gyro_z.push(this.IMU.gyro_z)
            this.mag_x.push(this.IMU.mag_x)
            this.mag_y.push(this.IMU.mag_y)
            this.mag_z.push(this.IMU.mag_z)
            this.bearing.push(this.IMU.bearing)

            this.nav_state.push(this.nav_status.nav_state_name)
            this.completed_wps.push(this.completed_wps)
            this.total_wps.push(this.total_wps)

            this.forward_back.push(this.Joystick.forward_back)
            this.left_right.push(this.Joystick.left_right)
            this.dampen.push(this.Joystick.dampen)
            this.kill.push(this.Joystick.kill)
            this.restart.push(this.Joystick.restart)

            if (this.num_logs > (seconds_to_save / update_rate) + overflow_amt) {
                this.num_logs -= overflow_amt

                this.degrees_north.splice(0, overflow_amt)
                this.minutes_north.splice(0, overflow_amt)
                this.degrees_west.splice(0, overflow_amt)
                this.minutes_west.splice(0, overflow_amt)

                this.accel_x.splice(0, overflow_amt)
                this.accel_y.splice(0, overflow_amt)
                this.accel_z.splice(0, overflow_amt)
                this.gyro_x.splice(0, overflow_amt)
                this.gyro_y.splice(0, overflow_amt)
                this.gyro_z.splice(0, overflow_amt)
                this.mag_x.splice(0, overflow_amt)
                this.mag_y.splice(0, overflow_amt)
                this.mag_z.splice(0, overflow_amt)
                this.bearing.splice(0, overflow_amt)

                this.nav_state.splice(0, overflow_amt)
                this.completed_wps.splice(0, overflow_amt)
                this.total_wps.splice(0, overflow_amt)

                this.forward_back.splice(0, overflow_amt)
                this.left_right.splice(0, overflow_amt)
                this.dampen.splice(0, overflow_amt)
                this.kill.splice(0, overflow_amt)
                this.restart.splice(0, overflow_amt)
            }

        }, update_rate * 1000)
    },

    methods: {
        download_log() {
            var csv = 'Timestamp,Degrees North,Minutes North,Degrees East,Minutes East,Accel X, Accel Y, Accel Z, Gyro X, Gyro Y, Gyro Z, Mag X, Mag Y, Mag Z, Bearing, Nav State, Waypoints Completed, Total Waypoints, Forward/Back, Left/Right, Dampen, Kill, Restart\n'

            for (let i = 0; i < this.num_logs; i++) {
                csv += this.timestamp[i] + ','
                
                csv += this.degrees_north[i] + ','
                csv += this.minutes_north[i] + ','
                csv += this.degrees_west[i] + ','
                csv += this.minutes_west[i]

                csv += this.accel_x[i] + ','
                csv += this.accel_y[i] + ','
                csv += this.accel_z[i] + ','
                csv += this.gyro_x[i] + ','
                csv += this.gyro_y[i] + ','
                csv += this.gyro_z[i] + ','
                csv += this.mag_x[i] + ','
                csv += this.mag_y[i] + ','
                csv += this.mag_z[i] + ','
                csv += this.bearing[i] + ','

                csv += this.nav_state[i] + ','
                csv += this.completed_wps[i] + ','
                csv += this.total_wps[i] + ','

                csv += this.forward_back[i] + ','
                csv += this.left_right[i] + ','
                csv += this.dampen[i] + ','
                csv += this.kill[i] + ','
                csv += this.restart[i]

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
