<template>
    <div class="log">
        <label for="loggin_toggle">{{logging_on ? 'Logging On':'Logging Off'}}</label>
        <input type="checkbox" v-model="logging_on" id="loggin_toggle">

        <button v-on:click="download_log()">Save log</button>
    </div>
</template>

<script>

import {convertDMS} from '../utils.js'
import {mapGetters} from 'vuex'

export default {
    data() {
        return {
            logging_on: false,
            num_logs: 0,
            timestamp: [],
            enabled: [],

            // odom
            degrees_lat: [],
            minutes_lat: [],
            degrees_lon: [],
            minutes_lon: [],
            odom_bearing: [],
            odom_speed: [],

            // IMU
            accel_x: [],
            accel_y: [],
            accel_z: [],
            gyro_x: [],
            gyro_y: [],
            gyro_z: [],
            mag_x: [],
            mag_y: [],
            mag_z: [],
            roll: [],
            pitch: [],
            yaw: [],
            calib_sys: [],
            calib_gyro: [],
            calib_accel: [],
            calib_mag: [],
            imu_bearing: [],

            // GPS
            gps_lat_deg: [],
            gps_lat_min: [],
            gps_lon_deg: [],
            gps_lon_min: [],
            gps_bearing: [],
            gps_speed: [],

            // nav_state
            nav_state: [],
            completed_wps: [],
            total_wps: [],
            first_waypoint_lat: [],
            first_waypoint_lon: [],

            // Joystick
            forward_back: [],
            left_right: [],
            dampen: [],
            kill: [],
            restart: [],

            // TargetList
            bearing0: [],
            distance0: [],
            id0: [],
            bearing1: [],
            distance1: [],
            id1: []

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

        GPS: {
            type: Object,
            required: true
        },

        nav_status: {
            type: Object,
            required: true
        },

        route: {
            type: Object,
            required: true
        },

        Joystick: {
            type: Object,
            required: true
        },

        TargetList: {
            type: Object,
            required: true
        }
    },

    computed: {
        ...mapGetters('autonomy', {
            autonEnabled: 'autonEnabled',
            odom_format: 'odomFormat',
            route: 'route'
        }),

        formatted_odom: function() {
            return {
                lat: convertDMS({d: this.odom.latitude_deg, m: this.odom.latitude_min, s: 0}, this.odom_format),
                lon: convertDMS({d: this.odom.longitude_deg, m: this.odom.longitude_min, s: 0}, this.odom_format)
            }
        }
    },

    created: function() {
        // time between interations in seconds
        const update_rate = 0.5

        const seconds_to_save = 1800
        const overflow_amt = 60

        window.setInterval(() => {
            console.log(this.TargetList)
            console.log(this.TargetList[0])
            console.log(this.TargetList[1])
            if (this.logging_on == 1) {
                this.num_logs += 1

                const time = new Date(Date.now())
                const time_string = time.toTimeString().substring(0,17) + ' ' + time.toDateString()
                this.timestamp.push(time_string)
                this.enabled.push(this.autonEnabled)

                this.degrees_lat.push(this.formatted_odom.lat.d)
                this.minutes_lat.push(this.formatted_odom.lat.m)
                this.degrees_lon.push(this.formatted_odom.lon.d)
                this.minutes_lon.push(this.formatted_odom.lon.m)
                this.odom_bearing.push(this.odom.bearing_deg)
                this.odom_speed.push(this.odom.speed)

                this.accel_x.push(this.IMU.accel_x_g)
                this.accel_y.push(this.IMU.accel_y_g)
                this.accel_z.push(this.IMU.accel_z_g)
                this.gyro_x.push(this.IMU.gyro_x_dps)
                this.gyro_y.push(this.IMU.gyro_y_dps)
                this.gyro_z.push(this.IMU.gyro_z_dps)
                this.mag_x.push(this.IMU.mag_x_uT)
                this.mag_y.push(this.IMU.mag_y_uT)
                this.mag_z.push(this.IMU.mag_z_uT)
                this.roll.push(this.IMU.roll_rad)
                this.pitch.push(this.IMU.pitch_rad)
                this.yaw.push(this.IMU.yaw_rad)
                this.calib_sys.push(this.IMU.calibration_sys)
                this.calib_gyro.push(this.IMU.calibration_gyro)
                this.calib_accel.push(this.IMU.calibration_accel)
                this.calib_mag.push(this.IMU.calibration_mag)
                this.imu_bearing.push(this.IMU.bearing_deg)

                this.gps_lat_deg.push(this.GPS.latitude_deg)
                this.gps_lat_min.push(this.GPS.latitude_min)
                this.gps_lon_deg.push(this.GPS.longitude_deg)
                this.gps_lon_min.push(this.GPS.longitude_min)
                this.gps_bearing.push(this.GPS.bearing_deg)
                this.gps_speed.push(this.GPS.speed)

                this.nav_state.push(this.nav_status.nav_state_name)
                this.completed_wps.push(this.nav_status.completed_wps)
                this.total_wps.push(this.nav_status.total_wps)

                if (this.route.length > 0) {
                    this.first_waypoint_lat.push(this.route[0].latLng.lat)
                    this.first_waypoint_lon.push(this.route[0].latLng.lng)
                }
                else {
                    this.first_waypoint_lat.push("(empty course)")
                    this.first_waypoint_lon.push("(empty course)")
                }

                this.forward_back.push(this.Joystick.forward_back)
                this.left_right.push(this.Joystick.left_right)
                this.dampen.push(this.Joystick.dampen)
                this.kill.push(this.Joystick.kill)
                this.restart.push(this.Joystick.restart)

                this.bearing0.push(this.TargetList[0].bearing)
                this.distance0.push(this.TargetList[0].distance)
                this.id0.push(this.TargetList[0].id)
                this.bearing1.push(this.TargetList[1].bearing)
                this.distance1.push(this.TargetList[1].distance)
                this.id1.push(this.TargetList[1].id)

                if (this.num_logs > (seconds_to_save / update_rate) + overflow_amt) {
                    this.num_logs -= overflow_amt
                    this.timestamp.splice(0, overflow_amt)
                    this.enabled.splice(0, overflow_amt)

                    this.degrees_lat.splice(0, overflow_amt)
                    this.minutes_lat.splice(0, overflow_amt)
                    this.degrees_lon.splice(0, overflow_amt)
                    this.minutes_lon.splice(0, overflow_amt)
                    this.odom_bearing.splice(0, overflow_amt)
                    this.odom_speed.splice(0, overflow_amt)

                    this.accel_x.splice(0, overflow_amt)
                    this.accel_y.splice(0, overflow_amt)
                    this.accel_z.splice(0, overflow_amt)
                    this.gyro_x.splice(0, overflow_amt)
                    this.gyro_y.splice(0, overflow_amt)
                    this.gyro_z.splice(0, overflow_amt)
                    this.mag_x.splice(0, overflow_amt)
                    this.mag_y.splice(0, overflow_amt)
                    this.mag_z.splice(0, overflow_amt)
                    this.roll.splice(0, overflow_amt)
                    this.pitch.splice(0, overflow_amt)
                    this.yaw.splice(0, overflow_amt)
                    this.calib_sys.splice(0, overflow_amt)
                    this.calib_gyro.splice(0, overflow_amt)
                    this.calib_accel.splice(0, overflow_amt)
                    this.calib_mag.splice(0, overflow_amt)
                    this.imu_bearing.splice(0, overflow_amt)

                    this.gps_lat_deg.splice(0, overflow_amt)
                    this.gps_lat_min.splice(0, overflow_amt)
                    this.gps_lon_deg.splice(0, overflow_amt)
                    this.gps_lon_min.splice(0, overflow_amt)
                    this.gps_bearing.splice(0, overflow_amt)
                    this.gps_speed.splice(0, overflow_amt)

                    this.nav_state.splice(0, overflow_amt)
                    this.completed_wps.splice(0, overflow_amt)
                    this.total_wps.splice(0, overflow_amt)
                    this.first_waypoint_lat.splice(0, overflow_amt)
                    this.first_waypoint_lon.splice(0, overflow_amt)

                    this.forward_back.splice(0, overflow_amt)
                    this.left_right.splice(0, overflow_amt)
                    this.dampen.splice(0, overflow_amt)
                    this.kill.splice(0, overflow_amt)
                    this.restart.splice(0, overflow_amt)

                    this.bearing0.splice(0,overflow_amt)
                    this.distance0.splice(0,overflow_amt)
                    this.id0.splice(0,overflow_amt)
                    this.bearing1.splice(0,overflow_amt)
                    this.distance1.splice(0,overflow_amt)
                    this.id1.splice(0,overflow_amt)
                }
            }
        }, update_rate * 1000)
    },

    methods: {
        download_log() {
            var csv = 'Timestamp,Auton Enabled,Odom Degrees Lat,Odom Minutes Lat,Odom Degrees Lon,Odom Minutes Lon,Odom bearing,Odom speed,Accel X,Accel Y,Accel Z,Gyro X,Gyro Y,Gyro Z,Mag X,Mag Y,Mag Z,Roll,Pitch,Yaw,Calibration Sys,Calibration Gyro,Calibration Accel,Calibration Mag,IMU Bearing,GPS Degrees Lat,GPS Minutes Lat,GPS Degrees Lon,GPS Minutes Lon,GPS Bearing,GPS Speed,Nav State,Waypoints Completed,Total Waypoints,First Waypoint Lat,First Waypoint Lon,Forward/Back,Left/Right,Dampen,Kill,Restart,Bearing0,Distance0,id0,Bearing1,Distance1,id1\n'

            for (let i = 0; i < this.num_logs; i++) {
                csv += this.timestamp[i] + ','
                csv += this.enabled[i] + ','
                
                csv += this.degrees_lat[i] + ','
                csv += this.minutes_lat[i] + ','
                csv += this.degrees_lon[i] + ','
                csv += this.minutes_lon[i] + ','
                csv += this.odom_bearing[i] + ','
                csv += this.odom_speed[i] + ','

                csv += this.accel_x[i] + ','
                csv += this.accel_y[i] + ','
                csv += this.accel_z[i] + ','
                csv += this.gyro_x[i] + ','
                csv += this.gyro_y[i] + ','
                csv += this.gyro_z[i] + ','
                csv += this.mag_x[i] + ','
                csv += this.mag_y[i] + ','
                csv += this.mag_z[i] + ','
                csv += this.roll[i] + ','
                csv += this.pitch[i] + ','
                csv += this.yaw[i] + ','
                csv += this.calib_sys[i] + ','
                csv += this.calib_gyro[i] + ','
                csv += this.calib_accel[i] + ','
                csv += this.calib_mag[i] + ','
                csv += this.imu_bearing[i] + ','

                csv += this.gps_lat_deg[i] + ','
                csv += this.gps_lat_min[i] + ','
                csv += this.gps_lon_deg[i] + ','
                csv += this.gps_lon_min[i] + ','
                csv += this.gps_bearing[i] + ','
                csv += this.gps_speed[i] + ','

                csv += this.nav_state[i] + ','
                csv += this.completed_wps[i] + ','
                csv += this.total_wps[i] + ','
                csv += this.first_waypoint_lat[i] + ','
                csv += this.first_waypoint_lon[i] + ','

                csv += this.forward_back[i] + ','
                csv += this.left_right[i] + ','
                csv += this.dampen[i] + ','
                csv += this.kill[i] + ','
                csv += this.restart[i] + ','

                csv += this.bearing0[i] + ','
                csv += this.distance0[i] + ','
                csv += this.id0[i] + ','
                csv += this.bearing1[i] + ','
                csv += this.distance1[i] + ','
                csv += this.id1[i]

                csv += '\n'
            }

            var hiddenElement = document.createElement('a')
            hiddenElement.href = 'data:text/csv;charset=utf-8,' + encodeURI(csv)
            hiddenElement.target = '_blank'

            const time = new Date(Date.now())
            const time_string = time.toTimeString().substring(0,17) + ' ' + time.toDateString()

            hiddenElement.download = 'AutonLog-' + time_string + '.csv'
            hiddenElement.click()

            // Remove data that was just saved

            this.num_logs = 0
            this.timestamp = []
            this.enabled = []

            this.degrees_lat = []
            this.minutes_lat = []
            this.degrees_lon = []
            this.minutes_lon = []
            this.odom_bearing = []
            this.odom_speed = []

            this.accel_x = []
            this.accel_y = []
            this.accel_z = []
            this.gyro_x = []
            this.gyro_y = []
            this.gyro_z = []
            this.mag_x = []
            this.mag_y = []
            this.mag_z = []
            this.roll = []
            this.pitch = []
            this.yaw = []
            this.calib_sys = []
            this.calib_gyro = []
            this.calib_accel = []
            this.calib_mag = []
            this.imu_bearing = []

            this.gps_lat_deg = []
            this.gps_lat_min = []
            this.gps_lon_deg = []
            this.gps_lon_min = []
            this.gps_bearing = []
            this.gps_speed = []

            this.nav_state = []
            this.completed_wps = []
            this.total_wps = []
            this.first_waypoint_lat = []
            this.first_waypoint_lon = []

            this.forward_back = []
            this.left_right = []
            this.dampen = []
            this.kill = []
            this.restart = []

            this.bearing0 = []
            this.distance0 = []
            this.id0 = []
            this.bearing1 = []
            this.distance1 = []
            this.id1 = []
        }
    }
}
</script>

<style scoped>
    .log {
        padding: 10px
    }
</style>
