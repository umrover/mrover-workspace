import Vue from 'vue'
import Vuex from 'vuex'

Vue.use(Vuex)

const debug = process.env.NODE_ENV !== 'production'

export default new Vuex.Store({
    state: {
        websocket_connected: false,
        connected: false,
        odom: {
            latitude_deg: 0,
            longitude_deg: 0,
            bearing_deg: 0
        }
    },
    mutations: {
        websocketConnectionState (state, c) {
            console.log('New websocket connection state:', c)
            state.websocket_connected = c
        },
        connectionState (state, c) {
            console.log('New connection state:', c)
            state.connected = c
        },
        lcmMessage (state, msg) {
            console.log('Received on:', msg.topic)
            if (msg.topic === '/odom') {
                state.odom = msg.message
            }
        }
    },
    strict: debug
})
