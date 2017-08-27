import Vue from 'vue'
import App from './App.vue'
import store from './store.js'
import LCMBridge from './bridge.js'

const WEBSOCKET_URL = 'ws://localhost:8001'

new Vue({
    el: '#app',
    store,
    render: h => h(App)
})

new LCMBridge(
    WEBSOCKET_URL,
    // Update WebSocket connection state
    (c) => store.commit('websocketConnectionState', c),
    // Update LCM connection state
    (c) => store.commit('connectionState', c),
    // Subscribed LCM message received
    (msg) => store.commit('lcmMessage', msg),
    // Subscriptions
    [
        { 
            'topic': '/odom',
            'type': 'Odometry'
        }
    ]
)
