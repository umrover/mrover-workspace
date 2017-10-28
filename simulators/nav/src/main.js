import App from './components/App.html'
import LCMBridge from 'bridge'

const app = new App({
    target: document.querySelector('main')
})

// TODO add connected indicators
const bridge = new LCMBridge("ws://localhost:8081",
    // Is websocket online?
    (online) => {
        if (!online) {
            app.set({
                'lcm_connected': false
            })
        }
    },
    // Is LCM online?
    (online) => {
        app.set({
            'lcm_connected': online
        })
    },
    // Message handler
    ({topic, message}) => {
        if (topic === '/joystick') {
            app.apply_joystick(message)
        }
    },
    // Subscriptions
    [ { topic: '/joystick', type: 'Joystick' } ])

app.on("odom", (odom) => {
    if (bridge.online) {
        odom.type = 'Odometry'
        bridge.publish('/odom', odom)
    }
})
