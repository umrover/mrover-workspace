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

var lcm_ = new LCMBridge(
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

window.addEventListener("gamepadconnected", function(e) {
  var gamepad = navigator.getGamepads()[e.gamepad.index];
  console.log("Gamepad connected at index %d: %s. %d buttons, %d axes.",
    e.gamepad.index, e.gamepad.id,
    e.gamepad.buttons.length, e.gamepad.axes.length);
});

window.setInterval(function(){
    var gamepads = navigator.getGamepads();
    var gamepad = gamepads[0];
    if(!gamepad)
        return;


    var joystickData = {'type' : 'Joystick',
                        'forward_back' : -gamepad.axes[1],
                        'left_right' : gamepad.axes[5],
                        'kill' : gamepad.buttons[10]['pressed']};

    lcm_.publish('/joystick', joystickData);

}, 100);