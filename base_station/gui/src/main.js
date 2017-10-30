import App from './components/App.html'
import LCMBridge from 'bridge'

const app = new App({
    target: document.querySelector('main')
})

const lcm_ = new LCMBridge(
    "ws://localhost:8001",
    // Update WebSocket connection state
    (online) => {
        app.set({
            websocket_connected: online
        })
    },
    // Update LCM connection state
    (online) => {
        app.set({
            lcm_connected: online
        })
    },
    // Subscribed LCM message received
    (msg) => app.lcm_message_recv(msg),
    // Subscriptions
    [
        { 
            'topic': '/odom',
            'type': 'Odometry'
        }
    ]
)

window.addEventListener("gamepadconnected", function(e) {
  const gamepad = navigator.getGamepads()[e.gamepad.index];
  console.log("Gamepad connected at index %d: %s. %d buttons, %d axes.",
    e.gamepad.index, e.gamepad.id,
    e.gamepad.buttons.length, e.gamepad.axes.length);
});

window.setInterval(function(){
    const gamepads = navigator.getGamepads();
    const gamepad = gamepads[0];
    if(!gamepad)
        return;


    const joystickData = {'type' : 'Joystick',
                        'forward_back' : -gamepad.axes[1],
                        'left_right' : gamepad.axes[5],
                        'kill' : gamepad.buttons[10]['pressed']};

    lcm_.publish('/joystick', joystickData);

}, 100);
