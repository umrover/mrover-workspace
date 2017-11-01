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

window.addEventListener("gamepadconnected", e => {
  const gamepad = navigator.getGamepads()[e.gamepad.index];
  console.log("Gamepad connected at index %d: %s. %d buttons, %d axes.",
    e.gamepad.index, e.gamepad.id,
    e.gamepad.buttons.length, e.gamepad.axes.length);
});

// Go to http://html5gamepad.com/ to check joystick button/axes indices
const JOYSYTICK_CONFIG = {
    "forward_back" : 1,
    "left_right" : 2,
    "kill" : 10
};

window.setInterval(() => {
    const gamepads = navigator.getGamepads();
    const gamepad = gamepads[0];
    if(!gamepad)
        return;


    const joystickData = {'type' : 'Joystick',
                        'forward_back' : -gamepad.axes[JOYSYTICK_CONFIG["forward_back"]],
                        'left_right' : gamepad.axes[JOYSYTICK_CONFIG["left_right"]],
                        'kill' : gamepad.buttons[JOYSYTICK_CONFIG["kill"]]['pressed']};

    console.log(joystickData[JOYSYTICK_CONFIG["forward_back"]]);

    lcm_.publish('/joystick', joystickData);

}, 100);
