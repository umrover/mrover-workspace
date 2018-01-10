import App from './components/App.html'
import Cameras from './components/Cameras.html'
import LCMBridge from 'bridge'

const main = document.querySelector('main')
let app = null
let cam = null
if (main.id == 'dashboard') {
  app = new App({target: main})
}
else if (main.id == 'camera') {
  cam = new Cameras({target: main})
}

const lcm_ = new LCMBridge(
  "ws://localhost:8001",
  // Update WebSocket connection state
  (online) => {
    if (app != null) {
      app.set({
        websocket_connected: online
      })
    }
  },
  // Update connection states
  (online) => {
    if (app != null) {
      app.set({
        lcm_connected: online[0]
      })
    }
    if (cam != null) {
      cam.set({
        connections: online.slice(1)
      })
    }
  },
  // Subscribed LCM message received
  (msg) => {
    if (app != null) {
      app.lcm_message_recv(msg)
    }
  },
  // Subscriptions
  [{
    'topic': '/odom',
    'type': 'Odometry'
  },
  {
    'topic': '/sensors',
    'type': 'Sensors'
  },
  {
    'topic': '/kill_switch',
    'type': 'Kill_switches'
  }]
)

window.addEventListener("gamepadconnected", e => {
  const gamepad = navigator.getGamepads()[e.gamepad.index];
  console.log("Gamepad connected at index %d: %s. %d buttons, %d axes.",
    e.gamepad.index, e.gamepad.id,
    e.gamepad.buttons.length, e.gamepad.axes.length);
});

// Go to http://html5gamepad.com/ to check joystick button/axes indices
const JOYSTICK_CONFIG = {
  "forward_back": 1,
  "left_right": 2,
  "dampen": 3,
  "kill": 4,
  "restart": 5
};

app.on("sensor_switch", (should_record) => {
    console.log(should_record);
    if (lcm_.online) {
        const msg={
          'type': 'SensorSwitch',
          'should_record': should_record
        }

        lcm_.publish('/sensor_switch', msg)
    }
});

window.setInterval(() => {
  const gamepads = navigator.getGamepads();
  const gamepad = gamepads[0];
  if (!gamepad)
    return;


  const joystickData = {
    'type': 'Joystick',
    'forward_back': -gamepad.axes[JOYSTICK_CONFIG["forward_back"]],
    'left_right': gamepad.axes[JOYSTICK_CONFIG["left_right"]],
    'dampen': gamepad.axes[JOYSTICK_CONFIG["dampen"]],
    'kill': gamepad.buttons[JOYSTICK_CONFIG["kill"]]['pressed'],
    'restart': gamepad.buttons[JOYSTICK_CONFIG["restart"]]['pressed']
  };

  console.log(joystickData['dampen']);

  lcm_.publish('/joystick', joystickData);

}, 100);
