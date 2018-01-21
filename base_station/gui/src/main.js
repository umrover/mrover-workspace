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
    if (cam != null){
      cam.lcm_message_recv(msg)
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
  },
  {
    'topic': '/carmera_servos',
    'type': 'CameraServos'
  }]
)

window.addEventListener("gamepadconnected", e => {
  const gamepad = navigator.getGamepads()[e.gamepad.index];
  console.log("Gamepad connected at index %d: %s. %d buttons, %d axes.",
    e.gamepad.index, e.gamepad.id,
    e.gamepad.buttons.length, e.gamepad.axes.length);
});



if(app !=null ){
  let servosSpeed = 1;
  const keysDown=[false, false, false, false];

  const servosMessages={
      'type':'CameraServos',
      'pan':0,
      'tilt':0
  };

  window.addEventListener('keydown', (e) => {
    if(e.keyCode==38)//Up
      keysDown[0]=true;
    else if(e.keyCode==40)//Down
      keysDown[1]=true;
    else if(e.keyCode==37)//Left
      keysDown[2]=true;
    else if(e.keyCode==39)//Right
      keysDown[3]=true;
    else if(e.keyCode==188)//comma /less than
      servosSpeed*=0.8;
    else if(e.keyCode==190)//period /greater than
      servosSpeed/=0.8;

    if(servosSpeed>1)
      servosSpeed=1;
    else if(servosSpeed<0.1)
      servosSpeed=Math.pow(0.8, 10);
  });

  window.addEventListener('keyup', (e) => {
    if(e.keyCode==38)//Up
      keysDown[0]=false;
    else if(e.keyCode==40)//Down
      keysDown[1]=false;
    else if(e.keyCode==37)//Left
      keysDown[2]=false;
    else if(e.keyCode==39)//Right
      keysDown[3]=false;
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
    if (gamepad){
      const joystickData = {
        'type': 'Joystick',
        'forward_back': -gamepad.axes[JOYSTICK_CONFIG["forward_back"]],
        'left_right': gamepad.axes[JOYSTICK_CONFIG["left_right"]],
        'dampen': gamepad.axes[JOYSTICK_CONFIG["dampen"]],
        'kill': gamepad.buttons[JOYSTICK_CONFIG["kill"]]['pressed'],
        'restart': gamepad.buttons[JOYSTICK_CONFIG["restart"]]['pressed']
      };
      lcm_.publish('/joystick', joystickData);
    }

    const pan=(keysDown[2]^keysDown[3] ? (keysDown[2]?-1:1) : 0);
    const tilt=(keysDown[0]^keysDown[1] ? (keysDown[0]?1:-1) : 0);

    servosMessages['pan']+=pan*servosSpeed/10;
    servosMessages['tilt']+=tilt*servosSpeed/10;

    const clamp = function(num, min, max) {
      return num <= min ? min : num >= max ? max : num;
    };

    servosMessages['pan'] = clamp(servosMessages['pan'], -1, 1);
    servosMessages['tilt'] = clamp(servosMessages['tilt'], -1, 1);

    lcm_.publish('/carmera_servos', servosMessages);

    console.log(servosMessages['pan']+", "+servosMessages['tilt']);

}, 100);
}
