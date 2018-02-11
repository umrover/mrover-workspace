import App from './components/App.html'
import Cameras from './components/Cameras.html'
import PidTune from './components/PidTune.html'
import LCMBridge from 'bridge'

const main = document.querySelector('main')
let app = null
let cam = null
let pidTune = null
if (main.id == 'dashboard') {
  app = new App({target: main})
}
else if (main.id == 'camera') {
  cam = new Cameras({target: main})
}else if (main.id == 'pid-tune'){
  pidTune = new PidTune({target: main})
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
    if (pidTune != null){
      console.log(msg);
      pidTune.lcm_message_recv(msg);
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
    'topic': '/temperature',
    'type': 'Temperature'
  },
  {
    'topic': '/kill_switch',
    'type': 'Kill_switches'
  },
  {
    'topic': '/camera_servos',
    'type': 'CameraServos'
  },
  {
    'topic':'/encoder',
    'type':'Encoder'
  }]
)

window.addEventListener("gamepadconnected", e => {
  // const gamepad = navigator.getGamepads()[e.gamepad.index];
  // console.log("Gamepad connected at index %d: %s. %d buttons, %d axes.",
  //  e.gamepad.index, e.gamepad.id,
  //  e.gamepad.buttons.length, e.gamepad.axes.length);
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

    if(e.keyCode>=49 && e.keyCode<=54)  //keys 1 to 6
      window.localStorage.setItem("last_camera_key", e.keyCode-48);

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

  const XBOX_CONFIG = {
    "shoulder_rotate": 0,
    "shoulder_tilt": 1,
    "elbow_tilt_forward": 6,
    "elbow_tilt_back": 7,
    "hand_rotate": 2,
    "hand_tilt": 3,
    "grip_close": 5,
    "grip_open": 4
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
    console.log(gamepads);
    for (let i = 0; i < 2; i++) {
      const gamepad = gamepads[i];
      if (gamepad) {
        if (gamepad.id.includes("Logitech")) {
          const joystickData = {
            'type': 'Joystick',
            'forward_back': gamepad.axes[JOYSTICK_CONFIG["forward_back"]],
            'left_right': gamepad.axes[JOYSTICK_CONFIG["left_right"]],
            'dampen': gamepad.axes[JOYSTICK_CONFIG["dampen"]],
            'kill': gamepad.buttons[JOYSTICK_CONFIG["kill"]]['pressed'],
            'restart': gamepad.buttons[JOYSTICK_CONFIG["restart"]]['pressed']
          };
          lcm_.publish('/drive_control', joystickData);
        }
        else if (gamepad.id.includes("Microsoft")) {
          if (app.refs.controls.get("arm")) {
            const xboxData = {
              'type': 'Xbox',
              'shoulder_rotate': gamepad.axes[XBOX_CONFIG["shoulder_rotate"]],
              'shoulder_tilt': gamepad.axes[XBOX_CONFIG["shoulder_tilt"]],
              'elbow_tilt_forward': gamepad.buttons[XBOX_CONFIG["elbow_tilt_forward"]]['pressed'],
              'elbow_tilt_back': gamepad.buttons[XBOX_CONFIG["elbow_tilt_back"]]['pressed'],
              'hand_rotate': gamepad.axes[XBOX_CONFIG["hand_rotate"]],
              'hand_tilt': gamepad.axes[XBOX_CONFIG["hand_tilt"]],
              'grip_close': gamepad.buttons[XBOX_CONFIG["grip_close"]]['pressed'],
              'grip_open': gamepad.buttons[XBOX_CONFIG["grip_open"]]['pressed']
            };
            lcm_.publish('/arm_control', xboxData);
          } else if (app.refs.controls.get("soil_ac")) {
            // send soil_ac controls
          }
        }
      }
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
  }, 100);

}else if(pidTune != null){
  pidTune.on("/set_params", (pidData) => {
    console.log("Setting Params");
    
    const msg={
      'type':'SetParam',
      'deviceID': pidData['deviceID'],
      'paramID': 1,  //talon_srx.Params.ProfileParamSlot0_P
      'value': pidData['kp']
    }
    lcm_.publish('/setparam', msg);

    msg['paramID'] = 2  //talon_srx.Params.ProfileParamSlot0_I
    msg['value'] = pidData['ki']
    lcm_.publish('/setparam', msg);

    msg['paramID'] = 3  //talon_srx.Params.ProfileParamSlot0_D
    msg['value'] = pidData['kd']
    lcm_.publish('/setparam', msg);

    msg['paramID'] = 4  //talon_srx.Params.ProfileParamSlot0_F
    msg['value'] = pidData['kf']
    lcm_.publish('/setparam', msg);
  });

  pidTune.on('/set_demand', (demandData)=>{
    demandData['type'] = 'SetDemand';
    lcm_.publish('/setdemand', demandData);
  });
}
