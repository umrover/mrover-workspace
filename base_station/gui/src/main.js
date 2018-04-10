import App from './components/App.html'
import Cameras from './components/Cameras.html'
import PidTune from './components/PidTune.html'
import Diagnostics from './components/Diagnostics.html'
import LCMBridge from 'bridge'

const main = document.querySelector('main')
let app = null
let cam = null
let pidTune = null
let diagnostics = null
if (main.id == 'dashboard') {
  app = new App({target: main})
}
else if (main.id == 'camera') {
  cam = new Cameras({target: main})
}else if (main.id == 'pid-tune'){
  pidTune = new PidTune({target: main})
}else if (main.id == 'diagnostics'){
  diagnostics = new Diagnostics({target: main})
}
const lcm_ = new LCMBridge(
  "ws://localhost:8001",
  // Update WebSocket connection state
  (online) => {
    if (app != null) {
      app.set({
        websocket_connected: online
      })
      lcm_.setHomePage();
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
      pidTune.lcm_message_recv(msg);
    }
    if (diagnostics != null){
      diagnostics.lcm_message_recv(msg);
    }

    //Displays debug
    if(msg['topic'] == '/debugMessage'){
      if(msg['message']['isError'])
        console.error(msg['message']['message'])
      else
        console.log(msg['message']['message'])
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
    'type': 'KillSwitch'
  },
  {
    'topic': '/camera_servos',
    'type': 'CameraServos'
  },
  {
    'topic':'/encoder',
    'type':'Encoder'
  },
  {
    'topic':'/nav_status',
    'type':'NavStatus'
  },
  {
    'topic': '/sa_motors',
    'type': 'SAMotors'
  },
  {
    'topic': '/debugMessage',
    'type': 'DebugMessage'
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
    "left_js_x": 0,
    "left_js_y": 1,
    "left_trigger": 6,
    "right_trigger": 7,
    "right_js_x": 2,
    "right_js_y": 3,
    "right_bumper": 5,
    "left_bumper": 4,
    "d_pad_up": 12,
    "d_pad_down": 13
  };

  app.on("sensor_switch", (should_record) => {
      if (lcm_.online) {
          const msg={
            'type': 'SensorSwitch',
            'should_record': should_record
          };

          lcm_.publish('/sensor_switch', msg);
      }
  });

  app.on("auton", (msg) => {
    if(lcm_.online){
      msg['type']='AutonState'
      lcm_.publish('/auton', msg);
    }
  });

  app.on('course', (course) => {
    if(lcm_.online){
      course['type'] = 'Course';
      lcm_.publish('/course', course);
    }
  });

  window.setInterval(() => {
    const gamepads = navigator.getGamepads();
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
          app.set({
            dampen: gamepad.axes[JOYSTICK_CONFIG["dampen"]]
          });
          lcm_.publish('/drive_control', joystickData);
        }
        else if (gamepad.id.includes("Microsoft")) {
          const xboxData = {
            'type': 'Xbox',
            'left_js_x': gamepad.axes[XBOX_CONFIG["left_js_x"]], //shoulder rotate
            'left_js_y': gamepad.axes[XBOX_CONFIG["left_js_y"]], //shoulder tilt
            'left_trigger': gamepad.buttons[XBOX_CONFIG["left_trigger"]]['pressed'], //elbow forward
            'right_trigger': gamepad.buttons[XBOX_CONFIG["right_trigger"]]['pressed'], //elbow back
            'right_js_x': gamepad.axes[XBOX_CONFIG["right_js_x"]], //hand rotate
            'right_js_y': gamepad.axes[XBOX_CONFIG["right_js_y"]], //hand tilt
            'right_bumper': gamepad.buttons[XBOX_CONFIG["right_bumper"]]['pressed'], //grip close
            'left_bumper': gamepad.buttons[XBOX_CONFIG["left_bumper"]]['pressed'], //grip open
            'd_pad_up': gamepad.buttons[XBOX_CONFIG["d_pad_up"]]['pressed'],
            'd_pad_down': gamepad.buttons[XBOX_CONFIG["d_pad_down"]]['pressed']
          };
          if (app.refs.controls.get("arm")) {
            lcm_.publish('/arm_control', xboxData);
          } else if (app.refs.controls.get("soil_ac")) {
            lcm_.publish('/sa_control', xboxData);
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
