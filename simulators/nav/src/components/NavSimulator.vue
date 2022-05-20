<!-- This file contains the NavSimulator component which contains all the major
     components (e.g. Header, Perception, Field, etc.). This is the main body
     of the simulator. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div
    id="nav-simulator"
    class="wrapper"
  >
    <Header />
    <Perception />
    <HotKeys />
    <div class="flex-grid">
      <!-- left half -->
      <div class="col left">
        <Field />
        <LCMCenter />
      </div>
      <!-- right half -->
      <div class="col">
        <ControlPanel />
        <FieldItems />
      </div>

      <!-- LCM Connection Statuses -->
      <HeartbeatMonitor
        :is-alive.sync="isNavAlive"
        :has-pulse.sync="navPulse"
      />
      <HeartbeatMonitor
        :is-alive.sync="isLocAlive"
        :has-pulse.sync="locPulse"
      />
      <HeartbeatMonitor
        :is-alive.sync="isPercepAlive"
        :has-pulse.sync="percepPulse"
      />
    </div>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import fnvPlus from 'fnv-plus';
import { Component, Vue, Watch } from 'vue-property-decorator';
import { Getter, Mutation } from 'vuex-class';
import { RADIO } from '../utils/constants';
import {
  applyJoystickCmdUtil,
  applyZedGimbalCmdUtil,
  randnBm,
  metersToOdom,
  odomToMeters,
  compassModDeg
} from '../utils/utils';
import {
  Joystick,
  NavStatus,
  ObstacleMessage,
  Odom,
  Speeds,
  TargetListMessage,
  Waypoint,
  ZedGimbalPosition,
  Point2D,
  ProjectedPointsMessage
} from '../utils/types';
import ControlPanel from './control_panel/ControlPanel.vue';
import Field from './field/Field.vue';
import FieldItems from './field_items/FieldItems.vue';
import Header from './header/Header.vue';
import HeartbeatMonitor from './common/HeartbeatMonitor.vue';
import HotKeys from './HotKeys.vue';
import LCMBridge from '../../deps/lcm_bridge_client/dist/bridge.js';
import LCMCenter from './lcm_center/LCMCenter.vue';
import Perception from './perception/Perception.vue';
import { state } from '../store/modules/simulatorState';

/**************************************************************************************************
 * Constants
 **************************************************************************************************/
/* Frequency in milliseconds to publish outgoing LCM messages at. */
const TIME_INTERVAL_MILLI = 100;

/* Number of milliseconds in a 1 second. */
const ONE_SECOND_MILLI = 1000;

/* Number of milliseconds in 2 seconds. */
const TWO_SECOND_MILLI = 2000;

@Component({
  components: {
    ControlPanel,
    Field,
    FieldItems,
    Header,
    HeartbeatMonitor,
    HotKeys,
    LCMCenter,
    Perception
  }
})
export default class NavSimulator extends Vue {
  /************************************************************************************************
   * Vuex Getters
   ************************************************************************************************/
  @Getter
  private readonly autonOn!:boolean;

  @Getter
  private readonly currOdom!:Odom;

  @Getter
  private readonly realOdom!:Odom;

  @Getter
  private readonly currSpeed!:Speeds;

  @Getter
  private readonly fieldCenterOdom!:Odom;

  @Getter
  private readonly joystick!:Joystick;

  @Getter
  private readonly locConnected!:boolean;

  @Getter
  private readonly navConnected!:boolean;

  @Getter
  private readonly obstacleMessage!:ObstacleMessage;

  @Getter
  private readonly paused!:boolean;

  @Getter
  private readonly percepConnected!:boolean;

  @Getter
  private readonly radioStrength!:number;

  @Getter
  private readonly repeaterLoc!:Odom|null;

  @Getter
  private readonly simulateLoc!:boolean;

  @Getter
  private readonly simulatePercep!:boolean;

  @Getter
  private readonly enableLCM!:boolean;

  @Getter
  private readonly takeStep!:boolean;

  @Getter
  private readonly targetList!:TargetListMessage;

  @Getter
  private readonly waypoints!:Waypoint[];

  @Getter
  private readonly zedGimbalCmd!:ZedGimbalPosition;

  @Getter
  private readonly zedGimbalPos!:ZedGimbalPosition;

  /************************************************************************************************
   * Vuex Mutations
   ************************************************************************************************/
  @Mutation
  private readonly flipLcmConnected!:(onOff:boolean)=>void;

  @Mutation
  private readonly flipLocConnected!:(onOff:boolean)=>void;

  @Mutation
  private readonly flipNavConnected!:(onOff:boolean)=>void;

  @Mutation
  private readonly flipPercepConnected!:(onOff:boolean)=>void;

  @Mutation
  private readonly setCurrOdom!:(newOdom:Odom)=>void;

  @Mutation
  private readonly setRealOdom!:(newOdom:Odom)=>void;

  @Mutation
  private readonly setJoystick!:(newJoystick:Joystick)=>void;

  @Mutation
  private readonly setNavStatus!:(newNavStatus:NavStatus)=>void;

  @Mutation
  private readonly setObstacleMessage!:(newObstacle:ObstacleMessage)=>void;

  @Mutation
  private readonly setProjectedPointsMessage!:(
    newProjectedPointsMessage:ProjectedPointsMessage
  )=>void;

  @Mutation
  private readonly setRadioStrength!:(strength:number)=>void;

  @Mutation
  private readonly setRepeaterLoc!:(newRepeaterLoc:Odom|null)=>void;

  @Mutation
  private readonly setTakeStep!:(takeStep:boolean)=>void;

  @Mutation
  private readonly setTargetList!:(newTargetList:TargetListMessage)=>void;

  @Mutation
  private readonly setZedGimbalCmd!:(newZedGimbalCmd:ZedGimbalPosition)=>void;

  @Mutation
  private readonly setZedGimbalPos!:(newZedGimbalPos:ZedGimbalPosition)=>void;

  /************************************************************************************************
   * Private Members
   ************************************************************************************************/
  /* Whether or not we are in the process of dropping the repeater. */
  private droppingRepeater = false;

  /* Interval to publish outgoing LCM messages with. */
  private intervalLcmPublish!:number;

  /* LCM Bridge Client for LCM communications in and out of simulator. */
  private lcmBridge!:LCMBridge;

  /* Has there been a sign of life from the localization program. */
  private locPulse = false;

  /* Has there been a sign of life from the navigation program. */
  private navPulse = false;

  /* Has there been a sign of life from the perception program. */
  private percepPulse = false;

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* Are we receiving lcm messages from the localization program */
  private get isLocAlive():boolean {
    return this.locConnected;
  }
  private set isLocAlive(newLocConnected:boolean) {
    this.flipLocConnected(newLocConnected);
  }

  /* Are we receiving lcm messages from the navigation program */
  private get isNavAlive():boolean {
    return this.navConnected;
  }
  private set isNavAlive(newNavConnected:boolean) {
    this.flipNavConnected(newNavConnected);
  }

  /* Are we receiving lcm messages from the perception program */
  private get isPercepAlive():boolean {
    return this.percepConnected;
  }
  private set isPercepAlive(newPercepConnected:boolean) {
    this.flipPercepConnected(newPercepConnected);
  }

  /************************************************************************************************
   * Watchers
   ************************************************************************************************/
  @Watch('paused')
  private onUnpause(paused:boolean):void {
    if (!paused) {
      this.applyJoystickCmd();
      this.applyZedGimbalCmd();
    }
  }

  @Watch('takeStep')
  private onTakeStep():void {
    this.applyJoystickCmd();
    this.applyZedGimbalCmd();
    this.setTakeStep(false);
  }

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* Apply the current joystick message to move the rover. Update the current
     odometry based on this movement. */
  private applyGPSNoise():Odom {
    const maxOffDist = 0.75;

    /* eslint no-magic-numbers: ["error", { "ignore": [100, 0, 1] }] */
    const factor = 100;
    const num1:number = randnBm(-state.simSettings.noiseGPSPercent * maxOffDist /
    factor, state.simSettings.noiseGPSPercent * maxOffDist / factor, 1);
    const num2:number = randnBm(-state.simSettings.noiseGPSPercent * maxOffDist /
    factor, state.simSettings.noiseGPSPercent * maxOffDist / factor, 1);
    const nextPointMeters:Point2D = odomToMeters(this.realOdom, this.fieldCenterOdom);
    nextPointMeters.x += num1;
    nextPointMeters.y += num2;
    const nextOdom:Odom = metersToOdom(nextPointMeters, this.fieldCenterOdom);

    /* Calculate new bearing without bearing noise */
    nextOdom.bearing_deg = compassModDeg(this.realOdom.bearing_deg);
    return nextOdom;
  }

  private applyJoystickCmd():void {
    /* if not simulating localization, nothing to do */
    if (!this.simulateLoc) {
      return;
    }
    const deltaTimeSeconds:number = TIME_INTERVAL_MILLI / ONE_SECOND_MILLI;
    this.setRealOdom(applyJoystickCmdUtil(this.realOdom, this.fieldCenterOdom, this.joystick,
                                          deltaTimeSeconds, this.currSpeed));
    this.setCurrOdom(this.applyGPSNoise());
  }

  /* Apply the current ZED gimbal command. Update the ZED gimbal based on the
     current ZED gimbal command. */
  private applyZedGimbalCmd():void {
    const deltaTimeSeconds:number = TIME_INTERVAL_MILLI / ONE_SECOND_MILLI;
    this.setZedGimbalPos(applyZedGimbalCmdUtil(this.zedGimbalPos, this.zedGimbalCmd,
                                               deltaTimeSeconds, this.currSpeed));
  }

  /* Drop the radio repeater. */
  private dropRepeater():void {
    if (this.repeaterLoc === null && !this.droppingRepeater) {
      this.droppingRepeater = true;

      /* Wait 2 seconds, then 'drop' repeater */
      setTimeout(() => {
        const repeaterLoc:Odom = { ...this.currOdom };
        this.setRepeaterLoc(repeaterLoc);
        this.droppingRepeater = false;
        this.setRadioStrength(RADIO.maxSignalStrength);
      }, TWO_SECOND_MILLI);
    }
  }

  /* For the code handling LCM publishing and subscribing, we are disabling the
     @typescript-eslint/no-explicit-any rules because the LCM Bridge is not
     written in typescript. This should not be done any other times in this
     project. We re-enable these rules immediately after we are done handling
     the publishing and subscribing of LCMs. */
  /* eslint-disable @typescript-eslint/no-explicit-any */

  /* Publish the given LCM message on the given channel. */
  private publish(channel:string, payload:any, odomOrObstacle:boolean):void {
    if (!this.enableLCM && !odomOrObstacle) {
      return;
    }
    this.lcmBridge.publish(channel, payload);
  }

  /************************************************************************************************
   * Vue Life Cycle
   ************************************************************************************************/
  /* Connect to the LCM Bridge and set up communications with the server. */
  private created():void {
    /* Set up LCM Bridge and subscriptions */
    this.lcmBridge = new LCMBridge(

      /* URL of the LCM Bridge Server */
      'ws://localhost:8001',

      /* Update WebSocket connection state. */
      (online) => {
        this.lcmBridge.setHomePage();
        this.flipLcmConnected(online);
      },

      /* Update connection states. */
      () => { /* Intentionally doing nothing. */ },

      /* Subscribed LCM message received */
      (msg) => {
        if (msg.topic === '/auton_drive_control') {
          this.setJoystick({
            forward_back: msg.message.left_percent_velocity,
            left_right: msg.message.right_percent_velocity
          });
          if (!this.paused) {
            this.applyJoystickCmd();
          }
        }
        else if (msg.topic === '/nav_status') {
          this.navPulse = true;
          this.setNavStatus(msg.message);
        }
        else if (msg.topic === '/obstacle') {
          if (!this.simulatePercep) {
            this.percepPulse = true;
            this.setObstacleMessage(msg.message);
          }
        }
        else if (msg.topic === '/odometry') {
          if (!this.simulateLoc) {
            this.locPulse = true;
            this.setCurrOdom(msg.message);
            this.setRealOdom(this.currOdom);
          }
        }
        else if (msg.topic === '/rr_drop_init') {
          this.dropRepeater();
        }
        else if (msg.topic === '/target_list') {
          if (!this.simulatePercep) {
            this.percepPulse = true;
            this.setTargetList(msg.message.targetList);
          }
        }
        else if (msg.topic === '/zed_gimbal_cmd') {
          this.setZedGimbalCmd(msg.message);
          if (!this.paused) {
            this.applyZedGimbalCmd();
          }
        }
        else if (msg.topic === '/debugMessage') {
          if (msg.message.isError) {
            console.error(msg.message.message);
          }
          else {
            console.log(msg.message.message);
          }
        }
        else if (msg.topic === '/projected_points') {
          this.setProjectedPointsMessage(msg.message);
        }
      },

      /* Subscriptions */
      [
        { topic: '/autonomous',     type: 'Joystick' },
        { topic: '/auton_drive_control',     type: 'AutonDriveControl' },
        { topic: '/nav_status',     type: 'NavStatus' },
        { topic: '/obstacle',       type: 'Obstacle' },
        { topic: '/odometry',       type: 'Odometry' },
        { topic: '/rr_drop_init',   type: 'RepeaterDropInit' },
        { topic: '/target_list',    type: 'TargetList' },
        { topic: '/zed_gimbal_cmd', type: 'ZedGimbalPosition' },
        { topic: '/debugMessage',   type: 'DebugMessage' },
        { topic: '/projected_points', type: 'ProjectedPoints' }
      ]
    );

    /* Set up publishing LCMs */
    this.intervalLcmPublish = window.setInterval(() => {
      this.publish('/auton', { type: 'AutonState', is_auton: this.autonOn }, false);

      if (this.simulateLoc) {
        const odom:any = Object.assign(this.currOdom, { type: 'Odometry' });
        this.publish('/odometry', odom, true);
      }

      if (this.simulatePercep) {
        const obs:any = Object.assign(this.obstacleMessage, { type: 'Obstacle' });
        this.publish('/obstacle', obs, true);

        /* eslint no-magic-numbers: ["error", { "ignore": [0, 1] }] */
        const targetList:any = { targetList: this.targetList, type: 'TargetList' };
        targetList.targetList[0].type = 'Target';
        targetList.targetList[1].type = 'Target';
        this.publish('/target_list', targetList, false);
      }

      if (this.repeaterLoc !== null) {
        this.publish('/rr_drop_complete', { type: 'RepeaterDrop' }, false);
      }

      this.publish('/radio', { type: 'RadioSignalStrength', signal_strength: this.radioStrength }, false);

      const course:any = {
        type: 'Course',
        num_waypoints: this.waypoints.length,
        waypoints: this.waypoints.map((waypoint) => ({
          type: 'Waypoint',
          gate: waypoint.gate,
          gate_width: waypoint.gate_width,
          id: waypoint.id,
          odom: Object.assign(waypoint.odom, { type: 'Odometry' }),
          search: waypoint.search
        }))
      };
      course.hash = fnvPlus.fast1a52(JSON.stringify(course));
      this.publish('/course', course, false);

      const zedGimbalPos:any = Object.assign(this.zedGimbalPos, { type: 'ZedGimbalPosition' });
      this.publish('/zed_gimbal_data', zedGimbalPos, false);
    }, TIME_INTERVAL_MILLI);
    /* eslint-enable @typescript-eslint/no-explicit-any */
  } /* created() */

  /* Stop sending periodic LCM messages before tearing down the application. */
  private beforeDestroy():void {
    window.clearInterval(this.intervalLcmPublish);
  }
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.col {
  display: flex;
  flex-direction: column;
  flex: 1;
}

.col.left {
  max-width: calc(99vw - 4px);
  margin-right: 5px;
  max-width: calc(min(600px, 75vh));
}

.flex-grid {
  display: flex;
  flex: 1;
  overflow: auto;
}

.wrapper {
  height: 100%;
  display: flex;
  flex-direction: column;
}
</style>
