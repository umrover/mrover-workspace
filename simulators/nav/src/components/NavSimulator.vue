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
    </div>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import fnvPlus from 'fnv-plus';
import { Component, Vue, Watch } from 'vue-property-decorator';
import { Getter, Mutation } from 'vuex-class';
import { RADIO } from '../utils/constants';
import { applyJoystick as applyJoystickUtil } from '../utils/utils';
import {
  Joystick,
  NavStatus,
  ObstacleMessage,
  Odom,
  Speeds,
  TargetListMessage,
  Waypoint
} from '../utils/types';
import ControlPanel from './control_panel/ControlPanel.vue';
import Field from './field/Field.vue';
import FieldItems from './field_items/FieldItems.vue';
import Header from './header/Header.vue';
import LCMBridge from '../../deps/lcm_bridge_client/dist/bridge.js';
import LCMCenter from './lcm_center/LCMCenter.vue';
import Perception from './perception/Perception.vue';

/**************************************************************************************************
 * Constants
 **************************************************************************************************/
/* Frequency in milliseconds to publish outgoing LCM messages at. */
const TIME_INTERVAL = 100;

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
  private readonly fieldCenterOdom!:Odom;

  @Getter
  private readonly joystick!:Joystick;

  @Getter
  private readonly currSpeed!:Speeds;

  @Getter
  private readonly obstacleMessage!:ObstacleMessage;

  @Getter
  private readonly paused!:boolean;

  @Getter
  private readonly radioStrength!:number;

  @Getter
  private readonly repeaterLoc!:Odom|null;

  @Getter
  private readonly simulateLocalization!:boolean;

  @Getter
  private readonly simulatePerception!:boolean;

  @Getter
  private readonly takeStep!:boolean;

  @Getter
  private readonly targetList!:TargetListMessage;

  @Getter
  private readonly waypoints!:Waypoint[];

  /************************************************************************************************
   * Vuex Mutations
   ************************************************************************************************/
  @Mutation
  private readonly flipLcmConnected!:(onOff:boolean)=>void;

  @Mutation
  private readonly setCurrOdom!:(newOdom:Odom)=>void;

  @Mutation
  private readonly setJoystick!:(newJoystick:Joystick)=>void;

  @Mutation
  private readonly setNavStatus!:(newNavStatus:NavStatus)=>void;

  @Mutation
  private readonly setObstacleMessage!:(newObstacle:ObstacleMessage)=>void;

  @Mutation
  private readonly setRadioStrength!:(strength:number)=>void;

  @Mutation
  private readonly setRepeaterLoc!:(newRepeaterLoc:Odom|null)=>void;

  @Mutation
  private readonly setTakeStep!:(takeStep:boolean)=>void;

  @Mutation
  private readonly setTargetList!:(newTargetList:TargetListMessage)=>void;

  /************************************************************************************************
   * Private Members
   ************************************************************************************************/
  /* LCM Bridge Client for LCM communications in and out of simulator. */
  private lcmBridge!:LCMBridge;

  /* Interval to publish outgoing LCM messages with. */
  private intervalLcmPublish!:number;

  /* Whether or not we are in the process of dropping the repeater. */
  private droppingRepeater = false;

  /************************************************************************************************
   * Watchers
   ************************************************************************************************/
  @Watch('paused')
  private onUnpause(paused:boolean):void {
    if (!paused) {
      this.applyJoystick();
    }
  }

  @Watch('takeStep')
  private onTakeStep():void {
    this.applyJoystick();
    this.setTakeStep(false);
  }

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* Apply the current joystick message to move the rover. Update the current
     odometry based on this movement. */
  private applyJoystick():void {
    /* if not simulating localization, nothing to do */
    if (!this.simulateLocalization) {
      return;
    }

    this.setCurrOdom(applyJoystickUtil(this.currOdom, this.fieldCenterOdom, this.joystick,
                                       TIME_INTERVAL / ONE_SECOND_MILLI, this.currSpeed));
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
  private publish(channel:string, payload:any):void {
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
        if (msg.topic === '/autonomous') {
          this.setJoystick({
            forward_back: msg.message.forward_back,
            left_right: msg.message.left_right
          });
          if (!this.paused) {
            this.applyJoystick();
          }
        }
        else if (msg.topic === '/nav_status') {
          this.setNavStatus(msg.message);
        }
        else if (msg.topic === '/obstacle') {
          if (!this.simulatePerception) {
            this.setObstacleMessage(msg.message);
          }
        }
        else if (msg.topic === '/odometry') {
          if (!this.simulateLocalization) {
            this.setCurrOdom(msg.message);
          }
        }
        else if (msg.topic === '/rr_drop_init') {
          this.dropRepeater();
        }
        else if (msg.topic === '/target_list') {
          if (!this.simulatePerception) {
            this.setTargetList(msg.message.targetList);
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
      },

      /* Subscriptions */
      [
        { topic: '/autonomous',   type: 'Joystick' },
        { topic: '/nav_status',    type: 'NavStatus' },
        { topic: '/obstacle',     type: 'Obstacle' },
        { topic: '/odometry',     type: 'Odometry' },
        { topic: '/rr_drop_init', type: 'RepeaterDropInit' },
        { topic: '/target_list',   type: 'TargetList' },
        { topic: '/debugMessage', type: 'DebugMessage' }
      ]
    );

    /* Set up publishing LCMs */
    this.intervalLcmPublish = window.setInterval(() => {
      this.publish('/auton', { type: 'AutonState', is_auton: this.autonOn });

      if (this.simulateLocalization) {
        const odom:any = Object.assign(this.currOdom, { type: 'Odometry' });
        this.publish('/odometry', odom);
      }

      if (this.simulatePerception) {
        const obs:any = Object.assign(this.obstacleMessage, { type: 'Obstacle' });
        this.publish('/obstacle', obs);

        const targetList:any = { targetList: this.targetList, type: 'TargetList' };
        targetList.targetList[0].type = 'Target';
        targetList.targetList[1].type = 'Target';
        this.publish('/target_list', targetList);
      }

      if (this.repeaterLoc !== null) {
        this.publish('/rr_drop_complete', { type: 'RepeaterDropComplete' });
      }

      this.publish('/radio', { type: 'RadioSignalStrength', signal_strength: this.radioStrength });

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
      this.publish('/course', course);
    }, TIME_INTERVAL);

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
