<!-- This file contains the Simulator's hot key logic. There is nothing to
     render in this file. Note that, for the purposes of not having code
     duplication, not all hotkeys live in this file. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <!-- Render nothing. -->
  <div v-hotkey.prevent="keymap" />
</template>

<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { Getter, Mutation } from 'vuex-class';
import { ZED } from '../utils/constants';
import {
  FieldItemType,
  Joystick,
  Odom,
  OdomFormat,
  ZedGimbalPosition
} from '../utils/types';

@Component({})
export default class HotKeys extends Vue {
  /************************************************************************************************
   * Vuex Getters
   ************************************************************************************************/
  @Getter
  private readonly autonOn!:boolean;

  @Getter
  private readonly currOdom!:Odom;

  @Getter
  private readonly drawMode!:FieldItemType;

  @Getter
  private readonly fieldCenterOdom!:Odom;

  @Getter
  private readonly paused!:boolean;

  @Getter
  private readonly simulateLoc!:boolean;

  @Getter
  private readonly simulatePercep!:boolean;

  @Getter
  private readonly zedGimbalPos!:ZedGimbalPosition;

  /************************************************************************************************
   * Vuex Mutations
   ************************************************************************************************/
  @Mutation
  private readonly clearRoverPath!:()=>void;

  @Mutation
  private readonly flipSimulateLoc!:(onOff:boolean)=>void;

  @Mutation
  private readonly flipSimulatePercep!:(onOff:boolean)=>void;

  @Mutation
  private readonly setAutonState!:(onOff:boolean)=>void;

  @Mutation
  private readonly setCurrOdom!:(newOdom:Odom)=>void;

  @Mutation
  private readonly setDrawMode!:(mode:FieldItemType)=>void;

  @Mutation
  private readonly setJoystick!:(newJoystick:Joystick)=>void;

  @Mutation
  private readonly setOdomFormat!:(newOdomFormat:OdomFormat)=>void;

  @Mutation
  private readonly setPaused!:(paused:boolean)=>void;

  @Mutation
  private readonly setStartLoc!:(newStartLoc:Odom)=>void;

  @Mutation
  private readonly setTakeStep!:(takeStep:boolean)=>void;

  @Mutation
  private readonly setZedGimbalCmd!:(newZedGimbalCmd:ZedGimbalPosition)=>void;

  @Mutation
  private readonly setZedGimbalPos!:(newZedGimbalPos:ZedGimbalPosition)=>void;

  /************************************************************************************************
   * Private Members
   ************************************************************************************************/
  /* Whether or not clicking on the field recenters the field. */
  private prevDrawMode:FieldItemType|null = null;

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* Mapping of hotkeys to functions. */
  get keymap():Record<string, (()=>void)|(Record<string, ()=>void>)> {
    return {
      'shift+enter': this.flipAutonState,
      'shift+space': this.pausePlay,
      'shift+alt+space': this.takeStep,
      'tab': this.setToNextDrawMode,
      'shift+tab': this.setToPrevDrawMode,
      'shift+1': this.setToWaypointDrawMode,
      'shift+2': this.setToArTagDrawMode,
      'shift+3': this.setToGateDrawMode,
      'shift+4': this.setToObstacleDrawMode,
      'shift+5': this.setToReferencePointDrawMode,
      'shift+c': {
        keydown: this.toggleMoveRoverModeOn,
        keyup: this.toggleMoveRoverModeOff
      },
      'shift+alt+c': this.updateStartLoc,
      'shift+ctrl+c': this.resetStartLoc,
      'shift+l': this.flipSimLoc,
      'shift+p': this.flipSimPercep,
      'shift+alt+d': this.setToDFormat,
      'shift+alt+m': this.setToDMFormat,
      'shift+alt+s': this.setToDMSFormat,
      'shift+up': this.manualDriveForward,
      'shift+down': this.manualDriveBackward,
      'shift+left': this.manualDriveLeft,
      'shift+alt+left': this.manualGimbalLeft,
      'shift+right': this.manualDriveRight,
      'shift+alt+right': this.manualGimbalRight
    };
  }

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* Turn on and off the rover. */
  private flipAutonState():void {
    this.setAutonState(!this.autonOn);
  } /* flipAutonState() */

  /* Turn on and off simulating localization. */
  private flipSimLoc():void {
    this.flipSimulateLoc(!this.simulateLoc);
  } /* flipSimLoc() */

  /* Turn on and off simulating perception. */
  private flipSimPercep():void {
    this.flipSimulatePercep(!this.simulatePercep);
  } /* flipSimPercep() */

  /* Apply one joystick command based on "manual" input. */
  private manaulDrive(forwardBack:number, leftRight:number):void {
    if (!this.autonOn || this.paused) {
      this.setJoystick({
        forward_back: forwardBack,
        left_right: leftRight
      });
      this.setZedGimbalCmd(this.zedGimbalPos);
      this.setTakeStep(true);
    }
  } /* manaulDrive() */

  /* Apply one joystick command to drive backward based on "manual" input. */
  private manualDriveBackward():void {
    this.manaulDrive(-1, 0);
  } /* manualDriveBackward() */

  /* Apply one joystick command to drive forward based on "manual" input. */
  private manualDriveForward():void {
    this.manaulDrive(1, 0);
  } /* manualDriveForward() */

  /* Apply one joystick command to turn left based on "manual" input. */
  private manualDriveLeft():void {
    this.manaulDrive(0, -1);
  } /* manualDriveLeft() */

  /* Apply one joystick command to turn right based on "manual" input. */
  private manualDriveRight():void {
    this.manaulDrive(0, 1);
  } /* manualDriveRight() */

  /* Set the ZED gimbal angle based on "manual" input. */
  private manualGimbal(newAngle:number):void {
    if (!this.autonOn || this.paused) {
      this.setZedGimbalPos({
        angle: newAngle
      });
      this.setZedGimbalCmd({
        angle: newAngle
      });
    }
  } /* manualGimbal() */

  /* Apply one command to turn the ZED gimbal left based on "manual" input. */
  private manualGimbalLeft():void {
    this.manualGimbal(Math.max(this.zedGimbalPos.angle - 1, ZED.gimbal.minAngle));
  } /* manualGimbalLeft() */

  /* Apply one command to turn the ZED gimbal right based on "manual" input. */
  private manualGimbalRight():void {
    this.manualGimbal(Math.min(this.zedGimbalPos.angle + 1, ZED.gimbal.maxAngle));
  } /* manualGimbalRight() */

  /* Pause and play the rover. */
  private pausePlay():void {
    this.setPaused(!this.paused);
  } /* pausePlay() */

  /* Move the rover's starting location back to the field center. */
  private resetStartLoc():void {
    if (!this.autonOn) {
      this.setStartLoc(this.fieldCenterOdom);
      this.setCurrOdom(this.fieldCenterOdom);
      this.clearRoverPath();
    }
  } /* resetStartLoc() */

  /* Rotate through the draw modes in the forwards direction. */
  private setToNextDrawMode():void {
    switch (this.drawMode) {
      case FieldItemType.WAYPOINT: {
        this.setDrawMode(FieldItemType.AR_TAG);
        break;
      }

      case FieldItemType.AR_TAG: {
        this.setDrawMode(FieldItemType.GATE);
        break;
      }

      case FieldItemType.GATE: {
        this.setDrawMode(FieldItemType.OBSTACLE);
        break;
      }

      case FieldItemType.OBSTACLE: {
        this.setDrawMode(FieldItemType.REFERENCE_POINT);
        break;
      }

      case FieldItemType.REFERENCE_POINT: {
        this.setDrawMode(FieldItemType.WAYPOINT);
        break;
      }

      /* no default */
    }
  } /* setToNextDrawMode() */

  /* Rotate through the draw modes in the backwards direction. */
  private setToPrevDrawMode():void {
    switch (this.drawMode) {
      case FieldItemType.WAYPOINT: {
        this.setDrawMode(FieldItemType.REFERENCE_POINT);
        break;
      }

      case FieldItemType.AR_TAG: {
        this.setDrawMode(FieldItemType.WAYPOINT);
        break;
      }

      case FieldItemType.GATE: {
        this.setDrawMode(FieldItemType.AR_TAG);
        break;
      }

      case FieldItemType.OBSTACLE: {
        this.setDrawMode(FieldItemType.GATE);
        break;
      }

      case FieldItemType.REFERENCE_POINT: {
        this.setDrawMode(FieldItemType.OBSTACLE);
        break;
      }

      /* no default */
    }
  } /* setToPrevDrawMode() */

  /* Set the draw mode to waypoint. */
  private setToWaypointDrawMode():void {
    this.setDrawMode(FieldItemType.WAYPOINT);
  } /* setToWaypointDrawMode() */

  /* Set the draw mode to ar tag. */
  private setToArTagDrawMode():void {
    this.setDrawMode(FieldItemType.AR_TAG);
  } /* setToArTagDrawMode() */

  /* Set the draw mode to gate. */
  private setToGateDrawMode():void {
    this.setDrawMode(FieldItemType.GATE);
  } /* setToGateDrawMode() */

  /* Set the draw mode to obstacle. */
  private setToObstacleDrawMode():void {
    this.setDrawMode(FieldItemType.OBSTACLE);
  } /* setToObstacleDrawMode() */

  /* Set the draw mode to reference point. */
  private setToReferencePointDrawMode():void {
    this.setDrawMode(FieldItemType.REFERENCE_POINT);
  } /* setToReferencePointDrawMode() */

  /* Set the odom format to degrees. */
  private setToDFormat():void {
    this.setOdomFormat(OdomFormat.D);
  } /* setToDFormat() */

  /* Set the odom format to degrees, minutes. */
  private setToDMFormat():void {
    this.setOdomFormat(OdomFormat.DM);
  } /* setToDMFormat() */

  /* Set the odom format to degrees, minutes, seconds. */
  private setToDMSFormat():void {
    this.setOdomFormat(OdomFormat.DMS);
  } /* setToDMSFormat() */

  /* Take a debug step if the rover is paused. */
  private takeStep():void {
    if (this.paused) {
      this.setTakeStep(true);
    }
  } /* takeStep() */

  /* Switch from MOVE_ROVER draw mode back to previous draw mode. */
  private toggleMoveRoverModeOff():void {
    if (this.prevDrawMode === null) {
      console.log('ERROR: Previous Draw Mode is null.');
      return;
    }
    this.setDrawMode(this.prevDrawMode);
    this.prevDrawMode = null;
  } /* toggleMoveRoverModeOff() */

  /* Switch to MOVE_ROVER draw mode and save previous draw mode. */
  private toggleMoveRoverModeOn():void {
    if (this.drawMode !== FieldItemType.MOVE_ROVER) {
      this.prevDrawMode = this.drawMode;
      this.setDrawMode(FieldItemType.MOVE_ROVER);
    }
  } /* toggleMoveRoverModeOn() */

  /* Make the current odom the new start location. */
  private updateStartLoc():void {
    if (!this.autonOn) {
      this.setStartLoc(this.currOdom);
      this.clearRoverPath();
    }
  } /* updateStartLoc() */
}
</script>
