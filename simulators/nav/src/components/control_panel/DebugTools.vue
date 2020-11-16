<!-- This file contains the DebugTools component. This contains IDE features
     (e.g. pause) and rover optons (e.g. field of view and speed). -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <fieldset class="box debug-tools">
    <legend>Debug Tools</legend>
    <!-- IDE Features (e.g. pause, step, etc.) -->
    <div class="ide-features">
      <Button
        name="Reset Rover"
        :disabled="!simulateLocalization"
        @clicked="resetRover"
      />
      <Checkbox
        class="play-pause"
        :on="!paused"
        :name="playPauseDisplay"
        :disabled="!simulateLocalization"
        @clicked="playPause"
      />
      <Button
        name="Step"
        :disabled="!paused || !simulateLocalization"
        @clicked="step"
      />
      <Checkbox
        :on="drawFovIn"
        name="Draw Rover Visibility"
        @clicked="toggleDrawFov"
      />
    </div>

    <div class="rover-options">
      <!-- Field of View Options -->
      <div class="fov-options left-col">
        <div class="setting">
          <p>FOV Angle:</p>
          <NumberInput
            :val.sync="fovAngleIn"
            :min="0"
            :max="360"
          />
          <p>º</p>
        </div>
        <div class="setting">
          <p>FOV Depth:</p>
          <NumberInput
            :val.sync="fovDepthIn"
            :min="0"
            :max="10"
            :step="0.5"
          />
          <p>m</p>
        </div>
      </div>

      <!-- Rover Speed -->
      <div class="speeds">
        <div class="setting">
          <p>Drive Speed:</p>
          <NumberInput
            :val.sync="driveSpeed"
            :min="driveSpeedMin"
            :max="5"
            :step="driveSpeedStep"
          />
          <p>m/s</p>
        </div>
        <div class="setting">
          <p>Turn Speed:</p>
          <NumberInput
            :val.sync="turnSpeed"
            :min="turnSpeedMin"
            :max="90"
            :step="turnSpeedStep"
          />
          <p>º/s</p>
        </div>
      </div>
    </div>
  </fieldset>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { Getter, Mutation } from 'vuex-class';
import { FieldOfViewOptions, Odom, Speeds } from '../../utils/types';
import Button from '../common/Button.vue';
import Checkbox from '../common/Checkbox.vue';
import NumberInput from '../common/NumberInput.vue';

/**************************************************************************************************
 * Constants
 **************************************************************************************************/
const MIN_DRIVE_SPEED = 0.1; /* meters/second */
const MIN_TURN_SPEED = 1; /* degrees/second */
const DRIVE_STEP = 0.5; /* meters/second */
const TURN_STEP = 5; /* degrees/second */

@Component({
  components: {
    Button,
    Checkbox,
    NumberInput
  }
})
export default class DebugTools extends Vue {
  /************************************************************************************************
   * Vuex Getters
   ************************************************************************************************/
  @Getter
  private readonly currSpeed!:Speeds;

  @Getter
  private readonly fieldCenterOdom!:Odom;

  @Getter
  private readonly fieldOfViewOptions!:FieldOfViewOptions;

  @Getter
  private readonly paused!:boolean;

  @Getter
  private readonly simulateLocalization!:boolean;

  @Getter
  private readonly takeStep!:boolean;

  /************************************************************************************************
   * Vuex Mutations
   ************************************************************************************************/
  @Mutation
  private readonly setAutonState!:(onOff:boolean)=>void;

  @Mutation
  private readonly setCurrOdom!:(newOdom:Odom)=>void;

  @Mutation
  private readonly setCurrSpeed!:(newSpeeds:Speeds)=>void;

  @Mutation
  private readonly setFieldOfViewOptions!:(options:FieldOfViewOptions)=>void;

  @Mutation
  private readonly setPaused!:(paused:boolean)=>void;

  @Mutation
  private readonly setRepeaterLoc!:(newRepeaterLoc:Odom|null)=>void;

  @Mutation
  private readonly setTakeStep!:(takeStep:boolean)=>void;

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* Displayed whether or not to draw the field of view. */
  private get drawFovIn():boolean {
    return this.fieldOfViewOptions.visible;
  }
  private set drawFovIn(newVisible:boolean) {
    this.setFieldOfViewOptions({
      angle: this.fovAngleIn,
      depth: this.fovDepthIn,
      visible: newVisible
    });
  }

  /* Displayed field of view angle. */
  private get fovAngleIn():number {
    return this.fieldOfViewOptions.angle;
  }
  private set fovAngleIn(newAngle:number) {
    this.setFieldOfViewOptions({
      angle: newAngle,
      depth: this.fovDepthIn,
      visible: this.drawFovIn
    });
  }

  /* Displayed field of view depth. */
  private get fovDepthIn():number {
    return this.fieldOfViewOptions.depth;
  }
  private set fovDepthIn(newDepth:number) {
    this.setFieldOfViewOptions({
      angle: this.fovAngleIn,
      depth: newDepth,
      visible: this.drawFovIn
    });
  }

  /* Word to display on Play/Pause button. */
  private get playPauseDisplay():string {
    if (this.paused) {
      return 'Play';
    }
    return 'Pause';
  }

  /* Drive Speed */
  private get driveSpeed():number {
    return this.currSpeed.drive;
  }
  private set driveSpeed(newSpeed:number) {
    /* Check for out of range caused by dynamic min value. */
    const speed:number = Math.max(newSpeed, MIN_DRIVE_SPEED);
    this.setCurrSpeed({
      drive: speed,
      turn: this.currSpeed.turn
    });
  }
  private get driveSpeedMin():number {
    return this.driveSpeed === MIN_DRIVE_SPEED ? MIN_DRIVE_SPEED : 0;
  }
  private get driveSpeedStep():number {
    return this.driveSpeed === MIN_DRIVE_SPEED ? DRIVE_STEP - MIN_DRIVE_SPEED : DRIVE_STEP;
  }

  /* Turn Speed */
  private get turnSpeed():number {
    return this.currSpeed.turn;
  }
  private set turnSpeed(newSpeed:number) {
    /* Check for out of range caused by dynamic min value. */
    const speed:number = Math.max(newSpeed, MIN_TURN_SPEED);
    this.setCurrSpeed({
      drive: this.currSpeed.drive,
      turn: speed
    });
  }
  private get turnSpeedMin():number {
    return this.turnSpeed === MIN_TURN_SPEED ? MIN_TURN_SPEED : 0;
  }
  private get turnSpeedStep():number {
    return this.turnSpeed === MIN_TURN_SPEED ? TURN_STEP - MIN_TURN_SPEED : TURN_STEP;
  }

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* Play/pause the rover. */
  private playPause():void {
    this.setPaused(!this.paused);
  } /* playPause() */

  /* Reset the rover to the starting state. */
  private resetRover():void {
    this.setCurrOdom(this.fieldCenterOdom);
    this.setAutonState(false);
    this.setPaused(false);
  } /* resetRover() */

  /* When paused, execute a single joystick command. */
  private step():void {
    this.setTakeStep(true);
  } /* step() */

  /* Flip on/off drawing the field of view. */
  private toggleDrawFov():void {
    this.drawFovIn = !this.drawFovIn;
  } /* toggleDrawFov() */
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.ide-features {
  display: flex;
  flex-wrap: wrap;
}

.left-col {
  margin-right: 20px;
}

p {
  display: inline-block;
}

.play-pause::v-deep p {
  width: 44px;
}

.rover-options {
  display: flex;
  flex-wrap: wrap;
}

.setting {
  margin-right: 5px;
}

.setting:last-child {
  margin-right: 5px;
}

::v-deep .button-container {
  flex: 1;
}

::v-deep .button-container input {
  min-height: 26px;
}
</style>
