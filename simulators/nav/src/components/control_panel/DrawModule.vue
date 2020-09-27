<!-- This file contains the DrawModule component which controls options for
     placing items on the field/canvas. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <fieldset class="box draw-module">
    <legend>Draw Module</legend>
    <RadioSelector
      :options="drawModeOptions"
      :selection="drawMode"
      @selected="selectDrawMode"
    />

    <div class="draw-options-container">
      <!-- OBSTACLE -->
      <div
        :class="{ 'invisible': drawMode !== DrawModeType.OBSTACLE }"
        class="draw-options obstacle-draw-options"
      >
        <div class="size">
          <p>Size:</p>
          <NumberInput
            :val.sync="obstacleSizeIn"
            :min="obstacleSizeStep"
            :max="50"
            :step="obstacleSizeStep"
          />
          <p>m</p>
        </div>
      </div>

      <!-- AR TAG -->
      <div
        :class="{ 'invisible': drawMode !== DrawModeType.AR_TAG }"
        class="ar-tag-draw-options draw-options"
      >
        <div class="targets">
          <div>
            <div>
              <p>Target ID:</p>
              <NumberInput
                :val.sync="arTagIdIn"
                :min="0"
                :max="249"
              />
            </div>
          </div>
        </div>
      </div>

      <!-- GATE -->
      <div
        :class="{ 'invisible': drawMode !== DrawModeType.GATE }"
        class="draw-options gate-draw-options"
      >
        <div class="options-row">
          <div>
            <div class="left-col">
              <p>Left Target ID:</p>
              <NumberInput
                :val.sync="gateLeftPostIdIn"
                :min="1"
                :max="249"
                :step="2"
              />
            </div>
            <div>
              <p>Right Target ID:</p>
              <NumberInput
                :val.sync="gateRightPostIdIn"
                :min="0"
                :max="248"
                :step="2"
              />
            </div>
          </div>
          <div>
            <div class="size">
              <p>Width:</p>
              <NumberInput
                :val.sync="gateWidthIn"
                :min="1"
                :max="4"
                :step="0.5"
              />
              <p>m</p>
            </div>
            <div class="orientation">
              <p>Orientation:</p>
              <!-- Extend range by 1 degree in each direction for continuous
                   circular range -->
              <NumberInput
                :val.sync="gateOrientationIn"
                :min="-1"
                :max="360"
              />
              <p>º</p>
            </div>
          </div>
        </div>
      </div>

      <!-- The largest draw-options must go last for styling purposes.
           Additionally it must have the class 'largest-draw-options'. -->
      <!-- WAYPOINT -->
      <div
        :class="{ 'invisible': drawMode !== DrawModeType.WAYPOINT }"
        class="draw-options largest-draw-options waypoint-draw-options"
      >
        <div class="options-row">
          <div class="left-col waypoint-target">
            <p>Associated Target ID:</p>
            <NumberInput
              :val.sync="waypointIdIn"
              :min="-1"
              :max="249"
            />
          </div>

          <div class="size">
            <p>Associated Gate Width:</p>
            <NumberInput
              :val.sync="waypointGateWidthIn"
              :min="1"
              :max="4"
              :step="0.5"
            />
            <p>m</p>
          </div>
        </div>

        <div class="search-options">
          <Checkbox
            :on="waypointIsSearch"
            name="Search Point"
            @clicked="toggleIsSearchPoint"
          />
          <Checkbox
            :on="waypointIsGateSearch"
            name="Gate Search Point"
            @clicked="toggleIsGateSearchPoint"
          />
        </div>
      </div>
    </div>
  </fieldset>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { Getter, Mutation } from 'vuex-class';
import {
  ArTagDrawOptions,
  FieldItemType,
  GateDrawOptions,
  ObstacleDrawOptions,
  RadioOption,
  WaypointDrawOptions
} from '../../utils/types';
import { compassModDeg } from '../../utils/utils';
import Button from '../common/Button.vue';
import Checkbox from '../common/Checkbox.vue';
import NumberInput from '../common/NumberInput.vue';
import RadioSelector from '../common/RadioSelector.vue';

/**************************************************************************************************
 * Constants
 **************************************************************************************************/
/* largest size of a small obstacle */
const SMALL_OBS_CUTOFF = 2; /* meters */

/* step size for a small obstacle */
const SMALL_OBS_STEP = 0.25; /* meters */

/* largest size of a medium obstacle */
const MED_OBS_CUTOFF = 5; /* meters */

/* step size for a medium obstacle */
const MED_OBS_STEP = 0.5; /* meters */

/* step size for a large obstacle */
const LARGE_OBS_STEP = 1; /* meters */

@Component({
  components: {
    Button,
    Checkbox,
    NumberInput,
    RadioSelector
  }
})
export default class DrawModeModule extends Vue {
  /************************************************************************************************
   * Types
   ************************************************************************************************/
  /* Redefine FieldItemType locally for use in the Template. */
  private readonly DrawModeType = FieldItemType;

  /************************************************************************************************
   * Vuex Getters
   ************************************************************************************************/
  @Getter
  private readonly arTagDrawOptions!:ArTagDrawOptions;

  @Getter
  private readonly drawMode!:FieldItemType;

  @Getter
  private readonly gateDrawOptions!:GateDrawOptions;

  @Getter
  private readonly obstacleDrawOptions!:ObstacleDrawOptions;

  @Getter
  private readonly waypointDrawOptions!:WaypointDrawOptions;

  /************************************************************************************************
   * Vuex Mutations
   ************************************************************************************************/
  @Mutation
  private readonly setArTagDrawOptions!:(options:ArTagDrawOptions)=>void;

  @Mutation
  private readonly setDrawMode!:(mode:FieldItemType)=>void;

  @Mutation
  private readonly setGateDrawOptions!:(options:GateDrawOptions)=>void;

  @Mutation
  private readonly setObstacleDrawOptions!:(options:ObstacleDrawOptions)=>void;

  @Mutation
  private readonly setWaypointDrawOptions!:(options:WaypointDrawOptions)=>void;

  /************************************************************************************************
   * Private Members
   ************************************************************************************************/
  /* Draw mode options */
  private readonly drawModeOptions:RadioOption<FieldItemType>[] = [
    { value: FieldItemType.WAYPOINT, name: 'Waypoint' },
    { value: FieldItemType.AR_TAG,   name: 'AR Tag' },
    { value: FieldItemType.GATE,     name: 'Gate' },
    { value: FieldItemType.OBSTACLE, name: 'Obstacle' }
  ];

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* -------------------------------- AR Tag -------------------------------- */
  /* ID of ar tag */
  private get arTagIdIn():number {
    return this.arTagDrawOptions.id;
  }
  private set arTagIdIn(newId:number) {
    this.setArTagDrawOptions({ id: newId });
  }

  /* --------------------------------- Gate --------------------------------- */
  /* ID of left gate post */
  private get gateLeftPostIdIn():number {
    return this.gateDrawOptions.leftPostId;
  }
  private set gateLeftPostIdIn(newLeftId:number) {
    this.setGateDrawOptions({
      leftPostId: newLeftId,
      rightPostId: this.gateRightPostIdIn,
      width: this.gateWidthIn,
      orientation: this.gateOrientationIn
    });
  }

  /* ID of right gate post */
  private get gateRightPostIdIn():number {
    return this.gateDrawOptions.rightPostId;
  }
  private set gateRightPostIdIn(newRightId:number) {
    this.setGateDrawOptions({
      leftPostId: this.gateLeftPostIdIn,
      rightPostId: newRightId,
      width: this.gateWidthIn,
      orientation: this.gateOrientationIn
    });
  }

  /* width of gate */
  private get gateWidthIn():number {
    return this.gateDrawOptions.width;
  }
  private set gateWidthIn(newWidth:number) {
    this.setGateDrawOptions({
      leftPostId: this.gateLeftPostIdIn,
      rightPostId: this.gateRightPostIdIn,
      width: newWidth,
      orientation: this.gateOrientationIn
    });
  }

  /* Orientation of gate in degrees from north (angle made from left to right
     post) */
  private get gateOrientationIn():number {
    return this.gateDrawOptions.orientation;
  }
  private set gateOrientationIn(newOrientation:number) {
    this.setGateDrawOptions({
      leftPostId: this.gateLeftPostIdIn,
      rightPostId: this.gateRightPostIdIn,
      width: this.gateWidthIn,
      orientation: compassModDeg(newOrientation)
    });
  }

  /* ------------------------------- Obstacle ------------------------------- */
  /* Size of obstacle */
  private get obstacleSizeIn():number {
    return this.obstacleDrawOptions.size;
  }
  private set obstacleSizeIn(newSize:number) {
    this.setObstacleDrawOptions({ size: newSize });
  }
  private get obstacleSizeStep():number {
    if (this.obstacleSizeIn < SMALL_OBS_CUTOFF) {
      return SMALL_OBS_STEP;
    }
    if (this.obstacleSizeIn < MED_OBS_CUTOFF) {
      return MED_OBS_STEP;
    }
    return LARGE_OBS_STEP;
  }

  /* ------------------------------- Waypoint ------------------------------- */
  /* ID of target to search for at waypoint. Note this is only relevant if
     search == true and gate == false. */
  private get waypointIdIn():number {
    return this.waypointDrawOptions.targetId;
  }
  private set waypointIdIn(newId:number) {
    this.setWaypointDrawOptions({
      targetId: newId,
      gate: this.waypointIsGateSearch,
      gate_width: this.waypointGateWidthIn,
      search: this.waypointIsSearch
    });
  }

  /* Whether or not waypoint is a gate search point */
  private get waypointIsGateSearch():boolean {
    return this.waypointDrawOptions.gate;
  }
  private set waypointIsGateSearch(newIsGateSearch:boolean) {
    this.setWaypointDrawOptions({
      targetId: this.waypointIdIn,
      gate: newIsGateSearch,
      gate_width: this.waypointGateWidthIn,
      search: this.waypointIsSearch
    });
  }

  /* Width of gate to search for at waypoint. Note this is only relevant if
     gate == true */
  private get waypointGateWidthIn():number {
    return this.waypointDrawOptions.gate_width;
  }
  private set waypointGateWidthIn(newWidth:number) {
    this.setWaypointDrawOptions({
      targetId: this.waypointIdIn,
      gate: this.waypointIsGateSearch,
      gate_width: newWidth,
      search: this.waypointIsSearch
    });
  }

  /* Whether or not a waypoint is a search point. Note this is only relevant if
     gate == true */
  private get waypointIsSearch():boolean {
    return this.waypointDrawOptions.search;
  }
  private set waypointIsSearch(newIsSearch:boolean) {
    this.setWaypointDrawOptions({
      targetId: this.waypointIdIn,
      gate: this.waypointIsGateSearch,
      gate_width: this.waypointGateWidthIn,
      search: newIsSearch
    });
  }

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* Set/unset the gate search option. */
  private toggleIsGateSearchPoint():void {
    this.waypointIsGateSearch = !this.waypointIsGateSearch;
    if (this.waypointIsGateSearch) {
      this.waypointIsSearch = true;
    }
  } /* toggleIsGateSearchPoint() */

  /* Set/unset the search option. */
  private toggleIsSearchPoint():void {
    this.waypointIsSearch = !this.waypointIsSearch;
    if (!this.waypointIsSearch) {
      this.waypointIsGateSearch = false;
    }
  } /* toggleIsSearchPoint() */

  /* Select current draw mode. */
  private selectDrawMode(selectedDrawMode:FieldItemType):void {
    this.setDrawMode(selectedDrawMode);
  } /* selectDrawMode() */
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.draw-options {
  position: absolute;
  transition: 0.5s;
  width: 100%;
}

.draw-options-container {
  position: relative;
}

.invisible {
  opacity: 0;
}
:not(.invisible) {
  z-index: 1;
}

.largest-draw-options {
  position: relative;
}

.left-col {
  margin-right: 20px;
}

.options-row {
  display: flex;
  flex-wrap: wrap;
}

p {
  display: inline-block;
}

.search-options {
  display: flex;
  flex-wrap: wrap;
}

.size input {
  min-width: 60px;
}
</style>
