<!-- This file contains the Waypoint component which is what displays a waypoint
     item in the field items component. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="waypoint-item">
    <div class="title-row">
      <b class="waypoint-item-name">Waypoint {{ index }}</b>
      <Button
        name="%D7"
        :invert-color="true"
        @clicked="deleteWaypointItem"
      />
    </div>
    <FieldItemInfo
      :location="waypoint.odom"
      :src="RoverLocationSource_.GPS"
    />
    <div class="draw-options-container">
      <div class="option search-options">
        <Checkbox
          :on="isSearchPoint"
          name="Search"
          @clicked="toggleIsSearchPoint"
        />
        <Checkbox
          :on="isGateSearchPoint"
          name="Gate Search"
          @clicked="toggleIsGateSearchPoint"
        />
      </div>
      <div class="search-options">
        <div class="option">
          <p>Target ID:</p>
          <NumberInput
            :val.sync="targetId"
            :min="-1"
            :max="249"
          />
        </div>
        <div class="option">
          <p>Gate Width:</p>
          <NumberInput
            :val.sync="gate_width"
            :min="1"
            :max="4"
            :step="0.5"
          />
          <p>m</p>
        </div>
      </div>
    </div>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Prop, Vue } from 'vue-property-decorator';
import { Mutation } from 'vuex-class';
import { RoverLocationSource, Waypoint } from '../../utils/types';
import Button from '../common/Button.vue';
import Checkbox from '../common/Checkbox.vue';
import NumberInput from '../common/NumberInput.vue';
import FieldItemInfo from './FieldItemInfo.vue';

@Component({
  components: {
    Button,
    Checkbox,
    NumberInput,
    FieldItemInfo
  }
})
export default class WaypointItem extends Vue {
  /************************************************************************************************
   * Types
   ************************************************************************************************/
  /* Redefine RoverLocationSource locally for use in the Template. */
  RoverLocationSource_ = RoverLocationSource;

  /************************************************************************************************
   * Props
   ************************************************************************************************/
  /* Index of waypoint item in list of waypoints. */
  @Prop({ required: true, type: Number })
  private readonly index!:number;

  /* Waypoint represented by this WaypointItem. */
  @Prop({ required: true, type: Object as ()=>Waypoint })
  private readonly waypoint!:Waypoint;

  /************************************************************************************************
   * Vuex Mutations
   ************************************************************************************************/
  @Mutation
  private readonly editWaypoint!:(updatedWaypoint:[Waypoint, number])=>void;

  @Mutation
  private readonly removeWaypoint!:(index:number)=>void;

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* Width of gate to search for at waypoint. Note this is only relevant if
     gate == true */
  private get gate_width():number {
    return this.waypoint.gate_width;
  }
  private set gate_width(newGateWidth:number) {
    this.editWaypoint([{
      gate: this.isGateSearchPoint,
      gate_width: newGateWidth,
      id: this.targetId,
      odom: this.waypoint.odom,
      search: this.isSearchPoint
    }, this.index]);
  }

  /* Whether or not the waypoint has at it. */
  private get isGateSearchPoint():boolean {
    return this.waypoint.gate;
  }
  private set isGateSearchPoint(newIsGateSearchPoint:boolean) {
    this.editWaypoint([{
      gate: newIsGateSearchPoint,
      gate_width: this.gate_width,
      id: this.targetId,
      odom: this.waypoint.odom,
      search: this.isSearchPoint
    }, this.index]);
  }

  /* Whether or not the waypoint is a search point. */
  private get isSearchPoint():boolean {
    return this.waypoint.search;
  }
  private set isSearchPoint(newIsSearchPoint:boolean) {
    this.editWaypoint([{
      gate: this.isGateSearchPoint,
      gate_width: this.gate_width,
      id: this.targetId,
      odom: this.waypoint.odom,
      search: newIsSearchPoint
    }, this.index]);
  }

  /* ID of target to search for at waypoint. Note this is only relevant if
     search == true and gate == false. */
  private get targetId():number {
    return this.waypoint.id;
  }
  private set targetId(newId:number) {
    this.editWaypoint([{
      gate: this.isGateSearchPoint,
      gate_width: this.gate_width,
      id: newId,
      odom: this.waypoint.odom,
      search: this.isSearchPoint
    }, this.index]);
  }

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* Delete the waypoint item and the corresponding waypoint. */
  private deleteWaypointItem():void {
    this.removeWaypoint(this.index);
  } /* deleteWaypointItem() */

  /* Set/unset the gate search option. */
  private toggleIsGateSearchPoint():void {
    this.isGateSearchPoint = !this.isGateSearchPoint;
    if (this.isGateSearchPoint) {
      this.isSearchPoint = true;
    }
  } /* toggleIsGateSearchPoint() */

  /* Set/unset the search option. */
  private toggleIsSearchPoint():void {
    this.isSearchPoint = !this.isSearchPoint;
    if (!this.isSearchPoint) {
      this.isGateSearchPoint = false;
    }
  } /* toggleIsSearchPoint() */
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.draw-options-container {
  display: flex;
  flex-wrap: wrap;
}

.option {
  margin-right: 10px;
}

.search-options {
  display: flex;
}

::v-deep .button-container input {
  min-height: 26px;
}

.waypoint-item {
  background-color: #BED;
}
</style>
