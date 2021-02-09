<!-- This file contains the Obstacle component which is what displays an
     obstacle item in the field items component. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="obstacle-item">
    <div class="title-row">
      <b class="obstacle-item-name">Obstacle {{ index }}</b>
      <Button
        name="%D7"
        :invert-color="true"
        @clicked="deleteObstacleItem"
      />
    </div>
    <FieldItemInfo
      :location="obstacle.odom"
      :src="RoverLocationSource_.ZED"
    />
    <div class="draw-options-container">
      <div>
        <p>Size:</p>
        <NumberInput
          :val.sync="size"
          :max="50"
          :min="stepSize"
          :step="stepSize"
        />
        <p>m</p>
      </div>
    </div>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Prop, Vue } from 'vue-property-decorator';
import { Mutation } from 'vuex-class';
import { Obstacle, RoverLocationSource } from '../../utils/types';
import Button from '../common/Button.vue';
import NumberInput from '../common/NumberInput.vue';
import FieldItemInfo from './FieldItemInfo.vue';

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
    NumberInput,
    FieldItemInfo
  }
})
export default class ObstacleItem extends Vue {
  /************************************************************************************************
   * Types
   ************************************************************************************************/
  /* Redefine RoverLocationSource locally for use in the Template. */
  RoverLocationSource_ = RoverLocationSource;

  /************************************************************************************************
   * Props
   ************************************************************************************************/
  /* Index of obstacle item in list of obstacles. */
  @Prop({ required: true, type: Number })
  private readonly index!:number;

  /* Obstacle represented by this ObstacleItem. */
  @Prop({ required: true, type: Object as ()=>Obstacle })
  private readonly obstacle!:Obstacle;

  /************************************************************************************************
   * Vuex Mutations
   ************************************************************************************************/
  @Mutation
  private readonly editObstacle!:(updatedObstacle:[Obstacle, number])=>void;

  @Mutation
  private readonly removeObstacle!:(index:number)=>void;

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* Size of obstacle. */
  private get size():number {
    return this.obstacle.size;
  }
  private set size(newSize:number) {
    this.editObstacle([{
      odom: this.obstacle.odom,
      size: newSize
    }, this.index]);
  }
  private get stepSize():number {
    if (this.size < SMALL_OBS_CUTOFF) {
      return SMALL_OBS_STEP;
    }
    if (this.size < MED_OBS_CUTOFF) {
      return MED_OBS_STEP;
    }
    return LARGE_OBS_STEP;
  }

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* Delete the obstacle item and the corresponding obstacle. */
  private deleteObstacleItem():void {
    this.removeObstacle(this.index);
  } /* deleteObstacleItem() */
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.obstacle-item {
  background-color: #fc8;
}

::v-deep.button-container input {
  min-height: 26px;
}
</style>
