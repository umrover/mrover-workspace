<!-- This file contains the GateItem component which is what displays a gate
     item in the field items component. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="gate-item">
    <div class="title-row">
      <b class="gate-item-name">Gate {{ index }}</b>
      <Button
        name="%D7"
        :invert-color="true"
        @clicked="deleteGateItem"
      />
    </div>
    <div class="posts">
      <div class="container left-post post">
        <b>Left Post</b>
        <FieldItemInfo
          :location="leftPostOdom"
          :src="RoverLocationSource_.ZED"
        />
      </div>
      <div class="container post right-post">
        <b>Right Post</b>
        <FieldItemInfo
          :location="rightPostOdom"
          :src="RoverLocationSource_.ZED"
        />
      </div>
    </div>
    <div class="draw-options-container">
      <div class="left-post-id option">
        <p>Left ID:</p>
        <NumberInput
          :val.sync="leftId"
          :max="249"
          :min="1"
          :step="2"
        />
      </div>
      <div class="option right-post-id">
        <p>Right ID:</p>
        <NumberInput
          :val.sync="rightId"
          :max="248"
          :min="0"
          :step="2"
        />
      </div>
      <div class="option">
        <p>Width:</p>
        <NumberInput
          :val.sync="width"
          :max="4"
          :min="1"
          :step="0.5"
        />
      </div>
      <div class="option">
        <p>Orientation:</p>
        <!-- Extend range by 1 degree in each direction to allow for continuous
             circular range -->
        <NumberInput
          :val.sync="orientation"
          :min="-1"
          :max="360"
        />
      </div>
    </div>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Prop, Vue } from 'vue-property-decorator';
import { Getter, Mutation } from 'vuex-class';
import { Gate, Odom, RoverLocationSource } from '../../utils/types';
import { compassModDeg, calcRelativeOdom } from '../../utils/utils';
import Button from '../common/Button.vue';
import NumberInput from '../common/NumberInput.vue';
import FieldItemInfo from './FieldItemInfo.vue';

@Component({
  components: {
    Button,
    NumberInput,
    FieldItemInfo
  }
})
export default class GateItem extends Vue {
  /************************************************************************************************
   * Types
   ************************************************************************************************/
  /* Redefine RoverLocationSource locally for use in the Template. */
  RoverLocationSource_ = RoverLocationSource;

  /************************************************************************************************
   * Props
   ************************************************************************************************/
  /* Index of gate item in list of gates. */
  @Prop({ required: true, type: Number })
  private readonly index!:number;

  /* Gate represented by this GateItem. */
  @Prop({ required: true, type: Object as ()=>Gate })
  private readonly gate!:Gate;

  /************************************************************************************************
   * Vuex Getters
   ************************************************************************************************/
  @Getter
  private readonly fieldCenterOdom!:Odom;

  /************************************************************************************************
   * Vuex Mutations
   ************************************************************************************************/
  @Mutation
  private readonly editGate!:(updatedGate:[Gate, number])=>void;

  @Mutation
  private readonly removeGate!:(index:number)=>void;

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* Width of gate. */
  private get width():number {
    return this.gate.width;
  }
  private set width(newWidth:number) {
    this.editGate([{
      leftId: this.gate.leftId,
      rightId: this.gate.rightId,
      odom: this.gate.odom,
      orientation: this.gate.orientation,
      width: newWidth
    }, this.index]);
  }

  /* ID of AR tag on left post. */
  private get leftId():number {
    return this.gate.leftId;
  }
  private set leftId(newLeftId:number) {
    this.editGate([{
      leftId: newLeftId,
      rightId: this.gate.rightId,
      odom: this.gate.odom,
      orientation: this.gate.orientation,
      width: this.gate.width
    }, this.index]);
  }

  /* GPS location of the left post. */
  private get leftPostOdom():Odom {
    return calcRelativeOdom(this.gate.odom, this.gate.orientation + 180,
                            this.gate.width / 2, this.fieldCenterOdom);
  }

  /* Orientation of gate in degrees from north (angle made from left to right
     post) */
  private get orientation():number {
    return this.gate.orientation;
  }
  private set orientation(newOrientation:number) {
    this.editGate([{
      leftId: this.gate.leftId,
      rightId: this.gate.rightId,
      odom: this.gate.odom,
      orientation: compassModDeg(newOrientation),
      width: this.gate.width
    }, this.index]);
  }

  /* ID of AR tag on right post. */
  private get rightId():number {
    return this.gate.rightId;
  }
  private set rightId(newRightId:number) {
    this.editGate([{
      leftId: this.gate.leftId,
      rightId: newRightId,
      odom: this.gate.odom,
      orientation: this.gate.orientation,
      width: this.gate.width
    }, this.index]);
  }

  /* GPS location of the right post. */
  private get rightPostOdom():Odom {
    return calcRelativeOdom(this.gate.odom, this.gate.orientation,
                            this.gate.width / 2, this.fieldCenterOdom);
  }

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* Delete the gate item and the corresponding gate. */
  private deleteGateItem():void {
    this.removeGate(this.index);
  } /* deleteGateItem() */
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.gate-item {
  background-color: #C9BBEE;
}

.left-post {
  background-color: #ff9;
}

.left-post-id::v-deep input {
  background-color: #ff9;
}

.option {
  margin-right: 10px;
}

.post {
  border: #555 solid 1px;
  flex: 1;
}

.post:not(:first-child) {
  margin-left: 5px;
}

.posts {
  display: flex;
  flex-wrap: wrap;
}

.right-post {
  background-color: #9df;
}

.right-post-id::v-deep input {
  background-color: #9df;
}

::v-deep.button-container input {
  min-height: 26px;
}
</style>
