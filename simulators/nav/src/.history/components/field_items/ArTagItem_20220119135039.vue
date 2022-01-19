<!-- This file contains the ArTagItem component which is what displays an AR tag
     item in the field items component. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="ar-tag-item">
    <div class="title-row">
      <b class="ar-tag-item-name">Ar Tag {{ index }}</b>
      <Button
        name="%D7"
        :invert-color="true"
        @clicked="deleteArTagItem"
      />
    </div>
    <FieldItemInfo
      :location="arTag.odom"
      :src="RoverLocationSource_.ZED"
    />
    <div class="draw-options-container">
      <div>
        <p>ID:</p>
        <NumberInput
          :val.sync="id"
          :max="249"
          :min="0"
        />
      </div>
    </div>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Prop, Vue } from 'vue-property-decorator';
import { Mutation } from 'vuex-class';
import { ArTag, RoverLocationSource } from '../../utils/types';
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
export default class ArTagItem extends Vue {
  /************************************************************************************************
   * Types
   ************************************************************************************************/
  /* Redefine RoverLocationSource locally for use in the Template. */
  RoverLocationSource_ = RoverLocationSource;

  /************************************************************************************************
   * Props
   ************************************************************************************************/
  /* Index of AR tag item in list of AR tags. */
  @Prop({ required: true, type: Number })
  private readonly index!:number;

  /* ArTag represented by this ArTagItem. */
  @Prop({ required: true, type: Object as ()=>ArTag })
  private readonly arTag!:ArTag;

  /************************************************************************************************
   * Vuex Mutations
   ************************************************************************************************/
  @Mutation
  private readonly editArTag!:(updatedArTag:[ArTag, number])=>void;

  @Mutation
  private readonly removeArTag!:(index:number)=>void;

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* ID of the AR Tag */
  private get id():number {
    return this.arTag.id;
  }
  private set id(newId:number) {
    this.editArTag([{
      id: newId,
      odom: this.arTag.odom,
      orientation: this.arTag.orientation
    }, this.index]);
  }

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* Delete the AR Tag item and the corresponding AR Tag. */
  private deleteArTagItem():void {
    this.removeArTag(this.index);
  } /* deleteArTagItem() */
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.ar-tag-item {
  background-color: #bffcc6;
}

::v-deep.button-container input {
  min-height: 26px;
}
</style>
