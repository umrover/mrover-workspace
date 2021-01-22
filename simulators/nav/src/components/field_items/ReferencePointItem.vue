<!-- This file contains the Waypoint component which is what displays a waypoint
     item in the field items component. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="reference-point-item">
    <div class="title-row">
      <b>Reference Point {{ index }}</b>
      <Button
        name="%D7"
        :color-scheme="redColorScheme"
        @clicked="deleteReferencePointItem"
      />
    </div>
    <FieldItemInfo
      :location="loc"
      :src="RoverLocationSource_.GPS"
    />
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Prop, Vue } from 'vue-property-decorator';
import { Mutation } from 'vuex-class';
import { BUTTON_COLOR_SCHEMES } from '../../utils/constants';
import {
  ColorScheme,
  ColorSchemeName,
  Odom,
  RoverLocationSource
} from '../../utils/types';
import Button from '../common/Button.vue';
import FieldItemInfo from './FieldItemInfo.vue';

@Component({
  components: {
    Button,
    FieldItemInfo
  }
})
export default class ReferencePointItem extends Vue {
  /************************************************************************************************
   * Types
   ************************************************************************************************/
  /* Redefine RoverLocationSource locally for use in the Template. */
  RoverLocationSource_ = RoverLocationSource;

  /************************************************************************************************
   * Props
   ************************************************************************************************/
  /* Index of reference point item in list of reference points. */
  @Prop({ required: true, type: Number })
  private readonly index!:number;

  /* Location represented by this ReferencePointItem. */
  @Prop({ required: true, type: Object as ()=>Odom })
  private readonly loc!:Odom;

  /************************************************************************************************
   * Vuex Mutations
   ************************************************************************************************/
  @Mutation
  private readonly removeReferencePoint!:(index:number)=>void;

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* Delete the reference point item. */
  private deleteReferencePointItem():void {
    this.removeReferencePoint(this.index);
  } /* deleteRefereceointItem() */

  /* Get the red color scheme for a button. */
  private get redColorScheme():ColorScheme {
    return BUTTON_COLOR_SCHEMES[ColorSchemeName.Red];
  }
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

.reference-point-item {
  background-color: lightyellow;
}
</style>
