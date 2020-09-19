<!-- This file contains the RadioRepeaterItem component which is what displays
     the radio repeater item in the field items component. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="rr-item">
    <div class="title-row">
      <b class="rr-item-name">Radio Repeater</b>
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
import { Odom, RoverLocationSource } from '../../utils/types';
import FieldItemInfo from './FieldItemInfo.vue';

@Component({
  components: {
    FieldItemInfo
  }
})
export default class RadioRepeaterItem extends Vue {
  /************************************************************************************************
   * Types
   ************************************************************************************************/
  /* Redefine RoverLocationSource locally for use in the Template. */
  RoverLocationSource_ = RoverLocationSource;

  /************************************************************************************************
   * Props
   ************************************************************************************************/
  /* Location of the radio repeater. */
  @Prop({ required: true, type: Object as ()=>Odom })
  private readonly loc!:Odom;
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.rr-item {
  background-color: #444;
}

::v-deep p, b {
  color: white;
}
</style>
