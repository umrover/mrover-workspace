<!-- This file contains the TargetListLCM component which displays the target
     list lcm. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="box">
    <fieldset class="target-list">
      <legend>TargetList</legend>
      <p>Target 1 Distance: {{ dist1 }} m</p>
      <p>Target 1 Bearing: {{ bear1 }}º</p>
      <p>Target 1 ID: {{ targetList[0].id }}</p>
      <div class="divider" />
      <p>Target 2 Distance: {{ dist2 }} m</p>
      <p>Target 2 Bearing: {{ bear2 }}º</p>
      <p>Target 2 ID: {{ targetList[1].id }}</p>
    </fieldset>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { Getter } from 'vuex-class';
import { TargetListMessage } from '../../utils/types';

@Component({})
export default class TargetListLCM extends Vue {
  /************************************************************************************************
   * Vuex Getters
   ************************************************************************************************/
  @Getter
  private readonly targetList!:TargetListMessage

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* distance to first target (if one exists) in the target list LCM */
  private get dist1():number {
    return Number(this.targetList[0].distance.toFixed(2));
  }

  /* bearing to first target (if one exists) in the target list LCM */
  private get bear1():number {
    return Number(this.targetList[0].bearing.toFixed(2));
  }

  /* distance to second target (if one exists) in the target list LCM */
  private get dist2():number {
    return Number(this.targetList[1].distance.toFixed(2));
  }

  /* bearing to second target (if one exists) in the target list LCM */
  private get bear2():number {
    return Number(this.targetList[1].bearing.toFixed(2));
  }
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.divider {
  border-top: 1px solid gray;
}

.target-list {
  min-width: 210px;
}
</style>
