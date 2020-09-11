<!-- This file contains the AutonStateLCM component which displays the auton
     state lcm as well as acts as the on/off button for autonomy. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="box">
    <fieldset
      :class="['auton-state', status.toLowerCase()]"
      @click="switchOnOff"
    >
      <legend>Auton State</legend>
      <p class="status">
        {{ status }}
      </p>
    </fieldset>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { Getter, Mutation } from 'vuex-class';

@Component({})
export default class AutonStateLCM extends Vue {
  /************************************************************************************************
   * Vuex Getters
   ************************************************************************************************/
  @Getter
  private readonly autonOn!:boolean;

  /************************************************************************************************
   * Vuex Mutations
   ************************************************************************************************/
  @Mutation
  private readonly setAutonState!:(onOff:boolean)=>void;

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* Whether or not autonomy is turned on or off */
  private get status():string {
    return this.autonOn ? 'On' : 'Off';
  }

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  private switchOnOff():void {
    this.setAutonState(!this.autonOn);
  } /* switchOnOff() */
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.auton-state {
  min-width: 95px;
  cursor: pointer;
}

.auton-state legend {
  background-color: lightgray;
  border-radius: 5px;
}

.status {
  text-align: center;
}
</style>
