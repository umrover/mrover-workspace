<!-- This file contains the ZedGimbalLCM component which displays the ZED gimbal
     current and desired positions. -->

<!--------------------------------- Template ---------------------------------->
<template>
  <div class="box">
    <fieldset class="zed-gimbal">
      <legend>ZED Gimbal</legend>
      <p>Current: {{ currentPos }}º, Desired: {{ desiredPos }}º</p>
    </fieldset>
  </div>
</template>


<!---------------------------------- Script ----------------------------------->
<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { Getter } from 'vuex-class';
import { ZedGimbalPosition } from '../../utils/types';

/**************************************************************************************************
 * Constants
 **************************************************************************************************/
const NUM_DECIMALS = 2;

@Component({})
export default class ZedGimbalLCM extends Vue {
  /************************************************************************************************
   * Vuex Getters
   ************************************************************************************************/
  @Getter
  private readonly zedGimbalCmd!:ZedGimbalPosition;

  @Getter
  private readonly zedGimbalPos!:ZedGimbalPosition;

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* Current angle of the ZED gimbal relative to rover's heading. */
  private get currentPos():number {
    return Number(this.zedGimbalPos.angle.toFixed(NUM_DECIMALS));
  }

  /* Desired angle of the ZED gimbal relative to rover's heading. */
  private get desiredPos():number {
    return Number(this.zedGimbalCmd.angle.toFixed(NUM_DECIMALS));
  }
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
</style>
