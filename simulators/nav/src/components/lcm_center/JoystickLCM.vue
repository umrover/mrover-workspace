<!-- This file contains the JoystickLCM component which displays the joystick
     lcm. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="box">
    <fieldset class="joystick">
      <legend>Joystick</legend>
      <p>Translational: {{ forwardBack }}</p>
      <p>Rotational: {{ leftRight }}</p>
    </fieldset>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { Getter } from 'vuex-class';
import { Joystick } from '../../utils/types';

/**************************************************************************************************
 * Constants
 **************************************************************************************************/
const NUM_DECIMALS = 5;

@Component({})
export default class JoystickLCM extends Vue {
  /************************************************************************************************
   * Vuex Getters
   ************************************************************************************************/
  @Getter
  private readonly joystick!:Joystick;

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* Forward/back joystick effort. */
  private get forwardBack():number {
    return Number(this.joystick.forward_back.toFixed(NUM_DECIMALS));
  }

  /* Left/right joystick effort. */
  private get leftRight():number {
    return Number(this.joystick.left_right.toFixed(NUM_DECIMALS));
  }
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.joystick {
  min-width: 165px;
}
</style>
