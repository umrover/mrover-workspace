<!-- This file contains the ObstacleLCM component which displays the obstacle
     lcm. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="box">
    <fieldset class="obstacle">
      <legend>Obstacle</legend>
      <p>Distance: {{ dist }} m</p>
      <p>Bearing: {{ bear }}º</p>
    </fieldset>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { Getter } from 'vuex-class';
import { ObstacleMessage, SensorSimulationMode } from '../../utils/types';

/**************************************************************************************************
 * Constants
 **************************************************************************************************/
const NUM_DECIMALS = 2;

@Component({})
export default class ObstacleLCM extends Vue {
  /************************************************************************************************
   * Vuex Getters
   ************************************************************************************************/
  @Getter
  private readonly obstacleMessage!:ObstacleMessage;

  @Getter
  private readonly obstacleMessageNoisy!:ObstacleMessage;

  @Getter
  private readonly simulatePercep!:SensorSimulationMode;


  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* bearing to turn to in the obstacle message LCM */
  private get bear():number {
    return Number(this.displayObsMsg.bearing.toFixed(NUM_DECIMALS));
  }

  /* Obstacle message to display depending on the current mode of
     simulatePercep. */
  private get displayObsMsg():ObstacleMessage {
    if (this.simulatePercep === SensorSimulationMode.OnWithNoise) {
      return this.obstacleMessageNoisy;
    }
    else {
      return this.obstacleMessage;
    }
  }

  /* distance to the closest obstacle (if one exists) in the obstacle message
     LCM */
  private get dist():number {
    return Number(this.displayObsMsg.distance.toFixed(NUM_DECIMALS));
  }
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.obstacle {
  min-width: 210px;
}
</style>
