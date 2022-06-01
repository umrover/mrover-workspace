<!-- This file contains the ObstacleLCM component which displays the obstacle
     lcm. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="box">
    <fieldset class="obstacle">
      <legend>Obstacle</legend>
      <p>bottom_left_coordinate_meters: {{ dist }} m</p>
      <p>top_right_coordinate_meters: {{ bear }}º</p>
    </fieldset>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { Getter } from 'vuex-class';
import { ObstacleMessage } from '../../utils/types';

@Component({})
export default class ObstacleLCM extends Vue {
  /************************************************************************************************
   * Vuex Getters
   ************************************************************************************************/
  @Getter
  private readonly obstacleMessage!:ObstacleMessage;


  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* distance to the closest obstacle (if one exists) in the obstacle message
     LCM */
  private get dist():number {
    return Number(this.obstacleMessage.numObstacles.toFixed(2));
  }

  /* bearing to turn to in the obstacle message LCM */
  private get bear():number {
    return Number(this.obstacleMessage.numObstacles.toFixed(2));
  }
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.obstacle {
  min-width: 210px;
}
</style>
