<!-- This file contains the NavStatusLCM component which displays the nav status
     lcm. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="box">
    <fieldset class="nav-status">
      <legend>NavStatus</legend>
      <p>State: {{ navStatus.nav_state_name }}</p>
      <p>Waypoints: {{ navStatus.completed_wps }} / {{ navStatus.total_wps }}</p>
    </fieldset>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { Getter } from 'vuex-class';
import { NavStatus } from '../../utils/types';

@Component({})
export default class NavStatusLCM extends Vue {
  /************************************************************************************************
   * Vuex Getters
   ************************************************************************************************/
  @Getter
  private readonly navStatus!:NavStatus;
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.nav-status {
  min-width: 270px;
}
</style>
