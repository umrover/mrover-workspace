<!-- This file contains the LCMCenter component which contains the components
     for displaying the various LCM messages (e.g. nav status and radio signal
     strength). Note that this includes turning on and off autonomy the
     auton state LCM. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="container lcm-center">
    <div class="row">
      <AutonStateLCM />
      <NavStatusLCM />
      <JoystickLCM />
    </div>
    <div class="row">
      <div class="col">
        <OdometryLCM />
        <ObstacleLCM />
      </div>
      <TargetListLCM />
    </div>
    <div class="row">
      <RadioRepeaterLCM />
    </div>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import AutonStateLCM from './AutonStateLCM.vue';
import JoystickLCM from './JoystickLCM.vue';
import NavStatusLCM from './NavStatusLCM.vue';
import ObstacleLCM from './ObstacleLCM.vue';
import OdometryLCM from './OdometryLCM.vue';
import RadioRepeaterLCM from './RadioRepeaterLCM.vue';
import TargetListLCM from './TargetListLCM.vue';

@Component({
  components: {
    AutonStateLCM,
    JoystickLCM,
    NavStatusLCM,
    ObstacleLCM,
    OdometryLCM,
    RadioRepeaterLCM,
    TargetListLCM
  }
})
export default class LCMCenter extends Vue {}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.col {
  display: flex;
  flex-direction: column;
  flex: 1;
}

.lcm-center {
  display: flex;
  flex-direction: column;
  margin-top: 5px;
  flex: 1;
  overflow: scroll;
}

.row {
  display: flex;
  flex-wrap: wrap;
  flex: none;
}
</style>
