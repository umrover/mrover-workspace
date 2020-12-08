<!-- This file contains the Header component which contains the logo, title,
     and the connection status indicators. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="container header">
    <img
      id="logo"
      src="/static/mrover.png"
      alt="MRover"
      title="MRover"
    >
    <h1 id="title">
      Navigation Simulator
    </h1>
    <div class="comms">
      <ul id="vitals">
        <li>
          <BinaryIndicator
            :val="lcmConnected"
            name="LCM Bridge"
          />
        </li>
        <li>
          <BinaryIndicator
            :val="navConnected"
            name="Navigation"
          />
        </li>
        <li>
          <BinaryIndicator
            :val="localizationOn"
            name="Localization"
          />
        </li>
        <li>
          <BinaryIndicator
            :val="perceptionOn"
            name="Perception"
          />
        </li>
      </ul>
    </div>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { Getter } from 'vuex-class';
import BinaryIndicator from '../common/BinaryIndicator.vue';

@Component({
  components: {
    BinaryIndicator
  }
})
export default class Header extends Vue {
  /************************************************************************************************
   * Vuex Getters
   ************************************************************************************************/
  @Getter
  private readonly navConnected!:boolean;

  @Getter
  private readonly lcmConnected!:boolean;

  @Getter
  private readonly locConnected!:boolean;

  @Getter
  private readonly percepConnected!:boolean;

  @Getter
  private readonly simulateLoc!:boolean;

  @Getter
  private readonly simulatePercep!:boolean;

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* Whether or not we are connected to or simulating localization. */
  private get localizationOn():boolean {
    return this.simulateLoc || this.locConnected;
  }

  /* Whether or not we are connected to or simulating perception. */
  private get perceptionOn():boolean {
    return this.simulatePercep || this.percepConnected;
  }
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.comms {
  display: flex;
  flex-grow: 1;
  justify-content: space-around;
  margin: 0;
}

.comms * {
  margin-top: 2px;
  margin-bottom: 2px;
}

.header {
  display: flex;
  align-items: center;
  flex-wrap: wrap;
  flex: initial;
  margin-bottom: 5px;
}

#logo {
  flex: initial;
  height: 44px; /* same as height of title */
  width: auto;
}

#title {
  display: inline;
  margin: 0 0 0 5px;
}

#vitals {
  display: flex;
  flex-wrap: wrap;
  padding: 0;
}

#vitals li {
  display: inline;
  float: left;
  padding: 0px 5px;
}
</style>
