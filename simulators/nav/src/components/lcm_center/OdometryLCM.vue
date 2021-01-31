<!-- This file contains the OdometryLCM component which displays the odometry
     lcm. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="box">
    <fieldset class="odometry">
      <legend>Odometry</legend>
      <p>Latitude: {{ latitude }}</p>
      <p>Longitude: {{ longitude }}</p>
      <p>Heading: {{ heading }}º</p>
    </fieldset>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { Getter } from 'vuex-class';
import { stringifyLatLon } from '../../utils/utils';
import { Odom, OdomFormat, SensorSimulationMode } from '../../utils/types';

/**************************************************************************************************
 * Constants
 **************************************************************************************************/
const HEADING_DECIMALS = 2;

@Component({})
export default class OdometryLCM extends Vue {
  /************************************************************************************************
   * Vuex Getters
   ************************************************************************************************/
  @Getter
  private readonly currOdom!:Odom;

  @Getter
  private readonly currOdomNoisy!:Odom;

  @Getter
  private readonly odomFormat!:OdomFormat;

  @Getter
  private readonly simulateLoc!:SensorSimulationMode;

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* Odometry to display depending on the current mode of simulateLoc. */
  private get displayOdom():Odom {
    if (this.simulateLoc === SensorSimulationMode.OnWithNoise) {
      return this.currOdomNoisy;
    }
    else {
      return this.currOdom;
    }
  }

  /* Stringified version of the rover's latitude. */
  private get latitude():string {
    return stringifyLatLon(this.displayOdom.latitude_deg, this.displayOdom.latitude_min,
                           'lat', this.odomFormat);
  }

  /* Stringified version of the rover's longitude. */
  private get longitude():string {
    return stringifyLatLon(this.displayOdom.longitude_deg, this.displayOdom.longitude_min,
                           'lon', this.odomFormat);
  }

  /* Stringified version of the rover's heading with fixed number of
     decimals. */
  private get heading():string {
    return this.displayOdom.bearing_deg.toFixed(HEADING_DECIMALS);
  }
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.odometry {
  min-width: 250px;
}
</style>
