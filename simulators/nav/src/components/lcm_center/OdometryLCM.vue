<!-- This file contains the OdometryLCM component which displays the odometry
     lcm. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="box">
    <fieldset class="odometry">
      <legend>Odometry</legend>
      <p>Latitude: {{ latitude }}</p>
      <p>Longitude: {{ longitude }}</p>
      <p>Heading: {{ currOdom.bearing_deg }}º</p>
    </fieldset>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { Getter } from 'vuex-class';
import { stringifyLatLon } from '../../utils/utils';
import { Odom, OdomFormat } from '../../utils/types';

@Component({})
export default class OdometryLCM extends Vue {
  /************************************************************************************************
   * Vuex Getters
   ************************************************************************************************/
  @Getter
  private readonly currOdom!:Odom;

  @Getter
  private readonly odomFormat!:OdomFormat;

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* Stringified version of the latitude of the field item. */
  private get latitude():string {
    return stringifyLatLon(this.currOdom.latitude_deg, this.currOdom.latitude_min,
                           'lat', this.odomFormat);
  }

  /* Stringified version of the longitude of the field item. */
  private get longitude():string {
    return stringifyLatLon(this.currOdom.longitude_deg, this.currOdom.longitude_min,
                           'lon', this.odomFormat);
  }
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.odometry {
  min-width: 250px;
}
</style>
