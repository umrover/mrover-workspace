<!-- This file contains the FieldItemInfo component which displays latitude,
     longitude, distance from rover, and bearing from rover in the various types
     of field item components. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="field-item-info">
    <div class="left-col">
      <p>Latitude: {{ latitude }}</p>
      <p>Longitude: {{ longitude }}</p>
    </div>
    <div>
      <!-- Note that this is the distance and bearing to the center of the field item. -->
      <p>Distance from Rover: {{ dist }} m</p>
      <p>Bearing from Rover: {{ bear }} º</p>
    </div>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Prop, Vue } from 'vue-property-decorator';
import { Getter } from 'vuex-class';
import { Odom, OdomFormat, RoverLocationSource } from '../../utils/types';
import {
  calcDistAndBear,
  calcRelativeBearing,
  calcRelativeOdom,
  radToDeg,
  stringifyLatLon
} from '../../utils/utils';
import { ROVER } from '../../utils/constants';

@Component({})
export default class FieldItemInfo extends Vue {
  /************************************************************************************************
   * Props
   ************************************************************************************************/
  /* Odometry location of the field item. */
  @Prop({ required: true, type: Object as ()=>Odom })
  private readonly location!:Odom;

  /* Where to measure relative distance and bearing from (i,e. gps or zed). */
  @Prop({ required: true, type: Number as ()=>RoverLocationSource })
  private readonly src!:RoverLocationSource;

  /************************************************************************************************
   * Vuex Getters
   ************************************************************************************************/
  @Getter
  private readonly currOdom!:Odom;

  @Getter
  private readonly fieldCenterOdom!:Odom;

  @Getter
  private readonly odomFormat!:OdomFormat;

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* Stringified version of the latitude of the field item. */
  private get latitude():string {
    return stringifyLatLon(this.location.latitude_deg, this.location.latitude_min,
                           'lat', this.odomFormat);
  }

  /* Stringified version of the longitude of the field item. */
  private get longitude():string {
    return stringifyLatLon(this.location.longitude_deg, this.location.longitude_min,
                           'lon', this.odomFormat);
  }

  /* Distance and bearing from the rover (gps or zed) to the field item. */
  private get distAndBear():[number, number] {
    let loc:Odom;
    switch (this.src) {
      case RoverLocationSource.GPS: {
        loc = this.currOdom;
        break;
      }

      case RoverLocationSource.ZED: {
        /* find gps coord of front-center of rover (location of ZED) */
        const zedLoc:Odom = calcRelativeOdom(this.currOdom, this.currOdom.bearing_deg,
                                             ROVER.length / 2, this.fieldCenterOdom);
        loc = zedLoc;
        break;
      }

      /* no default */
    }

    const [dist, bear]:[number, number] = calcDistAndBear(loc, this.location);
    return [dist, calcRelativeBearing(this.currOdom.bearing_deg, radToDeg(bear))];
  }

  /* Displayed distance from rover (gps or zed) to the field item. We case from
     a string back to a number so that we cut off trailing zeros. */
  private get dist():number {
    return Number(this.distAndBear[0].toFixed(2));
  }

  /* Displayed bearing from rover (gps or zed) to the field item. We case from
     a string back to a number so that we cut off trailing zeros. */
  private get bear():number {
    return Number(this.distAndBear[1].toFixed(2));
  }
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.field-item-info {
  display: flex;
  flex-wrap: wrap;
}

.left-col {
  margin-right: 20px;
}

p {
  margin: auto;
}
</style>
