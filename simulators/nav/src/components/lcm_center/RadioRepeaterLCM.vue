<!-- This file contains the RadioRepeaterLCM component which displays the
     repeater strength lcm as well as allows for updating the simulated radio
     signal strength. -->

<!--------------------------------- Template ---------------------------------->
<template>
  <div class="box">
    <fieldset :class="['radio-signal-strength', strengthStatus]">
      <legend>Radio Signal</legend>
      <p id="signal">
        Strength: {{ radioStrength }} dBm
      </p>
      <Button
        name="Update"
        :invert-color="invertColor"
        @clicked="updateSignalStrength"
      />
      <NumberInput
        :val.sync="signalStrengthIn"
        :min="minSignal"
        :max="maxSignal"
      />
    </fieldset>
  </div>
</template>


<!---------------------------------- Script ----------------------------------->
<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { Getter, Mutation } from 'vuex-class';
import { RADIO } from '../../utils/constants';
import Button from '../common/Button.vue';
import NumberInput from '../common/NumberInput.vue';

@Component({
  components: {
    Button,
    NumberInput
  }
})
export default class RadioRepeaterLCM extends Vue {
  /************************************************************************************************
   * Vuex Getters
   ************************************************************************************************/
  @Getter
  private readonly radioStrength!:number;

  /************************************************************************************************
   * Vuex Mutations
   ************************************************************************************************/
  @Mutation
  private readonly setRadioStrength!:(strength:number)=>void;

  /************************************************************************************************
   * Private Members
   ************************************************************************************************/
  /* Displayed signal strength update value. */
  private signalStrengthIn:number = RADIO.lowSignalStrengthThreshold - 1;

  /* Maximum valid signal strength. */
  private maxSignal:number = RADIO.maxSignalStrength;

  /* Minimum valid signal strength. */
  private minSignal:number = RADIO.minSignalStrength;

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* Whether or not to flip the color from green to red. */
  private get invertColor():boolean {
    return this.radioStrength < RADIO.lowSignalStrengthThreshold;
  }

  /* Determine if the radio signal strength is strong ('on') or weak ('off'). */
  private get strengthStatus():string {
    return this.radioStrength < RADIO.lowSignalStrengthThreshold ? 'weak' : 'strong';
  }

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* Update the signal strength value with signalStrengthIn. */
  private updateSignalStrength():void {
    this.setRadioStrength(this.signalStrengthIn);
  } /* updateSignalStrength() */
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
p {
  display: inline-block;
}

.radio-signal-strength legend {
  background-color: lightgray;
  border-radius: 5px;
}

#signal {
  width: 135px;
}

.strong {
  background-color: rgb(160, 255, 160);
}

.strong,
.strong p,
.weak,
.weak p {
  transition: 0.5s;
}

::v-deep .button-container input {
  min-height: 26px;
}

.weak {
  background-color: rgb(255, 90, 90);
}

.weak p,
.weak ::v-deep input {
  color: white;
  border-color: #eee;
}
</style>
