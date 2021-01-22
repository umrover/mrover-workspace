<!-- This file contains the SimSettings component which includes the Odom Format
     selector, the simulate localization checkbox, etc. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <fieldset class="box">
    <legend>Settings</legend>
    <div class="sim-settings">
      <div class="field-size first setting">
        <p>Field Size (m):</p>
        <NumberInput
          :val.sync="fieldSizeIn"
          :min="5"
          :max="1000"
        />
      </div>

      <div class="odom-format setting">
        <p>Odom Format:</p>
        <RadioSelector
          :options="odomFormatOptions"
          :selection="odomFormat"
          @selected="selectOdomFormat"
        />
      </div>

      <div class="setting">
        <Button
          name="Simulate Localization"
          :color-scheme="simulateLocColor"
          @clicked="flipSimLoc"
        />
      </div>
      <div class="setting">
        <Button
          name="Simulate Perception"
          :color-scheme="simulatePercepColor"
          @clicked="flipSimPercep"
        />
      </div>
    </div>
  </fieldset>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { Getter, Mutation } from 'vuex-class';
import { BUTTON_COLOR_SCHEMES } from '../../utils/constants';
import {
  ColorScheme,
  ColorSchemeName,
  OdomFormat,
  RadioOption,
  SensorSimulationMode
} from '../../utils/types';
import { rotateSensorSimulationMode } from '../../utils/utils';
import Button from '../common/Button.vue';
import NumberInput from '../common/NumberInput.vue';
import RadioSelector from '../common/RadioSelector.vue';

@Component({
  components: {
    Button,
    NumberInput,
    RadioSelector
  }
})
export default class SimSettings extends Vue {
  /************************************************************************************************
   * Vuex Getters
   ************************************************************************************************/
  @Getter
  private readonly fieldSize!:number;

  @Getter
  private readonly odomFormat!:OdomFormat;

  @Getter
  private readonly simulateLoc!:SensorSimulationMode;

  @Getter
  private readonly simulatePercep!:SensorSimulationMode;

  /************************************************************************************************
   * Vuex Mutations
   ************************************************************************************************/
  @Mutation
  private readonly setFieldSize!:(newFieldSize:number)=>void;

  @Mutation
  private readonly setOdomFormat!:(newOdomFormat:OdomFormat)=>void;

  @Mutation
  private readonly flipSimulateLoc!:(newMode:SensorSimulationMode)=>void;

  @Mutation
  private readonly flipSimulatePercep!:(newMode:SensorSimulationMode)=>void;

  /************************************************************************************************
   * Private Members
   ************************************************************************************************/
  /* Odom format options */
  private readonly odomFormatOptions:RadioOption<OdomFormat>[] = [
    { value: OdomFormat.D,   name: 'D'   },
    { value: OdomFormat.DM,  name: 'DM'  },
    { value: OdomFormat.DMS, name: 'DMS' }
  ];

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* Displayed field size */
  private get fieldSizeIn():number {
    return this.fieldSize;
  }
  private set fieldSizeIn(newFieldSize:number) {
    this.setFieldSize(newFieldSize);
  }

  /* Color of the simulate localization button */
  private get simulateLocColor():ColorScheme {
    switch (this.simulateLoc) {
      case SensorSimulationMode.OnWithNoise: {
        return BUTTON_COLOR_SCHEMES[ColorSchemeName.Green];
      }

      case SensorSimulationMode.OnNoNoise: {
        return BUTTON_COLOR_SCHEMES[ColorSchemeName.Yellow];
      }

      case SensorSimulationMode.Off: {
        return BUTTON_COLOR_SCHEMES[ColorSchemeName.Red];
      }

      default: {
        console.log('ERROR: Unknown sensor simulation mode.');
        return BUTTON_COLOR_SCHEMES[ColorSchemeName.Red];
      }
    }
  }

  /* Color of the simulate perception button */
  private get simulatePercepColor():ColorScheme {
    switch (this.simulatePercep) {
      case SensorSimulationMode.OnWithNoise: {
        return BUTTON_COLOR_SCHEMES[ColorSchemeName.Green];
      }

      case SensorSimulationMode.OnNoNoise: {
        return BUTTON_COLOR_SCHEMES[ColorSchemeName.Yellow];
      }

      case SensorSimulationMode.Off: {
        return BUTTON_COLOR_SCHEMES[ColorSchemeName.Red];
      }

      default: {
        console.log('ERROR: Unknown sensor simulation mode.');
        return BUTTON_COLOR_SCHEMES[ColorSchemeName.Red];
      }
    }
  }

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* Rotate to next simulation mode for localization. */
  private flipSimLoc():void {
    this.flipSimulateLoc(rotateSensorSimulationMode(this.simulateLoc));
  } /* flipSimLoc() */

  /* Rotate to next simulation mode for perception. */
  private flipSimPercep():void {
    this.flipSimulatePercep(rotateSensorSimulationMode(this.simulatePercep));
  } /* flipSimPercep() */

  /* Change current odom format selection. */
  private selectOdomFormat(selectedOdomFormat:OdomFormat):void {
    this.setOdomFormat(selectedOdomFormat);
  } /* selectOdomFormat() */
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.field-size p {
  margin-top: 4px;
}

.odom-format {
  display: flex;
  flex-wrap: wrap;
}

.odom-format p {
  margin-right: 3px;
}

p {
  display: inline-block;
  margin: auto;
}

.setting {
  margin-right: 10px;
}

.setting:last-child {
  margin-left: 0;
}

.sim-settings {
  display: flex;
  flex-wrap: wrap;
}

::v-deep .button-container input {
  min-height: 26px;
}
</style>
