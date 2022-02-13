<!-- This file contains an input component where the value must be a number. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="number-input">
    <input
      v-model="valDisplay"
      type="number"
      :min="min"
      :max="max"
      :step="step"
      @input="updateVal"
    >
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import {
  Component,
  Emit,
  Prop,
  Vue,
  Watch
} from 'vue-property-decorator';

@Component({})
export default class NumberInput extends Vue {
  /************************************************************************************************
   * Props
   ************************************************************************************************/
  /* Maximum valid value. Note that ((max - min) % step) must be true otherwise
     when doing out of bounds checks we may create an invalid value. */
  @Prop({ required: true })
  private readonly max!:number;

  /* Minimum valid value */
  @Prop({ required: true })
  private readonly min!:number;

  /* Step size */
  @Prop({ required: false, default: 1 })
  private readonly step!:number;

  /* Value of input. */
  @Prop({ required: true })
  private readonly val!:number;

  /************************************************************************************************
   * Private Members
   ************************************************************************************************/
  /* Get the displayed value. */
  private valDisplay!:number;

  /************************************************************************************************
   * Watchers
   ************************************************************************************************/
  /* In the case of dynamic min (and max) values, we may need to update val
     again on change. */
  @Watch('val')
  private onValChange():void {
    this.valDisplay = this.val;
  } /* onValChange() */

  /************************************************************************************************
   * Emitting Private Methods
   ************************************************************************************************/
  /* Update the value and make sure it is valid. */
  @Emit('update:val')
  private updateVal(event):number {
    let validVal = Number(event.target.value);

    /* Ensure value is multiple of step size. */
    if (((validVal - this.min) / this.step) % 1) {
      /* If increasing */
      if (validVal > this.val) {
        validVal += this.step - ((validVal - this.min) % this.step);
      }
      else {
        validVal -= (validVal - this.min) % this.step;
      }
    }

    /* Ensure valid is in valid range */
    if (validVal <= this.min) {
      validVal = this.min;
    }
    else if (validVal > this.max) {
      validVal = this.max;
    }

    this.valDisplay = validVal;
    return validVal;
  } /* updateVal() */

  /************************************************************************************************
   * Vue Life Cycle
   ************************************************************************************************/
  private beforeMount():void {
    this.valDisplay = this.val;
  } /* beforeMount() */
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style>
.number-input {
  display: inline-block;
}
</style>
