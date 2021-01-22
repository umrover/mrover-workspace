<!-- This files contains the HearbeatMonitor component which handles the logic
     for detecting the connection to other programs (e.g. Nav). -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <!-- Render nothing. -->
  <div />
</template>

<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import {
  Component,
  Emit,
  Prop,
  Vue
} from 'vue-property-decorator';

/**************************************************************************************************
 * Constants
 **************************************************************************************************/
/* Time between checks for a hearbeat. */
const HB_CHECK_INTERVAL = 500; /* milliseconds */

/* 3 seconds without a pulse indicates dead program */
const HB_TIMEOUT = 3;

/* 1000 milliseconds in a second */
const MILLI_TO_SEC = 1000;

/* Max number of heartbeat misses before program is considered dead. */
const HB_MAX_MISSED = HB_TIMEOUT / (HB_CHECK_INTERVAL / MILLI_TO_SEC); /* 3 seconds */

/* Component that monitors the connection to other programs (e.g. Nav). */
@Component({})
export default class HearbeatMonitor extends Vue {
  /************************************************************************************************
   * Props
   ************************************************************************************************/
  /* Is the corresponding program alive. Set by the heartbeat monitor to be
     used by the parent component. */
  @Prop({ required: true, type: Boolean })
  private isAlive!:boolean;

  /* Have there be a pulse for this program since the heartbeat monitor last
     checked. */
  @Prop({ required: true, type: Boolean })
  private hasPulse!:boolean;

  /************************************************************************************************
   * Private Members
   ************************************************************************************************/
  /* Interval to check pulse of program on. */
  private checkPulseInterval!:number;

  /* Number of hearbeats missed. */
  private missedBeats:number = HB_MAX_MISSED; /* initially dead until we find heartbeat */

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* Check if there has been a pulse since the last time we checked. */
  @Emit('update:isAlive')
  private checkForPulse():boolean {
    if (this.hasPulse) {
      this.missedBeats = 0;
      this.$emit('update:hasPulse', false);
    }
    else {
      /* Using max() so that we don't eventually run into integer overflow */
      this.missedBeats = Math.min(this.missedBeats + 1, HB_MAX_MISSED);
    }

    return this.missedBeats !== HB_MAX_MISSED;
  } /* check_for_pulse() */

  /************************************************************************************************
   * Vue Life Cycle
   ************************************************************************************************/
  /* Start checking for program pulse's on start up. */
  private created():void {
    this.checkPulseInterval = window.setInterval(this.checkForPulse, HB_CHECK_INTERVAL);
  } /* created() */

  /* Stop checking for program pulse before tearing down the application. */
  private beforeDestroy():void {
    window.clearInterval(this.checkPulseInterval);
  } /* beforeDestroy() */
}
</script>
