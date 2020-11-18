<!-- This file contains a simple checkbox component. An example of a Checkbox is
     the "Pause" button in the Debug Tools component. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div
    :class="{
      'checkbox': true,
      'on': !disabled && on,
      'off': !disabled && !on,
      'disabled': disabled
    }"
    @click="clicked"
  >
    <p>{{ name }}</p>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Prop, Vue } from 'vue-property-decorator';

@Component({})
export default class Checkbox extends Vue {
  /************************************************************************************************
   * Props
   ************************************************************************************************/
  /* If the checkbox is disabled or not. */
  @Prop({ default: false, type: Boolean })
  private readonly disabled!:boolean;

  /* Whether or not the checkbox is checked or not. */
  @Prop({ required: true })
  private readonly on!:boolean;

  /* Word to display on the checkbox. */
  @Prop({ required: true })
  private readonly name!:string;

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* If the button is enabled, emit a notification */
  private clicked():void {
    if (!this.disabled) {
      this.$emit('clicked');
    }
  } /* clicked() */
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.checkbox {
  cursor: pointer;
  border: 2px solid;
  border-radius: 5px;
  padding: 0 5px;
  text-align: center;
  margin: 2px;
  white-space: nowrap;
  flex: 1;
}

.checkbox.off {
  border-color: rgb(255, 40, 40);
}

.checkbox.on {
  border-color: #4c4;
}

.checkbox p {
  margin: auto;
}

.disabled {
  background-color: inherit;
  border-color: gray;
  cursor: not-allowed;
}
</style>
