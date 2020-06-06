<!-- This file contains a simple button component. An example of a Button is the
     "Reset Rover" Button in the Debug Tools component. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="button-container">
    <input
      type="button"
      :disabled="disabled"
      :value="htmlName"
      :class="{ 'enabled': !disabled, 'positive': !invertColor, 'negative': invertColor }"
      class="button"
      @click="$emit('clicked')"
    >
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Prop, Vue } from 'vue-property-decorator';

@Component({})
export default class Button extends Vue {
  /************************************************************************************************
   * Props
   ************************************************************************************************/
  /* If the button is disabled or not. */
  @Prop({ default: false, type: Boolean })
  private readonly disabled!:boolean;

  /* Word to display on the button. */
  @Prop({ required: true, type: String })
  private readonly name!:string;

  /* Normal color scheme is green and inverted color scheme is red on hover. */
  @Prop({ default: false, type: Boolean })
  private readonly invertColor!:boolean;

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* Unescaped version of the input name. This allows for displaying special
     characters in the HTML (e.g. %D7 -> ×). Note that the special characters
     are javascript not HTML. */
  private get htmlName():string {
    return unescape(this.name);
  }
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.button {
  transition: 0.5s;
}

.button-container {
  display: inline-flex;
  align-items: center;
}

.enabled:hover:not(:disabled) {
  cursor: pointer;
}

input {
  width: 100%;
  transition: 0.5s;
}

.negative:hover:not(:disabled) {
  background-color: rgb(255, 120, 120);
  color: white;
}

.positive:hover:not(:disabled) {
  background-color: rgb(190, 255, 190);
}
</style>
