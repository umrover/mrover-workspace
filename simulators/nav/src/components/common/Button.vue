<!-- This file contains a simple button component. An example of a Button is the
     "Reset Rover" Button in the Debug Tools component. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="button-container">
    <input
      type="button"
      :disabled="disabled"
      :value="htmlName"
      :class="{ 'enabled': !disabled }"
      :style="styles"
      class="button"
      @click="$emit('clicked')"
    >
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Prop, Vue } from 'vue-property-decorator';
import { ColorScheme } from '../../utils/types';

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

  /* Normal color scheme is green and inverted color scheme is red on hover. TODO */
  @Prop({ required: true })
  private readonly colorScheme!:ColorScheme;

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* Unescaped version of the input name. This allows for displaying special
     characters in the HTML (e.g. %D7 -> ×). Note that the special characters
     are javascript not HTML. */
  private get htmlName():string {
    return unescape(this.name);
  }

  private get styles():Record<string, string> {
    return {
      '--background-color': this.colorScheme.backgroundColor,
      '--background-color-hover': this.colorScheme.backgroundColorHover,
      '--border-color': this.colorScheme.borderColor,
      '--color': this.colorScheme.fontColor
    };
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

.enabled:not(:disabled) {
  background-color: var(--background-color);
  border-color: var(--border-color);
  color: var(--color);
}

.enabled:hover:not(:disabled) {
  cursor: pointer;
  background-color: var(--background-color-hover);
}

input {
  width: 100%;
  transition: 0.5s;
}
</style>
