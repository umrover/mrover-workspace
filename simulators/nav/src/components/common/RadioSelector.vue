<!-- This file contains a radio selector component which is a grouping of radio
     buttons. This component is templated so options can be of any type (but
     must be uniform). An example is the Odom Format options in the SimSettings
     component. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="radio-selector">
    <div
      v-for="option in options"
      :key="option.value"
      class="option"
    >
      <input
        :id="option.name"
        v-model="selectedOption"
        type="radio"
        :value="option.value"
      >
      <label :for="option.name">{{ option.name }}</label>
    </div>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Prop, Vue } from 'vue-property-decorator';
import { RadioOption } from '../../utils/types';

@Component({})
export default class RadioSelector<T> extends Vue {
  /************************************************************************************************
   * Props
   ************************************************************************************************/
  /* Possible options to select. */
  @Prop({ required: true })
  private readonly options!:RadioOption<T>[];

  /* Currently selected option. */
  @Prop({ required: true })
  private readonly selection!:T;

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* The currently selected option. */
  private get selectedOption():T {
    return this.selection;
  }
  private set selectedOption(newSelection:T) {
    this.$emit('selected', newSelection);
  }
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.option {
  margin: 2px;
  display: inline-flex;
  flex: 1;
  white-space: nowrap;
}

.radio-selector {
  display: flex;
  flex-wrap: wrap;
}

.radio-selector input[type="radio"] {
  opacity: 0;
  position: fixed;
  width: 0;
}

.radio-selector input[type="radio"]:checked + label {
  background-color:rgb(160, 255, 160);
  border-color: #4c4;
}

.radio-selector label {
  display: inline-block;
  background-color: #ddd;
  padding: 0 5px;
  border: 2px solid #444;
  border-radius: 4px;
  min-width: 30px;
  cursor: pointer;
  text-align: center;
  transition: 0.5s;
  flex: 1;
}

.radio-selector label:hover {
  background-color: rgb(190, 255, 190);
}
</style>
