<!-- This file contains the BinaryIndicator component which displays if the
     input variable is on (green) or off (red). -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="wrap">
    <span :class="['led', status]" />
    <span class="name">{{ name }}</span>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Prop, Vue } from 'vue-property-decorator';

@Component({})
export default class BinaryIndicator extends Vue {
  /************************************************************************************************
   * Props
   ************************************************************************************************/
  /* Input variable. */
  @Prop({ required: true, type: Boolean })
  private readonly val!:boolean;

  @Prop({ required: true, type: String })
  private readonly name!:string;

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* Css class corresponding to val's status. */
  private get status():string {
    return this.val ? 'on' : 'off';
  }
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.led {
  width: 16px;
  height: 16px;
  border-radius: 8px;
  border: 1px solid;
}

.name {
  margin-left: 5px;
}

.wrap {
  display: flex;
  align-items: center;
  height: 100%;
}
</style>
