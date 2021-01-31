<!-- This file contains the control panel component. This is the container for
     the SimSettings, DrawModule, and DebugTools components. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="container control-panel">
    <SimSettings />
    <div class="row">
      <DrawModule />
      <DebugTools />
    </div>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import DebugTools from './DebugTools.vue';
import DrawModule from './DrawModule.vue';
import SimSettings from './SimSettings.vue';

@Component({
  components: {
    DebugTools,
    DrawModule,
    SimSettings
  }
})
export default class ControlPanel extends Vue {}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.row {
  display: flex;
}
</style>
