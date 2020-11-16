<!-- This file contains the FieldItems component which is the container for the
     lists of AR tags, gates, obstacles, and waypoints. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="container field-items">
    <!-- Waypoint Items -->
    <fieldset>
      <legend>
        <div
          class="collapse-click-area"
          @click="collapseWps"
        >
          <div
            :class="{ 'collapsed': wpsCollapsed, 'minimized': wpsMinimized }"
            class="collapse-arrow"
          >
            <span>&#9662;</span>
          </div>
          <p><u>Waypoints</u> ({{ waypoints.length }})</p>
        </div>
        <div class="clear-btn-area waypoint-area-buttons">
          <Button
            name="Clear Waypoints"
            :invert-color="true"
            :disabled="waypoints.length === 0"
            @clicked="clearWps"
          />
          <Button
            name="Clear Field"
            :invert-color="true"
            :disabled="clearFieldDisabled"
            @clicked="clearField"
          />
        </div>
      </legend>
      <div
        class="collapsible"
        :style="{ 'height': wpsContainerHeight }"
      >
        <div
          ref="wpsContainer"
          class="field-item-group"
        >
          <div
            v-for="waypoint, i in waypoints"
            :key="`waypoint-${i}`"
            ref="wps"
            class="field-item-container"
          >
            <WaypointItem
              class="field-item"
              :waypoint="waypoint"
              :index="i"
            />
          </div>
        </div>
      </div>
    </fieldset>

    <!-- Ar Tag Items -->
    <fieldset>
      <legend>
        <div
          class="collapse-click-area"
          @click="collapseArTags"
        >
          <div
            :class="{ 'collapsed': arsCollapsed, 'minimized': arsMinimized }"
            class="collapse-arrow"
          >
            <span>&#9662;</span>
          </div>
          <p><u>AR Tags</u> ({{ arTags.length }})</p>
        </div>
        <Button
          class="clear-btn-area"
          name="Clear Ar Tags"
          :invert-color="true"
          :disabled="arTags.length === 0"
          @clicked="clearArs"
        />
      </legend>
      <div
        class="collapsible"
        :style="{ 'height': arContainerHeight }"
      >
        <div
          ref="arContainer"
          class="field-item-group"
        >
          <div
            v-for="arTag, i in arTags"
            :key="`ar-tag-${i}`"
            ref="arTags"
            class="field-item-container"
          >
            <ArTagItem
              class="field-item"
              :ar-tag="arTag"
              :index="i"
            />
          </div>
        </div>
      </div>
    </fieldset>

    <!-- Gate Items -->
    <fieldset>
      <legend>
        <div
          class="collapse-click-area"
          @click="collapseGates"
        >
          <div
            :class="{ 'collapsed': gatesCollapsed, 'minimized': gatesMinimized }"
            class="collapse-arrow"
          >
            <span>&#9662;</span>
          </div>
          <p><u>Gates</u> ({{ gates.length }})</p>
        </div>
        <Button
          class="clear-btn-area"
          name="Clear Gates"
          :invert-color="true"
          :disabled="gates.length === 0"
          @clicked="clearGates"
        />
      </legend>
      <div
        class="collapsible"
        :style="{ 'height': gatesContainerHeight }"
      >
        <div
          ref="gatesContainer"
          class="field-item-group"
        >
          <div
            v-for="gate, i in gates"
            :key="`gate-${i}`"
            ref="gates"
            class="field-item-container"
          >
            <GateItem
              class="field-item"
              :gate="gate"
              :index="i"
            />
          </div>
        </div>
      </div>
    </fieldset>

    <!-- Obstacle Items -->
    <fieldset>
      <legend>
        <div
          class="collapse-click-area"
          @click="collapseObs"
        >
          <div
            :class="{ 'collapsed': obsCollapsed, 'minimized': obsMinimized }"
            class="collapse-arrow"
          >
            <span>&#9662;</span>
          </div>
          <p><u>Obstacles</u> ({{ obstacles.length }})</p>
        </div>
        <Button
          class="clear-btn-area"
          name="Clear Obstacles"
          :invert-color="true"
          :disabled="obstacles.length === 0"
          @clicked="clearObs"
        />
      </legend>
      <div
        class="collapsible"
        :style="{ 'height': obsContainerHeight }"
      >
        <div
          ref="obsContainer"
          class="field-item-group"
        >
          <div
            v-for="obstacle, i in obstacles"
            :key="`obstacle-${i}`"
            ref="obs"
            class="field-item-container"
          >
            <ObstacleItem
              class="field-item"
              :obstacle="obstacle"
              :index="i"
            />
          </div>
        </div>
      </div>
    </fieldset>

    <!-- Radio Repeater -->
    <fieldset>
      <legend>
        <div
          class="collapse-click-area"
          @click="collapseRr"
        >
          <div
            :class="{ 'collapsed': rrCollapsed }"
            class="collapse-arrow"
          >
            <span>&#9662;</span>
          </div>
          <p><u>Radio Repeater</u><span v-if="repeaterLoc === null"> (not dropped)</span></p>
        </div>
        <Button
          class="clear-btn-area"
          name="Reset Radio Repeater"
          :invert-color="true"
          :disabled="repeaterLoc === null"
          @clicked="resetRr"
        />
      </legend>
      <div
        class="collapsible"
        :style="{ 'height': rrContainerHeight }"
      >
        <div
          ref="rrContainer"
          class="field-item-group"
        >
          <div
            v-if="repeaterLoc !== null"
            ref="rr"
            class="field-item-container"
          >
            <RadioRepeaterItem
              class="field-item"
              :loc="repeaterLoc"
            />
          </div>
        </div>
      </div>
    </fieldset>
  </div>
</template>

<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import {
  Component,
  Ref,
  Watch,
  Vue
} from 'vue-property-decorator';
import { Getter, Mutation } from 'vuex-class';
import {
  ArTag,
  Gate,
  Obstacle,
  Odom,
  OdomFormat,
  Waypoint
} from '../../utils/types';
import ArTagItem from './ArTagItem.vue';
import Button from '../common/Button.vue';
import GateItem from './GateItem.vue';
import ObstacleItem from './ObstacleItem.vue';
import RadioRepeaterItem from './RadioRepeaterItem.vue';
import WaypointItem from './WaypointItem.vue';

@Component({
  components: {
    ArTagItem,
    Button,
    GateItem,
    ObstacleItem,
    RadioRepeaterItem,
    WaypointItem
  }
})
export default class FieldItems extends Vue {
  /************************************************************************************************
   * Vuex Getters
   ************************************************************************************************/
  @Getter
  private readonly arTags!:ArTag[];

  @Getter
  private readonly gates!:Gate[];

  @Getter
  private readonly obstacles!:Obstacle[];

  @Getter
  private readonly odomFormat!:OdomFormat;

  @Getter
  private readonly repeaterLoc!:Odom|null;

  @Getter
  private readonly waypoints!:Waypoint[];

  /************************************************************************************************
   * Vuex Mutations
   ************************************************************************************************/
  @Mutation
  private readonly removeArTag!:(index:number)=>void;

  @Mutation
  private readonly removeGate!:(index:number)=>void;

  @Mutation
  private readonly removeObstacle!:(index:number)=>void;

  @Mutation
  private readonly removeWaypoint!:(index:number)=>void;

  @Mutation
  private readonly setRepeaterLoc!:(newRepeaterLoc:Odom|null)=>void;

  /************************************************************************************************
   * HTML Refs
   ************************************************************************************************/
  /* List of ArTagItem elements. */
  @Ref('arTags')
  private arsHtml!:HTMLDivElement[];

  /* Div element wrapping list of ArTagItem elements. */
  @Ref('arContainer')
  private arsHtmlContainer!:HTMLDivElement;

  /* List of GateItem elements. */
  @Ref('gates')
  private gatesHtml!:HTMLDivElement[];

  /* Div element wrapping list of GateItem elements. */
  @Ref('gatesContainer')
  private gatesHtmlContainer!:HTMLDivElement;

  /* List of ObstacleItem elements. */
  @Ref('obs')
  private obsHtml!:HTMLDivElement[];

  /* Div element wrapping list of ObstacleItem elements. */
  @Ref('obsContainer')
  private obsHtmlContainer!:HTMLDivElement;

  /* RadioRepeaterItem element. */
  @Ref('rr')
  private rrHtml!:HTMLDivElement;

  /* Div element wrapping RadioRepeaterItem element. */
  @Ref('rrContainer')
  private rrHtmlContainer!:HTMLDivElement;

  /* List of WaypointItem elements. */
  @Ref('wps')
  private  wpsHtml!:HTMLDivElement[];

  /* Div element wrapping list of WaypointItem elements. */
  @Ref('wpsContainer')
  private wpsHtmlContainer!:HTMLDivElement;

  /************************************************************************************************
   * Private Members
   ************************************************************************************************/
  private arsCollapsed = false; /* true <-> collapsed */
  private arsMinimized = false; /* true <-> minimized */
  private arContainerHeight = '0px';

  private gatesCollapsed = false; /* true <-> collapsed */
  private gatesMinimized = false; /* true <-> minimized */
  private gatesContainerHeight = '0px';

  private obsCollapsed = false; /* true <-> collapsed */
  private obsMinimized = false; /* true <-> minimized */
  private obsContainerHeight = '0px';

  private rrCollapsed = false; /* true <-> collapsed */
  private rrContainerHeight = '0px';

  private wpsCollapsed = false; /* true <-> collapsed */
  private wpsMinimized = false; /* true <-> minimized */
  private wpsContainerHeight = '0px';

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* Whether or not the clear field button is disabled. */
  private get clearFieldDisabled():boolean {
    return this.arTags.length === 0 && this.gates.length === 0 &&
           this.obstacles.length === 0 && this.waypoints.length === 0;
  }

  /************************************************************************************************
   * Watchers
   ************************************************************************************************/
  @Watch('odomFormat')
  private onOdomFormatChange():void {
    this.onUpdate();
  }

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* Delete all ar tag field items. */
  private clearArs():void {
    while (this.arTags.length) {
      this.removeArTag(0);
    }
  } /* clearArs() */

  /* Delete all field items. */
  private clearField():void {
    this.clearArs();
    this.clearGates();
    this.clearObs();
    this.clearWps();
  } /* clearField() */

  /* Delete all gate field items. */
  private clearGates():void {
    while (this.gates.length) {
      this.removeGate(0);
    }
  } /* clearGates() */

  /* Delete all obstacle field items. */
  private clearObs():void {
    while (this.obstacles.length) {
      this.removeObstacle(0);
    }
  } /* clearObs() */

  /* Delete all waypoint field items. */
  private clearWps():void {
    while (this.waypoints.length) {
      this.removeWaypoint(0);
    }
  } /* clearWps() */

  /* Collapse, minimize, or expand AR tags list. */
  private collapseArTags():void {
    /* collapse */
    if (this.arsMinimized) {
      this.arsMinimized = false;
      this.arsCollapsed = true;
    }

    /* expand */
    else if (this.arsCollapsed) {
      this.arsHtmlContainer.scrollIntoView();
      this.arsCollapsed = false;
    }

    /* minimize */
    else {
      this.arsMinimized = true;
    }
    this.updateArsHeight();
  } /* collapseArTags() */

  /* Collapse, minimize, or expand gates list. */
  private collapseGates():void {
    /* collapse */
    if (this.gatesMinimized) {
      this.gatesMinimized = false;
      this.gatesCollapsed = true;
    }

    /* expand */
    else if (this.gatesCollapsed) {
      this.gatesHtmlContainer.scrollIntoView();
      this.gatesCollapsed = false;
    }

    /* minimize */
    else {
      this.gatesMinimized = true;
    }
    this.updateGatesHeight();
  } /* collapseGates() */

  /* Collapse, minimize, or expand obstacles list. */
  private collapseObs():void {
    /* collapse */
    if (this.obsMinimized) {
      this.obsMinimized = false;
      this.obsCollapsed = true;
    }

    /* expand */
    else if (this.obsCollapsed) {
      this.obsHtmlContainer.scrollIntoView();
      this.obsCollapsed = false;
    }

    /* minimize */
    else {
      this.obsMinimized = true;
    }
    this.updateObsHeight();
  } /* collapseObs() */

  /* Collapse or expanse radio repeater item display. */
  private collapseRr():void {
    this.rrCollapsed = !this.rrCollapsed;
    this.updateRrHeight();
  } /* collapseRr() */

  /* Collapse, minimize, or expand waypoints list. */
  private collapseWps():void {
    /* collapse */
    if (this.wpsMinimized) {
      this.wpsMinimized = false;
      this.wpsCollapsed = true;
    }

    /* expand */
    else if (this.wpsCollapsed) {
      this.wpsHtmlContainer.scrollIntoView();
      this.wpsCollapsed = false;
    }

    /* minimize */
    else {
      this.wpsMinimized = true;
    }
    this.updateWpsHeight();
  } /* collapseWps() */

  /* Update height of the ar tags container, gates container, obstacles
     container, and waypoints container. */
  private onUpdate():void {
    this.updateArsHeight();
    this.updateGatesHeight();
    this.updateObsHeight();
    this.updateRrHeight();
    this.updateWpsHeight();
  } /* onUpdate() */

  /* Reset/pick-up radio repeater. */
  private resetRr():void {
    this.setRepeaterLoc(null);
  } /* resetRr() */

  /* Update the height of the ar tags container. */
  private updateArsHeight():void {
    let height = 0;
    if (this.arTags.length) {
      if (this.arsMinimized) {
        height = this.arsHtml[0].scrollHeight;
      }
      else if (!this.arsCollapsed) {
        height = this.arsHtmlContainer.scrollHeight;
      }
      this.arsHtml.forEach((arTag, i) => {
        this.arsHtml[i].style.height = `${arTag.children[0].scrollHeight}px`;
      });
    }
    this.arContainerHeight = `${height}px`;
  } /* updateArsHeight() */

  /* Update the height of the gates container. */
  private updateGatesHeight():void {
    let height = 0;
    if (this.gates.length) {
      if (this.gatesMinimized) {
        height = this.gatesHtml[0].scrollHeight;
      }
      else if (!this.gatesCollapsed) {
        height = this.gatesHtmlContainer.scrollHeight;
      }
      this.gatesHtml.forEach((gate, i) => {
        this.gatesHtml[i].style.height = `${gate.children[0].scrollHeight}px`;
      });
    }
    this.gatesContainerHeight = `${height}px`;
  } /* updateGatesHeight() */

  /* Update the height of the obstacles container. */
  private updateObsHeight():void {
    let height = 0;
    if (this.obstacles.length) {
      if (this.obsMinimized) {
        height = this.obsHtml[0].scrollHeight;
      }
      else if (!this.obsCollapsed) {
        height = this.obsHtmlContainer.scrollHeight;
      }
      this.obsHtml.forEach((obstacle, i) => {
        this.obsHtml[i].style.height = `${obstacle.children[0].scrollHeight}px`;
      });
    }
    this.obsContainerHeight = `${height}px`;
  } /* updateObsHeight() */

  /* Update the height of the radio repeater container. */
  private updateRrHeight():void {
    let height = 0;
    if (this.repeaterLoc !== null && !this.rrCollapsed) {
      height = this.rrHtml.scrollHeight;
    }
    this.rrContainerHeight = `${height}px`;
  } /* updateRrHeight() */

  /* Update the height of the waypoints container. */
  private updateWpsHeight():void {
    let height = 0;
    if (this.waypoints.length) {
      if (this.wpsMinimized) {
        height = this.wpsHtml[0].scrollHeight;
      }
      else if (!this.wpsCollapsed) {
        height = this.wpsHtmlContainer.scrollHeight;
      }
      this.wpsHtml.forEach((wp, i) => {
        this.wpsHtml[i].style.height = `${wp.children[0].scrollHeight}px`;
      });
    }
    this.wpsContainerHeight = `${height}px`;
  } /* updateWpsHeight() */

  /************************************************************************************************
   * Vue Life Cycle
   ************************************************************************************************/
  private mounted():void {
    window.addEventListener('resize', this.onUpdate);
  } /* mounted() */

  private updated():void {
    this.onUpdate();
  } /* updated() */

  private beforeDestroy():void {
    window.removeEventListener('resize', this.onUpdate);
  } /* beforeDestroy() */
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
.clear-btn-area {
  margin-left: 10px;
}

.collapse-arrow {
  width: 10px;
  display: inline-block;
  transition: 0.5s;
}

.collapse-click-area {
  display: inline-block;
  cursor: pointer;
}

.collapsed {
  transform: rotate(-180deg);
}

.collapsible {
  overflow: scroll;
  scroll-snap-type: y mandatory;
  transition: height 0.5s;
}

.field-item {
  border: 1px solid black;
  border-radius: 5px;
  padding: 2px 5px;
}

.field-item-container {
  margin-top: 3px;
  scroll-snap-align: start;
}

.field-item-container:first-child {
  margin-top: 0;
}

.field-items {
  margin-top: 5px;
  flex: 1;
  overflow: scroll;
}

fieldset {
  border: 1px solid lightgray;
}

legend {
  display: flex;
  width: 100%;
}

.minimized {
  transform: rotate(-90deg);
}

p {
  display: inline-block;
  margin: auto;
}

::v-deep .button-container {
  display: inline-flex;
  font-size: small;
}

::v-deep .draw-options-container {
  display: flex;
  flex-wrap: wrap;
}

::v-deep .field-item p {
  margin: auto;
}

::v-deep .title-row {
  display: flex;
  justify-content: space-between;
}

::v-deep .title-row b {
  margin: auto 0;
}

.waypoint-area-buttons {
  display: inline-flex;
  flex-grow: 1;
  justify-content: space-between;
}
</style>
