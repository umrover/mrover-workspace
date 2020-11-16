<!-- This file contains the Field component. This includes uploading/downloading
     test cases as well as drawing items on the canvas. -->

<!------------------------------------------- Template -------------------------------------------->
<template>
  <div class="field-container">
    <!-- Ensure that canvas is a square. -->
    <div id="aspect-ratio" />

    <div class="field">
      <!-- Upload/Download -->
      <div class="upload-download">
        <div class="upload">
          <label for="upload-btn">
            <i class="fa fa-upload" />
          </label>
          <input
            id="upload-btn"
            ref="uploadInput"
            type="file"
            @change="uploadTestCase"
          >
        </div>
        <div
          ref="download_container"
          class="download"
        >
          <i
            class="fa fa-download"
            @click="downloadTestCase"
          />

          <!-- Hidden element used for downloading test cases. -->
          <a
            id="download-data"
            ref="dataToDownload"
          />
        </div>
      </div>

      <!-- Field -->
      <canvas
        ref="fieldCanvas"
        v-drawWaypoints="canvasWaypoints"
        v-drawObstacles="canvasObstacles"
        v-drawArTags="canvasArTags"
        v-drawRepeater="canvasRepeater"
        v-drawRover="canvasRover"
        @click="addFieldItem"
      />
    </div>
  </div>
</template>


<!-------------------------------------------- Script --------------------------------------------->
<script lang="ts">
import {
  Component,
  Ref,
  Vue,
  Watch
} from 'vue-property-decorator';
import {
  Getter,
  Mutation
} from 'vuex-class';
import {
  ArTag,
  ArTagDrawOptions,
  FieldItemType,
  FieldOfViewOptions,
  FieldState,
  Gate,
  GateDrawOptions,
  Obstacle,
  ObstacleDrawOptions,
  Odom,
  Point2D,
  Waypoint,
  WaypointDrawOptions
} from '../../utils/types';
import { canvasToOdom } from '../../utils/utils';
import CanvasArTags from './ar_tags';
import CanvasObstacles from './obstacles';
import CanvasRepeater from './repeater';
import CanvasRover from './rover';
import CanvasWaypoints from './waypoints';

/* multiply the number of pixels in each dimension of the canvas by pixel
   density, this provides added definition to the canvas drawings */
const PIXEL_DENSITY = 4;

@Component({
  directives: {

    /* Note: We are casting canvasElement because the custom directives accept an
       `Element` as the first parameter while the functions they call need an
       `HTMLCanvasElement`. */
    drawArTags(canvasElement:Element, binding):void {
      binding.value.drawArTags(canvasElement as HTMLCanvasElement);
    },

    drawObstacles(canvasElement:Element, binding):void {
      binding.value.drawObstacles(canvasElement as HTMLCanvasElement);
    },

    drawRepeater(canvasElement:Element, binding):void {
      binding.value.drawRepeater(canvasElement as HTMLCanvasElement);
    },

    drawRover(canvasElement:Element, binding):void {
      binding.value.drawRover(canvasElement as HTMLCanvasElement);
    },

    drawWaypoints(canvasElement:Element, binding):void {
      binding.value.drawWaypoints(canvasElement as HTMLCanvasElement);
    }
  }
})
export default class Field extends Vue {
  /************************************************************************************************
   * Vuex Getters
   ************************************************************************************************/
  @Getter
  private readonly arTagDrawOptions!:ArTagDrawOptions;

  @Getter
  private readonly arTags!:ArTag[];

  @Getter
  private readonly canvasHeight!:number;

  @Getter
  private readonly currOdom!:Odom;

  @Getter
  private readonly drawMode!:FieldItemType;

  @Getter
  private readonly fieldCenterOdom!:Odom;

  @Getter
  private readonly fieldOfViewOptions!:FieldOfViewOptions;

  @Getter
  private readonly fieldSize!:number;

  @Getter
  private readonly fieldState!:FieldState;

  @Getter
  private readonly gateDrawOptions!:GateDrawOptions;

  @Getter
  private readonly gates!:Gate[];

  @Getter
  private readonly obstacleDrawOptions!:ObstacleDrawOptions;

  @Getter
  private readonly obstacles!:Obstacle[];

  @Getter
  private readonly repeaterLoc!:Odom|null;

  @Getter
  private readonly waypoints!:Waypoint[];

  @Getter
  private readonly waypointDrawOptions!:WaypointDrawOptions;

  /************************************************************************************************
   * Vuex Mutations
   ************************************************************************************************/
  @Mutation
  private readonly pushArTag!:(newArTag:ArTag)=>void;

  @Mutation
  private readonly pushGate!:(newGate:Gate)=>void;

  @Mutation
  private readonly pushObstacle!:(newObstacle:Obstacle)=>void;

  @Mutation
  private readonly pushWaypoint!:(newWaypoint:Waypoint)=>void;

  @Mutation
  private readonly setArTag!:(newArTags:ArTag[])=>void;

  @Mutation
  private readonly setFieldState!:(newState:FieldState)=>void

  @Mutation
  private readonly setGates!:(newGates:Gate[])=>void;

  @Mutation
  private readonly setObstacles!:(newObstacles:Obstacle[])=>void;

  @Mutation
  private readonly setWaypoints!:(newWaypoints:Waypoint[])=>void;

  @Mutation
  private readonly updateCanvasHeight!:(newCanvasHeight:number)=>void;

  /************************************************************************************************
   * HTML Refs
   ************************************************************************************************/
  /* Div containing download component. */
  @Ref()
  private dataToDownload!:HTMLAnchorElement;

  /* Canvas element. */
  @Ref()
  private fieldCanvas!:HTMLCanvasElement;

  /* Input element for uploading test cases. */
  @Ref()
  private uploadInput!:HTMLInputElement;

  /************************************************************************************************
   * Private Members
   ************************************************************************************************/
  private scale = -1; /* pixels / meters, -1 indicates not yet set */

  /************************************************************************************************
   * Local Getters/Setters
   ************************************************************************************************/
  /* Object for drawing ar tags on canvas */
  private get canvasArTags():CanvasArTags {
    return new CanvasArTags(this.arTags, this.gates, this.fieldCenterOdom, this.scale);
  }

  /* Object for drawing ar obstacles on canvas. */
  private get canvasObstacles():CanvasObstacles {
    return new CanvasObstacles(this.obstacles, this.fieldCenterOdom, this.scale);
  }

  /* Object for drawing radio repeater on canvas. */
  private get canvasRepeater():CanvasRepeater {
    return new CanvasRepeater(this.repeaterLoc, this.fieldCenterOdom, this.scale);
  }

  /* Object for drawing rover on canvas. */
  private get canvasRover():CanvasRover {
    return new CanvasRover(this.currOdom, this.fieldCenterOdom, this.scale,
                           this.fieldOfViewOptions);
  }

  /* Object for drawing waypoints on canvas. */
  private get canvasWaypoints():CanvasWaypoints {
    return new CanvasWaypoints(this.waypoints, this.fieldCenterOdom, this.scale);
  }

  /************************************************************************************************
   * Watchers
   ************************************************************************************************/
  @Watch('fieldSize')
  private onFieldSizeChange():void {
    this.updateScale();
  } /* onFieldSizeChange() */


  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* When field is clicked, add the field item corresponding to the current
     draw mode. */
  private addFieldItem(event:MouseEvent):void {
    /* Update scale in case of window resizing */
    // this.updateScale();

    const clickLoc:Point2D = {
      x: event.offsetX * PIXEL_DENSITY,
      y: event.offsetY * PIXEL_DENSITY
    };
    const clickOdom:Odom = canvasToOdom(clickLoc, this.canvasHeight, this.scale,
                                        this.fieldCenterOdom);
    switch (this.drawMode) {
      case FieldItemType.AR_TAG: {
        this.pushArTag({
          id: this.arTagDrawOptions.id,
          odom: clickOdom,
          orientation: 0
        });
        break;
      }

      case FieldItemType.GATE: {
        this.pushGate({
          leftId: this.gateDrawOptions.leftPostId,
          rightId: this.gateDrawOptions.rightPostId,
          odom: clickOdom,
          orientation: this.gateDrawOptions.orientation,
          width: this.gateDrawOptions.width
        });
        break;
      }

      case FieldItemType.OBSTACLE: {
        this.pushObstacle({
          odom: clickOdom,
          size: this.obstacleDrawOptions.size
        });
        break;
      }

      case FieldItemType.WAYPOINT: {
        this.pushWaypoint({
          gate: this.waypointDrawOptions.gate,
          gate_width: this.waypointDrawOptions.gate_width,
          id: this.waypointDrawOptions.targetId,
          odom: clickOdom,
          search: this.waypointDrawOptions.search
        });
        break;
      }

      /* no default */
    }
  } /* addFieldItem() */

  /* Download a test case in JSON format with the current field items on the
     canvas. */
  private downloadTestCase():void {
    const testCaseData:string = JSON.stringify(this.fieldState, null, 2);
    const dataToDownload = `data:text/plain;charset=utf-8,${encodeURIComponent(testCaseData)}`;

    this.dataToDownload.setAttribute('href', dataToDownload);
    this.dataToDownload.setAttribute('download', `test_case_${new Date().toJSON()}.json`);
    this.dataToDownload.click();
    this.dataToDownload.removeAttribute('href');
    this.dataToDownload.removeAttribute('download');
  } /* downloadTestCase() */

  /* Update scale when the field size changes. */
  private updateScale():void {
    this.updateCanvasHeight(this.fieldCanvas.scrollHeight * PIXEL_DENSITY);
    this.scale = this.canvasHeight / this.fieldSize;
    this.fieldCanvas.height = this.canvasHeight;
    this.fieldCanvas.width = this.canvasHeight;
  } /* updateScale() */

  /* Upload test case and add field items to field. */
  private uploadTestCase():void {
    /* If no file uploaded */
    if (this.uploadInput.files === null || this.uploadInput.files.length === 0) {
      return;
    }

    const file:File = this.uploadInput.files[0];
    const reader:FileReader = new FileReader();
    reader.onload = (e):void => {
      if (e.target !== null) {
        const data:string|ArrayBuffer|null = e.target.result;
        if (typeof data === 'string') {
          const testCaseData:FieldState = JSON.parse(data);
          this.setFieldState(testCaseData);
        }
      }
    };
    reader.readAsText(file);
  } /* uploadTestCase() */

  /************************************************************************************************
   * Vue Life Cycle
   ************************************************************************************************/
  private mounted():void {
    this.updateScale();
    window.addEventListener('resize', this.updateScale);
  } /* mounted() */

  private beforeDestroy():void {
    window.removeEventListener('resize', this.updateScale);
  } /* beforeDestroy() */
}
</script>


<!----------------------------------------- Scoped Style ------------------------------------------>
<style scoped>
#aspect-ratio {
  margin-top: 100%; /* 1:1 aspect ratio */
}

#download-data {
  display: none;
}

.fa:hover {
  cursor: pointer;
}

.fa-upload {
  margin-right: 5px;
}

.field {
  position: absolute;
  top: 0;
  bottom: 0;
  left: 0;
  right: 0;
  border: 2px black solid;
  cursor: crosshair;
}

.field canvas {
  height: 100%;
  width: 100%;
  background-color: tan;
}

.field-container {
  display: inline-block;
  position: relative;
  width: 100%;
}

input[type="file"] {
  display: none;
}

.upload-download {
  display: flex;
  position: absolute;
  bottom: 0;
  right: 0;
  margin-right: 5px;
}
</style>
