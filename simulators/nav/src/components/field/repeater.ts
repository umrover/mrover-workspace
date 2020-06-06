/* This file contains the CanvasRepeater class for drawing the radio repeater
   on the canvas. */

import { degToRad, odomToCanvas } from '../../utils/utils';
import { Odom, Point2D } from '../../utils/types';

/**************************************************************************************************
 * Constants
 **************************************************************************************************/
/* repeater dimensions */
const WIDTH = 0.5; /* meters */
const HEIGHT = 0.25; /* meters */
const ANTENNA_LEN = 0.5; /* meters */
const LFT_ANTENNA_ANGLE = 70; /* degrees */
const LFT_ANTENNA_ANGLE_RAD:number = degToRad(LFT_ANTENNA_ANGLE); /* radians */
const RT_ANTENNA_ANGLE = 45; /* degrees */
const RT_ANTENNA_ANGLE_RAD:number = degToRad(RT_ANTENNA_ANGLE); /* radians */

/* Class for drawing the radio repeater on the field canvas. */
export default class CanvasRepeater {
  /************************************************************************************************
   * Private Members
   ************************************************************************************************/
  /* GPS point of the center of the canvas */
  private canvasCent!:Odom;

  /* Location of radio repeater (null if not dropped) */
  private odom!:Odom|null;

  /* scale of the canvas in pixels/meter */
  private scale!:number;

  /* Repeater dimensions in pixels */
  private readonly scaledWidth!:number;
  private readonly scaledHeight!:number;
  private readonly scaledAntennaLen!:number;

  /************************************************************************************************
   * Public Methods
   ************************************************************************************************/
  /* Initialize CanvasRepeater instance by calculating scaled dimensions. */
  constructor(
      odom:Odom|null,
      canvasCent:Odom,
      scale:number /* pixels/meter */
  ) {
    this.odom = odom;
    this.canvasCent = canvasCent;
    this.scale = scale;

    this.scaledWidth = this.scale * WIDTH;
    this.scaledHeight = this.scale * HEIGHT;
    this.scaledAntennaLen = this.scale * ANTENNA_LEN;
  } /* constructor() */

  /* Draw radio repeater on the canvas if it has been "dropped". */
  drawRepeater(canvas:HTMLCanvasElement):void {
    /* if field component not yet mounted */
    if (this.scale === -1) {
      return;
    }

    /* if repeater not yet dropped */
    if (this.odom === null) {
      return;
    }

    /* get canvas context */
    const ctx:CanvasRenderingContext2D = canvas.getContext('2d') as CanvasRenderingContext2D;

    /* draw repeater */
    const loc:Point2D = odomToCanvas(this.odom, this.canvasCent, canvas.height, this.scale);

    ctx.translate(loc.x, loc.y);
    ctx.rotate(degToRad(this.odom.bearing_deg));

    /* main body */
    ctx.fillStyle = '#444';
    ctx.fillRect(-this.scaledWidth / 2, -this.scaledHeight / 2,
                 this.scaledWidth, this.scaledHeight);

    /* antenna */
    ctx.strokeStyle = '#444';
    ctx.beginPath();
    ctx.lineWidth = 3;
    ctx.moveTo(-this.scaledAntennaLen * Math.cos(LFT_ANTENNA_ANGLE_RAD),
               -this.scaledAntennaLen * Math.sin(LFT_ANTENNA_ANGLE_RAD));
    ctx.lineTo(0, -this.scaledHeight / 2);
    ctx.lineTo(this.scaledAntennaLen * Math.cos(RT_ANTENNA_ANGLE_RAD),
               -this.scaledAntennaLen * Math.sin(RT_ANTENNA_ANGLE_RAD));
    ctx.stroke();

    ctx.rotate(-degToRad(this.odom.bearing_deg));
    ctx.translate(-loc.x, -loc.y);
  } /* drawRepeater() */
} /* CanvasRepeater */
