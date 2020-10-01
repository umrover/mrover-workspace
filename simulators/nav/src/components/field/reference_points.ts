/* This file contains the CanvasReferencePoints class for drawing reference
   points on the canvas. */

import { odomToCanvas } from '../../utils/utils';
import { Odom, Point2D } from '../../utils/types';

/**************************************************************************************************
 * Constants
 **************************************************************************************************/
/* reference point dimensions in pixels */
const REFERENCE_POINT_SIZE = 30;

/* Class for drawing reference points on the field canvas. */
export default class CanvasReferencePoints {
  /************************************************************************************************
   * Private Members
   ************************************************************************************************/
  /* GPS point of the center of the canvas */
  private canvasCent!:Odom;

  /* Canvas context */
  private ctx!:CanvasRenderingContext2D;

  /* scale of the canvas in pixels/meter */
  private scale!:number;

  /* list of all reference points on the field */
  private referencePoints:Odom[];

  /************************************************************************************************
   * Public Methods
   ************************************************************************************************/
  /* Initialize CanvasWaypoints instance. */
  constructor(
      referencePoints:Odom[],
      canvasCent:Odom,
      scale:number /* pixels/meter */
  ) {
    this.referencePoints = referencePoints;
    this.canvasCent = canvasCent;
    this.scale = scale;
  }

  /* Draw all waypoints on the canvas. */
  drawReferencePoints(canvas:HTMLCanvasElement):void {
    /* if field component not yet mounted */
    if (this.scale === -1) {
      return;
    }

    /* get canvas context */
    this.ctx = canvas.getContext('2d') as CanvasRenderingContext2D;

    /* set draw options */
    this.ctx.strokeStyle = 'black';
    this.ctx.lineWidth = 3;
    this.ctx.font = '40px sans-serif';
    this.ctx.textAlign = 'center';
    this.ctx.textBaseline = 'middle';

    /* draw waypoints */
    this.referencePoints.forEach((pt, i) => {
      const loc:Point2D = odomToCanvas(pt, this.canvasCent, canvas.height, this.scale);
      this.ctx.translate(loc.x, loc.y);
      this.drawReferencePoint(String(i));
      this.ctx.translate(-loc.x, -loc.y);
    });
  } /* drawReferencePoints() */

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* Draw a single waypoint on the canvas. */
  private drawReferencePoint(label:string):void {
    this.ctx.beginPath();
    this.ctx.arc(0, 0, REFERENCE_POINT_SIZE, 0, 2 * Math.PI, false);
    this.ctx.fillStyle = 'lightyellow';
    this.ctx.fill();
    this.ctx.stroke();
    this.ctx.fillStyle = 'black';
    this.ctx.fillText(label, 0, 0);
  } /* drawReferencePoint() */
} /* CanvasReferencePoints */
