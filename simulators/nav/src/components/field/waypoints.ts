/* This file contains the CanvasWaypoints class for drawing waypoints on the
   canvas. */

import { odomToCanvas } from '../../utils/utils';
import { Odom, Point2D, Waypoint } from '../../utils/types';

/**************************************************************************************************
 * Constants
 **************************************************************************************************/
/* waypoint dimensions in pixels */
const WAYPOINT_SIZE = 40;

/* Class for drawing waypoints on the field canvas. */
export default class CanvasWaypoints {
  /************************************************************************************************
   * Private Members
   ************************************************************************************************/
  /* GPS point of the center of the canvas */
  private canvasCent!:Odom;

  /* Canvas context */
  private ctx!:CanvasRenderingContext2D;

  /* scale of the canvas in pixels/meter */
  private scale!:number;

  /* list of all waypoints on the field */
  private waypoints:Waypoint[];

  /************************************************************************************************
   * Public Methods
   ************************************************************************************************/
  /* Initialize CanvasWaypoints instance. */
  constructor(
      waypoints:Waypoint[],
      canvasCent:Odom,
      scale:number /* pixels/meter */
  ) {
    this.waypoints = waypoints;
    this.canvasCent = canvasCent;
    this.scale = scale;
  }

  /* Draw all waypoints on the canvas. */
  drawWaypoints(canvas:HTMLCanvasElement):void {
    /* if field component not yet mounted */
    if (this.scale === -1) {
      return;
    }

    /* get canvas context */
    this.ctx = canvas.getContext('2d') as CanvasRenderingContext2D;

    /* Only clear the canvas with the first element draw (i.e. the first v-draw
       function in the canvas element in the Field component ). */
    this.ctx.clearRect(0, 0, canvas.width, canvas.height);

    /* set draw options */
    this.ctx.strokeStyle = 'black';
    this.ctx.lineWidth = 3;
    this.ctx.font = '40px sans-serif';
    this.ctx.textAlign = 'center';
    this.ctx.textBaseline = 'middle';

    /* draw waypoints */
    this.waypoints.forEach((waypoint, i) => {
      const loc:Point2D = odomToCanvas(waypoint.odom, this.canvasCent, canvas.height, this.scale);
      this.ctx.translate(loc.x, loc.y);
      this.drawWaypoint(String(i));
      this.ctx.translate(-loc.x, -loc.y);
    });
  } /* drawWaypoints() */

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* Draw a single waypoint on the canvas. */
  private drawWaypoint(label:string):void {
    this.ctx.beginPath();
    this.ctx.arc(0, 0, WAYPOINT_SIZE, 0, 2 * Math.PI, false);
    this.ctx.fillStyle = '#BED';
    this.ctx.fill();
    this.ctx.stroke();
    this.ctx.fillStyle = 'black';
    this.ctx.fillText(label, 0, 0);
  } /* drawWaypoint() */
} /* CanvasWaypoints */
