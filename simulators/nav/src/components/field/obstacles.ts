/* This file contains the CanvasObstacles class for drawing obstacles on the
   canvas. */

import { odomToCanvas } from '../../utils/utils';
import { Obstacle, Odom, Point2D } from '../../utils/types';

/**************************************************************************************************
 * Constants
 **************************************************************************************************/
/* Space needed to draw label inside obstacle in pixels. */
const LABEL_SPACE_REQUIRED = 30;

/* Class for drawing obstacles on the field canvas. */
export default class CanvasObstacles {
  /************************************************************************************************
   * Private Members
   ************************************************************************************************/
  /* GPS point of the center of the canvas */
  private canvasCent!:Odom;

  /* Canvas context */
  private ctx!:CanvasRenderingContext2D;

  /* list of all obstacles on field */
  private obstacles!:Obstacle[];

  /* scale of the canvas in pixels/meter */
  private scale!:number;

  /************************************************************************************************
   * Public Methods
   ************************************************************************************************/
  /* Initialize CanvasObstacles instance. */
  constructor(
      obstacles:Obstacle[],
      canvasCent:Odom,
      scale:number /* pixels/meter */
  ) {
    this.obstacles = obstacles;
    this.canvasCent = canvasCent;
    this.scale = scale;
  }

  /* Draw all obstacles on the canvas. */
  drawObstacles(canvas:HTMLCanvasElement):void {
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

    /* draw obstacles */
    this.obstacles.forEach((obstacle, i) => {
      const loc:Point2D = odomToCanvas(obstacle.odom, this.canvasCent, canvas.height, this.scale);
      this.ctx.translate(loc.x, loc.y);
      this.drawObstacle(obstacle.size, String(i));
      this.ctx.translate(-loc.x, -loc.y);
    });
  } /* drawObstacles() */

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* Draw a single obstacle on the canvas. */
  private drawObstacle(size:number, label:string):void {
    this.ctx.beginPath();
    const radius = this.scale * size / 2;
    this.ctx.arc(0, 0, radius, 0, 2 * Math.PI, false);
    this.ctx.fillStyle = '#ffb670';
    this.ctx.fill();
    this.ctx.stroke();
    this.ctx.fillStyle = 'black';

    /* If obstacle is large enough (30 pixels), draw label inside obstacle */
    if (radius > LABEL_SPACE_REQUIRED) {
      this.ctx.textAlign = 'center';
      this.ctx.textBaseline = 'middle';
      this.ctx.fillText(label, 0, 0);
    }
    else {
      this.ctx.textAlign = 'left';
      this.ctx.textBaseline = 'bottom';
      this.ctx.fillText(label, radius, -radius);
    }
  } /* drawObstacle() */
} /* CanvasObstacles */
