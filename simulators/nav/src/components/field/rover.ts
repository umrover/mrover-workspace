/* This file contains the CanvasRover class for drawing the rover on the
   canvas. */

import {
  compassToCanvasRad,
  degToRad,
  odomToCanvas
} from '../../utils/utils';
import { FieldOfViewOptions, Odom, Point2D } from '../../utils/types';
import { ROVER } from '../../utils/constants';

/**************************************************************************************************
 * Constants
 **************************************************************************************************/
/* Rover dimensions in meters. Width is the direction of the rover's width
   not the item's width. The same goes for length. */
/* distance between edge of rover body and items like ebox and ZED */
const EDGE_OFFSET = 0.07;
const EBOX_LEN = 0.3;
const WHEEL_DEPTH = 0.15;
const WHEEL_DIAM = 0.25;
const ZED_LEN = 0.05;
const ZED_WIDTH = 0.5;

/* Class for drawing the rover on the field canvas. */
export default class CanvasRover {
  /************************************************************************************************
   * Private Members
   ************************************************************************************************/
  /* Rover dimensions in pixels */
  private scaledEdgeOffset!:number;
  private scaledEboxLen!:number;
  private scaledFovDepth!:number;
  private scaledRoverLen!:number;
  private scaledRoverWdth!:number;
  private scaledWheelDpth!:number;
  private scaledWheelDiam!:number;
  private scaledZedLen!:number;
  private scaledZedWdth!:number;

  /* GPS point of the center of the canvas */
  private canvasCent!:Odom;

  /* Location of the rover */
  private currOdom!:Odom;

  /* Canvas context */
  private ctx!:CanvasRenderingContext2D;

  /* Field of view options */
  private fov!:FieldOfViewOptions;

  /* scale of the canvas in pixels/meter */
  private scale!:number;

  /************************************************************************************************
   * Public Methods
   ************************************************************************************************/
  /* Initialize CanvasRover instance by calculating scaled dimensions. */
  constructor(
      currOdom:Odom,
      canvasCent:Odom,
      scale:number, /* pixels/meter */
      fov:FieldOfViewOptions
  ) {
    this.currOdom = currOdom;
    this.canvasCent = canvasCent;
    this.scale = scale;
    this.fov = fov;

    this.scaledEdgeOffset = EDGE_OFFSET * this.scale;
    this.scaledEboxLen = EBOX_LEN * this.scale;
    this.scaledFovDepth = this.fov.depth * this.scale;
    this.scaledRoverLen = ROVER.length * this.scale;
    this.scaledRoverWdth = ROVER.width * this.scale;
    this.scaledWheelDpth = WHEEL_DEPTH * this.scale;
    this.scaledWheelDiam = WHEEL_DIAM * this.scale;
    this.scaledZedLen = ZED_LEN * this.scale;
    this.scaledZedWdth = ZED_WIDTH * this.scale;
  } /* constructor() */

  /* Draw rover on the canvas. */
  drawRover(canvas:HTMLCanvasElement):void {
    /* if field component not yet mounted */
    if (this.scale === -1) {
      return;
    }

    /* get canvas context */
    this.ctx = canvas.getContext('2d') as CanvasRenderingContext2D;

    const loc:Point2D = odomToCanvas(this.currOdom, this.canvasCent, canvas.height, this.scale);
    this.ctx.translate(loc.x, loc.y);
    this.ctx.rotate(degToRad(this.currOdom.bearing_deg));

    this.drawBody();
    this.drawEbox();
    this.drawWheels();
    this.drawZed();
    if (this.fov.visible) {
      this.drawFov();
    }

    this.ctx.rotate(-degToRad(this.currOdom.bearing_deg));
    this.ctx.translate(-loc.x, -loc.y);
  } /* drawRover() */

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* Draw body of rover on the canvas. */
  private drawBody():void {
    const bodyWdth:number = this.scaledRoverWdth - (2 * this.scaledWheelDpth);
    const bodyLen:number = this.scaledRoverLen;
    this.ctx.fillStyle = 'black';
    this.ctx.fillRect(-bodyWdth / 2, -bodyLen / 2, bodyWdth, bodyLen);
  } /* drawBody() */

  /* Draw ebox on the rover. */
  private drawEbox():void {
    const scaledEboxWdth:number = this.scaledRoverWdth - (2 * this.scaledWheelDpth) -
                                  (2 * this.scaledEdgeOffset);
    const eboxLoc:Point2D = {
      x: 0,
      y: ((this.scaledRoverLen - this.scaledEboxLen) / 2) - this.scaledEdgeOffset
    };
    const startCorner:Point2D = {
      x: eboxLoc.x - (scaledEboxWdth / 2),
      y: eboxLoc.y - (this.scaledEboxLen / 2)
    };
    this.ctx.fillStyle = 'white';
    this.ctx.fillRect(startCorner.x, startCorner.y, scaledEboxWdth, this.scaledEboxLen);
  } /* drawEbox() */

  /* Draw the field of view on the canvas. */
  private drawFov():void {
    const roverEyeLoc:Point2D = {
      x: 0,
      y: -this.scaledRoverLen / 2
    };
    this.ctx.strokeStyle = 'black';
    this.ctx.lineWidth = 3;
    this.ctx.beginPath();
    this.ctx.moveTo(roverEyeLoc.x, roverEyeLoc.y);
    this.ctx.arc(roverEyeLoc.x, roverEyeLoc.y, this.scaledFovDepth,
                 compassToCanvasRad(-degToRad(this.fov.angle / 2)),
                 compassToCanvasRad(degToRad(this.fov.angle / 2)),
                 false);
    this.ctx.lineTo(roverEyeLoc.x, roverEyeLoc.y);
    this.ctx.stroke();
  } /* drawFov() */

  /* Draw the wheels on the rover. */
  private drawWheels():void {
    const distToWheel:Point2D = {
      x: (this.scaledRoverWdth - this.scaledWheelDpth) / 2,
      y: (this.scaledRoverLen - this.scaledWheelDiam) / 2
    };
    const wheelLocations:Point2D[] = [
      { x: -distToWheel.x, y: -distToWheel.y },
      { x: -distToWheel.x, y:  distToWheel.y },
      { x:  distToWheel.x, y: -distToWheel.y },
      { x:  distToWheel.x, y:  distToWheel.y }
    ];

    this.ctx.fillStyle = '#404040';
    wheelLocations.forEach((wheelLoc:Point2D) => {
      const startCorner:Point2D = {
        x: wheelLoc.x - (this.scaledWheelDpth / 2),
        y: wheelLoc.y - (this.scaledWheelDiam / 2)
      };
      this.ctx.fillRect(startCorner.x, startCorner.y,
                        this.scaledWheelDpth, this.scaledWheelDiam);
    });
  } /* drawWheels() */

  /* Draw the ZED on the rover. */
  private drawZed():void {
    const zedLoc:Point2D = {
      x: 0,
      y: this.scaledEdgeOffset + ((this.scaledZedLen - this.scaledRoverLen) / 2)
    };
    const startCorner:Point2D = {
      x: zedLoc.x - (this.scaledZedWdth / 2),
      y: zedLoc.y - (this.scaledZedLen / 2)
    };
    this.ctx.fillStyle = 'silver';
    this.ctx.fillRect(startCorner.x, startCorner.y, this.scaledZedWdth, this.scaledZedLen);
  } /* drawZed() */
} /* CanvasRover */
