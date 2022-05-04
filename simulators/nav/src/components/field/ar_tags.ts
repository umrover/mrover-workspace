/* This file contains the CanvasArTags class for drawing ar tags and gates on
   the canvas. */

import { POST } from '../../utils/constants';
import {
  ArTag,
  Gate,
  Odom,
  Point2D,
  FieldOfViewOptions
} from '../../utils/types';
import {
  compassModDeg,
  degToRad,
  odomToCanvas,
  randnBm,
  getGaussianThres,
  canvasToOdom
} from '../../utils/utils';
import { state } from '../../store/modules/simulatorState';

/**************************************************************************************************
 * Constants
 **************************************************************************************************/
/* Post features */
const POST_INNER_ANGLE = 60; /* degrees, equilateral triangle */

/* Gate directionality arrow dimensions */
const ARROW_TAIL_LEN = 80; /* pixels */
const ARROW_HEAD_LEN = 20; /* pixels */
const ARROW_HEAD_ANGLE = 30; /* degrees */

/* Space needed to draw directionality arrow in pixels. */
const ARROW_SPACE_REQUIRED = 40;

/* Shift arrow up */
const ARROW_OFFSET_Y = 10;

/* Post visual (triangle around drawn post) dimensions in pixels */
const VISUAL_SIDE_LEN = 50;

/* Post visual (triangle around drawn post) dimensions in pixels */
const SMALLSIDELEN = 25;

/* Class for drawing ar tags (and gates) on the field canvas. */
export default class CanvasArTags {
  /************************************************************************************************
   * Private Members
   ************************************************************************************************/
  /* list of all AR tags on the field */
  private posts!:ArTag[];

  /* Gate directionality arrow dimensions in pixels */
  private readonly arrowHeadWdth:number = ARROW_HEAD_LEN * Math.tan(degToRad(ARROW_HEAD_ANGLE));

  /* list of all ar tags on field */
  private arTags!:ArTag[];

  /* list of all ar tags on field */
  private falseArTags!:ArTag[];

  /* GPS point of the center of the canvas */
  private canvasCent!:Odom;

  /* Location of the rover */
  private currOdom!:Odom;

  /* Canvas context */
  private ctx!:CanvasRenderingContext2D;

  /* Field of view options */
  private fov!:FieldOfViewOptions;

  /* Field of view options */
  private fieldSize!:number;

  /* list of all gates on the field */
  private gates!:Gate[];

  /* Function to push an area to the rover's FOV converage */
  private setFalseArTag!:(newArTags:ArTag[])=>void;

  /* scale of the canvas in pixels/meter */
  private scale!:number;

  /* post dimensions in pixels */
  private readonly scaledPostSideLen!:number;

  /************************************************************************************************
   * Public Methods
   ************************************************************************************************/
  /* Initialize CanvasArTags instance by calculating scaled dimensions. */
  constructor(
      arTags:ArTag[],
      canvasCent:Odom,
      currOdom:Odom,
      fov:FieldOfViewOptions,
      fieldSize:number,
      gates:Gate[],
      posts:ArTag[],
      setFalseArTag:(newArTags:ArTag[])=>void,
      scale:number /* pixels/meter */
  ) {
    this.arTags = arTags;
    this.canvasCent = canvasCent;
    this.currOdom = currOdom;
    this.fov = fov;
    this.fieldSize = fieldSize;
    this.gates = gates;
    this.posts = posts;
    this.setFalseArTag = setFalseArTag;
    this.scale = scale;
    this.scaledPostSideLen = this.scale * POST.sideLen;
    this.falseArTags = [];
  } /* constructor() */

  /* Draw all ar tags on the canvas. */
  drawArTags(canvas:HTMLCanvasElement):void {
    /* if field component not yet mounted */
    if (this.scale === -1) {
      return;
    }

    /* get canvas context */
    this.ctx = canvas.getContext('2d') as CanvasRenderingContext2D;

    /* set draw options */
    this.ctx.strokeStyle = 'black';
    this.ctx.lineWidth = 2;
    this.ctx.font = '40px sans-serif';

    /* draw posts */
    this.posts.forEach((post, i) => {
      const loc:Point2D = odomToCanvas(post.odom, this.canvasCent, canvas.height, this.scale);
      this.ctx.translate(loc.x, loc.y);
      if (post.isHidden === true) {
        this.drawPost(String(i), post.id, 0, VISUAL_SIDE_LEN, '#d2b48c', true);
      }
      else {
        this.drawPost(String(i), post.id, 0, VISUAL_SIDE_LEN, '#bffcc6', true);
      }
      this.ctx.translate(-loc.x, -loc.y);
    });

    /* draw gates */
    this.gates.forEach((gate, i) => {
      const loc:Point2D = odomToCanvas(gate.odom, this.canvasCent, canvas.height, this.scale);
      this.ctx.translate(loc.x, loc.y);
      this.ctx.rotate(degToRad(gate.orientation));
      this.drawGate(String(i), gate);
      this.ctx.rotate(-degToRad(gate.orientation));
      this.ctx.translate(-loc.x, -loc.y);
    });

    this.ctx.save();

    /* Translate canvas to rover's position */
    const loc:Point2D = odomToCanvas(this.currOdom, this.canvasCent, canvas.height, this.scale);
    this.ctx.translate(loc.x, loc.y);
    this.ctx.rotate(degToRad(this.currOdom.bearing_deg));

    this.drawFalsePos(canvas);
    this.ctx.restore();
  } /* drawArTags() */

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* Draw a single gate on the canvas. */
  private drawGate(label:string, gate:Gate):void {
    /* Add width of post to gate width so distance between inside edges of posts
       is gate width. */
    const gateWidthPx:number = (gate.width * this.scale) + this.scaledPostSideLen;

    /* Draw left gate post. */
    this.ctx.translate(0, gateWidthPx / 2);
    this.ctx.rotate(degToRad(-90));
    this.drawPost('', gate.leftId, gate.orientation - 90, VISUAL_SIDE_LEN, '#ff9', true);
    this.ctx.rotate(degToRad(90));

    /* Draw right gate post. */
    this.ctx.translate(0, -gateWidthPx);
    this.ctx.rotate(degToRad(-90));
    this.drawPost('', gate.rightId, gate.orientation - 90, VISUAL_SIDE_LEN, '#9df', true);
    this.ctx.rotate(degToRad(90));
    this.ctx.translate(0, gateWidthPx / 2);

    /* Draw directionality arrow (if there is enough room). */
    if (this.scale > ARROW_SPACE_REQUIRED) {
      this.ctx.beginPath();
      this.ctx.moveTo(ARROW_TAIL_LEN, 0);
      this.ctx.lineTo(0, 0);
      this.ctx.lineTo(ARROW_HEAD_LEN,  this.arrowHeadWdth);
      this.ctx.lineTo(ARROW_HEAD_LEN, -this.arrowHeadWdth);
      this.ctx.lineTo(0, 0);
      this.ctx.stroke();
      this.ctx.fillStyle = 'black';
      this.ctx.fill();
    }

    /* Label gate. */
    this.ctx.textAlign = 'center';
    this.ctx.textBaseline = 'middle';
    this.ctx.fillStyle = 'black';
    if (this.scale > ARROW_SPACE_REQUIRED) {
      const flip:number = compassModDeg(gate.orientation) > 90 &&
                          compassModDeg(gate.orientation) <= 270 ? -1 : 1;
      const arrowLoc:Point2D = {
        x: (ARROW_TAIL_LEN + ARROW_HEAD_LEN) / 2,
        y: (this.arrowHeadWdth + ARROW_OFFSET_Y) * flip
      };
      this.ctx.translate(arrowLoc.x, arrowLoc.y);
      this.ctx.rotate(degToRad(-gate.orientation));
      this.ctx.fillText(label, 0, 0);
      this.ctx.rotate(degToRad(gate.orientation));
      this.ctx.translate(-arrowLoc.x, -arrowLoc.y);
    }
    else {
      this.ctx.rotate(degToRad(-gate.orientation));
      this.ctx.fillText(label, 0, 0);
      this.ctx.rotate(degToRad(gate.orientation));
    }
  } /* drawGate() */

  /* Draw a single post (i.e. gate post or ar tag) on the canvas at the given angle (in degrees) */
  private drawPost(label:string, id:number, angle:number, triSideLen:number, color:string,
      showID:boolean):void {
    /* Draw visual triangle so tag is visable even when zoomed out */
    this.drawTriangle(triSideLen, color, false);

    /* Draw actual sized triangle */
    this.drawTriangle(this.scaledPostSideLen, color);

    /* Label tag with index */
    this.ctx.rotate(degToRad(-angle));
    this.ctx.textAlign = 'left';
    this.ctx.textBaseline = 'bottom';
    const sideLen:number = Math.max(this.scaledPostSideLen, triSideLen);
    const offsetX:number = sideLen * Math.cos(degToRad(POST_INNER_ANGLE));
    this.ctx.fillStyle = 'black';
    this.ctx.fillText(label, offsetX, 0);

    /* Annotate tag id */
    if (showID) {
      this.ctx.textAlign = 'center';
      this.ctx.textBaseline = 'top';
      const offsetY:number = sideLen * Math.sin(degToRad(POST_INNER_ANGLE)) * 1;
      this.ctx.fillText(`ID: ${id}`, 0, offsetY);
      this.ctx.rotate(degToRad(angle));
    }
  } /* drawPost() */

  /* Draw a triangle on the canvas with a side lenght of sideLen pixels. */
  private drawTriangle(sideLen:number, color:string, drawOutline = true):void {
    const triHalfHght:number = sideLen * Math.sin(degToRad(POST_INNER_ANGLE)) / 2;
    const triHalfWdth:number = sideLen * Math.cos(degToRad(POST_INNER_ANGLE));
    const top:Point2D =   { x: 0,            y: -triHalfHght };
    const btmLf:Point2D = { x: -triHalfWdth, y:  triHalfHght };
    const btmRt:Point2D = { x:  triHalfWdth, y:  triHalfHght };

    this.ctx.beginPath();
    this.ctx.moveTo(top.x, top.y);
    this.ctx.lineTo(btmLf.x, btmLf.y);
    this.ctx.lineTo(btmRt.x, btmRt.y);
    this.ctx.lineTo(top.x, top.y);
    this.ctx.fillStyle = color;
    this.ctx.fill();
    if (drawOutline) {
      this.ctx.stroke();
    }
  } /* drawTriangle() */

  /* False Positives */
  private drawFalsePos(canvas:HTMLCanvasElement):void {
    const max:number = Math.min(state.simSettings.maxFalsePos, this.arTags.length);

    // for each false positive, generate and display on canvas
    const thres = getGaussianThres(state.simSettings.noiseFalsePosPercent);
    for (let i = 0; i < max; i += 1) {
      const num:number = randnBm(0, 1, 1);
      const isFalsePos:boolean = num < thres;

      if (isFalsePos) {
        /* generate r and theta */
        const r:number = randnBm(0, this.fov.depth, 1);
        const theta:number = randnBm(0, this.fov.angle, 1);
        const half:number = this.fov.angle / 2;
        const diff:number = 90 - half;
        const angle:number = theta + diff;

        // convert r and theta to rectangular coordinates in meters
        const xCoordMeters:number = r * Math.cos(angle * Math.PI / 180.0);
        const yCoordmeters:number = r * Math.sin(angle * Math.PI / 180.0);

        this.ctx = canvas.getContext('2d') as CanvasRenderingContext2D;
        const pCanvas:Point2D = { x: xCoordMeters * this.scale, y: yCoordmeters * this.scale };

        // pick a random ar tag and push it to falseArTags
        const randIdx = Math.floor(Math.random() * this.arTags.length);

        const newFalseArTag:ArTag = { ...this.arTags[randIdx] };
        newFalseArTag.odom = canvasToOdom(pCanvas, this.fieldSize, this.scale, this.canvasCent);
        this.falseArTags.push(newFalseArTag);

        // draw False Positive point directly
        this.ctx.translate(pCanvas.x, -pCanvas.y);
        this.drawPost(String(newFalseArTag.id), newFalseArTag.id, 0, SMALLSIDELEN, 'red', false);
        this.ctx.translate(-pCanvas.x, pCanvas.y);

        // this.ctx.fillRect(pCanvas.x, -pCanvas.y, SMALLSIDELEN, SMALLSIDELEN);
      }
    }

    this.setFalseArTag(this.falseArTags);
  }
} /* CanvasArTags */
