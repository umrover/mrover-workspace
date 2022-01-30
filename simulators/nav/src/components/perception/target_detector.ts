/* This files contains the TargetDetector class which contains the logic for
   detecting targets. */

import {
  ArTag,
  FieldOfViewOptions,
  Gate,
  Odom,
  TargetMessage,
  TargetListMessage,
  ZedGimbalPosition
} from '../../utils/types';
import {
  arTagCompare,
  calcDistAndBear,
  calcRelativeBearing,
  calcRelativeOdom,
  compassModDeg,
  degToRad,
  radToDeg
} from '../../utils/utils';
import { Interval, OpenIntervalHeap } from './open_interval_heap';
import {
  PERCEPTION,
  POST,
  ROVER,
  Zscores
} from '../../utils/constants';
import { state } from '../../store/modules/simulatorState';

function randnBm(min, max, skew):number {
  let u = 0;
  let v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  const factor = 2.0;
  let num:number = Math.sqrt(-1 * factor * Math.log(u)) * Math.cos(factor * Math.PI * v);
  const a = 10.0;
  const b = 0.5;
  num = (num / a) + b;
  if (num > 1 || num < 0) {
    num = randnBm(min, max, skew);
  }
  else {
    num **= skew;
    num *= max - min;
    num += min;
  }
  return num;
}

// indZscore is from 0 to 19
function getGaussianThres(indZscore):number {
  const sd = 0.2;
  const mu = 0.5;
  const x = Zscores[indZscore] * sd;

  return mu + x;
}

/* Class that performs target dectection calculations. */
export default class TargetDetector {
  /************************************************************************************************
   * Private Members
   ************************************************************************************************/
  /* list of all ar tags on field */
  private arTags!:ArTag[];

  /* Current rover GPS location */
  private currOdom!:Odom;

  /* GPS point of the center of the canvas */
  private fieldCenterOdom!:Odom;

  /* Rover field of view parameters */
  private fov:FieldOfViewOptions;

  /* list of all gates on field */
  private gates!:Gate[];

  /* ar tags and gate posts */
  private posts!:ArTag[];

  /* posts visible to the rover */
  private visiblePosts!:ArTag[];

  /* position of the ZED's gimbal */
  private zedGimbalPos!:ZedGimbalPosition;

  /* location of the ZED (i.e. rover's eyes) */
  private zedOdom!:Odom;

  /************************************************************************************************
   * Public Methods
   ************************************************************************************************/
  /* Initialize ObstacleDetector. */
  constructor(
      currOdom:Odom,
      zedGimbalPos:ZedGimbalPosition,
      arTags:ArTag[],
      gates:Gate[],
      fov:FieldOfViewOptions,
      fieldCenterOdom:Odom
  ) {
    this.currOdom = currOdom;
    this.zedGimbalPos = zedGimbalPos;
    this.arTags = arTags;
    this.gates = gates;
    this.fov = fov;
    this.fieldCenterOdom = fieldCenterOdom;

    this.getPosts();
    this.updateZedOdom();
  } /* constructor() */

  /* Calculate the TargetList LCM message. */
  computeTargetList():TargetListMessage {
    /* Step 1: filter out targets not in field of view */
    this.visiblePosts = this.posts.filter((post, i) => this.isPostVisible(post, i));

    /* Step 2: Sort visible posts from left to right */
    this.visiblePosts.sort((post1:ArTag, post2:ArTag) => arTagCompare(this.zedOdom, post1, post2));

    /* Step 3: get first entry in target list (left most post) */
    let targetLeft:TargetMessage = {
      bearing: 0,
      distance: -1,
      id: -1
    };
    if (this.visiblePosts.length) {
      const [leftDist, leftBear]:[number, number] = calcDistAndBear(this.zedOdom,
                                                                    this.visiblePosts[0].odom);
      targetLeft = {
        bearing: calcRelativeBearing(this.zedOdom.bearing_deg, radToDeg(leftBear)),
        distance: leftDist,
        id: this.visiblePosts[0].id
      };
    }

    /* Step 4: get second entry in target list (right most post) */
    let targetRight:TargetMessage = {
      bearing: 0,
      distance: -1,
      id: -1
    };
    if (this.visiblePosts.length > 1) {
      const lastIndex:number = this.visiblePosts.length - 1;
      const [rightDist, rightBear]:[number, number] =
          calcDistAndBear(this.zedOdom, this.visiblePosts[lastIndex].odom);
      targetRight = {
        bearing: calcRelativeBearing(this.zedOdom.bearing_deg, radToDeg(rightBear)),
        distance: rightDist,
        id: this.visiblePosts[lastIndex].id
      };
    }

    return [targetLeft, targetRight];
  } /* computeTargetList() */

  /* Update posts list on change. */
  updateArTags(newArTags:ArTag[]):void {
    this.arTags = Object.assign([], newArTags);
    this.getPosts();
  } /* updateArTags() */

  /* Update zedOdom on currOdom change. */
  updateCurrOdom(newCurrOdom:Odom):void {
    this.currOdom = newCurrOdom;
    this.updateZedOdom();
  } /* updateCurrOdom() */

  /* Update field of view options on change. */
  updateFov(newFov:FieldOfViewOptions):void {
    this.fov = newFov;
  } /* updateFov() */

  /* Update gates list on change. */
  updateGates(newGates:Gate[]):void {
    this.gates = Object.assign([], newGates);
    this.getPosts();
  } /* updateGates() */

  /* Update ZED gimbal position on change. */
  updateZedGimbalPos(newZedGimbalPos:ZedGimbalPosition):void {
    this.zedGimbalPos = newZedGimbalPos;
    this.updateZedOdom();
  }

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* Get the list of posts from the lists of ar tags and gates. */
  private getPosts():void {
    this.posts = Object.assign([], this.arTags);
    this.gates.forEach((gate) => {
      const rightPostLoc:Odom = calcRelativeOdom(gate.odom,
                                                 gate.orientation,
                                                 gate.width / 2,
                                                 this.fieldCenterOdom);
      this.posts.push({
        id: gate.rightId,
        odom: rightPostLoc,
        orientation: gate.orientation + 90,
        isHidden: false
      });

      const leftPostLoc:Odom = calcRelativeOdom(gate.odom,
                                                gate.orientation + 180,
                                                gate.width / 2,
                                                this.fieldCenterOdom);
      this.posts.push({
        id: gate.leftId,
        odom: leftPostLoc,
        orientation: gate.orientation + 90,
        isHidden: false
      });
    });
  } /* getPosts() */

  /* If the center is in view, then at least half the target is so consider
     it visible. If beyond depth of view, consider it visible if two corners
     are visible. */
  private isPostVisible(orgPost:ArTag, index:number):boolean {
    const post = orgPost;
    const [dist, bear]:[number, number] = calcDistAndBear(this.zedOdom, post.odom);
    const relBear:number = calcRelativeBearing(this.zedOdom.bearing_deg, radToDeg(bear));

    /* Special Case: guassian noise */
    const num:number = randnBm(0, 1, 1);

    // console.log(state.simSettings.noisePercent);
    const divisor = 18.0;
    const factor = 1.0 / divisor;
    const indThres = Math.round(state.simSettings.noisePercent / factor);
    const thres = getGaussianThres(indThres);

    if (num < thres) {
      post.isHidden = true;
      return false;
    }
    else {
      post.isHidden = false;
    }

    /* Step 1: Check if post is blocked by other posts */
    /* Find edges of current post */
    let postLeftEdge:[number, number] = [dist, relBear];
    let postRightEdge:[number, number] = [dist, relBear];

    /* list of distances and relative bearings to post corners */
    const cornerInfos:[number, number][] = [];
    const internalAngle:number = (POST.numSides - 2) * 180 / POST.numSides;
    const postHeight:number = POST.sideLen * Math.sin(degToRad(internalAngle));

    for (let angle = 0; angle < 360; angle += 360 / POST.numSides) {
      /* get corner info */
      const cornerLoc:Odom = calcRelativeOdom(post.odom,
                                              compassModDeg(post.orientation + angle),
                                              postHeight / 2,
                                              this.fieldCenterOdom);
      const [cornerDist, cornerBear]:[number, number] = calcDistAndBear(this.zedOdom, cornerLoc);
      const relCornerBear:number = calcRelativeBearing(this.zedOdom.bearing_deg,
                                                       radToDeg(cornerBear));
      cornerInfos.push([cornerDist, relCornerBear]);

      /* update post edges */
      if (relCornerBear <= postLeftEdge[1]) {
        postLeftEdge = [cornerDist, relCornerBear];
      }
      else if (relCornerBear >= postRightEdge[1]) {
        postRightEdge = [cornerDist, relCornerBear];
      }
    }

    /* Find ranges of other posts */
    const closedIntervals:Interval[] = [];
    this.posts.forEach((otherPost, i) => {
      if (i !== index) {
        const [otherDist, otherBear]:[number, number] = calcDistAndBear(this.zedOdom,
                                                                        otherPost.odom);
        const otherRelBear:number = calcRelativeBearing(this.zedOdom.bearing_deg,
                                                        radToDeg(otherBear));

        let otherPostLeftEdge:[number, number] = [otherDist, otherRelBear];
        let otherPostRightEdge:[number, number] = [otherDist, otherRelBear];
        for (let angle = 0; angle < 360; angle += 360 / POST.numSides) {
          /* get corner info */
          const cornerLoc:Odom = calcRelativeOdom(otherPost.odom,
                                                  compassModDeg(post.orientation + angle),
                                                  postHeight / 2,
                                                  this.fieldCenterOdom);
          const [cornerDist, cornerBear]:[number, number] = calcDistAndBear(this.zedOdom,
                                                                            cornerLoc);
          const relCornerBear:number = calcRelativeBearing(this.zedOdom.bearing_deg,
                                                           radToDeg(cornerBear));

          /* update post edges */
          if (relCornerBear <= otherPostLeftEdge[1]) {
            otherPostLeftEdge = [cornerDist, relCornerBear];
          }
          else if (relCornerBear >= otherPostRightEdge[1]) {
            otherPostRightEdge = [cornerDist, relCornerBear];
          }
        }

        /* If overlaps on left half */
        if (postLeftEdge[1] >= otherPostLeftEdge[1] &&
            postLeftEdge[1] <= otherPostRightEdge[1] &&
            postLeftEdge[0] > otherPostRightEdge[0]) {
          closedIntervals.push([otherPostLeftEdge[1], otherPostRightEdge[1]]);
        }

        /* If overlaps on the right half */
        else if (postRightEdge[1] >= otherPostLeftEdge[1] &&
                 postRightEdge[1] <= otherPostRightEdge[1] &&
                 postRightEdge[0] > otherPostLeftEdge[0]) {
          closedIntervals.push([otherPostLeftEdge[1], otherPostRightEdge[1]]);
        }
      }
    });

    const intervalHeap:OpenIntervalHeap = new OpenIntervalHeap(postLeftEdge[1], postRightEdge[1],
                                                               closedIntervals);

    /* If less than 50% of tag unblocked, we can't see it */
    const openInterval:Interval|null = intervalHeap.getNextOpenInterval();
    if (openInterval === null ||
        (openInterval[1] - openInterval[0] < (postRightEdge[1] - postLeftEdge[1]) / 2)) {
      return false;
    }

    /* Step 2: Check if post is in field of view */
    /* Case 1: post is too close to see */
    if (dist < PERCEPTION.minVisibleDistTag) {
      return false;
    }

    /* Case 2: post center is within field of view */
    else if (dist <= this.fov.depth &&
        relBear >= -this.fov.angle / 2 &&
        relBear <= this.fov.angle / 2) {
      return true;
    }

    /* Case 3: post center is within field of view angles but beyond depth.
               We consider these posts visible if we can see at 2 of the 3
               corners. */
    else if (relBear >= -this.fov.angle / 2 && relBear <= this.fov.angle / 2) {
      let visibleCorners = 0;

      for (let i = 0; i < POST.numSides; i += 1) {
        const [cornerDist, relCornerBear]:[number, number] = cornerInfos[i];
        if (cornerDist <= this.fov.depth &&
            relCornerBear >= -this.fov.angle / 2 &&
            relCornerBear <= this.fov.angle / 2) {
          visibleCorners += 1;
        }
      }
      return visibleCorners >= 2;
    }

    /* Case 4: post is outside field of view angles */
    else {
      return false;
    }
  } /* isPostVisible() */

  /* Calculate the odometry of the ZED */
  private updateZedOdom():void {
    this.zedOdom = calcRelativeOdom(this.currOdom, this.currOdom.bearing_deg,
                                    ROVER.length / 2, this.fieldCenterOdom);
    this.zedOdom.bearing_deg = compassModDeg(this.currOdom.bearing_deg + this.zedGimbalPos.angle);
  } /* updateZedOdom() */
} /* TargetDetector */
