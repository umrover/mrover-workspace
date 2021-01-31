/* This files contains the ObstacleDetector class which handles the logic for
   detecting obstacles. */

import {
  FieldOfViewOptions,
  Obstacle,
  ObstacleMessage,
  Odom,
  Point2D,
  ZedGimbalPosition
} from '../../utils/types';
import {
  calcDistAndBear,
  calcRelativeBearing,
  calcRelativeOdom,
  canvasToCompassRad,
  compassModDeg,
  odomToCanvas,
  radToDeg,
  transformPoint
} from '../../utils/utils';
import { Interval, OpenIntervalHeap } from './open_interval_heap';
import { ROVER } from '../../utils/constants';

/**************************************************************************************************
 * Constants
 **************************************************************************************************/
/* minimum size an obstacle must be in order to be detected */
const MIN_OBS_SIZE = 0.1; /* meters */

/* Distance there must be between object and rover in meters. */
const OBS_PAD = 0.25;

/* Class that performs obstacle detection calculations. */
export default class ObstacleDetector {
  /************************************************************************************************
   * Private Members
   ************************************************************************************************/
  /* Height (and width) of canvas in pixels */
  private canvasHeight!:number;

  /* Current rover GPS location */
  private currOdom!:Odom;

  /* GPS point of the center of the canvas */
  private fieldCenterOdom!:Odom;

  /* Rover field of view parameters */
  private fov:FieldOfViewOptions;

  /* distance to the (closest) obstacle */
  private obsDist = -1; /* meters */

  /* list of all obstacles on field */
  private obstacles!:Obstacle[];

  /* scale of the canvas in pixels/meter */
  private scale!:number;

  /* obstacles visible to the rover */
  private visibleObstacles!:Obstacle[];

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
      obstacles:Obstacle[],
      fov:FieldOfViewOptions,
      fieldCenterOdom:Odom,
      canvasHeight:number, /* pixels */
      scale:number /* pixels/meter */
  ) {
    this.canvasHeight = canvasHeight;
    this.currOdom = currOdom;
    this.zedGimbalPos = zedGimbalPos;
    this.fieldCenterOdom = fieldCenterOdom;
    this.fov = fov;
    this.obstacles = obstacles;
    this.scale = scale;

    this.updateZedOdom();
  } /* constructor() */

  /* Calculate the Obstacle LCM message. */
  computeObsMsg():ObstacleMessage {
    let obsMsg:ObstacleMessage|null = null;

    /* Step 1: filter out obstacles not in field of view */
    this.visibleObstacles = this.obstacles.filter((obs) => this.isObsVisible(obs));

    /* Step 2: find distance to closest obstacle in path (-1 if no obstacles) */
    this.findClosestObs();

    /* Step 3: Check if our current path clear */
    obsMsg = this.isPathClear(this.zedOdom.bearing_deg);
    if (obsMsg !== null) {
      obsMsg.distance = -1;
      return obsMsg;
    }

    /* Step 4: find open intervals */
    const closedIntervals:Interval[] = [];
    this.visibleObstacles.forEach((obs) => {
      /* [meters, radians from north] */
      const [dist, bear]:[number, number] = calcDistAndBear(this.zedOdom, obs.odom);

      /* relative angle from rover to obstacle in degrees */
      const relBear:number = calcRelativeBearing(this.zedOdom.bearing_deg, radToDeg(bear));

      /* angle formed by center of obstacle, rover, and edge of obstacle in
         degrees (equates to solving an SSS triangle). This uses law of signs to
         find the angles of the triangle: cos(A) = (b^2 + c^2 - a^2) / 2bc.
         Note that since this will form an isosceles triangle, c = b.
         A visual of this can be viewed on the team google drive:
         https://drive.google.com/open?id=12x-ImdIOt_TJqijZ7b8_DF2l7Spkxc7y */
      const numerator:number = (2 * (dist ** 2)) - ((obs.size / 2) ** 2);
      const denominator:number = 2 * (dist ** 2);
      if (denominator !== 0) {
        /* Bound ratio by -1 and 1 so that acos doesn't return NaN. This could
           happen if we are on top of the obstacle. */
        const ratio:number = Math.max(-1, Math.min(1, numerator / denominator));
        const deltaBear:number = radToDeg(Math.abs(Math.acos(ratio)));
        closedIntervals.push([relBear - deltaBear, relBear + deltaBear]);
      }
      else {
        /* If distance to obstacle is 0, it would cover our entire field of
           view. */
        closedIntervals.push([-this.fov.angle / 2, this.fov.angle / 2]);
      }
    });
    const intervalHeap:OpenIntervalHeap = new OpenIntervalHeap(-this.fov.angle / 2,
                                                               this.fov.angle / 2, closedIntervals);

    /* Step 5: Pick biggest interval that rover can fit through */
    /* path width = rover width + 0.25 meters on each side */
    const pathWdth = ROVER.width + (2 * OBS_PAD); /* meters */

    /* degrees */
    const minIntervalSize:number = 2 * radToDeg(Math.asin(pathWdth / 2 / this.fov.depth));

    let openInterval:Interval|null = intervalHeap.getNextOpenInterval();
    while (openInterval !== null) {
      /* If the interval is too small for the rover, then all remaining intervals are as well. */
      if (openInterval[1] - openInterval[0] < minIntervalSize) {
        break;
      }

      /* Step 5b: Search through sub-intervals of size minIntervalSize within openInterval. */
      /* If interval is to the right, search sub-intervals left to right */
      if ((openInterval[0] + openInterval[1]) / 2 > 0) {
        for (let start:number = openInterval[0]; /* relative degrees */
          start <= openInterval[1] - minIntervalSize;
          start += 1) {
          const end:number = start + minIntervalSize; /* relative degrees */
          const angle = start + (end / 2); /* relative degrees */
          obsMsg = this.isPathClear(compassModDeg(this.zedOdom.bearing_deg + angle));
          if (obsMsg !== null) {
            return obsMsg;
          }
        }
      }

      /* If interval is to the left, search sub-intervals right to left */
      else {
        for (let end:number = openInterval[1]; /* relative degrees */
          end >= openInterval[0] + minIntervalSize;
          end -= 1) {
          const start:number = end - minIntervalSize; /* relative degrees */
          const angle = start + (end / 2); /* relative degrees */
          obsMsg = this.isPathClear(compassModDeg(this.zedOdom.bearing_deg + angle));
          if (obsMsg !== null) {
            return obsMsg;
          }
        }
      }

      /* Go to next open interval. */
      openInterval = intervalHeap.getNextOpenInterval();
    }

    /* Step 6: If no intervals work, pick left or right edge of field of view */
    let angle!:number;

    /* If left side has more open, go left of fov */
    if (intervalHeap.minOccupied > 0) {
      angle = intervalHeap.minOccupied - (minIntervalSize / 2);
    }

    /* If right side has more open, go right of fov */
    else if (intervalHeap.maxOccupied < 0) {
      angle = intervalHeap.maxOccupied + (minIntervalSize / 2);
    }

    /* If right side has more open, go right of fov */
    else if (Math.abs(intervalHeap.minOccupied) > intervalHeap.maxOccupied) {
      angle = intervalHeap.maxOccupied + (minIntervalSize / 2);
    }

    /* If left side has more open, go left of fov */
    else if (Math.abs(intervalHeap.minOccupied) < intervalHeap.maxOccupied) {
      angle = intervalHeap.minOccupied - (minIntervalSize / 2);
    }

    /* If both sides equally occupied, randomly go right of fov */
    else {
      angle = intervalHeap.maxOccupied + (minIntervalSize / 2);
    }

    return {
      distance: this.obsDist,
      bearing: angle
    };
  } /* computeObsMsg() */

  /* Update canvas height on change. */
  updateCanvasHeight(newCanvasHeight:number /* pixels */):void {
    this.canvasHeight = newCanvasHeight;
  } /* updateCanvasHeight() */

  /* Update zedOdom on currOdom change. */
  updateCurrOdom(newCurrOdom:Odom):void {
    this.currOdom = newCurrOdom;
    this.updateZedOdom();
  } /* updateCurrOdom() */

  /* Update field of view options on change. */
  updateFov(newFov:FieldOfViewOptions):void {
    this.fov = newFov;
  } /* updateFov() */

  /* Update obstacles list on change. */
  updateObstacles(newObstacles:Obstacle[]):void {
    this.obstacles = newObstacles;
  } /* updateObstacles() */

  /* Update canvas field scale on change. */
  updateScale(newScale:number):void {
    this.scale = newScale;
  } /* updateScale() */

  /* Update ZED gimbal position on change. */
  updateZedGimbalPos(newZedGimbalPos:ZedGimbalPosition):void {
    this.zedGimbalPos = newZedGimbalPos;
    this.updateZedOdom();
  }

  /************************************************************************************************
   * Private Methods
   ************************************************************************************************/
  /* Set obsDist to the distance to the closest obstacle. */
  private findClosestObs():void {
    let minDist = Infinity;
    for (let i = 0; i < this.visibleObstacles.length; i += 1) {
      const obs:Obstacle = this.visibleObstacles[i];
      let dist:number = calcDistAndBear(this.zedOdom, obs.odom)[0];
      dist -= obs.size / 2;
      if (dist < minDist) {
        minDist = dist;
      }
    }

    if (minDist === Infinity) {
      this.obsDist = -1;
    }
    else {
      if (minDist < 0) {
        minDist = 0;
      }
      this.obsDist = minDist;
    }
  } /* findClosestObs() */

  /* Does there appear to be a clear path in the direction of angle (angle is
     in degrees from north)? */
  private isPathClear(angle:number):ObstacleMessage|null {
    for (let i = 0; i < this.visibleObstacles.length; i += 1) {
      if (this.isObsInPath(this.visibleObstacles[i], angle)) {
        return null;
      }
    }

    return {
      distance: this.obsDist, /* Will be -1 if okay to go straight ahead (i.e. bearing = 0) */
      bearing: calcRelativeBearing(this.zedOdom.bearing_deg, angle)
    };
  } /* isPathClear() */

  /* Is obs in a path in the direction of angle. Note that this does not take
     into account the field of view angles. This works because when this
     function is called we have already filtered out the non-visible obstacles.
     A visual of this can be viewed on the team google drive:
     https://drive.google.com/open?id=19d0Meyjjbjb5X7W5VQUFeIY3iJwj1PPV */
  private isObsInPath(obs:Obstacle, angle:number /* degrees */):boolean {
    /* Locations of objects in pixels */
    const obsLocPx:Point2D = odomToCanvas(obs.odom, this.fieldCenterOdom,
                                          this.canvasHeight, this.scale);

    const zedLocPx:Point2D = odomToCanvas(this.zedOdom, this.fieldCenterOdom,
                                          this.canvasHeight, this.scale);

    /* Location of the obstacle in the transformed coordinates using the ZED
       as the origin and `angle` as y-axis */
    const obsLocPrime:Point2D = transformPoint(obsLocPx, zedLocPx, angle);

    const dist:number = calcDistAndBear(this.zedOdom, obs.odom)[0]; /* meters */

    /* path width = rover width + 0.25 meters on each side */
    const pathWdth = ROVER.width + (2 * OBS_PAD); /* meters */
    const pathWdthPx = this.scale * pathWdth; /* pixels */

    /* meters */
    const depthY:number = Math.sqrt((this.fov.depth ** 2) - ((pathWdth / 2) ** 2));
    const depthYPx:number = this.scale * depthY; /* pixels */

    /* CAUTION: note that positive y indicates behind the zed location because
                of the reversed direction of the y-axis on the canvas. */

    /* Case 1: obstacle center is in visible path. */
    if (dist <= this.fov.depth && obsLocPrime.y <= 0 &&
        obsLocPrime.x >= -pathWdthPx / 2 && obsLocPrime.x <= pathWdthPx / 2) {
      return true;
    }

    /* Case 2: obstacle center is in path beyond field of view depth */
    else if (obsLocPrime.y <= 0 && obsLocPrime.x >= -pathWdthPx / 2 &&
             obsLocPrime.x <= pathWdthPx / 2) {
      /* If some part of the obstacle is in the field of view */
      return dist <= this.fov.depth + (obs.size / 2);
    }

    /* Case 3: obstacle center is in path and behind zed */
    else if (obsLocPrime.x >= -pathWdthPx / 2 && obsLocPrime.x <= pathWdthPx / 2) {
      return obsLocPrime.y < this.scale * -obs.size / 2;
    }

    /* Case 4: obstacle center is left of path and behind zed */
    else if (obsLocPrime.x < -pathWdthPx / 2 && obsLocPrime.y > 0) {
      const corner:Odom = calcRelativeOdom(this.zedOdom,
                                           compassModDeg(this.zedOdom.bearing_deg - 90),
                                           pathWdth / 2,
                                           this.fieldCenterOdom);
      const cornerDist:number = calcDistAndBear(corner, obs.odom)[0];
      return cornerDist <= obs.size / 2;
    }

    /* Case 5: obstacle center is left of path and beyond field of view depth */
    else if (obsLocPrime.x < -pathWdthPx / 2 && -obsLocPrime.y > depthYPx) {
      const angleToCorner = radToDeg(canvasToCompassRad(Math.atan2(-depthY, -pathWdth / 2)));
      const corner:Odom = calcRelativeOdom(this.zedOdom,
                                           compassModDeg(this.zedOdom.bearing_deg +
                                                         angleToCorner),
                                           this.fov.depth,
                                           this.fieldCenterOdom);
      const cornerDist:number = calcDistAndBear(corner, obs.odom)[0];
      return cornerDist <= obs.size / 2;
    }

    /* Case 6: obstacle center is left of path between field of view depth and
               ZED (0 depth) */
    else if (obsLocPrime.x < -pathWdthPx / 2)  {
      return obsLocPrime.x >= (-pathWdthPx / 2) + (this.scale * -obs.size / 2);
    }

    /* Case 7: obstacle center is right of path and behind zed */
    else if (obsLocPrime.y > 0) {
      const corner:Odom = calcRelativeOdom(this.zedOdom,
                                           compassModDeg(this.zedOdom.bearing_deg + 90),
                                           pathWdth / 2,
                                           this.fieldCenterOdom);
      const rightDist:number = calcDistAndBear(corner, obs.odom)[0];
      return rightDist <= obs.size / 2;
    }

    /* Case 8: obstacle center is right of path and beyond field of view
               depth */
    else if (-obsLocPrime.y > depthYPx) {
      const angleToCorner = radToDeg(canvasToCompassRad(Math.atan2(-depthY, pathWdth / 2)));
      const corner:Odom = calcRelativeOdom(this.zedOdom,
                                           compassModDeg(this.zedOdom.bearing_deg + angleToCorner),
                                           this.fov.depth,
                                           this.fieldCenterOdom);
      const cornerDist:number = calcDistAndBear(corner, obs.odom)[0];
      return cornerDist <= obs.size / 2;
    }

    /* Case 9: obstacle center is right of path between field of view depth and
               ZED (0 depth) */
    else {
      return obsLocPrime.x <= (pathWdthPx / 2) + (this.scale * obs.size / 2);
    }
  } /* isObsInPath() */

  /* Determine if obs is currently visible.
     A visual of this can be viewed on the team google drive:
     https://drive.google.com/open?id=1ph_1FzBkFkgKH5QcgHnnoaKyqOWukiUx */
  private isObsVisible(obs:Obstacle):boolean {
    const [dist, bear]:[number, number] = calcDistAndBear(this.zedOdom, obs.odom);
    const relBear:number = calcRelativeBearing(this.zedOdom.bearing_deg, radToDeg(bear));

    /* Case 1: obstacle center is within field of view */
    if (dist <= this.fov.depth && relBear >= -this.fov.angle / 2 &&
        relBear <= this.fov.angle / 2) {
      /* If half of the obstacle is above the threshold size, we can see it.
         CAUTION: This assumes that the field of view angle is at least twice
         MIN_OBS_SIZE. */
      if (obs.size / 2 >= MIN_OBS_SIZE) {
        return true;
      }

      /* If on the left half of our view, is the right half plus the visible
         left half of the obstacle at least MIN_OBS_SIZE */
      if (relBear <= 0) {
        /* Locations of objects in pixels */
        const obsLocPx:Point2D = odomToCanvas(obs.odom, this.fieldCenterOdom,
                                              this.canvasHeight, this.scale);
        const zedLocPx:Point2D = odomToCanvas(this.zedOdom, this.fieldCenterOdom,
                                              this.canvasHeight, this.scale);

        /* Absolute bearing from north (in degrees) to left edge of field of
           view */
        const angle:number = compassModDeg(this.zedOdom.bearing_deg - (this.fov.angle / 2));

        /* Location of the obstacle in the transformed coordinates using the ZED
           as the origin and left edge of field of view as y-axis */
        const obsLocPrime:Point2D = transformPoint(obsLocPx, zedLocPx, angle);

        /* If obstacle is completely visible (note that this assumes that if an
           obstacle covers the entire field of view, it must be large enough to
           be visible) */
        if (obsLocPrime.x / this.scale >= obs.size / 2) {
          return obs.size >= MIN_OBS_SIZE;
        }

        /* Otherwise check if "enough" of it is visible. */
        return (obs.size / 2) + (obsLocPrime.x / this.scale) >= MIN_OBS_SIZE;
      }

      /* If on the right half of our view, is the left half plus the visible
         right half of the obstacle at least MIN_OBS_SIZE */
      else {
        /* Locations of objects in pixels */
        const obsLocPx:Point2D = odomToCanvas(obs.odom, this.fieldCenterOdom,
                                              this.canvasHeight, this.scale);
        const zedLocPx:Point2D = odomToCanvas(this.zedOdom, this.fieldCenterOdom,
                                              this.canvasHeight, this.scale);

        /* Absolute bearing from north (in degrees) to right edge of field of
           view */
        const angle:number = compassModDeg(this.zedOdom.bearing_deg + (this.fov.angle / 2));

        /* Location of the obstacle in the transformed coordinates using the ZED
           as the origin and right edge of field of view as y-axis */
        const obsLocPrime:Point2D = transformPoint(obsLocPx, zedLocPx, angle);

        /* If obstacle is completely visible (note that this assumes that if an
           obstacle covers the entire field of view, it must be large enough to
           be visible) */
        if (-obsLocPrime.x / this.scale >= obs.size / 2) {
          return obs.size >= MIN_OBS_SIZE;
        }

        /* Otherwise check if "enough" of it is visible. */
        return (obs.size / 2) + (-obsLocPrime.x / this.scale) >= MIN_OBS_SIZE;
      }
    }

    /* Case 2: obstacle center is within field of view angles, beyond depth */
    else if (relBear >= -this.fov.angle / 2 && relBear <= this.fov.angle / 2) {
      return dist <= this.fov.depth + (obs.size / 2) - MIN_OBS_SIZE;
    }

    /* Case 3: obstacle center is left of field of view, within depth */
    else if (relBear < -this.fov.angle / 2 && dist <= this.fov.depth) {
      /* Locations of objects in pixels */
      const obsLocPx:Point2D = odomToCanvas(obs.odom, this.fieldCenterOdom,
                                            this.canvasHeight, this.scale);
      const zedLocPx:Point2D = odomToCanvas(this.zedOdom, this.fieldCenterOdom,
                                            this.canvasHeight, this.scale);

      /* Absolute bearing from north (in degrees) to left edge of field of
         view */
      const angle:number = compassModDeg(this.zedOdom.bearing_deg - (this.fov.angle / 2));

      /* Location of the obstacle in the transformed coordinates using the ZED
         as the origin and left edge of field of view as y-axis */
      const obsLocPrime:Point2D = transformPoint(obsLocPx, zedLocPx, angle);

      return -obsLocPrime.x <= this.scale * ((obs.size / 2) - MIN_OBS_SIZE);
    }

    /* Case 4: obstacle center is left of field of view, beyond depth */
    else if (relBear < -1 * this.fov.angle / 2) {
      /* GPS location of the left corner of the field of view. */
      const corner:Odom = calcRelativeOdom(this.zedOdom,
                                           compassModDeg(this.zedOdom.bearing_deg -
                                                         (this.fov.angle / 2)),
                                           this.fov.depth,
                                           this.fieldCenterOdom);
      const cornerDist:number = calcDistAndBear(corner, obs.odom)[0];
      return cornerDist <= (obs.size / 2) - MIN_OBS_SIZE;
    }

    /* Case 5: obstacle center is right of field of view, within depth */
    else if (dist <= this.fov.depth) {
      /* Locations of objects in pixels */
      const obsLocPx:Point2D = odomToCanvas(obs.odom, this.fieldCenterOdom,
                                            this.canvasHeight, this.scale);
      const zedLocPx:Point2D = odomToCanvas(this.zedOdom, this.fieldCenterOdom,
                                            this.canvasHeight, this.scale);

      /* Absolute bearing from north (in degrees) to right edge of field of
         view */
      const angle:number = compassModDeg(this.zedOdom.bearing_deg + (this.fov.angle / 2));

      /* Location of the obstacle in the transformed coordinates using the ZED
         as the origin and right edge of field of view as y-axis */
      const obsLocPrime:Point2D = transformPoint(obsLocPx, zedLocPx, angle);

      return obsLocPrime.x <= this.scale * ((obs.size / 2) - MIN_OBS_SIZE);
    }

    /* Case 6: obstacle center is right of field of view, beyond depth */
    else {
      /* GPS location of the right corner of the field of view. */
      const corner:Odom = calcRelativeOdom(this.zedOdom,
                                           compassModDeg(this.zedOdom.bearing_deg +
                                                         (this.fov.angle / 2)),
                                           this.fov.depth,
                                           this.fieldCenterOdom);
      const cornerDist:number = calcDistAndBear(corner, obs.odom)[0];
      return cornerDist <= (obs.size / 2) - MIN_OBS_SIZE;
    }
  } /* isObsVisible() */

  /* Calculate the odometry of the ZED */
  private updateZedOdom():void {
    this.zedOdom = calcRelativeOdom(this.currOdom, this.currOdom.bearing_deg,
                                    ROVER.length / 2, this.fieldCenterOdom);
    this.zedOdom.bearing_deg = compassModDeg(this.currOdom.bearing_deg + this.zedGimbalPos.angle);
  } /* updateZedOdom() */
} /* ObstacleDetector */
