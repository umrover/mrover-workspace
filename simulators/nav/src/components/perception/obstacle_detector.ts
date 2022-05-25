/* This files contains the ObstacleDetector class which handles the logic for
   detecting obstacles. */

import {
  FieldOfViewOptions,
  ObstacleField,
  Odom,
  Point2D,
  ZedGimbalPosition,
  Obstacle,
  ObstacleMessage
} from '../../utils/types';
import {
  calcDistAndBear,
  calcRelativeBearing,
  calcRelativeOdom,
  canvasToCompassRad,
  compassModDeg,
  odomToCanvas,
  radToDeg,
  transformPoint,
  obstacleFieldToBoundingBox
} from '../../utils/utils';
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
  private obstacles!:ObstacleField[];

  /* scale of the canvas in pixels/meter */
  private scale!:number;

  /* obstacles visible to the rover */
  private visibleObstacles!:ObstacleField[];

  /* position of the ZED's gimbal */
  private zedGimbalPos!:ZedGimbalPosition;

  /* location of the ZED (i.e. rover's eyes) */
  private zedOdom!:Odom;

  /* size of the Obstacle */
  private size!:number;

  /************************************************************************************************
   * Public Methods
   ************************************************************************************************/
  /* Initialize ObstacleDetector. */
  constructor(
      currOdom:Odom,
      zedGimbalPos:ZedGimbalPosition,
      obstacles:ObstacleField[],
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

    // Conver all visible obstacles to bounding box
    const obstaclesList:Obstacle[] = [];
    this.visibleObstacles.forEach((obsField) => {
      const obs = obstacleFieldToBoundingBox(obsField, this.currOdom);
      obstaclesList.push(obs);
    });

    obsMsg = { numObstacles: obstaclesList.length, obstacles: obstaclesList };

    return obsMsg;
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
  updateObstacles(newObstacles:ObstacleField[]):void {
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
    let tempSize = 0;
    for (let i = 0; i < this.visibleObstacles.length; i += 1) {
      const obs:ObstacleField = this.visibleObstacles[i];
      let dist:number = calcDistAndBear(this.zedOdom, obs.odom)[0];
      dist -= obs.size / 2;
      if (dist < minDist) {
        minDist = dist;
        tempSize = this.visibleObstacles[i].size;
      }
    }

    if (minDist === Infinity) {
      this.obsDist = -1;
      this.size = 0;
    }
    else {
      if (minDist < 0) {
        minDist = 0;
      }
      this.obsDist = minDist;
      this.size = tempSize;
    }
  } /* findClosestObs() */

  /* Does there appear to be a clear path in the direction of angle (angle is
     in degrees from north)? */
  // private isPathClear(angle:number):ObstacleOld|null {
  //   for (let i = 0; i < this.visibleObstacles.length; i += 1) {
  //     if (this.isObsInPath(this.visibleObstacles[i], angle)) {
  //       return null;
  //     }
  //   }

  //   return {
  //     distance: this.obsDist, /* Will be -1 if okay to go straight ahead (i.e. bearing = 0) */
  //     bearing: calcRelativeBearing(this.zedOdom.bearing_deg, angle),
  //     size: this.size
  //   };
  // } /* isPathClear() */

  /* Is obs in a path in the direction of angle. Note that this does not take
     into account the field of view angles. This works because when this
     function is called we have already filtered out the non-visible obstacles.
     A visual of this can be viewed on the team google drive:
     https://drive.google.com/open?id=19d0Meyjjbjb5X7W5VQUFeIY3iJwj1PPV */
  private isObsInPath(obs:ObstacleField, angle:number /* degrees */):boolean {
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
  private isObsVisible(obs:ObstacleField):boolean {
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
