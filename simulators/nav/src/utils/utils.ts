/* This file contains utility functions used throughout the application. */

import {
  ArTag,
  Joystick,
  Odom,
  OdomFormat,
  Point2D,
  Speeds
} from './types';

/**************************************************************************************************
 * Constants
 **************************************************************************************************/
/* Number of decimals to display on the degrees (when in OdomFormat is D) */
const DEG_DECIMALS = 8;

/* Number of decimals to display on the minutes (when in OdomFormat is DM) */
const MIN_DECIMALS = 6;

/* Number of decimals to display on the seconds (when in OdomFormat is DMS) */
const SEC_DECIMALS = 4;

/* Radius of Earth in meters */
const EARTH_RADIUS = 6371000.0;

/**************************************************************************************************
 * Public Utility Functions
 **************************************************************************************************/
/* Calculate new gps location based a joystick command.
   CAUTION: This function assumes constant speeds over
   the time interval */
export function applyJoystick(
    currOdom:Odom,
    canvasCent:Odom,
    command:Joystick,
    deltaTime:number, /* seconds */
    currSpeed:Speeds
):Odom {
  /* Find change in bearing */
  const deltaBear:number = deltaTime * currSpeed.turn * command.left_right;
  const deltaBearRad:number = Math.abs(degToRad(deltaBear));

  /* Find distance travelled */
  const dist:number = deltaTime * currSpeed.drive * command.forward_back;

  /* The path we drive forms an arc segment of a circle where our start and
     end locations are on the edge of the circle. delta_bearing is the arc
     angle of our arc segment. The straight line between our start and end
     locations forms a chord of this circle. */
  /* Find the radius of the circle. This utilizes the following equations:
     * circumference = 2 * PI * radius
     * arc_length = circumference * arc_angle / 360
     * [DERIVED]: radius = arc_length / arc_angle */
  const radius:number = deltaBear !== 0 ? Math.abs(dist) / deltaBearRad : 0;

  /* Find chord length using law of sines. */
  const chordLen:number =  Math.sin(deltaBearRad) * radius /
                           Math.sin((Math.PI / 2) - (deltaBearRad / 2)) * (dist > 0 ? 1 : -1);

  /* Calculate new location */
  let halfwayBear:number = degToRad(currOdom.bearing_deg + (deltaBear / 2));
  halfwayBear = compassModRad(halfwayBear);
  const nextPointMeters:Point2D = odomToMeters(currOdom, canvasCent);
  nextPointMeters.x += chordLen * Math.cos(compassToCanvasRad(halfwayBear));
  nextPointMeters.y += chordLen * Math.sin(compassToCanvasRad(halfwayBear));
  const nextOdom:Odom = metersToOdom(nextPointMeters, canvasCent);

  /* Calculate new bearing */
  nextOdom.bearing_deg = deltaBear;
  nextOdom.bearing_deg = compassModDeg(currOdom.bearing_deg + deltaBear);

  return nextOdom;
} /* applyJoystick() */


/* Compare ArTags. ArTags on the left (relative to the source) are "less than"
   ArTags on the right. Ties are broken by the smaller distance (compared to the
   source). */
export function arTagCompare(source:Odom, tag0:ArTag, tag1:ArTag):number {
  const [dist0, bear0]:[number, number] = calcDistAndBear(source, tag0.odom);
  const relBear0 = calcRelativeBearing(source.bearing_deg, radToDeg(bear0));

  const [dist1, bear1]:[number, number] = calcDistAndBear(source, tag1.odom);
  const relBear1 = calcRelativeBearing(source.bearing_deg, radToDeg(bear1));

  if (relBear0 === relBear1) {
    return dist0 - dist1;
  }
  else {
    return relBear0 - relBear1;
  }
} /* arTagCompare() */


/* Calculate the distance and bearing (in radians) from north from o1 to o2. */
export function calcDistAndBear(o1:Odom, o2:Odom):[number, number] {
  const p1:Point2D = degToRad2D(odomToPoint(o1));
  const p2:Point2D = degToRad2D(odomToPoint(o2));
  const avgLatRad = (p1.y + p2.y) / 2;

  const dLatRad:number = p2.y - p1.y;
  const dLonRad:number = p2.x - p1.x;

  /* Calculate radius of slice of earth at current latitude in meters. */
  const currLatRadius = calcLatRadius(avgLatRad);

  /* Multiply by -1 because canvas increases y-axis downward rather than
     upward. */
  const dLatMeters:number = EARTH_RADIUS * dLatRad * -1;

  const dLonMeters:number = currLatRadius * dLonRad;

  const dist = Math.sqrt((dLatMeters * dLatMeters) + (dLonMeters * dLonMeters));
  const bearing = canvasToCompassRad(Math.atan2(dLatMeters, dLonMeters));

  return [dist, bearing];
} /* calcDistAndBear() */


/* Calculate the bearing to an object relative to another object's heading using
   the bearing from north between the objects. */
export function calcRelativeBearing(
    currHeading:number, /* degrees */
    absBear:number /* degrees */
):number {
  let relBear:number = absBear - currHeading;
  if (relBear > 180) {
    relBear -= 360;
  }
  else if (relBear < -180) {
    relBear += 360;
  }
  return relBear;
} /* calcRelativeBearing() */


/* Calculate the odometry point that is absDist away from currOdom
   in the direction of absBear. */
export function calcRelativeOdom(
    currOdom:Odom,
    absBear:number, /* degrees from north */
    absDist:number, /* meters */
    canvasCent:Odom
):Odom {
  const relOdomRad:Point2D = degToRad2D(odomToPoint(currOdom));
  const canvasAngleRad:number = compassToCanvasRad(degToRad(absBear));

  /* Multiply by -1 because canvas y-axis increases downward rather than up. */
  const dLatRad:number = metersToLatRad(Math.sin(canvasAngleRad) * absDist) * -1;
  const canvasCentRad:Point2D = degToRad2D(odomToPoint(canvasCent));
  const dLonRad:number = metersToLonRad(Math.cos(canvasAngleRad) * absDist, canvasCentRad);
  relOdomRad.x += dLonRad;
  relOdomRad.y += dLatRad;
  return pointToOdom(radToDeg2D(relOdomRad));
} /* calcRelativeOdom() */


/* Convert an angle from radians on the simulator field to the
   equivalent on a compass.
   The simulator has 0 at "3 o'clock" and goes counter clockwise. */
export function canvasToCompassRad(angle:number):number {
  return compassModRad(angle + (Math.PI / 2));
} /* canvasToCompassRad() */


/* Convert from pixels to corresponding gps point. */
export function canvasToOdom(
    point:Point2D, /* pixels */
    canvasSize:number, /* pixels */
    scale:number, /* pixels/meter */
    canvasCent:Odom
):Odom {
  return metersToOdom(canvasToMeters(point, canvasSize, scale), canvasCent);
} /* canvasToOdom() */


/* Calculate the positive modulo in degrees using 360 as modulus. */
export function compassModDeg(angle:number):number {
  return ((angle % 360) + 360) % 360;
} /* compassModDeg() */


/* Convert an angle from radians on a compass to the
   equivalent on the simulator field.
   The simulator has 0 at "3 o'clock" and goes counter clockwise. */
export function compassToCanvasRad(angle:number):number {
  return compassModRad(angle - (Math.PI / 2));
} /* compassToCanvasRad() */


/* Convert from degrees to radians */
export function degToRad(angle:number):number {
  return angle * Math.PI / 180;
} /* degToRad() */


export function odomToCanvas(
    point:Odom,
    canvasCent:Odom,
    canvasSize:number, /* pixels */
    scale:number
):Point2D {
  return metersToCanvas(odomToMeters(point, canvasCent), canvasSize, scale);
} /* odomToCanvas() */


/* Convert from radians to degrees. */
export function radToDeg(angle:number):number {
  return angle * 180 / Math.PI;
} /* radToDeg() */


/* Convert latitude or longitude into a string. */
export function stringifyLatLon(
    deg:number,
    min:number,
    latLon:string, /* 'lat' or 'lon' */
    format:OdomFormat
):string {
  let dir:string;
  if (latLon === 'lat') {
    dir = deg < 0 ? 'S' : 'N';
  }
  else {
    dir = deg < 0 ? 'W' : 'E';
  }
  const magnitude:string = convertDMS(format, deg, min);
  return `${magnitude} ${dir}`;
} /* stringifyLatLon() */


/* Rotate a point and then translate it to a new origin. */
export function transformPoint(point:Point2D, origin:Point2D, angle:number):Point2D {
  return translatePoint(rotatePoint(point, origin, angle), origin);
} /* transformPoint() */


/**************************************************************************************************
 * Private Utility Functions
 **************************************************************************************************/
/* Find center point of canvas in pixels.
   CAUTION: this assumes a square canvas. */
function calcCanvasCenter(size:number):Point2D {
  return {
    x: size / 2,
    y: size / 2
  };
} /* calcCanvasCenter() */


/* Calculate radius of earth at a given latitude in meters. */
function calcLatRadius(lat:number):number {
  return Math.abs(EARTH_RADIUS * Math.cos(lat));
} /* calcLatRadius() */


/* Convert from pixels (origin is top-left of canvas) to meters (origin
   is middle of canvas). */
function canvasToMeters(
    point:Point2D, /* pixels */
    canvasSize:number, /* pixels */
    scale:number /* pixels/meter */
):Point2D {
  const canvasCent:Point2D = calcCanvasCenter(canvasSize);
  return {
    x: (point.x - canvasCent.x) / scale,
    y: (point.y - canvasCent.y) / scale
  };
} /* canvasToMeters() */


/* Calculate the positive modulo in radians using 2 pi as modulus. */
function compassModRad(angle:number):number {
  return ((angle % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
} /* compassModRad() */


/* Convert the input coordinate to the format corresponding to the input format.
   Note that any format can be input. */
function convertDMS(
    format:OdomFormat,
    deg:number,
    min = 0,
    sec = 0
):string {
  let degOut:number = Math.abs(deg) + (Math.abs(min) / 60) + (Math.abs(sec) / (60 * 60));
  if (format === OdomFormat.D) {
    /* Format output */
    degOut = Number(degOut.toFixed(DEG_DECIMALS));

    return `${degOut}º`;
  }
  else if (format === OdomFormat.DM) {
    let minOut:number = (degOut - Math.trunc(degOut)) * 60;

    /* Format output */
    degOut = Math.trunc(degOut);
    minOut = Number(minOut.toFixed(MIN_DECIMALS));

    return `${degOut}º ${minOut}'`;
  }
  else {
    let minOut:number = (degOut - Math.trunc(degOut)) * 60;
    let secOut:number = (minOut - Math.trunc(minOut)) * 60;

    /* Format output */
    degOut = Math.trunc(degOut);
    minOut = Math.trunc(minOut);
    secOut = Number(secOut.toFixed(SEC_DECIMALS));

    return `${degOut}º ${minOut}' ${secOut}"`;
  }
} /* convertDMS() */


/* Convert (lat, lon) from degrees to radians */
function degToRad2D(coords:Point2D):Point2D {
  return {
    x: degToRad(coords.x),
    y: degToRad(coords.y)
  };
} /* degToRad2D() */


/* Convert from meters (origin is middle of canvas) to pixels (origin is
   top-left of canvas). */
function metersToCanvas(
    point:Point2D /* meters */,
    canvasSize:number /* pixels */,
    scale:number /* pixels/meter */
):Point2D {
  const canvasCent:Point2D = calcCanvasCenter(canvasSize);
  return {
    x: (point.x * scale) + canvasCent.x,
    y: (point.y * scale) + canvasCent.y
  };
} /* metersToCanvas() */


/* Convert meters to latitiude radians on the earth's surface */
function metersToLatRad(meters:number):number {
  return meters / EARTH_RADIUS;
} /* metersToLatRad() */


/* Convert meters to longitude radians on the earth's surface */
function metersToLonRad(meters:number, canvasCentRad:Point2D):number {
  /* Calculate radius of slice of earth at current latitude in meters. */
  const currLatRadius:number = calcLatRadius(canvasCentRad.y);

  return meters / currLatRadius;
} /* metersToLonRad() */


/* Convert point in meters to latitude and longitude */
function metersToOdom(point:Point2D, canvasCent:Odom):Odom {
  const canvasCentRad:Point2D = degToRad2D(odomToPoint(canvasCent));

  /* Calculate change in latitude using:
     * radius_radians = arc_length / arc_angle
     Multiply by -1 because canvas y-axis increases downward rather than up. */
  const dLatRad:number = metersToLatRad(point.y) * -1;

  /* Calculate change in longitude. */
  const dLonRad:number = metersToLonRad(point.x, canvasCentRad);

  const coordOut = {
    x: canvasCentRad.x + dLonRad,
    y: canvasCentRad.y + dLatRad
  };
  return pointToOdom(radToDeg2D(coordOut));
} /* metersToOdom() */


/* Convert from Odom to Point2D in meters (origin at middle of canvas) */
function odomToMeters(point:Odom, canvasCent:Odom):Point2D {
  const [dist, bearing]:[number, number] = calcDistAndBear(canvasCent, point);
  const theta:number = compassToCanvasRad(bearing);
  return {
    x: dist * Math.cos(theta),
    y: dist * Math.sin(theta)
  };
} /* odomToMeters() */


/* Convert odom from (Degrees, Minutes) to (Degrees) */
function odomToPoint(odom:Odom):Point2D {
  return {
    x: odom.longitude_deg + (odom.longitude_min / 60.0),
    y: odom.latitude_deg + (odom.latitude_min / 60.0)
  };
} /* odomToPoint() */


/* Convert point from (Degrees) to (Degrees, Minutes) */
function pointToOdom(point:Point2D):Odom {
  return {
    latitude_deg: Math.trunc(point.y),
    latitude_min: (point.y % 1) * 60,
    longitude_deg: Math.trunc(point.x),
    longitude_min: (point.x % 1) * 60,
    bearing_deg: 0,
    speed: -1
  };
} /* pointToOdom() */


/* Convert (lat, lon) from radians to degrees */
function radToDeg2D(coords:Point2D):Point2D {
  return {
    x: radToDeg(coords.x),
    y: radToDeg(coords.y)
  };
} /* radToDeg2D() */


/* Find location of point after rotating around origin. */
function rotatePoint(
    point:Point2D /* pixels */,
    origin:Point2D /* pixels */,
    angle:number /* degrees of rotation on compass */
):Point2D {
  /* Multiply by -1 because compass and canvas rotate opposite directions */
  const angleRad:number = -1 * degToRad(angle);
  return {
    x: ((point.x - origin.x) * Math.cos(angleRad)) +
        ((origin.y - point.y) * Math.sin(angleRad)) + origin.x,
    y: origin.y - (((point.x - origin.x) * Math.sin(angleRad)) -
        ((origin.y - point.y) * Math.cos(angleRad)))
  };
} /* rotatePoint() */


/* Translate a point from the origin (0, 0) to the input origin. Return the
   coordinates of the input point relative to the input origin. Note that the
   y-axis is positive in the downwards direction like in the canvas. */
function translatePoint(point:Point2D, origin:Point2D):Point2D {
  return {
    x: point.x - origin.x,
    y: origin.y - point.y
  };
} /* translatePoint() */
