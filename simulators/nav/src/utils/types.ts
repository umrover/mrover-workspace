/* This file contains the types (enums, interfaces, and types) used throughout
   the application. */


/* Interface for representing AR tags on the field. */
export interface ArTag {
  id:number;
  odom:Odom;
  orientation:number; /* degrees from north */
  isHidden:boolean;
}


/* Interface representing the options for drawing AR tags used in the DrawModule
   component. */
export interface ArTagDrawOptions {
  id:number;
}


/* Interface representing the current debug settings used in the DebugTools
   component. */
export interface DebugOptions {
  fieldOfView:FieldOfViewOptions;
  paused:boolean;
  roverPathVisible:boolean;
  takeStep:boolean;
}


/* Interface representing all the draw options including which mode we are in
   and the specific options for each mode used in the DrawModule component. */
export interface DrawOptions {
  arTag:ArTagDrawOptions;
  gate:GateDrawOptions;
  mode:FieldItemType;
  obstacle:ObstacleDrawOptions;
  waypoint:WaypointDrawOptions;
}


/* Enum representing the types of field items used in the DrawModule and Field
   components. */
export enum FieldItemType {
  WAYPOINT,
  OBSTACLE,
  AR_TAG,
  GATE,
  REFERENCE_POINT,
  MOVE_ROVER
}


/* Interface representing the rover's field of view used in the Perception,
   DebugTools, and Field components. */
export interface FieldOfViewOptions {
  angle:number; /* degrees, total field of view angle */
  depth:number; /* meters */
  visible:boolean;
}


/* Interface representing the field state used in the field state module of the
   store. */
export interface FieldState {
  arTags:ArTag[];
  canvasHeight:number; /* pixels */
  centerOdom:Odom;
  gates:Gate[];
  obstacles:Obstacle[];
  referencePoints:Odom[];
  repeaterLoc:Odom|null;
  size:number;
  waypoints:Waypoint[];
}


/* Interface for representing gates on the field. */
export interface Gate {
  leftId:number;
  rightId:number;
  odom:Odom; /* center of two gate posts */
  orientation:number; /* degrees from north */
  width:number; /* meters */
}


/* Interface representing the options for drawing gates used in the DrawModule
   component. */
export interface GateDrawOptions {
  leftPostId:number;
  rightPostId:number;
  width:number; /* meters */
  orientation:number; /* degrees, angle from north of gate (post 1 to post 2) */
}


/* Interface representing a joystick command. This must be the same as the
   joystick LCM. */
export interface Joystick {
  forward_back:number;
  left_right:number;
}


/* Interface representing the statuses of the various LCM connections of the
   simulator to other rover programs. */
export interface LCMConnections {
  lcmBridge:boolean;
  localization:boolean;
  nav:boolean;
  perception:boolean;
}


/* Interface representing the NavStatus LCM. This must be the same as the
   NavStatus LCM. */
export interface NavStatus {
  nav_state_name:string;
  completed_wps:number;
  total_wps:number;
}


/* Interface for representing obstacles on the field. */
export interface Obstacle {
  odom:Odom;
  size:number; /* meters */
}


/* Interface representing the Obstacle LCM. This must be the same as the
   Obstacle LCM. */
export interface ObstacleMessage {
  distance:number; /* meters from rover */
  bearing:number; /* degrees from rover */
}


/* Interface representing the options for drawing obstacles used in the
   DrawModule component. */
export interface ObstacleDrawOptions {
  size:number; /* meters */
}


/* Interface representing the Odom LCM. This must be the same as the Odom
   LCM. */
export interface Odom {
  latitude_deg:number;
  latitude_min:number;
  longitude_deg:number;
  longitude_min:number;
  bearing_deg:number; /* degrees from north */
  speed:number;
}


/* Enum representing the different odom formatting options. */
export enum OdomFormat {
  D,  /* degrees */
  DM, /* degrees, minutes */
  DMS /* degrees, minutes, seconds */
}


// /* Data structure storing the information needed to draw the rover's path. */
// export interface Path {
//   // fullPath:Point2D[]; /* list of canvas locations */
//   path:PathSnapshot[]; /* list of sets of 4 canvas locations */
// }


//  Data structure representing a single instance of the path
// export interface PathSnapshot {
//   loc:Odom
// }

/* Interface repressenting the constant values used to define the perception
   system. */
export interface PerceptionConstants {

  /* minimum distance a tag's center must be away to see it, meters */
  minVisibleDistTag:number;
}


/* Interface representing a point in 2D space for use on the canvas and various
   field calculations. */
export interface Point2D {
  x:number;
  y:number;
}


/* Interface representing the constant values used to define the size of a
   post. */
export interface PostConstants {
  numSides:number;
  sideLen:number; /* meters */
}


/* Templated interface representing an option in a RadioSelector component. */
export interface RadioOption<T> {
  value:T;
  name:string;
}


/* Interface representing the constant values used to define the strong and weak
   signal strength ranges for the radio signal. */
export interface RepeaterConstants {

  /* Maximum valid signal strength. */
  maxSignalStrength:number;

  /* Minimum valid signal strength. */
  minSignalStrength:number;

  /* Cutoff between strong and weak signal strength. Less than this value is
     weak. */
  lowSignalStrengthThreshold:number;
}


/* Interface representing the constant values used to define the size of the
   rover. */
export interface RoverConstants {
  length:number; /* meters */
  width:number; /* meters */
}


/* Enum representing the different source locations for measuring distance from
   the rover. This is used in measuring distances from objects like the ZED and
   GPS differently. */
export enum RoverLocationSource {
  GPS,
  ZED
}


/* Interface representing the rover state used in the rover state module of the
   store. */
export interface RoverState {
  currOdom:Odom;
  currSpeed:Speeds;
  joystick:Joystick;
  navStatus:NavStatus;
  obstacleMessage:ObstacleMessage;
  radioSignalStrength:number;
  targetList:TargetListMessage;
  zedGimbalCmd:ZedGimbalPosition; /* Desired position of the ZED gimbal. */
  zedGimbalPos:ZedGimbalPosition; /* Current position of the ZED gimbal. */
}


/* Interface representing the different settings related to simulating different
   aspects of the environment that can be turned on and off. */
export interface SimulationSettings {
  simulateLoc:boolean;
  simulatePercep:boolean;
  noisePercent:number;
}


/* Interface representing the simulator state used in the simulator state
   module of the store. */
export interface SimulatorState {
  autonOn:boolean;
  debugOptions:DebugOptions;
  drawOptions:DrawOptions;
  lcmConnections:LCMConnections;
  odomFormat:OdomFormat;
  path:Odom[];
  simSettings:SimulationSettings;
  startLoc:Odom;
}


/* Interface representing the speed at which the rover can move. This is the
   current speed and not the maximum speed. */
export interface Speeds {
  drive:number; /* m/s */

  /* used for both the rover turning and the ZED gimbal turning */
  turn:number; /* degrees/s */
}


/* Interface representing the Target LCM. This must be the same as the Target
   LCM. */
export interface TargetMessage {
  bearing:number; /* degrees from rover's heading */
  distance:number; /* meters from rover */
  id:number;
}


/* Type representing the TargetList LCM. This must be the same as the
   TargetList LCM. */
export type TargetListMessage = [TargetMessage, TargetMessage];


/* Interface representing the Waypoint LCM. This must be the same as the
   Waypoint LCM. This is also used for representing waypoints on the field. */
export interface Waypoint {
  gate:boolean; /* is there an associated gate */
  gate_width:number; /* associated gate width, meters */
  id:number; /* associated target id */
  odom:Odom;
  search:boolean; /* is this is a search point */
}


/* Interface representing the options for drawing waypoints used in the
   DrawModule component. */
export interface WaypointDrawOptions {
  gate:boolean; /* is there an associated gate */
  gate_width:number; /* associated gate width, meters */
  search:boolean; /* is this is a search point */
  targetId:number; /* associated target id */
}


/* Type representing a set of wheel locations */
export type WheelLocs = [
  Point2D, /* front left */
  Point2D, /* front right */
  Point2D, /* back left */
  Point2D  /* back right */
];

/* Enum representing the wheel positions */
export enum WheelPositions {
  FrontLeft,
  FrontRight,
  BackLeft,
  BackRight
}

/* Interface representing the constant values used to define the ZED and ZED
   gimbal. */
export interface ZedConstants {
  gimbal:{
    minAngle:number; /* degrees */
    maxAngle:number; /* degrees */
  };
}


/* Position for ZED gimbal */
export interface ZedGimbalPosition {
  angle:number; /* absolute angle from rover's heading, -180 to 180 */
}
