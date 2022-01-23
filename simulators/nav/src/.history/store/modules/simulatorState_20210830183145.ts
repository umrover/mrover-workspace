/* This file contains the state variables, getters, and mutators for the
   simulator. */

import {
  ArTagDrawOptions,
  FieldItemType,
  FieldOfViewOptions,
  GateDrawOptions,
  ObstacleDrawOptions,
  Odom,
  OdomFormat,
  SimulatorState,
  WaypointDrawOptions
} from '../../utils/types';


/**************************************************************************************************
 * Constants
 **************************************************************************************************/
// const MAX_RECENT_PATH_LEN = 100;
const BEARING_DIFF_THRESHOLD = 0.1;
const MINUTE_DIFF_THRESHOLD = 0.00000001;

const state:SimulatorState = {
  autonOn: false,

  debugOptions: {
    fieldOfView: {
      angle: 110,
      depth: 3,
      visible: true
    },
    paused: false,
    roverPathVisible: true,
    takeStep: false
  },

  drawOptions: {
    arTag: {
      id: 0
    },
    gate: {
      leftPostId: 3,
      rightPostId: 4,
      width: 3,
      orientation: 0
    },
    mode: FieldItemType.WAYPOINT,
    obstacle: {
      size: 0.5
    },
    waypoint: {
      targetId: -1,
      gate: false,
      gate_width: 3,
      search: true
    }
  },

  lcmConnections: {
    lcmBridge: false,
    localization: false,
    nav: false,
    perception: false
  },

  odomFormat: OdomFormat.DM,

  path: [],

  simSettings: {
    simulateLoc: true,
    simulatePercep: true
  },

  startLoc: {
    latitude_deg: 38,
    latitude_min: 24.38,
    longitude_deg: -110,
    longitude_min: -47.51,
    bearing_deg: 0,
    speed: -1
  }
};


const getters = {
  arTagDrawOptions: (simState:SimulatorState):ArTagDrawOptions => simState.drawOptions.arTag,

  autonOn: (simState:SimulatorState):boolean => simState.autonOn,

  drawMode: (simState:SimulatorState):FieldItemType => simState.drawOptions.mode,

  fieldOfViewOptions: (simState:SimulatorState):FieldOfViewOptions => simState.debugOptions.
      fieldOfView,

  gateDrawOptions: (simState:SimulatorState):GateDrawOptions => simState.drawOptions.gate,

  lcmConnected: (simState:SimulatorState):boolean => simState.lcmConnections.lcmBridge,

  locConnected: (simState:SimulatorState):boolean => simState.lcmConnections.localization,

  navConnected: (simState:SimulatorState):boolean => simState.lcmConnections.nav,

  obstacleDrawOptions: (simState:SimulatorState):ObstacleDrawOptions => simState.drawOptions.
      obstacle,

  odomFormat: (simState:SimulatorState):OdomFormat => simState.odomFormat,

  paused: (simState:SimulatorState):boolean => simState.debugOptions.paused,

  percepConnected: (simState:SimulatorState):boolean => simState.lcmConnections.perception,

  roverPath: (simState:SimulatorState):Odom[] => simState.path,

  roverPathVisible: (simState:SimulatorState):boolean => simState.debugOptions.roverPathVisible,

  simulateLoc: (simState:SimulatorState):boolean => simState.simSettings.simulateLoc,

  simulatePercep: (simState:SimulatorState):boolean => simState.simSettings.simulatePercep,

  startLoc: (simState:SimulatorState):Odom => simState.startLoc,

  takeStep: (simState:SimulatorState):boolean => simState.debugOptions.takeStep,

  waypointDrawOptions: (simState:SimulatorState):WaypointDrawOptions => simState.drawOptions.
      waypoint
};


const mutations = {
  clearRoverPath: (simState:SimulatorState):void => {
    simState.path = [];
  },

  flipLcmConnected: (simState:SimulatorState, onOff:boolean):void => {
    simState.lcmConnections.lcmBridge = onOff;
  },

  flipLocConnected: (simState:SimulatorState, onOff:boolean):void => {
    simState.lcmConnections.localization = onOff;
  },

  flipNavConnected: (simState:SimulatorState, onOff:boolean):void => {
    simState.lcmConnections.nav = onOff;
  },

  flipPercepConnected: (simState:SimulatorState, onOff:boolean):void => {
    simState.lcmConnections.perception = onOff;
  },

  flipSimulateLoc: (simState:SimulatorState, onOff:boolean):void => {
    simState.simSettings.simulateLoc = onOff;
  },

  flipSimulatePercep: (simState:SimulatorState, onOff:boolean):void => {
    simState.simSettings.simulatePercep = onOff;
  },

  pushToRoverPath: (simState:SimulatorState, currLoc:Odom):void => {
    if (simState.path.length === 0) {
      simState.path.push(JSON.parse(JSON.stringify(currLoc)));
      return;
    }

    const prevLoc:Odom = simState.path[simState.path.length - 1];
    if (Math.abs(prevLoc.bearing_deg - currLoc.bearing_deg) > BEARING_DIFF_THRESHOLD ||
        prevLoc.latitude_deg !== currLoc.latitude_deg ||
        Math.abs(prevLoc.latitude_min - currLoc.latitude_min) > MINUTE_DIFF_THRESHOLD ||
        prevLoc.longitude_deg !== currLoc.longitude_deg ||
        Math.abs(prevLoc.longitude_min - currLoc.longitude_min) > MINUTE_DIFF_THRESHOLD) {
      simState.path.push(JSON.parse(JSON.stringify(currLoc)));
    }
  },

  setArTagDrawOptions: (simState:SimulatorState, options:ArTagDrawOptions):void => {
    Object.assign(simState.drawOptions.arTag, options);
  },

  setAutonState: (a:SimulatorState, onOff:boolean):void => {
    a.autonOn = onOff;
  },

  setDrawMode: (simState:SimulatorState, mode:FieldItemType):void => {
    simState.drawOptions.mode = mode;
  },

  setFieldOfViewOptions: (simState:SimulatorState, options:FieldOfViewOptions):void => {
    Object.assign(simState.debugOptions.fieldOfView, options);
  },

  setGateDrawOptions: (simState:SimulatorState, options:GateDrawOptions):void => {
    Object.assign(simState.drawOptions.gate, options);
  },

  setObstacleDrawOptions: (simState:SimulatorState, options:ObstacleDrawOptions):void => {
    Object.assign(simState.drawOptions.obstacle, options);
  },

  setOdomFormat: (simState:SimulatorState, format:OdomFormat):void => {
    simState.odomFormat = format;
  },

  setPaused: (simState:SimulatorState, paused:boolean):void => {
    simState.debugOptions.paused = paused;
  },

  setRoverPathVisible: (simState:SimulatorState, onOff:boolean):void => {
    simState.debugOptions.roverPathVisible = onOff;
  },

  setStartLoc: (simState:SimulatorState, newStartLoc:Odom):void => {
    Object.assign(simState.startLoc, newStartLoc);
  },

  setTakeStep: (simState:SimulatorState, takeStep:boolean):void => {
    simState.debugOptions.takeStep = takeStep;
  },

  setWaypointDrawOptions: (simState:SimulatorState, options:WaypointDrawOptions):void => {
    Object.assign(simState.drawOptions.waypoint, options);
  }
};


export default {
  state,
  getters,
  mutations
};

/**************************************************************************************************
 * Private Helper Functions
 **************************************************************************************************/
// /* Performs a list push operation specific to paths. This is the same as a
//    normal push operation except that it will not push duplicate
// function pathPush()
