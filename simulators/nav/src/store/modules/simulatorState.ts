/* This file contains the state variables, getters, and mutators for the
   simulator. */

import {
  ArTagDrawOptions,
  FieldItemType,
  FieldOfViewOptions,
  GateDrawOptions,
  ObstacleDrawOptions,
  OdomFormat,
  SimulatorState,
  WaypointDrawOptions
} from '../../utils/types';


const state:SimulatorState = {
  autonOn: false,

  debugOptions: {
    fieldOfView: {
      angle: 110,
      depth: 3,
      visible: true
    },
    paused: false,
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

  simSettings: {
    simulateLoc: true,
    simulatePercep: true
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

  simulateLoc: (simState:SimulatorState):boolean => simState.simSettings.simulateLoc,

  simulatePercep: (simState:SimulatorState):boolean => simState.simSettings.simulatePercep,

  takeStep: (simState:SimulatorState):boolean => simState.debugOptions.takeStep,

  waypointDrawOptions: (simState:SimulatorState):WaypointDrawOptions => simState.drawOptions.
      waypoint
};


const mutations = {
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
