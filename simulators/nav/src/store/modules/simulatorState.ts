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
    auton: false,
    lcmBridge: false,
    localization: false,
    perception: false
  },

  odomFormat: OdomFormat.DM,

  simSettings: {
    simulateLocalization: true,
    simulatePerception: true
  }
};


const getters = {
  arTagDrawOptions: (simState:SimulatorState):ArTagDrawOptions => simState.drawOptions.arTag,

  autonConnected: (simState:SimulatorState):boolean => simState.lcmConnections.auton,

  autonOn: (simState:SimulatorState):boolean => simState.autonOn,

  drawMode: (simState:SimulatorState):FieldItemType => simState.drawOptions.mode,

  fieldOfViewOptions: (simState:SimulatorState):FieldOfViewOptions => simState.debugOptions.
      fieldOfView,

  gateDrawOptions: (simState:SimulatorState):GateDrawOptions => simState.drawOptions.gate,

  lcmConnected: (simState:SimulatorState):boolean => simState.lcmConnections.lcmBridge,

  localizationConnected: (simState:SimulatorState):boolean => simState.lcmConnections.localization,

  obstacleDrawOptions: (simState:SimulatorState):ObstacleDrawOptions => simState.drawOptions.
      obstacle,

  odomFormat: (simState:SimulatorState):OdomFormat => simState.odomFormat,

  paused: (simState:SimulatorState):boolean => simState.debugOptions.paused,

  perceptionConnected: (simState:SimulatorState):boolean => simState.lcmConnections.perception,

  simulateLocalization: (simState:SimulatorState):boolean => simState.simSettings.
      simulateLocalization,

  simulatePerception: (simState:SimulatorState):boolean => simState.simSettings.simulatePerception,

  takeStep: (simState:SimulatorState):boolean => simState.debugOptions.takeStep,

  waypointDrawOptions: (simState:SimulatorState):WaypointDrawOptions => simState.drawOptions.
      waypoint
};


const mutations = {
  flipAutonConnected: (simState:SimulatorState, onOff:boolean):void => {
    simState.lcmConnections.auton = onOff;
  },

  flipLcmConnected: (simState:SimulatorState, onOff:boolean):void => {
    simState.lcmConnections.lcmBridge = onOff;
  },

  flipLocalizationConnected: (simState:SimulatorState, onOff:boolean):void => {
    simState.lcmConnections.localization = onOff;
  },

  flipPerceptionConnected: (simState:SimulatorState, onOff:boolean):void => {
    simState.lcmConnections.perception = onOff;
  },

  flipSimulateLocalization: (simState:SimulatorState, onOff:boolean):void => {
    simState.simSettings.simulateLocalization = onOff;
  },

  flipSimulatePerception: (simState:SimulatorState, onOff:boolean):void => {
    simState.simSettings.simulatePerception = onOff;
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
