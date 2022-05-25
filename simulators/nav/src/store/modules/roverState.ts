/* This file contains the state variables, getters, and mutators for the
   simulated rover. */

import {
  Joystick,
  NavStatus,
  ObstacleMessage,
  Odom,
  RoverState,
  Speeds,
  TargetListMessage,
  ProjectedPointsMessage,
  ZedGimbalPosition
} from '../../utils/types';


const state:RoverState = {
  currOdom: {
    latitude_deg: 38,
    latitude_min: 24.38,
    longitude_deg: -110,
    longitude_min: -47.51,
    bearing_deg: 0,
    speed: -1
  },

  realOdom: {
    latitude_deg: 38,
    latitude_min: 24.38,
    longitude_deg: -110,
    longitude_min: -47.51,
    bearing_deg: 0,
    speed: -1
  },

  currSpeed: {
    drive: 2,
    turn: 30
  },

  joystick: {
    forward_back: 0,
    left_right: 0
  },

  navStatus: {
    nav_state_name: 'Unknown',
    completed_wps: 0,
    total_wps: 0
  },

  obstacleMessage: {
    numObstacles: -1,
    obstacles: []
  },

  projectedPointsMessage: {
    pattern_size: 0,
    points: [],
    path_type: 'NA'
  },

  radioSignalStrength: 100,

  targetList: [
    {
      distance: -1,
      bearing: 0,
      id: -1
    },
    {
      distance: -1,
      bearing: 0,
      id: -1
    }
  ],

  zedGimbalCmd: {
    angle: 0
  },

  zedGimbalPos: {
    angle: 0
  }
};


const getters = {
  currOdom: (roverState:RoverState):Odom => roverState.currOdom,

  realOdom: (roverState:RoverState):Odom => roverState.realOdom,

  joystick: (roverState:RoverState):Joystick => roverState.joystick,

  currSpeed: (roverState:RoverState):Speeds => roverState.currSpeed,

  navStatus: (roverState:RoverState):NavStatus => roverState.navStatus,

  obstacleMessage: (roverState:RoverState):ObstacleMessage => roverState.obstacleMessage,

  projectedPointsMessage:
  (roverState:RoverState):ProjectedPointsMessage => roverState.projectedPointsMessage,

  radioStrength: (roverState:RoverState):number => roverState.radioSignalStrength,

  targetList: (roverState:RoverState):TargetListMessage => roverState.targetList,

  zedGimbalCmd: (roverState:RoverState):ZedGimbalPosition => roverState.zedGimbalCmd,

  zedGimbalPos: (roverState:RoverState):ZedGimbalPosition => roverState.zedGimbalPos
};


const mutations = {
  setCurrOdom: (roverState:RoverState, newOdom:Odom):void => {
    Object.assign(roverState.currOdom, newOdom);
  },

  setRealOdom: (roverState:RoverState, newOdom:Odom):void => {
    Object.assign(roverState.realOdom, newOdom);
  },

  setCurrSpeed: (roverState:RoverState, newSpeeds:Speeds):void => {
    Object.assign(roverState.currSpeed, newSpeeds);
  },

  setJoystick: (roverState:RoverState, newJoystick:Joystick):void => {
    Object.assign(roverState.joystick, newJoystick);
  },

  setNavStatus: (roverState:RoverState, newNavStatus:NavStatus):void => {
    Object.assign(roverState.navStatus, newNavStatus);
  },

  setObstacleMessage: (roverState:RoverState, newObstacle:ObstacleMessage):void => {
    Object.assign(roverState.obstacleMessage, newObstacle);
  },

  setRadioStrength: (roverState:RoverState, strength:number):void => {
    roverState.radioSignalStrength = strength;
  },

  setTargetList: (roverState:RoverState, newTargetList:TargetListMessage):void => {
    roverState.targetList = newTargetList;
  },

  setZedGimbalCmd: (roverState:RoverState, newZedGimbalCmd:ZedGimbalPosition):void => {
    Object.assign(roverState.zedGimbalCmd, newZedGimbalCmd);
  },

  setZedGimbalPos: (roverState:RoverState, newZedGimbalPos:ZedGimbalPosition):void => {
    Object.assign(roverState.zedGimbalPos, newZedGimbalPos);
  },

  setProjectedPointsMessage: (roverState:RoverState,
      newProjectedPointsMessage:ProjectedPointsMessage):void => {
    Object.assign(roverState.projectedPointsMessage, newProjectedPointsMessage);
  }
};


export default {
  state,
  getters,
  mutations
};
