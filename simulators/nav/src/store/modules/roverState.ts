/* This file contains the state variables, getters, and mutators for the
   simulated rover. */

import {
  Joystick,
  NavStatus,
  ObstacleMessage,
  Odom,
  RoverState,
  Speeds,
  TargetListMessage
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
    detected: false, /* this will be deprecated so don't use */
    distance: -1,
    bearing: 0
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
  ]
};


const getters = {
  currOdom: (roverState:RoverState):Odom => roverState.currOdom,

  joystick: (roverState:RoverState):Joystick => roverState.joystick,

  currSpeed: (roverState:RoverState):Speeds => roverState.currSpeed,

  navStatus: (roverState:RoverState):NavStatus => roverState.navStatus,

  obstacleMessage: (roverState:RoverState):ObstacleMessage => roverState.obstacleMessage,

  radioStrength: (roverState:RoverState):number => roverState.radioSignalStrength,

  targetList: (roverState:RoverState):TargetListMessage => roverState.targetList
};


const mutations = {
  setCurrOdom: (roverState:RoverState, newOdom:Odom):void => {
    Object.assign(roverState.currOdom, newOdom);
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
  }
};


export default {
  state,
  getters,
  mutations
};
