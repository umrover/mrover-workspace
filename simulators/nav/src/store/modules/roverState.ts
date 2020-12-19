/* This file contains the state variables, getters, and mutators for the
   simulated rover. */

import {
  createNoisyObs,
  createNoisyOdom,
  createNoisyTargetList
} from '../../utils/noise_utils';
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

  currOdomNoisy: {
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

  noiseSetttings: {
    locNoise: {
      headingStdev: 5,
      latLonStdev: 1
    },
    percepNoise: {
      obsFalsePos: 0.1,
      obsFalseNeg: 0.1,
      tagFalsePos: 0.1,
      tagFalseNeg: 0.1,
      tagIdFalses: 0.1,
      bearStddev: 5,
      distStddev: 0.5
    }
  },

  obstacleMessage: {
    distance: -1,
    bearing: 0
  },

  obstacleMessageNoisy: {
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
  ],

  targetListNoisy: [
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

  currOdomNoisy: (roverState:RoverState):Odom => roverState.currOdomNoisy,

  joystick: (roverState:RoverState):Joystick => roverState.joystick,

  currSpeed: (roverState:RoverState):Speeds => roverState.currSpeed,

  navStatus: (roverState:RoverState):NavStatus => roverState.navStatus,

  obstacleMessage: (roverState:RoverState):ObstacleMessage => roverState.obstacleMessage,

  obstacleMessageNoisy: (roverState:RoverState):ObstacleMessage => roverState.obstacleMessageNoisy,

  radioStrength: (roverState:RoverState):number => roverState.radioSignalStrength,

  targetList: (roverState:RoverState):TargetListMessage => roverState.targetList,

  noisyTargetList: (roverState:RoverState):TargetListMessage => roverState.targetListNoisy
};


const mutations = {
  setCurrOdom: (roverState:RoverState, newOdom:Odom):void => {
    Object.assign(roverState.currOdom, newOdom);
    roverState.currOdomNoisy = createNoisyOdom(newOdom, roverState.noiseSetttings.locNoise);
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
    roverState.obstacleMessageNoisy = createNoisyObs(newObstacle,
                                                     roverState.noiseSetttings.percepNoise);
  },

  setRadioStrength: (roverState:RoverState, strength:number):void => {
    roverState.radioSignalStrength = strength;
  },

  setTargetList: (roverState:RoverState, newTargetList:TargetListMessage):void => {
    roverState.targetList = newTargetList;
    roverState.targetListNoisy = createNoisyTargetList(newTargetList,
                                                       roverState.noiseSetttings.percepNoise);
  }
};


export default {
  state,
  getters,
  mutations
};
