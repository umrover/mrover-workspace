/* This file contains constants defined outside of this codebase (e.g. the size
   of the rover). */

import {
  ColorScheme,
  ColorSchemeName,
  PerceptionConstants,
  PostConstants,
  RepeaterConstants,
  RoverConstants,
  ZedConstants
} from './types';


/* Button */
export const BUTTON_COLOR_SCHEMES:Record<ColorSchemeName, ColorScheme> = {
  Green: {
    backgroundColor: '#a0ffa0',
    backgroundColorHover: '#beffbe',
    borderColor: '#44cc44',
    fontColor: 'black'
  },
  GreenGray: {
    backgroundColor: '#a0ffa0',
    backgroundColorHover: '#beffbe',
    borderColor: '#444',
    fontColor: 'black'
  },
  Red: {
    backgroundColor: '#ff5a5a',
    backgroundColorHover: '#ff7878',
    borderColor: '#ff2828',
    fontColor: 'white'
  },
  RedWhite: {
    backgroundColor: '#ff5a5a',
    backgroundColorHover: '#ff7878',
    borderColor: 'white',
    fontColor: 'white'
  },
  Yellow: {
    backgroundColor: '#ffff99',
    backgroundColorHover: '#ffffb7',
    borderColor: '#ffff5d',
    fontColor: 'black'
  }
};


/* Define the perception system constants */
export const PERCEPTION:PerceptionConstants = {
  minVisibleDistTag: 0.25
};


/* Define the size of a post. */
export const POST:PostConstants = {
  numSides: 3,
  sideLen: 0.2
};


/* Define ranges of strong and weak radio signal strengths. */
export const RADIO:RepeaterConstants = {
  maxSignalStrength: 100,
  minSignalStrength: 0,
  lowSignalStrengthThreshold: 30
};


/* Define the size of the rover. */
export const ROVER:RoverConstants = {
  length: 1.25,
  width: 1.25
};

/* Define ZED and ZED gimbal constants. */
export const ZED:ZedConstants = {
  gimbal: {
    minAngle: -180,
    maxAngle: 180
  }
};
