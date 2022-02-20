/* This file contains constants defined outside of this codebase (e.g. the size
   of the rover). */

import {
  PerceptionConstants,
  PostConstants,
  RepeaterConstants,
  RoverConstants,
  ZedConstants
} from './types';


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

/* eslint no-magic-numbers: ["error", { "ignore": [-3.4,-1.6,
  -1.22,
  -0.97,
  -0.77,
  -0.61,
  -0.44,
  -0.3,
  -0.14,
  0,
  0.13,
  0.28,
  0.42,
  0.58,
  0.74,
  0.96,
  1.22,
  1.56,
  3.49] }] */

// cut positive - z scores need to change 
export const Zscores = [
  -3.49, // 0   index 0 = -1
  -1.28  // 0.1 index 1 = -0.9
  -0.84, // 0.2 index 2 = -0.8
  -0.52, // 0.3 index 3 = -0.7
  -0.25, // 0.4 index 4 = -0.6
  0 // 0.5 index 5
];


