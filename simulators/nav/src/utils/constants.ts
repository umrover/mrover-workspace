/* This file contains constants defined outside of this codebase (e.g. the size
   of the rover). */

import {
  PostConstants,
  RepeaterConstants,
  RoverConstants
} from './types';


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
