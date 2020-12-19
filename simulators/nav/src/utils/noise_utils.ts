/* This file contains utility functions for helping create noisy data. */

import Z from 'random-z';
import store  from '../store';
import {
  Odom,
  LocNoiseSettings,
  ObstacleMessage,
  PercepNoiseSettings,
  TargetListMessage,
  TargetMessage
} from './types';
import {
  metersToOdom
} from './utils';

/**************************************************************************************************
 * Constants
 **************************************************************************************************/
/* Maximum AR tag ID number */
const MAX_TAG_ID = 249;

/**************************************************************************************************
 * Public Utility Functions
 **************************************************************************************************/
/* Create a noisy version of currObs by adding false positives/negatives. */
export function createNoisyObs(
    obsMsg:ObstacleMessage,
    percepNoise:PercepNoiseSettings
):ObstacleMessage {
  const noisyObsMsg:ObstacleMessage = { ...obsMsg };

  /* If an obstacle is in the path. */
  if (obsMsg.distance !== -1) {
    /* determine if this will be a false negative */
    if (Math.random() < percepNoise.obsFalseNeg) {
      noisyObsMsg.distance = -1;
      noisyObsMsg.bearing = 0;
    }

    /* otherwise add guassian noise */
    else {
      noisyObsMsg.bearing += Z() * percepNoise.bearStddev;
      noisyObsMsg.distance += Z() * percepNoise.distStddev;
    }
  }

  /* Otherwise there is no obstacle in the path and this will be a false
     positive */
  else if (Math.random() < percepNoise.obsFalsePos) {
    noisyObsMsg.bearing = getRandomFovAngle();
    noisyObsMsg.distance = getRandomFovDepth();
  }
  return noisyObsMsg;
} /* createNoisyObs() */


/* Create a noisy version of currOdom by adding normally distributed noise. */
export function createNoisyOdom(
    currOdom:Odom,
    locNoise:LocNoiseSettings
):Odom {
  /* add lat/lon noise */
  const noisyOdom:Odom = metersToOdom({
    x: Z() * locNoise.latLonStdev,
    y: Z() * locNoise.latLonStdev
  }, currOdom);

  /* add heading noise */
  noisyOdom.bearing_deg += Z() * locNoise.headingStdev;

  return noisyOdom;
} /* createNoisyOdom() */


/* Create a noisy version of currTargetList by adding false
   positives/negatives. */
export function createNoisyTargetList(
    currTargetList:TargetListMessage,
    percepNoise:PercepNoiseSettings
):TargetListMessage {
  const noisyTargetList:TargetListMessage = { ...currTargetList };

  /* Determine which will be false negatives. */
  const lenTargetList:number = Object.values(noisyTargetList).length;
  Object.values(noisyTargetList).reverse().some((target:TargetMessage, i:number) => {
    const index:number = lenTargetList - i - 1;

    /* If this is a true positive */
    if (target.distance !== -1) {
      /* Determine if this should be a false negative */
      if (Math.random() < percepNoise.tagFalseNeg) {
        noisyTargetList[index].bearing = 0;
        noisyTargetList[index].distance = -1;
        noisyTargetList[index].id = -1;
      }

      /* otherwise add noise */
      else {
        noisyTargetList[index].bearing += Z() * percepNoise.bearStddev;
        noisyTargetList[index].distance += Z() * percepNoise.distStddev;
        if (Math.random() < percepNoise.tagIdFalses) {
          noisyTargetList[index].id = getRandomTagId();
        }
      }
    }

    /* stop looping (return true) if this is a target */
    return noisyTargetList[index].distance !== -1;
  }, noisyTargetList);

  /* Determine which will be false positives. */
  Object.values(noisyTargetList).some((target:TargetMessage, index:number) => {
    /* If this is a true negative */
    if (target.distance === -1) {
      /* Determine if this should be a false positive */
      if (Math.random() < percepNoise.tagFalsePos) {
        noisyTargetList[index].bearing = getRandomFovAngle();
        noisyTargetList[index].distance = getRandomFovDepth();
        noisyTargetList[index].id = getRandomTagId();
      }
    }

    /* stop looping (return true) if this is a not a target */
    return noisyTargetList[index].distance === -1;
  }, noisyTargetList);

  return noisyTargetList;
} /* createNoisyTargetList() */


/**************************************************************************************************
 * Private Utility Functions
 **************************************************************************************************/
/* Get a uniformly random angle from the rover's field of view. */
function getRandomFovAngle():number {
  const fovAngle:number = store.getters.fieldOfViewOptions.angle;
  return (Math.random() * fovAngle) - (fovAngle / 2);
} /* getRandomFovAngle() */


/* Get a uniformly random depth from the rover's field of view. */
function getRandomFovDepth():number {
  const fovDepth:number = store.getters.fieldOfViewOptions.depth;
  return Math.random() * fovDepth;
} /* getRandomFovDepth() */

/* Get a uniformly random AR tag ID. */
function getRandomTagId():number {
  return Math.round(Math.random() * MAX_TAG_ID);
}
