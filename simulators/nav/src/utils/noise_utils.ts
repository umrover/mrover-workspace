/* This file contains utility functions for helping create noisy data. */

import Z from 'random-z';
import {
  Odom,
  OdomNoiseSettings
} from './types';

/**************************************************************************************************
 * Public Utility Functions
 **************************************************************************************************/
/* */
export function createNoisyOdom(
    currOdom:Odom,
    odomNoise:OdomNoiseSettings
):Odom {
  const noisyOdom:Odom = { ...currOdom };
  console.log(odomNoise);
  noisyOdom.bearing_deg += Z() * odomNoise.headingStdev;
  return noisyOdom;
}

/* */
export function func(currOdom:Odom):void {
  console.log(currOdom);
}
