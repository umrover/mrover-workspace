/* This file contains utility functions for helping create noisy data. */

import Z from 'random-z';
import {
  Odom,
  OdomNoiseSettings
} from './types';
import {
  metersToOdom
} from './utils';

/**************************************************************************************************
 * Public Utility Functions
 **************************************************************************************************/
/* Create a noisy version of currOdom by adding normally distributed noise. */
export function createNoisyOdom(
    currOdom:Odom,
    odomNoise:OdomNoiseSettings
):Odom {
  /* add lat/lon noise */
  const noisyOdom:Odom = metersToOdom({
    x: Z() * odomNoise.latLonStdev,
    y: Z() * odomNoise.latLonStdev
  }, currOdom);

  /* add heading noise */
  noisyOdom.bearing_deg += Z() * odomNoise.headingStdev;

  return noisyOdom;
} /* createNoisyOdom() */

/* */
export function func(currOdom:Odom):void {
  console.log(currOdom);
}
