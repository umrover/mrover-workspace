/* This file contains the OpenIntervalHeap class which handles the logic for
   finding open intervals in a given range. This is used for tasks such as
   finding an open area to drive through in obstacle detection. */

/* Type representing an interval. Interval is inclusive.
   Interval[0] must be <= Interval[1] */
export type Interval = [number, number];


/* Comparator for Intervals. Smaller start values are less, breaking ties with
   smaller end values. */
function intervalCompare(interval0:Interval, interval1:Interval):number {
  if (interval0[0] === interval1[0]) {
    return interval0[1] - interval1[1];
  }
  return interval0[0] - interval1[0];
} /* intervalCompare() */


/* Type representing [size of interval, interval] pair. */
type SizedInterval = [number, Interval];


/* Comporator for SizedIntervals. Smaller intervals are less, breaking ties with
   intervalCompare. */
function sizedIntervalCompare(interval0:SizedInterval, interval1:SizedInterval):number {
  if (interval0[0] === interval1[0]) {
    return intervalCompare(interval0[1], interval1[1]);
  }
  return interval0[0] - interval1[0];
} /* sizedIntervalCompare() */


/* Class for finding open intervals in a given range that has certain intervals
   that are occupied. */
export class OpenIntervalHeap {
  /************************************************************************************************
   * Public Members
   ************************************************************************************************/
  maxOccupied!:number; /* Minimum occupied value */
  minOccupied!:number; /* Maximum occupied value */

  /************************************************************************************************
   * Private Members
   ************************************************************************************************/
  /* Occupied intervals */
  private closedIntervals:Interval[];

  /* End of the global range */
  private end!:number;

  /* Unoccupied intervals (with their corresponding sizes first) */
  private openIntervals:SizedInterval[] = [];

  /* Start of the global range */
  private start!:number;

  /* Initialize ObstacleDetector. */
  constructor(
      start:number,
      end:number,
      closedIntervals:Interval[]
  ) {
    this.start = start;
    this.end = end;
    this.closedIntervals = closedIntervals.sort(intervalCompare);

    /* combine intervals */
    const intervalsToRemove:number[] = [];
    for (let index = 0; index < this.closedIntervals.length - 1; index += 1) {
      /* completely disjoint intervals */
      if (this.closedIntervals[index][1] < this.closedIntervals[index + 1][0]) {
        continue;
      }

      /* completely encompassing intervals */
      else if (this.closedIntervals[index][1] >= this.closedIntervals[index + 1][1]) {
        intervalsToRemove.push(index);
      }

      /* overlapping intervals */
      else {
        this.closedIntervals[index + 1][0] = this.closedIntervals[index][0];
        intervalsToRemove.push(index);
      }
    }

    /* remove intervals accounted for in other intervals */
    intervalsToRemove.reverse().forEach((index) => {
      this.closedIntervals.splice(index, 1);
    });

    /* If any closed intervals */
    if (this.closedIntervals.length) {
      /* trim intervals with start and end */
      while (this.start > this.closedIntervals[0][1]) {
        this.closedIntervals.splice(0, 1);
      }
      if (this.start > this.closedIntervals[0][0]) {
        this.closedIntervals[0][0] = this.start;
      }
      let lastIndex:number = this.closedIntervals.length - 1;
      while (this.end < this.closedIntervals[lastIndex][0]) {
        this.closedIntervals.splice(lastIndex, 1);
        lastIndex -= 1;
      }
      if (this.end < this.closedIntervals[lastIndex][1]) {
        this.closedIntervals[lastIndex][1] = this.end;
      }

      /* find min/max occupied valeus */
      this.minOccupied = this.closedIntervals[0][0];
      this.maxOccupied = this.closedIntervals[lastIndex][1];

      /* build open interval tree */
      if (this.start < this.closedIntervals[0][0]) {
        const size:number = this.closedIntervals[0][0] - this.start;
        const closedRange:Interval = [this.start, this.closedIntervals[0][0]];
        this.openIntervals.push([size, closedRange]);
      }
      for (let index = 0; index < this.closedIntervals.length - 1; index += 1) {
        const size:number = this.closedIntervals[index + 1][0] - this.closedIntervals[index][1];
        const closedRange:Interval = [
          this.closedIntervals[index][1],
          this.closedIntervals[index + 1][0]
        ];
        this.openIntervals.push([size, closedRange]);
      }
      if (this.end > this.closedIntervals[lastIndex][1]) {
        const size:number = this.end - this.closedIntervals[lastIndex][1];
        const closedRange:Interval = [this.closedIntervals[lastIndex][1], this.end];
        this.openIntervals.push([size, closedRange]);
      }
      this.openIntervals.sort(sizedIntervalCompare).reverse();
    }
    else {
      this.minOccupied = end;
      this.maxOccupied = start;
      this.openIntervals.push([start - end, [start, end]]);
    }
  } /* constructor() */

  /* Get largest open interval and remove it from the heap. If no intervals
     left, return null. */
  getNextOpenInterval():Interval|null {
    if (!this.openIntervals.length) {
      return null;
    }

    const openInterval:Interval = this.openIntervals[0][1];
    this.openIntervals.splice(0, 1);
    return openInterval;
  } /* getNextOpenInterval() */
}
