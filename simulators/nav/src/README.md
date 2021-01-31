# <img src="static/mrover.png" alt="MRover Logo" width="30"/> MRover Navigation Simulator

## Table of Contents
[Directory Overview](#directory-overview)<br/>
[Code Overview](#code-overview)<br/>
[Related Documents](#related-documents)<br/>

---

<!---------------------------- Directory Overview ---------------------------->
## Directory Overview
The `src` directory contains all the source code for the project.

---

<!------------------------------- Code Overview ------------------------------>
## Code Overview
This sections gives descriptions of the various directories and files in the `src` directory. This continues the code overview started in the the [main `README`](../README.md).

#### `components`
The `components` directory contains all of the Vue components that make up the simulator as well as the back-end logic that specifically makes each of those components work. For more information on the code in this directory, see [`components/README.md`](./components/README.md).

#### `router`
The `router` directory contains logic which directs users to various pages within the application. Currently, the application only has one page so users are automatically directed there on the page load.

#### `static`
The `static` directory contains static files (e.g. images) such as the MRover logo.

#### `store`
The `store` directory contains logic for maintaining the internal state of the simulator. The simulator state is broken up into 3 modules (this is not necessary but was done to keep a single store file from becoming too large).

* `fieldState.ts` manages state related to the simulator field and the items on the simulator field (e.g. waypoints, obstacles, etc.). This can loosely be thought of as data that make up the rover's environment.
* `roverState.ts` manages state related to the rover (e.g. location, perception data, etc.). This can loosely be thought of as data that the real rover would keep track of.
* `simulatorState.ts` manages state related specifically to the simulator (e.g. debugging tools, draw mode options, etc.). This can loosely be thought of as data that would not exist outside of the simulator.

There are some things that could potentially go in more than one of these modules (e.g. `radioSignalStrength`). If you find yourself adding a piece of state like this, it is important to be consistent with what has already been done (i.e. two or more pieces of state that you would expect to find together, should be together). Beyond that, use your best judgement to place state in the module that makes the most sense (or if this project changes dramatically, it may be useful to add an entirely new module).

#### `utils`
The `utils` directory contains code that is reused throughout the project. This mainly includes constants, type definitions, and utility functions (i.e. functions used in more than one component).

#### `.eslint.js`
`.eslint.js` specifies the linter configurations.

#### `app.ts`
`app.ts` creates the Vue application to inject into the `#app` element in `index.html`.

#### `App.vue`
`App.vue` is the root component for the entire project. It contains a single `NavSimulator` component. It is likely true that you can skip over this component and go right to `NavSimulator` when diving into this project however it is good to know that this component exists as it truly is what `app.ts` injects into `index.html`.

#### `index.html`
`index.html` is the starting point of this project that gets rendered on the webpage. While it doesn't look like much, this is where the entire Vue applicated gets injected into.

---

<!----------------------------- Related Documents ---------------------------->
## Related Documents
* [project `README`](../README.md)
* [`components` `README`](./components/README.md)
