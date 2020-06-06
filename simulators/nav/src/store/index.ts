/* This file contains the store for the application which manages the
   application state. This includes all the state managed by the various state
   modules (e.g. fieldState, etc.). */

import Vue from 'vue';
import Vuex from 'vuex';
import fieldState from './modules/fieldState';
import roverState from './modules/roverState';
import simulatorState from './modules/simulatorState';


Vue.use(Vuex);

export default new Vuex.Store({
  modules: {
    fieldState,
    roverState,
    simulatorState
  }
});
