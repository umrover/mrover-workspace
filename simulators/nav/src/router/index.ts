/* This file contains the router for the application which directs users to
   various pages within the application. The application currently only has one
   page. */

import Vue from 'vue';
import Router from 'vue-router';
import NavSimulator from '../components/NavSimulator.vue';

Vue.use(Router);

export default new Router({
  routes: [
    {
      path: '/',
      name: 'Simulator',
      component: NavSimulator
    }
  ]
});
