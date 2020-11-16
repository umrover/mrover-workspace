/* This file creates the Vue application to inject into the #app element in
   index.html. */

import Vue from 'vue';
import App from './App.vue';
import router from './router';
import store from './store';

new Vue({
  el: '#app',
  router,
  store,
  components: { App },
  template: '<App/>'
});
