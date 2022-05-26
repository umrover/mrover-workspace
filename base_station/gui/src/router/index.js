import Vue from 'vue'
import Router from 'vue-router'
import Menu from '../components/Menu.vue'
import ERDTask from '../components/ERDTask.vue'
import ESTask from '../components/ESTask.vue'
import AutonTask from '../components/AutonTask.vue'
import PidTune from '../components/PidTune.vue'
import LCMEcho from '../components/LCMEcho.vue'
import SATask from '../components/SATask.vue'
import ISHTask from '../components/ISHTask.vue'
import LCMSend from '../components/LCMSend.vue'

Vue.use(Router)

export default new Router({
  routes: [
    {
      path: '/',
      name: 'Menu',
      component: Menu
    },
    {
      path: '/ERDTask',
      name: 'ERDTask',
      component: ERDTask
    },
    {
      path: '/ESTask',
      name: 'ESTask',
      component: ESTask
    },
    {
      path: '/PidTune',
      name:'PidTune',
      component: PidTune
    },
    {
      path: '/LCMEcho',
      name: 'LCMEcho',
      component: LCMEcho
    },
    {
      path: '/SATask',
      name: 'SATask',
      component: SATask
    },
    {
      path: '/ISHTask',
      name: 'ISHTask',
      component: ISHTask
    },
    {
      path: '/AutonTask',
      name: 'AutonTask',
      component: AutonTask
    },
    {
      path: '/LCMSend',
      name: 'LCMSend',
      component: LCMSend
    }
  ]
})
