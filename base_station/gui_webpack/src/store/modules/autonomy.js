import L from '../../leafletRover.js'

// initial state
const state = {
  route: [],
  autonEnabled: false
}

// getters
const getters = {
  route: state => state.route
}

// mutations
const mutations = {
  setRoute (commit, newRoute) {
    state.route = newRoute
  },

  setAutonMode (commit, newAutonEnabled) {
    state.autonEnabled = newAutonEnabled
  }
}

export default {
  namespaced: true,
  state,
  getters,
  mutations
}
