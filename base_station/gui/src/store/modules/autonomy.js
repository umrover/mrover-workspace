// initial state
const state = {
  route: [],
  waypointList: [],
  autonEnabled: false,
  odomFormat: "DM",
}

// getters
const getters = {
  route: state => state.route,
  waypointList: state => state.waypointList,
  autonEnabled: state => state.autonEnabled,
  odomFormat: state => state.odomFormat
}

// mutations
const mutations = {
  setRoute (commit, newRoute) {
    state.route = newRoute
  },

  setAutonMode (commit, newAutonEnabled) {
    state.autonEnabled = newAutonEnabled
  },

  setWaypointList (commit, newList) {
    state.waypointList = newList
  },

  setOdomFormat (commit, newOdomFormat) {
    state.odomFormat = newOdomFormat
  }
}

export default {
  namespaced: true,
  state,
  getters,
  mutations
}
