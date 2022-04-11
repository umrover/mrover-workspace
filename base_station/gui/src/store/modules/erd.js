// initial state
const state = {
  waypointList: [],
  odomFormat: "DM",
  clickPoint: {
    lat: 0,
    lon: 0
  },
}

// getters
const getters = {
  waypointList: state => state.waypointList,
  odomFormat: state => state.odomFormat,
  clickPoint: state => state.clickPoint
}

// mutations
const mutations = {
  setWaypointList (commit, newList) {
    state.waypointList = newList
  },

  setOdomFormat (commit, newOdomFormat) {
    state.odomFormat = newOdomFormat
  },

  setClickPoint (commit, newClickPoint) {
    state.clickPoint = newClickPoint
  }
}

export default {
  namespaced: true,
  state,
  getters,
  mutations
}
