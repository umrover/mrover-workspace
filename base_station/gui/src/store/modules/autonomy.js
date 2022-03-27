// initial state
const state = {
  route: [],
  waypointList: [],
  autonEnabled: false,
  odomFormat: "DM",

  playbackEnabled: false,
  playbackLat: [],
  playbackLon: [],
  playbackBearing: [],
  playbackGpsBearing: []
}

// getters
const getters = {
  route: state => state.route,
  waypointList: state => state.waypointList,
  autonEnabled: state => state.autonEnabled,
  odomFormat: state => state.odomFormat,
  playbackEnabled: state => state.playbackEnabled,
  playback: state => state.playback,
  playbackLat: state => state.playbackLat,
  playbackLon: state => state.playbackLon,
  playbackBearing: state => state.playbackBearing,
  playbackGpsBearing: state => state.playbackGpsBearing,
  playbackLength: state => state.playbackLat.length
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
  },

  setPlaybackEnabled (commit, newPlaybackEnabled) {
    state.playbackEnabled = newPlaybackEnabled
  },

  setPlaybackLat (commit, lat) {
    state.playbackLat = lat
  },

  setPlaybackLon (commit, lon) {
    state.playbackLon = lon
  },

  setPlaybackBearing (commit, bearing) {
    state.playbackBearing = bearing
  },

  setPlaybackGpsBearing (commit, bearing) {
    state.playbackGpsBearing = bearing
  }
}

export default {
  namespaced: true,
  state,
  getters,
  mutations
}
