// initial state
const state = {
  route: [],
  waypointList: [],
  autonEnabled: false,
  odomFormat: "DM",
  clickPoint: {
    lat: 0,
    lon: 0
  },

  playbackEnabled: false,
  playbackOdomLat: [],
  playbackOdomLon: [],
  playbackOdomBearing: [],
  playbackGpsBearing: [],
  playbackTargetBearing: []
}

// getters
const getters = {
  route: state => state.route,
  waypointList: state => state.waypointList,
  autonEnabled: state => state.autonEnabled,
  odomFormat: state => state.odomFormat,
  clickPoint: state => state.clickPoint,
  playbackEnabled: state => state.playbackEnabled,
  playback: state => state.playback,
  playbackOdomLat: state => state.playbackOdomLat,
  playbackOdomLon: state => state.playbackOdomLon,
  playbackOdomBearing: state => state.playbackOdomBearing,
  playbackGpsBearing: state => state.playbackGpsBearing,
  playbackTargetBearing: state => state.playbackTargetBearing,
  playbackLength: state => state.playbackOdomLat.length
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

  setClickPoint (commit, newClickPoint) {
    state.clickPoint = newClickPoint
  },

  setPlaybackEnabled (commit, newPlaybackEnabled) {
    state.playbackEnabled = newPlaybackEnabled
  },

  setPlaybackOdomLat (commit, lat) {
    state.playbackOdomLat = lat
  },

  setPlaybackOdomLon (commit, lon) {
    state.playbackOdomLon = lon
  },

  setPlaybackOdomBearing (commit, bearing) {
    state.playbackOdomBearing = bearing
  },

  setPlaybackGpsBearing (commit, bearing) {
    state.playbackGpsBearing = bearing
  },

  setPlaybackTargetBearing (commit, bearing) {
    state.playbackTargetBearing = bearing
  }
}

export default {
  namespaced: true,
  state,
  getters,
  mutations
}
