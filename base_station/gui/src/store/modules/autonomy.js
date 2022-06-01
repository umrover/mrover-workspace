// initial state
const state = {
  route: [],
  waypointList: [],
  highlightedWaypoint: -1,
  autonEnabled: false,
  teleopEnabled: true,
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
  highlightedWaypoint: state => state.highlightedWaypoint,
  autonEnabled: state => state.autonEnabled,
  teleopEnabled: state => state.teleopEnabled,
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

  setTeleopMode (commit, newTeleopEnabled) {
    state.teleopEnabled = newTeleopEnabled
  },

  setWaypointList (commit, newList) {
    state.waypointList = newList
  },

  setHighlightedWaypoint (commit, newWaypoint) {
    state.highlightedWaypoint = newWaypoint
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
