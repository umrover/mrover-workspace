"""
This file contains the objects used in Nav's odometry filter.
"""


class NavState:
    """Class for nav states. Must match Nav's states exactly."""
    Off = "Off"
    Done = "Done"
    Turn = "Turn"
    Drive = "Drive"
    SearchFaceNorth = "Search Face North"
    SearchSpin = "Search Spin"
    SearchSpinWait = "Search Spin Wait"
    ChangeSearchAlg = "Change Search Algorithm"
    SearchTurn = "Search Turn"
    SearchDrive = "Search Drive"
    TurnToBall = "Turn to Ball"
    DriveToBall = "Drive to Ball"
    TurnAroundObs = "Turn Around Obstacle"
    DriveAroundObs = "Drive Around Obstacle"
    SearchTurnAroundObs = "Search Turn Around Obstacle"
    SearchDriveAroundObs = "Search Drive Around Obstacle"
    Unknown = "Unknown"

    StationaryStates = [Off, Done, SearchSpinWait, ChangeSearchAlg]
    RotationalStates = [Turn, SearchFaceNorth, SearchSpin,
                        SearchTurn, TurnToBall, TurnAroundObs,
                        SearchTurnAroundObs]
    TranslationalStates = [Drive, SearchDrive, DriveToBall,
                           DriveAroundObs, SearchDriveAroundObs]


class Odom:
    """Class for clean odometry data."""
    def __init__(self):
        """Initialize Odom data to None."""
        self._lat_deg = None
        self._lat_min = None
        self._long_deg = None
        self._long_min = None
        self._bearing = None
        self._speed = None

    def __str__(self):
        """String representation of an Odom."""
        odom = "Odom: lat_deg = {}\n".format(self._lat_deg)
        odom += "      lat_min = {}\n".format(self._lat_min)
        odom += "      long_deg = {}\n".format(self._long_deg)
        odom += "      long_min = {}\n".format(self._long_min)
        odom += "      bearing = {}\n".format(self._bearing)
        return odom

    def __repr__(self):
        """String representation of an Odom."""
        return str(self)

    def update_location(self, locEstimate):
        """Update location data."""
        if locEstimate is not None:
            self._lat_deg = locEstimate._lat_deg
            self._lat_min = locEstimate._lat_min
            self._long_deg = locEstimate._long_deg
            self._long_min = locEstimate._long_min

    def update_bearing(self, bearingEstimate):
        """Update bearing data."""
        if bearingEstimate is not None:
            self._bearing = bearingEstimate._bearing


class LocationEstimate:
    """Class for storing data on a location estimate."""

    def __init__(self, lat_deg=None, lat_min=None, long_deg=None,
                 long_min=None, weight=None):
        """Initialize location estimate."""
        self._lat_deg = lat_deg
        self._lat_min = lat_min
        self._long_deg = long_deg
        self._long_min = long_min
        self._weight = weight

    def __str__(self):
        """String representation of a LocationEstimate."""
        loc = "Location Estimate: lat_deg = {}\n".format(self._lat_deg)
        loc += "                   lat_min = {}\n".format(self._lat_min)
        loc += "                   long_deg = {}\n".format(self._long_deg)
        loc += "                   long_min = {}\n".format(self._long_min)
        loc += "                   weight = {}\n".format(self._weight)
        return loc

    def __repr__(self):
        """String representation of a LocationEstimate."""
        return str(self)

    @classmethod
    def from_raw_gps(cls, raw_gps, weight):
        """Convert a raw gps object into a LocationEstimate."""
        if raw_gps is None:
            return None
        return cls(raw_gps._lat_deg,
                   raw_gps._lat_min,
                   raw_gps._long_deg,
                   raw_gps._long_min,
                   weight)

    @classmethod
    def from_sensor_package(cls, sensor_package, weight):
        """Convert a raw sensor package object into a LocationEstimate."""
        if sensor_package is None:
            return None
        return cls(sensor_package._lat_deg,
                   sensor_package._lat_min,
                   sensor_package._long_deg,
                   sensor_package._long_min,
                   weight)

    def update_weight(self, weight):
        """Update the weight of the estimate."""
        self._weight = weight

    def check_validity(self):
        """Check if this is a usable location estimate."""
        if self._lat_deg is None:
            return None
        if self._lat_min is None:
            return None
        if self._long_deg is None:
            return None
        if self._long_min is None:
            return None
        if self._weight is None or self._weight == 0:
            return None
        return self

    def derive_location(self):
        """Fix location data after calculations."""
        sign_lat = 1 if self._lat_deg > 0 else -1
        sign_long = 1 if self._long_deg > 0 else -1

        self._lat_deg = abs(self._lat_deg)
        self._lat_min = abs(self._lat_min)
        self._long_deg = abs(self._long_deg)
        self._long_min = abs(self._long_min)

        self._lat_deg += self._lat_min // 60
        self._lat_min %= 60
        self._long_deg += self._long_min // 60
        self._long_min %= 60

        self._lat_deg *= sign_lat
        self._lat_min *= sign_lat
        self._long_deg *= sign_long
        self._long_min *= sign_long


class BearingEstimate:
    """Class for storing data on a bearing estimate."""

    def __init__(self, bearing, weight=None):
        """Initialize bearing estimate."""
        self._bearing = bearing
        self._weight = weight

    def __str__(self):
        """String representation of a BearingEstimate."""
        bear = "Bearing Estimate: bearing = {}\n".format(self._bearing)
        bear += "                  weight = {}\n".format(self._weight)
        return bear

    def __repr__(self):
        """String representation of a BearingEstimate."""
        return str(self)

    def update_weight(self, weight):
        """Update the weight of the estimate."""
        self._weight = weight

    def check_validity(self):
        """Check if this is a usable bearing estimate."""
        if self._bearing is None:
            return None
        if self._weight is None or self._weight == 0:
            return None
        return self


class Acceleration:
    """
    Class containing accelerations broken into the x, y, and z components.
    """
    def __init__(self, north, east, z):
        self.north = north
        self.east = east
        self.z = z


class Velocity:
    """Class containing velocities broken into the x, y, and z components."""
    def __init__(self, north, east, z):
        self.north = north
        self.east = east
        self.z = z
