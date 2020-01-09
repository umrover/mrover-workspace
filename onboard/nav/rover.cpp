#include "rover.hpp"

#include "utilities.hpp"
#include "rover_msgs/Joystick.hpp"
#include <cmath>
#include <iostream>

// Constructs a rover status object and initializes the navigation
// state to off.
Rover::RoverStatus::RoverStatus()
    : mCurrentState( NavState::Off )
{
    mAutonState.is_auton = false;
} // RoverStatus()

// Gets a reference to the rover's current navigation state.
NavState& Rover::RoverStatus::currentState()
{
    return mCurrentState;
} // currentState()

// Gets a reference to the rover's current auton state.
AutonState& Rover::RoverStatus::autonState()
{
    return mAutonState;
} // autonState()

// Gets a reference to the rover's course.
Course& Rover::RoverStatus::course()
{
    return mCourse;
} // course()

// Gets a reference to the rover's path.
queue<Waypoint>& Rover::RoverStatus::path()
{
    return mPath;
} // path()

// Gets a reference to the rover's current obstacle information.
Obstacle& Rover::RoverStatus::obstacle()
{
    return mObstacle;
} // obstacle()

// Gets a reference to the rover's current odometry information.
Odometry& Rover::RoverStatus::odometry()
{
    return mOdometry;
} // odometry()

// Gets a reference to the rover's first target's current information.
Target& Rover::RoverStatus::target()
{
    return mTarget1;
} // target()

Target& Rover::RoverStatus::target2() {
    return mTarget2;
}

unsigned Rover::RoverStatus::getPathTargets()
{
  return mPathTargets;
} // getPathTargets()

// Assignment operator for the rover status object. Does a "deep" copy
// where necessary.
Rover::RoverStatus& Rover::RoverStatus::operator=( Rover::RoverStatus& newRoverStatus )
{
    mAutonState = newRoverStatus.autonState();
    mCourse = newRoverStatus.course();
    mPathTargets = 0;

    while( !mPath.empty() )
    {
        mPath.pop();
    }
    for( int courseIndex = 0; courseIndex < mCourse.num_waypoints; ++courseIndex )
    {
        auto &wp = mCourse.waypoints[ courseIndex ];
        mPath.push( wp );
        if (wp.search) {
            ++mPathTargets;
        }
    }
    mObstacle = newRoverStatus.obstacle();
    mOdometry = newRoverStatus.odometry();
    mTarget1 = newRoverStatus.target();
    mTarget2 = newRoverStatus.target2();
    return *this;
} // operator=

// Constructs a rover object with the given configuration file and lcm
// object with which to use for communications.
Rover::Rover( const rapidjson::Document& config, lcm::LCM& lcmObject )
    : mRoverConfig( config )
    , mLcmObject( lcmObject )
    , mDistancePid( config[ "distancePid" ][ "kP" ].GetDouble(),
                    config[ "distancePid" ][ "kI" ].GetDouble(),
                    config[ "distancePid" ][ "kD" ].GetDouble() )
    , mBearingPid( config[ "bearingPid" ][ "kP" ].GetDouble(),
                   config[ "bearingPid" ][ "kI" ].GetDouble(),
                   config[ "bearingPid" ][ "kD" ].GetDouble() )
    , mLongMeterInMinutes( -1 )
{
} // Rover()

// Sends a joystick command to drive forward from the current odometry
// to the destination odometry. This joystick command will also turn
// the rover small amounts as "course corrections".
// The return value indicates if the rover has arrived or if it is
// on-course or off-course.
DriveStatus Rover::drive( const Odometry& destination )
{
    double distance = estimateNoneuclid( mRoverStatus.odometry(), destination );
    double bearing = calcBearing( mRoverStatus.odometry(), destination );
    return drive( distance, bearing, false );
} // drive()

// Sends a joystick command to drive forward from the current odometry
// in the direction of bearing. The distance is used to determine how
// quickly to drive forward. This joystick command will also turn the
// rover small amounts as "course corrections". target indicates
// if the rover is driving to a target rather than a waypoint and
// determines which distance threshold to use.
// The return value indicates if the rover has arrived or if it is
// on-course or off-course.
DriveStatus Rover::drive( const double distance, const double bearing, const bool target )
{
    if( (!target && distance < mRoverConfig[ "navThresholds" ][ "waypointDistance" ].GetDouble()) ||
        (target && distance < mRoverConfig[ "navThresholds" ][ "targetDistance" ].GetDouble()) )
    {
        return DriveStatus::Arrived;
    }

    double destinationBearing = mod( bearing, 360 );
    throughZero( destinationBearing, mRoverStatus.odometry().bearing_deg ); // will go off course if inside if because through zero not calculated

    if( fabs( destinationBearing - mRoverStatus.odometry().bearing_deg ) < mRoverConfig[ "navThresholds" ][ "drivingBearing" ].GetDouble() )
    {
        double distanceEffort = mDistancePid.update( -1 * distance, 0 );
        double turningEffort = mBearingPid.update( mRoverStatus.odometry().bearing_deg, destinationBearing );
        publishJoystick( distanceEffort, turningEffort, false );
        return DriveStatus::OnCourse;
    }
    cerr << "offcourse\n";
    return DriveStatus::OffCourse;
} // drive()


// Sends a joystick command to turn the rover toward the destination
// odometry. Returns true if the rover has finished turning, false
// otherwise.
bool Rover::turn( Odometry& destination )
{
    double bearing = calcBearing( mRoverStatus.odometry(), destination );
    return turn( bearing );
} // turn()

// Sends a joystick command to turn the rover. The bearing is the
// absolute bearing. Returns true if the rover has finished turning, false
// otherwise.
bool Rover::turn( double bearing )
{
    bearing = mod(bearing, 360);
    throughZero( bearing, mRoverStatus.odometry().bearing_deg );
    double turningBearingThreshold;
    if( isTurningAroundObstacle( mRoverStatus.currentState() ) )
    {
        turningBearingThreshold = 0;
    }
    else
    {
        turningBearingThreshold = mRoverConfig[ "navThresholds" ][ "turningBearing" ].GetDouble();
    }
    if( fabs( bearing - mRoverStatus.odometry().bearing_deg ) <= turningBearingThreshold )
    {
        return true;
    }
    double turningEffort = mBearingPid.update( mRoverStatus.odometry().bearing_deg, bearing );
    double minTurningEffort = mRoverConfig[ "navThresholds" ][ "minTurningEffort" ].GetDouble() * (turningEffort < 0 ? -1 : 1);
    if( isTurningAroundObstacle( mRoverStatus.currentState() ) && fabs(turningEffort) < minTurningEffort )
    {
        turningEffort = minTurningEffort;
    }
    publishJoystick( 0, turningEffort, false );
    return false;
} // turn()

// Sends a joystick command to stop the rover.
void Rover::stop()
{
    publishJoystick( 0, 0, false );
} // stop()

// Checks if the rover should be updated based on what information in
// the rover's status has changed. Returns true if the rover was
// updated, false otherwise.
// TODO: unconditionally update everygthing. When abstracting search class
// we got rid of NavStates TurnToTarget and DriveToTarget (oops) fix this soon :P
bool Rover::updateRover( RoverStatus newRoverStatus )
{
    // Rover currently on.
    if( mRoverStatus.autonState().is_auton )
    {
        // Rover turned off
        if( !newRoverStatus.autonState().is_auton )
        {
            mRoverStatus.autonState() = newRoverStatus.autonState();
            return true;
        }
        if( ( mRoverStatus.currentState() == NavState::TurnToTarget ||
              mRoverStatus.currentState() == NavState::DriveToTarget ) &&
            !isEqual( mRoverStatus.target(), newRoverStatus.target() ) )
        {
            mRoverStatus.obstacle() = newRoverStatus.obstacle();
            mRoverStatus.odometry() = newRoverStatus.odometry();
            mRoverStatus.target() = newRoverStatus.target();
            return true;
        }

        if( isTurningAroundObstacle( mRoverStatus.currentState() ) &&
            ( !isEqual( mRoverStatus.obstacle(), newRoverStatus.obstacle() ) ||
              !isEqual( mRoverStatus.odometry(), newRoverStatus.odometry() ) ) )
        {
            mRoverStatus.obstacle() = newRoverStatus.obstacle();
            mRoverStatus.odometry() = newRoverStatus.odometry();
            mRoverStatus.target() = newRoverStatus.target();
            return true;
        }

        if( !isEqual( mRoverStatus.obstacle(), newRoverStatus.obstacle() ) ||
            !isEqual( mRoverStatus.odometry(), newRoverStatus.odometry() ) ||
            !isEqual( mRoverStatus.target(), newRoverStatus.target() ) )
        {
            mRoverStatus.obstacle() = newRoverStatus.obstacle();
            mRoverStatus.odometry() = newRoverStatus.odometry();
            mRoverStatus.target() = newRoverStatus.target();
            return true;
        }
        return false;
    }

    // Rover currently off.
    else
    {
        // Rover turned on.
        if( newRoverStatus.autonState().is_auton )
        {
            mRoverStatus = newRoverStatus;
            // Calculate longitude minutes/meter conversion.
            mLongMeterInMinutes = 60 / ( EARTH_CIRCUM * cos( degreeToRadian(
                mRoverStatus.odometry().latitude_deg, mRoverStatus.odometry().latitude_min ) ) / 360 );
            return true;
        }
        return false;
    }
} // updateRover()

// Calculates the conversion from minutes to meters based on the
// rover's current latitude.
const double Rover::longMeterInMinutes() const
{
    return mLongMeterInMinutes;
}

// Gets the rover's status object.
Rover::RoverStatus& Rover::roverStatus()
{
    return mRoverStatus;
} // roverStatus()

// Gets the rover's driving pid object.
PidLoop& Rover::distancePid()
{
    return mDistancePid;
} // distancePid()

// Gets the rover's turning pid object.
PidLoop& Rover::bearingPid()
{
    return mBearingPid;
} // bearingPid()

// Publishes a joystick command with the given forwardBack and
// leftRight efforts.
void Rover::publishJoystick( const double forwardBack, const double leftRight, const bool kill )
{
    Joystick joystick;
    // power limit (0 = 50%, 1 = 0%, -1 = 100% power)
    joystick.dampen = mRoverConfig[ "joystick" ][ "dampen" ].GetDouble();
    double drivingPower = mRoverConfig[ "joystick" ][ "drivingPower" ].GetDouble();
    joystick.forward_back = drivingPower * forwardBack;
    double bearingPower = mRoverConfig[ "joystick" ][ "bearingPower" ].GetDouble();
    joystick.left_right = bearingPower * leftRight;
    joystick.kill = kill;
    string joystickChannel = mRoverConfig[ "lcmChannels" ][ "joystickChannel" ].GetString();
    mLcmObject.publish( joystickChannel, &joystick );
} // publishJoystick()

// Returns true if the two obstacle messages are equal, false
// otherwise.
bool Rover::isEqual( const Obstacle& obstacle1, const Obstacle& obstacle2 ) const
{
    if( obstacle1.detected == obstacle2.detected &&
        obstacle1.bearing == obstacle2.bearing )
    {
        return true;
    }
    return false;
} // isEqual( Obstacle )

// Returns true if the two odometry messages are equal, false
// otherwise.
bool Rover::isEqual( const Odometry& odometry1, const Odometry& odometry2 ) const
{
    if( odometry1.latitude_deg == odometry2.latitude_deg &&
        odometry1.latitude_min == odometry2.latitude_min &&
        odometry1.longitude_deg == odometry2.longitude_deg &&
        odometry1.longitude_min == odometry2.longitude_min &&
        odometry1.bearing_deg == odometry2.bearing_deg )
    {
        return true;
    }
    return false;
} // isEqual( Odometry )

// Returns true if the two target messages are equal, false
// otherwise.
bool Rover::isEqual( const Target& target1, const Target& target2 ) const
{
    if( target1.distance == target2.distance &&
        target1.bearing == target2.bearing )
    {
        return true;
    }
    return false;
} // isEqual( Target )

// Return true if the current state is TurnAroundObs or SearchTurnAroundObs,
// false otherwise.
bool Rover::isTurningAroundObstacle( const NavState currentState ) const
{
    if( currentState == NavState::TurnAroundObs ||
        currentState == NavState::SearchTurnAroundObs )
    {
        return true;
    }
    return false;
} // isTurningAroundObstacle()

/*************************************************************************/
/* TODOS */
/*************************************************************************/
